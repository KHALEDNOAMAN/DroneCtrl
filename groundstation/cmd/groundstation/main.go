// Command groundstation receives DroneCtrl MAVLink telemetry and serves it
// over HTTP: a live dashboard, JSON, Server-Sent Events and Prometheus
// metrics.
//
// Live:    groundstation -udp :14550 -http :8080
// Replay:  groundstation -replay firmware/sil/logs/signal_loss.tlog -speed 1
// Check:   groundstation -replay hover.tlog -check
//
// -check replays as fast as possible, prints what it saw, and exits non-zero
// if any frame was rejected or no heartbeat arrived. CI runs it over every
// log the SIL harness writes, which makes this service a third, independent
// reader of the firmware's telemetry.
package main

import (
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"log/slog"
	"net"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/mavlink"
	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/server"
	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/tlog"
	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/vehicle"
)

func main() {
	udpAddr := flag.String("udp", ":14550", "UDP address to receive MAVLink on")
	httpAddr := flag.String("http", ":8080", "HTTP address to serve on")
	replay := flag.String("replay", "", "replay a .tlog instead of listening on UDP")
	speed := flag.Float64("speed", 1, "replay speed, 0 for as fast as possible")
	loop := flag.Bool("loop", false, "replay the log again and again (demo deployments)")
	check := flag.Bool("check", false, "replay as fast as possible, print a summary, exit non-zero on errors")
	flag.Parse()

	log := slog.New(slog.NewJSONHandler(os.Stderr, nil))

	if *check {
		if *replay == "" {
			fmt.Fprintln(os.Stderr, "-check needs -replay")
			os.Exit(2)
		}
		os.Exit(runCheck(*replay))
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var (
		mu     sync.Mutex
		parser = mavlink.NewParser()
	)
	store := vehicle.NewStore(nil)
	linkStats := func() mavlink.Stats { mu.Lock(); defer mu.Unlock(); return parser.Stats }
	srv := server.New(store, linkStats)
	feed := func(b []byte) {
		mu.Lock()
		defer mu.Unlock()
		parser.Push(b, store.Apply)
	}

	httpSrv := &http.Server{Addr: *httpAddr, Handler: srv.Handler(), ReadHeaderTimeout: 5 * time.Second}
	go func() {
		log.Info("http listening", "addr", *httpAddr)
		if err := httpSrv.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
			log.Error("http server failed", "err", err)
			stop()
		}
	}()

	if *replay != "" {
		srv.SetReady(true)
		go func() {
			for {
				if err := replayFile(ctx, *replay, *speed, feed); err != nil && ctx.Err() == nil {
					log.Error("replay failed", "err", err)
					stop()
					return
				}
				if !*loop || ctx.Err() != nil {
					log.Info("replay finished", "file", *replay)
					return
				}
			}
		}()
	} else {
		conn, err := net.ListenPacket("udp", *udpAddr)
		if err != nil {
			log.Error("udp listen failed", "addr", *udpAddr, "err", err)
			os.Exit(1)
		}
		srv.SetReady(true)
		log.Info("udp listening", "addr", conn.LocalAddr().String())
		go func() {
			<-ctx.Done()
			_ = conn.Close()
		}()
		go func() {
			buf := make([]byte, 65535)
			for {
				n, _, err := conn.ReadFrom(buf)
				if err != nil {
					if ctx.Err() == nil {
						log.Error("udp read failed", "err", err)
					}
					return
				}
				feed(buf[:n])
			}
		}()
	}

	<-ctx.Done()
	srv.SetReady(false)
	shutdown, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_ = httpSrv.Shutdown(shutdown)
	log.Info("stopped")
}

// replayFile feeds a tlog into the parser, pacing by its own timestamps.
func replayFile(ctx context.Context, path string, speed float64, feed func([]byte)) error {
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	defer f.Close()
	r := tlog.NewReader(f)
	start := time.Now()
	var first uint64
	for i := 0; ; i++ {
		rec, err := r.Next()
		if errors.Is(err, io.EOF) {
			return nil
		}
		if err != nil {
			return err
		}
		if i == 0 {
			first = rec.Usec
		}
		if speed > 0 {
			due := start.Add(time.Duration(float64(rec.Usec-first)/speed) * time.Microsecond)
			if d := time.Until(due); d > 0 {
				select {
				case <-ctx.Done():
					return nil
				case <-time.After(d):
				}
			}
		}
		feed(rec.Frame)
	}
}

// runCheck replays a log on the log's own clock and reports what it found.
func runCheck(path string) int {
	f, err := os.Open(path)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		return 1
	}
	defer f.Close()

	var logNow time.Time
	store := vehicle.NewStore(func() time.Time { return logNow })
	p := mavlink.NewParser()
	r := tlog.NewReader(f)
	epoch := time.Unix(0, 0)
	for {
		rec, err := r.Next()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			return 1
		}
		logNow = epoch.Add(time.Duration(rec.Usec) * time.Microsecond)
		p.Push(rec.Frame, store.Apply)
	}

	vs := store.Snapshot()
	out := map[string]any{"file": path, "link": p.Stats, "vehicles": vs}
	b, _ := json.MarshalIndent(out, "", "  ")
	fmt.Println(string(b))

	switch {
	case p.Stats.CRCErrors > 0:
		fmt.Fprintf(os.Stderr, "FAIL: %d frames rejected\n", p.Stats.CRCErrors)
		return 1
	case p.Stats.SeqGaps > 0:
		fmt.Fprintf(os.Stderr, "FAIL: %d sequence gaps in a lossless log\n", p.Stats.SeqGaps)
		return 1
	case len(vs) == 0 || vs[0].LastHeartbeat.IsZero():
		fmt.Fprintln(os.Stderr, "FAIL: no heartbeat")
		return 1
	}
	return 0
}
