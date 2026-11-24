// Package server exposes vehicle state over HTTP: JSON for tools, Server-Sent
// Events for live views, Prometheus text for monitoring, and health probes
// for Kubernetes.
package server

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"net/http"
	"sort"
	"strconv"
	"strings"
	"sync/atomic"
	"time"

	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/mavlink"
	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/vehicle"
)

//go:embed index.html
var indexHTML []byte

// LinkStats reports the parser's counters. Supplied by whoever owns the
// parser, so the server never touches it from another goroutine.
type LinkStats func() mavlink.Stats

// Server is the HTTP side of the ground station.
type Server struct {
	store *vehicle.Store
	link  LinkStats
	ready atomic.Bool
	// StreamInterval is how often /api/stream pushes a snapshot.
	StreamInterval time.Duration
}

// New returns a server over a store.
func New(store *vehicle.Store, link LinkStats) *Server {
	return &Server{store: store, link: link, StreamInterval: 200 * time.Millisecond}
}

// SetReady marks the telemetry input as up. Until then /readyz fails, so a
// pod whose UDP socket did not bind never receives traffic.
func (s *Server) SetReady(v bool) { s.ready.Store(v) }

// Handler returns the routes.
func (s *Server) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("GET /{$}", func(w http.ResponseWriter, _ *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		_, _ = w.Write(indexHTML)
	})
	mux.HandleFunc("GET /healthz", func(w http.ResponseWriter, _ *http.Request) {
		_, _ = w.Write([]byte("ok\n"))
	})
	mux.HandleFunc("GET /readyz", func(w http.ResponseWriter, _ *http.Request) {
		if !s.ready.Load() {
			http.Error(w, "telemetry input not ready", http.StatusServiceUnavailable)
			return
		}
		_, _ = w.Write([]byte("ready\n"))
	})
	mux.HandleFunc("GET /api/vehicles", func(w http.ResponseWriter, _ *http.Request) {
		writeJSON(w, s.store.Snapshot())
	})
	mux.HandleFunc("GET /api/vehicles/{id}", func(w http.ResponseWriter, r *http.Request) {
		id, err := strconv.ParseUint(r.PathValue("id"), 10, 8)
		if err != nil {
			http.Error(w, "system id is 0..255", http.StatusBadRequest)
			return
		}
		v, ok := s.store.Get(uint8(id))
		if !ok {
			http.Error(w, "no such vehicle", http.StatusNotFound)
			return
		}
		writeJSON(w, v)
	})
	mux.HandleFunc("GET /api/link", func(w http.ResponseWriter, _ *http.Request) {
		writeJSON(w, s.link())
	})
	mux.HandleFunc("GET /api/stream", s.stream)
	mux.HandleFunc("GET /metrics", s.metrics)
	return mux
}

func writeJSON(w http.ResponseWriter, v any) {
	w.Header().Set("Content-Type", "application/json")
	enc := json.NewEncoder(w)
	enc.SetIndent("", "  ")
	_ = enc.Encode(v)
}

// stream pushes the vehicle list as Server-Sent Events until the client goes.
func (s *Server) stream(w http.ResponseWriter, r *http.Request) {
	flusher, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming unsupported", http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")
	t := time.NewTicker(s.StreamInterval)
	defer t.Stop()
	for {
		b, _ := json.Marshal(s.store.Snapshot())
		if _, err := fmt.Fprintf(w, "data: %s\n\n", b); err != nil {
			return
		}
		flusher.Flush()
		select {
		case <-r.Context().Done():
			return
		case <-t.C:
		}
	}
}

// metrics writes the Prometheus text exposition format by hand: eleven
// gauges and counters do not justify a client library and its dependency
// tree in a service that otherwise needs only the standard library.
func (s *Server) metrics(w http.ResponseWriter, _ *http.Request) {
	w.Header().Set("Content-Type", "text/plain; version=0.0.4")
	var b strings.Builder

	help := func(name, kind, text string) {
		fmt.Fprintf(&b, "# HELP %s %s\n# TYPE %s %s\n", name, text, name, kind)
	}

	ls := s.link()
	help("dronectrl_mavlink_frames_total", "counter", "MAVLink frames that passed their checksum.")
	fmt.Fprintf(&b, "dronectrl_mavlink_frames_total %d\n", ls.FramesOK)
	help("dronectrl_mavlink_crc_errors_total", "counter", "Frames rejected on checksum.")
	fmt.Fprintf(&b, "dronectrl_mavlink_crc_errors_total %d\n", ls.CRCErrors)
	help("dronectrl_mavlink_seq_gaps_total", "counter", "Frames inferred lost from sequence numbers.")
	fmt.Fprintf(&b, "dronectrl_mavlink_seq_gaps_total %d\n", ls.SeqGaps)
	help("dronectrl_mavlink_unknown_total", "counter", "Well formed frames of messages this service does not decode.")
	fmt.Fprintf(&b, "dronectrl_mavlink_unknown_total %d\n", ls.UnknownIDs)

	vs := s.store.Snapshot()
	type series struct {
		name, kind, help string
		value            func(v vehicle.State) (float64, bool)
	}
	b2f := func(x bool) float64 {
		if x {
			return 1
		}
		return 0
	}
	value := func(key string) func(v vehicle.State) (float64, bool) {
		return func(v vehicle.State) (float64, bool) { x, ok := v.Values[key]; return x, ok }
	}
	all := []series{
		{"dronectrl_vehicle_connected", "gauge", "1 while heartbeats arrive within the timeout.",
			func(v vehicle.State) (float64, bool) { return b2f(v.Connected), true }},
		{"dronectrl_vehicle_armed", "gauge", "1 while the vehicle reports itself armed.",
			func(v vehicle.State) (float64, bool) { return b2f(v.Armed), true }},
		{"dronectrl_vehicle_in_failsafe", "gauge", "1 in FAILSAFE_LAND or FAILSAFE_CUT.",
			func(v vehicle.State) (float64, bool) {
				return b2f(strings.HasPrefix(v.FlightState, "FAILSAFE")), true
			}},
		{"dronectrl_attitude_roll_radians", "gauge", "Estimated roll.",
			func(v vehicle.State) (float64, bool) { return v.RollRad, true }},
		{"dronectrl_attitude_pitch_radians", "gauge", "Estimated pitch.",
			func(v vehicle.State) (float64, bool) { return v.PitchRad, true }},
		{"dronectrl_battery_volts", "gauge", "Pack voltage.",
			func(v vehicle.State) (float64, bool) { return v.BatteryVolts, v.BatteryVolts > 0 }},
		{"dronectrl_loop_load_ratio", "gauge", "Control step time over the loop period.",
			func(v vehicle.State) (float64, bool) { return v.LoadPercent / 100, true }},
		{"dronectrl_loop_jitter_max_microseconds", "gauge", "Worst period deviation in the last second.",
			value("loop_jit")},
		{"dronectrl_loop_period_stddev_microseconds", "gauge", "Loop period standard deviation in the last second.",
			value("loop_sd")},
		{"dronectrl_loop_overruns", "gauge", "Missed loop deadlines since boot, as the vehicle reports it.",
			value("loop_ovr")},
		{"dronectrl_ekf_innovation_g", "gauge", "Accelerometer innovation magnitude.",
			value("ekf_inn")},
	}
	for _, sr := range all {
		help(sr.name, sr.kind, sr.help)
		for _, v := range vs {
			x, ok := sr.value(v)
			if !ok {
				continue
			}
			fmt.Fprintf(&b, "%s{system=\"%d\"} %g\n", sr.name, v.SystemID, x)
		}
	}

	// One series per possible state, 1 for the current one. Putting the state
	// on every gauge as a label would start new series on each transition.
	help("dronectrl_vehicle_flight_state", "gauge", "1 for the flight state the vehicle is in, 0 for the others.")
	for _, v := range vs {
		for _, st := range vehicle.FlightStates {
			fmt.Fprintf(&b, "dronectrl_vehicle_flight_state{system=\"%d\",state=%q} %g\n",
				v.SystemID, st, b2f(v.FlightState == st))
		}
	}

	help("dronectrl_vehicle_frames_total", "counter", "Frames received per vehicle and message.")
	for _, v := range vs {
		names := make([]string, 0, len(v.Frames))
		for k := range v.Frames {
			names = append(names, k)
		}
		sort.Strings(names)
		for _, k := range names {
			fmt.Fprintf(&b, "dronectrl_vehicle_frames_total{system=\"%d\",message=%q} %d\n", v.SystemID, k, v.Frames[k])
		}
	}
	_, _ = w.Write([]byte(b.String()))
}
