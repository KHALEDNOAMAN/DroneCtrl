package vehicle

import (
	"errors"
	"io"
	"os"
	"testing"
	"time"

	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/mavlink"
	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/tlog"
)

func replay(t *testing.T, path string) *Store {
	t.Helper()
	f, err := os.Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	var now time.Time
	s := NewStore(func() time.Time { return now })
	p := mavlink.NewParser()
	r := tlog.NewReader(f)
	for {
		rec, err := r.Next()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			t.Fatal(err)
		}
		now = time.Unix(0, 0).Add(time.Duration(rec.Usec) * time.Microsecond)
		p.Push(rec.Frame, s.Apply)
	}
	return s
}

// The SIL signal loss scenario, as the ground station sees it: armed, then a
// failsafe landing, then the cut once the landing ramp reaches zero.
func TestSignalLossTimeline(t *testing.T) {
	s := replay(t, "../../testdata/signal_loss.tlog")
	vs := s.Snapshot()
	if len(vs) != 1 {
		t.Fatalf("%d vehicles, want 1", len(vs))
	}
	v := vs[0]
	var path []string
	for _, e := range v.StateEvents {
		path = append(path, e.To)
	}
	// The first heartbeat leaves on the first control step, by which time the
	// scripted pilot is already holding the arm gesture.
	want := []string{"ARMING", "ARMED", "FAILSAFE_LAND", "FAILSAFE_CUT"}
	if len(path) != len(want) {
		t.Fatalf("state path %v, want %v", path, want)
	}
	for i := range want {
		if path[i] != want[i] {
			t.Fatalf("state path %v, want %v", path, want)
		}
	}
	if v.Fault != "SIGNAL_LOST" {
		t.Fatalf("fault %s, want SIGNAL_LOST", v.Fault)
	}
	if _, ok := v.Values["loop_jit"]; !ok {
		t.Fatal("loop timing values missing")
	}
	if v.BatteryVolts < 12 || v.BatteryVolts > 13 {
		t.Fatalf("battery %.2f V, the scenario flies a 12.4 V pack", v.BatteryVolts)
	}
}

func TestConnectedFollowsHeartbeatTimeout(t *testing.T) {
	now := time.Unix(100, 0)
	s := NewStore(func() time.Time { return now })
	e := mavlink.Encoder{SystemID: 7, ComponentID: 1}
	b, _ := e.Frame(mavlink.MsgHeartbeat, mavlink.Heartbeat{Type: 2, BaseMode: 128, CustomMode: 2}.Encode())
	p := mavlink.NewParser()
	p.Push(b, s.Apply)

	v, _ := s.Get(7)
	if !v.Connected || !v.Armed || v.FlightState != "ARMED" {
		t.Fatalf("after a heartbeat: %+v", v)
	}
	now = now.Add(HeartbeatTimeout + time.Millisecond)
	if v, _ := s.Get(7); v.Connected {
		t.Fatal("still connected after the heartbeat timeout")
	}
}
