package server

import (
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/mavlink"
	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/vehicle"
)

func fixture(t *testing.T) (*Server, *httptest.Server) {
	t.Helper()
	store := vehicle.NewStore(nil)
	p := mavlink.NewParser()
	e := mavlink.Encoder{SystemID: 1, ComponentID: 1}
	for _, fr := range []struct {
		id uint32
		b  []byte
	}{
		{mavlink.MsgHeartbeat, mavlink.Heartbeat{Type: 2, BaseMode: 128 | 16 | 1, CustomMode: 3 | 1<<8}.Encode()},
		{mavlink.MsgAttitude, mavlink.Attitude{Roll: 0.1, Pitch: -0.05}.Encode()},
	} {
		b, _ := e.Frame(fr.id, fr.b)
		p.Push(b, store.Apply)
	}
	srv := New(store, func() mavlink.Stats { return p.Stats })
	return srv, httptest.NewServer(srv.Handler())
}

func get(t *testing.T, url string) (int, string) {
	t.Helper()
	resp, err := http.Get(url)
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	b, _ := io.ReadAll(resp.Body)
	return resp.StatusCode, string(b)
}

func TestReadinessFollowsInput(t *testing.T) {
	srv, ts := fixture(t)
	defer ts.Close()
	if code, _ := get(t, ts.URL+"/readyz"); code != http.StatusServiceUnavailable {
		t.Fatalf("readyz before input: %d", code)
	}
	srv.SetReady(true)
	if code, _ := get(t, ts.URL+"/readyz"); code != http.StatusOK {
		t.Fatalf("readyz after input: %d", code)
	}
	if code, _ := get(t, ts.URL+"/healthz"); code != http.StatusOK {
		t.Fatalf("healthz: %d", code)
	}
}

func TestVehicleAPI(t *testing.T) {
	_, ts := fixture(t)
	defer ts.Close()
	code, body := get(t, ts.URL+"/api/vehicles/1")
	if code != http.StatusOK {
		t.Fatalf("status %d", code)
	}
	var v vehicle.State
	if err := json.Unmarshal([]byte(body), &v); err != nil {
		t.Fatal(err)
	}
	if v.FlightState != "FAILSAFE_LAND" || v.Fault != "SIGNAL_LOST" || !v.Armed {
		t.Fatalf("decoded %+v", v)
	}
	if code, _ := get(t, ts.URL+"/api/vehicles/9"); code != http.StatusNotFound {
		t.Fatalf("unknown vehicle: %d", code)
	}
	if code, _ := get(t, ts.URL+"/api/vehicles/abc"); code != http.StatusBadRequest {
		t.Fatalf("bad id: %d", code)
	}
}

func TestMetricsExposition(t *testing.T) {
	_, ts := fixture(t)
	defer ts.Close()
	_, body := get(t, ts.URL+"/metrics")
	for _, want := range []string{
		"# TYPE dronectrl_mavlink_frames_total counter",
		"dronectrl_mavlink_frames_total 2",
		`dronectrl_vehicle_armed{system="1"} 1`,
		`dronectrl_vehicle_in_failsafe{system="1"} 1`,
		`dronectrl_vehicle_flight_state{system="1",state="FAILSAFE_LAND"} 1`,
		`dronectrl_vehicle_flight_state{system="1",state="ARMED"} 0`,
		`dronectrl_vehicle_frames_total{system="1",message="ATTITUDE"} 1`,
	} {
		if !strings.Contains(body, want) {
			t.Errorf("metrics missing %q", want)
		}
	}
	// Every sample line must parse as name{labels} value.
	for _, line := range strings.Split(strings.TrimSpace(body), "\n") {
		if strings.HasPrefix(line, "#") {
			continue
		}
		if f := strings.Fields(line); len(f) != 2 {
			t.Errorf("malformed sample %q", line)
		}
	}
}

func TestDashboardServed(t *testing.T) {
	_, ts := fixture(t)
	defer ts.Close()
	code, body := get(t, ts.URL+"/")
	if code != http.StatusOK || !strings.Contains(body, "DroneCtrl ground station") {
		t.Fatalf("dashboard: %d", code)
	}
}
