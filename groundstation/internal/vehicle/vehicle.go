// Package vehicle keeps the latest known state of every vehicle the ground
// station has heard from, keyed by MAVLink system id.
package vehicle

import (
	"sort"
	"sync"
	"time"

	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/mavlink"
)

// DroneCtrl encodes its flight state and fault cause in HEARTBEAT.custom_mode
// (firmware/include/mavlink_telemetry.h). These are the names in
// firmware/include/flight_state.h, in enum order.
var (
	FlightStates = []string{"DISARMED", "ARMING", "ARMED", "FAILSAFE_LAND", "FAILSAFE_CUT"}
	FaultCauses  = []string{"NONE", "SIGNAL_LOST", "LOW_BATTERY", "EXCESSIVE_TILT", "ESTIMATOR_DIVERGED"}
)

// HeartbeatTimeout is how long without a HEARTBEAT before a vehicle counts
// as lost. Three missed 1 Hz heartbeats, the usual ground station convention.
const HeartbeatTimeout = 3 * time.Second

// State is one vehicle, as last reported. Safe to copy.
type State struct {
	SystemID      uint8     `json:"system_id"`
	FirstSeen     time.Time `json:"first_seen"`
	LastHeartbeat time.Time `json:"last_heartbeat"`
	Connected     bool      `json:"connected"`

	FlightState string `json:"flight_state"`
	Fault       string `json:"fault"`
	Armed       bool   `json:"armed"`
	HIL         bool   `json:"hil"`

	RollRad      float64 `json:"roll_rad"`
	PitchRad     float64 `json:"pitch_rad"`
	RollRateRad  float64 `json:"roll_rate_rad_s"`
	PitchRateRad float64 `json:"pitch_rate_rad_s"`
	YawRateRad   float64 `json:"yaw_rate_rad_s"`
	TimeBootMS   uint32  `json:"time_boot_ms"`

	BatteryVolts  float64 `json:"battery_volts"`
	LoadPercent   float64 `json:"load_percent"`
	SensorsHealth uint32  `json:"sensors_health"`
	LoopOverruns  uint16  `json:"loop_overruns"`

	// NAMED_VALUE_FLOAT by name: loop_jit, loop_max, loop_sd, loop_ovr,
	// ekf_bp, ekf_bq, ekf_inn (docs/mavlink.md).
	Values map[string]float64 `json:"values"`

	Frames      map[string]uint64 `json:"frames"`
	StateEvents []Event           `json:"state_events"`
}

// Event is a flight state transition, kept so a failsafe is visible after
// the fact rather than only while it is happening.
type Event struct {
	At    time.Time `json:"at"`
	From  string    `json:"from"`
	To    string    `json:"to"`
	Fault string    `json:"fault"`
}

const maxEvents = 50

var msgNames = map[uint32]string{
	mavlink.MsgHeartbeat: "HEARTBEAT", mavlink.MsgSysStatus: "SYS_STATUS",
	mavlink.MsgAttitude: "ATTITUDE", mavlink.MsgNamedValueFloat: "NAMED_VALUE_FLOAT",
	mavlink.MsgHilActuatorControls: "HIL_ACTUATOR_CONTROLS", mavlink.MsgHilSensor: "HIL_SENSOR",
	mavlink.MsgRcChannelsOverride: "RC_CHANNELS_OVERRIDE",
}

// Store is the set of known vehicles. Safe for concurrent use.
type Store struct {
	mu       sync.RWMutex
	vehicles map[uint8]*State
	now      func() time.Time
}

// NewStore returns an empty store using the given clock (time.Now if nil).
// Replay passes the log's own clock, so heartbeat timeouts follow log time.
func NewStore(now func() time.Time) *Store {
	if now == nil {
		now = time.Now
	}
	return &Store{vehicles: map[uint8]*State{}, now: now}
}

// Apply folds one verified frame into the vehicle it came from.
func (s *Store) Apply(f mavlink.Frame) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	v, ok := s.vehicles[f.SystemID]
	if !ok {
		v = &State{
			SystemID: f.SystemID, FirstSeen: now, FlightState: "UNKNOWN", Fault: "UNKNOWN",
			Values: map[string]float64{}, Frames: map[string]uint64{},
		}
		s.vehicles[f.SystemID] = v
	}
	name := msgNames[f.MsgID]
	if name == "" {
		name = "OTHER"
	}
	v.Frames[name]++

	switch f.MsgID {
	case mavlink.MsgHeartbeat:
		hb := mavlink.DecodeHeartbeat(f.Payload)
		v.LastHeartbeat = now
		state := lookup(FlightStates, int(hb.CustomMode&0xFF))
		fault := lookup(FaultCauses, int((hb.CustomMode>>8)&0xFF))
		if state != v.FlightState {
			v.StateEvents = append(v.StateEvents, Event{At: now, From: v.FlightState, To: state, Fault: fault})
			if len(v.StateEvents) > maxEvents {
				v.StateEvents = v.StateEvents[len(v.StateEvents)-maxEvents:]
			}
		}
		v.FlightState = state
		v.Fault = fault
		v.Armed = hb.Armed()
		v.HIL = hb.BaseMode&32 != 0
	case mavlink.MsgAttitude:
		a := mavlink.DecodeAttitude(f.Payload)
		v.TimeBootMS = a.TimeBootMS
		v.RollRad, v.PitchRad = float64(a.Roll), float64(a.Pitch)
		v.RollRateRad, v.PitchRateRad, v.YawRateRad = float64(a.RollSpeed), float64(a.PitchSpeed), float64(a.YawSpeed)
	case mavlink.MsgSysStatus:
		st := mavlink.DecodeSysStatus(f.Payload)
		if st.VoltageMV != 0xFFFF {
			v.BatteryVolts = float64(st.VoltageMV) / 1000
		}
		v.LoadPercent = float64(st.Load) / 10
		v.SensorsHealth = st.SensorsHealth
		v.LoopOverruns = st.ErrorsCount[0]
	case mavlink.MsgNamedValueFloat:
		nv := mavlink.DecodeNamedValueFloat(f.Payload)
		v.Values[nv.Name] = float64(nv.Value)
	}
}

func lookup(names []string, i int) string {
	if i >= 0 && i < len(names) {
		return names[i]
	}
	return "UNKNOWN"
}

// Snapshot returns a deep copy of every vehicle, sorted by system id, with
// Connected computed against the store's clock.
func (s *Store) Snapshot() []State {
	s.mu.RLock()
	defer s.mu.RUnlock()
	now := s.now()
	out := make([]State, 0, len(s.vehicles))
	for _, v := range s.vehicles {
		c := *v
		c.Connected = !v.LastHeartbeat.IsZero() && now.Sub(v.LastHeartbeat) <= HeartbeatTimeout
		c.Values = make(map[string]float64, len(v.Values))
		for k, x := range v.Values {
			c.Values[k] = x
		}
		c.Frames = make(map[string]uint64, len(v.Frames))
		for k, x := range v.Frames {
			c.Frames[k] = x
		}
		c.StateEvents = append([]Event(nil), v.StateEvents...)
		out = append(out, c)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].SystemID < out[j].SystemID })
	return out
}

// Get returns one vehicle.
func (s *Store) Get(id uint8) (State, bool) {
	for _, v := range s.Snapshot() {
		if v.SystemID == id {
			return v, true
		}
	}
	return State{}, false
}
