#pragma once

#include <stddef.h>
#include <stdint.h>

#include "flight_loop.h"
#include "flight_state.h"
#include "loop_timing.h"
#include "mavlink_lite.h"

/**
 * What the flight controller says about itself over MAVLink, and how often.
 *
 * The control loop hands this a snapshot each cycle and it decides which
 * frames are due. It never blocks and never decides what a frame costs to
 * send: the caller passes an emit callback that returns false when the link
 * has no room, and a refused frame is counted and dropped rather than queued.
 * On the Nano that is the difference between telemetry and a control loop
 * that stalls waiting on a 64 byte serial buffer.
 *
 * Rates are chosen to fit the link, and the host tests check the arithmetic:
 * at the default rates the stream needs well under half of a 115200 baud UART.
 */

namespace telemetry {

struct Snapshot {
    uint32_t time_ms = 0;
    float roll_rad = 0.0f, pitch_rad = 0.0f;
    float p = 0.0f, q = 0.0f, r = 0.0f; ///< rad/s
    FlightState state = FlightState::DISARMED;
    FaultCause fault = FaultCause::NONE;
    float battery_v = 0.0f;
    float bias_p_dps = 0.0f, bias_q_dps = 0.0f;
    float innovation_g = 0.0f;
    LoopTiming::Report timing;
    uint32_t total_overruns = 0;
    /** Frames the inbound parser rejected, reported as link errors. */
    uint32_t link_errors = 0;
    bool receiver_ok = true;
    bool hil = false;
};

/**
 * The parts of a snapshot that come from the controller itself. Shared by
 * main.cpp and the SIL harness so the telemetry a ground station sees from a
 * simulated flight is built exactly as the board builds it.
 */
inline Snapshot snapshotOf(const FlightLoop& flight, const ImuSample& s, uint32_t now_ms,
                           float battery_v) {
    Snapshot snap;
    snap.time_ms = now_ms;
    snap.roll_rad = flight.ekf.getRollRad();
    snap.pitch_rad = flight.ekf.getPitchRad();
    snap.p = s.p;
    snap.q = s.q;
    snap.r = s.r;
    snap.state = flight.fsm.state();
    snap.fault = flight.fsm.fault();
    snap.battery_v = battery_v;
    snap.bias_p_dps = flight.ekf.getBiasP() * 57.2957795f;
    snap.bias_q_dps = flight.ekf.getBiasQ() * 57.2957795f;
    const float i0 = flight.ekf.getInnovation(0), i1 = flight.ekf.getInnovation(1),
                i2 = flight.ekf.getInnovation(2);
    snap.innovation_g = sqrtf(i0 * i0 + i1 * i1 + i2 * i2);
    return snap;
}

struct Rates {
    uint16_t heartbeat_ms = 1000;
    uint16_t sys_status_ms = 500;
    uint16_t attitude_ms = 40; ///< 25 Hz; the ESP32 build runs 50 Hz
    /** One named value per tick, cycling, so they cost one frame at a time. */
    uint16_t named_value_ms = 100;
};

/** HEARTBEAT for a flight state. Mapping is documented in docs/mavlink.md. */
inline mavlink::Heartbeat heartbeatFor(FlightState state, FaultCause fault, bool hil) {
    using namespace mavlink;
    Heartbeat hb;
    hb.type = kMavTypeQuadrotor;
    hb.autopilot = kMavAutopilotGeneric;
    // custom_mode carries our own state and fault so a ground station that
    // knows DroneCtrl can show them exactly; anything else still gets the
    // standard flags below.
    hb.custom_mode = static_cast<uint32_t>(state) | (static_cast<uint32_t>(fault) << 8);
    hb.base_mode = kModeFlagCustomEnabled | kModeFlagStabilizeEnabled | kModeFlagManualInput;
    if (hil) hb.base_mode |= kModeFlagHilEnabled;

    switch (state) {
    case FlightState::DISARMED:
    case FlightState::ARMING:
        hb.system_status = kMavStateStandby;
        break;
    case FlightState::ARMED:
        hb.system_status = kMavStateActive;
        hb.base_mode |= kModeFlagSafetyArmed;
        break;
    case FlightState::FAILSAFE_LAND:
        // Still flying, under the state machine's control: armed, critical.
        hb.system_status = kMavStateCritical;
        hb.base_mode |= kModeFlagSafetyArmed;
        break;
    case FlightState::FAILSAFE_CUT:
        hb.system_status = kMavStateEmergency;
        break;
    }
    return hb;
}

class Scheduler {
public:
    explicit Scheduler(const Rates& rates = Rates()) : rates_(rates) {}

    mavlink::Endpoint& endpoint() { return ep_; }

    /**
     * Emit whatever is due. emit(const uint8_t* frame, size_t len) returns
     * false if the link cannot take the frame right now.
     */
    template <typename Emit>
    void poll(const Snapshot& s, Emit&& emit) {
        if (due(last_hb_, rates_.heartbeat_ms, s.time_ms)) {
            send(heartbeatFor(s.state, s.fault, s.hil), emit);
        }
        if (due(last_att_, rates_.attitude_ms, s.time_ms)) {
            mavlink::Attitude a;
            a.time_boot_ms = s.time_ms;
            a.roll = s.roll_rad;
            a.pitch = s.pitch_rad;
            a.yaw = 0.0f; // no magnetometer, so no heading to report
            a.rollspeed = s.p;
            a.pitchspeed = s.q;
            a.yawspeed = s.r;
            send(a, emit);
        }
        if (due(last_sys_, rates_.sys_status_ms, s.time_ms)) {
            send(sysStatus(s), emit);
        }
        if (due(last_nv_, rates_.named_value_ms, s.time_ms)) {
            send(namedValue(s, nv_index_), emit);
            nv_index_ = static_cast<uint8_t>((nv_index_ + 1) % kNamedValues);
        }
    }

    uint32_t framesSent() const { return sent_; }
    uint32_t framesDropped() const { return dropped_; }
    uint32_t bytesSent() const { return bytes_; }

    static constexpr uint8_t kNamedValues = 7;

    /** Exposed for the tests: the name and value of named slot i. */
    static mavlink::NamedValueFloat namedValue(const Snapshot& s, uint8_t i) {
        mavlink::NamedValueFloat nv;
        nv.time_boot_ms = s.time_ms;
        switch (i) {
        case 0: nv.setName("loop_jit"); nv.value = (float)s.timing.jitter_max_us; break;
        case 1: nv.setName("loop_max"); nv.value = (float)s.timing.period_max_us; break;
        case 2: nv.setName("loop_sd"); nv.value = s.timing.period_stddev_us; break;
        case 3: nv.setName("loop_ovr"); nv.value = (float)s.total_overruns; break;
        case 4: nv.setName("ekf_bp"); nv.value = s.bias_p_dps; break;
        case 5: nv.setName("ekf_bq"); nv.value = s.bias_q_dps; break;
        default: nv.setName("ekf_inn"); nv.value = s.innovation_g; break;
        }
        return nv;
    }

    static mavlink::SysStatus sysStatus(const Snapshot& s) {
        using namespace mavlink;
        SysStatus st;
        const uint32_t present = kSensorGyro | kSensorAccel | kSensorRateControl |
                                 kSensorAttitudeStabilization | kSensorMotorOutputs |
                                 kSensorRcReceiver | kSensorAhrs | kSensorBattery;
        st.sensors_present = present;
        st.sensors_enabled = present;
        uint32_t health = present;
        if (!s.receiver_ok) health &= ~kSensorRcReceiver;
        if (s.fault == FaultCause::ESTIMATOR_DIVERGED) health &= ~(kSensorAhrs | kSensorGyro | kSensorAccel);
        if (s.fault == FaultCause::LOW_BATTERY) health &= ~kSensorBattery;
        st.sensors_health = health;

        float load = s.timing.utilisation * 1000.0f;
        if (load < 0.0f) load = 0.0f;
        if (load > 1000.0f) load = 1000.0f;
        st.load = static_cast<uint16_t>(load + 0.5f);

        const float mv = s.battery_v * 1000.0f;
        st.voltage_battery = (mv > 0.0f && mv < 65535.0f) ? static_cast<uint16_t>(mv + 0.5f) : 0xFFFF;
        st.current_battery = -1;
        st.battery_remaining = -1;
        st.errors_comm = s.link_errors > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(s.link_errors);
        // errors_count1 carries missed loop deadlines, which is the closest
        // thing the standard message has to "the autopilot fell behind".
        st.errors_count[0] = s.total_overruns > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(s.total_overruns);
        return st;
    }

private:
    Rates rates_;
    mavlink::Endpoint ep_;
    uint32_t last_hb_ = 0, last_att_ = 0, last_sys_ = 0, last_nv_ = 0;
    bool started_ = false;
    uint8_t nv_index_ = 0;
    uint32_t sent_ = 0, dropped_ = 0, bytes_ = 0;

    /**
     * True once per period. Everything is due on the first call, so a ground
     * station sees a heartbeat the moment the link comes up. Unsigned
     * subtraction, so millis() rollover is harmless.
     */
    bool due(uint32_t& last, uint16_t period_ms, uint32_t now) {
        if (period_ms == 0) return false;
        if (!started_) {
            started_ = true;
            last_hb_ = last_att_ = last_sys_ = last_nv_ = now - 0xFFFFu;
        }
        if (now - last < period_ms) return false;
        last = now;
        return true;
    }

    template <typename Msg, typename Emit>
    void send(const Msg& msg, Emit& emit) {
        uint8_t frame[mavlink::frameSize(mavlink::SysStatus::kLen)];
        const uint8_t seq_before = ep_.seq;
        const size_t n = mavlink::encode(frame, ep_, msg);
        if (n == 0) return;
        if (emit(static_cast<const uint8_t*>(frame), n)) {
            ++sent_;
            bytes_ += static_cast<uint32_t>(n);
        } else {
            // Give the sequence number back: a frame that never left is not
            // a gap the receiver should count as loss.
            ep_.seq = seq_before;
            ++dropped_;
        }
    }
};

} // namespace telemetry
