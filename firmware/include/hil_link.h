#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "flight_loop.h"
#include "mavlink_lite.h"

/**
 * Hardware in the loop over MAVLink.
 *
 * In the HIL build the flight controller runs on the real board, but its IMU
 * and its motors are a simulator's. The simulator sends HIL_SENSOR, the board
 * runs one FlightLoop step on it, and replies with HIL_ACTUATOR_CONTROLS. It
 * is lockstep: the board steps once per sample it receives and the simulator
 * waits for the reply before advancing physics, so a slow link slows the
 * simulation down instead of starving the controller. Sticks come in as
 * RC_CHANNELS_OVERRIDE; when they stop coming, the receiver is treated as
 * lost exactly as the real one is, and the failsafe flies the airframe down.
 *
 * What HIL adds over SIL is the board: its real clock, its real float unit,
 * its real serial driver, and the real time the step takes on it. The
 * controller code is identical in all three, by construction (flight_loop.h).
 *
 * Frames and units
 * ----------------
 * MAVLink is body FRD (x forward, y right, z down) in SI units. The firmware's
 * rates already follow FRD: positive roll is right side down, positive pitch
 * nose up, and its Euler kinematics are the standard FRD ones. Its
 * accelerometer convention is the reaction one, +1 g on z at rest, where an
 * FRD accelerometer reads -9.81 m/s^2. So rates pass straight through and the
 * specific force is negated and scaled. The SIL loopback flies every scenario
 * through this conversion, which is what pins the signs down.
 *
 * Motor order in HIL_ACTUATOR_CONTROLS is DroneCtrl's own, FL FR BL BR in
 * controls[0..3], normalised so 0 is ESC_MIN and 1 is ESC_MAX. That is not
 * PX4's order. A simulator other than ours has to map it, and docs/hil.md
 * says so in its first paragraph.
 *
 * No Arduino dependency: the board code feeds it bytes and writes out what it
 * returns, and the SIL harness does the same with a byte buffer.
 */

namespace hil {

constexpr float kStandardGravity = 9.80665f;
/** Same window the PWM receiver uses before declaring the link lost. */
constexpr uint32_t kRcTimeoutUs = 500000;

inline ImuSample fromHilSensor(const mavlink::HilSensor& m) {
    ImuSample s;
    s.p = m.xgyro;
    s.q = m.ygyro;
    s.r = m.zgyro;
    s.ax = -m.xacc / kStandardGravity;
    s.ay = -m.yacc / kStandardGravity;
    s.az = -m.zacc / kStandardGravity;
    return s;
}

/** The inverse, for the simulator side and the loopback test. */
inline mavlink::HilSensor toHilSensor(const ImuSample& s, uint64_t time_usec) {
    mavlink::HilSensor m;
    m.time_usec = time_usec;
    m.xgyro = s.p;
    m.ygyro = s.q;
    m.zgyro = s.r;
    m.xacc = -s.ax * kStandardGravity;
    m.yacc = -s.ay * kStandardGravity;
    m.zacc = -s.az * kStandardGravity;
    m.fields_updated = 0x3F; // accel and gyro, all three axes
    return m;
}

/** One RC channel in microseconds to the receiver's normalised range. */
inline float rcAxis(uint16_t us) {
    float v = (static_cast<float>(us) - 1500.0f) / 500.0f;
    return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
}

inline float rcThrottle(uint16_t us) {
    float v = (static_cast<float>(us) - 1000.0f) / 1000.0f;
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

/** Channels are AETR, the same order receiver.h reads its pins in. */
inline PilotInput fromRcOverride(const mavlink::RcChannelsOverride& rc) {
    PilotInput in;
    in.roll = rcAxis(rc.chan[0]);
    in.pitch = rcAxis(rc.chan[1]);
    in.throttle = rcThrottle(rc.chan[2]);
    in.yaw = rcAxis(rc.chan[3]);
    in.signal_lost = false;
    return in;
}

inline mavlink::HilActuatorControls toActuators(const float pwm[4], uint64_t time_usec,
                                                 bool armed) {
    mavlink::HilActuatorControls m;
    m.time_usec = time_usec;
    const float span = static_cast<float>(ESC_MAX) - static_cast<float>(ESC_MIN);
    for (int i = 0; i < 4; ++i) {
        float u = (pwm[i] - static_cast<float>(ESC_MIN)) / span;
        m.controls[i] = u < 0.0f ? 0.0f : (u > 1.0f ? 1.0f : u);
    }
    m.mode = mavlink::kModeFlagHilEnabled | (armed ? mavlink::kModeFlagSafetyArmed : 0);
    return m;
}

inline void pwmFromActuators(const mavlink::HilActuatorControls& m, float pwm[4]) {
    const float span = static_cast<float>(ESC_MAX) - static_cast<float>(ESC_MIN);
    for (int i = 0; i < 4; ++i) pwm[i] = static_cast<float>(ESC_MIN) + m.controls[i] * span;
}

/**
 * Board side of the link. Push every received byte; onSensor is called with
 * each verified HIL_SENSOR, already converted, and its timestamp.
 */
class Link {
public:
    template <typename OnSensor>
    void push(uint8_t byte, OnSensor&& on_sensor) {
        parser_.push(byte, [&](const mavlink::Frame& f) {
            mavlink::HilSensor hs;
            mavlink::RcChannelsOverride rc;
            if (f.decode(hs)) {
                last_sensor_usec_ = hs.time_usec;
                ++sensor_frames_;
                on_sensor(fromHilSensor(hs), hs.time_usec);
            } else if (f.decode(rc)) {
                pilot_ = fromRcOverride(rc);
                last_rc_usec_ = last_sensor_usec_;
                have_rc_ = true;
            }
        });
    }

    /**
     * Sticks as of simulator time now_usec. Stale or never received means
     * signal lost, with every axis centred and the throttle closed, exactly
     * as receiver.h reports a dead receiver.
     */
    PilotInput pilot(uint64_t now_usec) const {
        if (!have_rc_ || now_usec - last_rc_usec_ > kRcTimeoutUs) {
            PilotInput lost;
            lost.signal_lost = true;
            return lost;
        }
        return pilot_;
    }

    /** Frame buffer size for one HIL_ACTUATOR_CONTROLS reply. */
    static constexpr size_t kActuatorFrame = mavlink::frameSize(mavlink::HilActuatorControls::kLen);

    template <size_t N>
    size_t encodeActuators(uint8_t (&out)[N], const float pwm[4], uint64_t time_usec, bool armed) {
        return mavlink::encode(out, ep_, toActuators(pwm, time_usec, armed));
    }

    mavlink::Endpoint& endpoint() { return ep_; }
    const mavlink::ParserStats& stats() const { return parser_.stats(); }
    uint32_t sensorFrames() const { return sensor_frames_; }

private:
    mavlink::Parser parser_;
    mavlink::Endpoint ep_;
    PilotInput pilot_;
    bool have_rc_ = false;
    uint64_t last_rc_usec_ = 0;
    uint64_t last_sensor_usec_ = 0;
    uint32_t sensor_frames_ = 0;
};

} // namespace hil
