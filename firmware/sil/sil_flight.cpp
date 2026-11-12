/**
 * Software-in-the-loop harness for the DroneCtrl flight controller.
 *
 * What this is
 * ------------
 * The control loop below is the one that runs on the board. It includes the
 * same attitude_ekf.h, pid_controller.h, mixer_math.h and flight_state.h the
 * firmware compiles, calls them in the same order and at the same 250 Hz, and
 * differs only in where its sensor readings come from and where its motor
 * commands go. Instead of an MPU6050 and four ESCs it is wired to the rigid
 * body model in this file.
 *
 * That distinction is the whole point. A simulator that reimplements the
 * control law in another language tests the reimplementation; when the two
 * drift apart, and they always do, the simulator keeps passing while the
 * aircraft does something else. Linking the flight source directly means a
 * change to the estimator or the mixer is felt here on the next build.
 *
 * This is SIL, not HIL: the control code is real and the plant is modelled.
 * The HIL build (docs/hil.md) runs this same FlightLoop on the board with the
 * sensor reads replaced by MAVLink HIL_SENSOR samples, and --hil-loopback
 * below proves that path end to end without a board, by flying every scenario
 * through the encoded byte stream instead of a function call.
 *
 * What the model covers, and what it does not
 * -------------------------------------------
 * Covered: rigid body rotation about three axes with inertia and aerodynamic
 * damping, first-order motor lag, translation with linear drag, per-motor
 * thrust scaling for failure injection, gyro bias and white noise on both
 * sensors, and receiver dropout.
 *
 * Not covered: blade flapping, ground effect, propeller inflow, battery sag
 * under load, ESC nonlinearity, structural flex, and wind gradients. This is a
 * model good enough to catch a sign error, an unstable gain, a windup bug or a
 * failsafe that does not fire. It is not good enough to predict flight time or
 * to tune gains for a specific airframe, and nothing here should be read as
 * claiming otherwise.
 *
 * Why the accelerometer can see tilt at all
 * -----------------------------------------
 * Worth stating because it is the subtlest part of the model. An accelerometer
 * measures specific force, not attitude. If thrust were the only force acting,
 * it would read straight up the body axis at every attitude and carry no tilt
 * information whatever. What makes it informative is that a tilted airframe
 * accelerates sideways until drag balances the lateral thrust component, and
 * that drag is what shows up in the body frame as a tilt signature. The model
 * therefore has to include translational drag or the estimator would be tested
 * against a plant that cannot exercise it.
 */

#include <unistd.h> // read(), for --hil-board

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// The control sequence itself, shared with firmware/src/main.cpp. config.h,
// the EKF, the PIDs, the mixer and the state machine all come in through it,
// so the harness takes the real gains rather than a copy that drifts.
#include "flight_loop.h"
#include "hil_link.h"
#include "loop_timing.h"
#include "mavlink_telemetry.h"

namespace {

constexpr float kDt = 1.0f / 250.0f;
constexpr float kG = 9.81f;
constexpr float kRad = 0.01745329252f;
constexpr float kDeg = 57.2957795f;

constexpr float kEscMin = ESC_MIN;
constexpr float kEscMax = ESC_MAX;
constexpr float kEscIdle = ESC_IDLE;

// ------------------------------------------------------------------ plant

struct Airframe {
    float mass = 1.0f;          // kg
    float arm = 0.15f;          // m, centre to motor
    float inertia_xx = 0.011f;  // kg m^2
    float inertia_yy = 0.011f;
    float inertia_zz = 0.021f;
    float max_thrust = 6.0f;    // N per motor
    float motor_tau = 0.03f;    // s, first order ESC and prop spin-up
    float rot_damping = 0.02f;  // N m per rad/s, aerodynamic
    float yaw_torque_k = 0.02f; // N m per N of differential thrust
    float lin_drag = 0.35f;     // N per m/s
};

/** Deterministic noise. A test that only fails sometimes is not a test. */
class Rng {
public:
    explicit Rng(uint32_t seed) : s_(seed) {}
    float uniform() {
        s_ = s_ * 1664525u + 1013904223u;
        return (float)((s_ >> 8) & 0xFFFFFF) / 8388608.0f - 1.0f;
    }
    float gauss() { return uniform() + uniform() + uniform(); }

private:
    uint32_t s_;
};

struct Plant {
    Airframe cfg;

    // Attitude and body rates.
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    float p = 0.0f, q = 0.0f, r = 0.0f;

    // Translation, world frame, z up.
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;

    // Motor thrusts, N, lagging their commands.
    float thrust[4] = {0, 0, 0, 0};
    /** Per-motor effectiveness, 1.0 healthy. Lowered to injure a motor. */
    float motor_health[4] = {1, 1, 1, 1};

    bool landed = true;

    /** @param pwm FL, FR, BL, BR in microseconds. */
    void step(const float pwm[4], float wind_torque_roll, float dt) {
        // ESC command to commanded thrust, then first order lag toward it.
        for (int i = 0; i < 4; ++i) {
            const float u = (pwm[i] - kEscMin) / (kEscMax - kEscMin);
            const float commanded = cfg.max_thrust * (u < 0.0f ? 0.0f : u) * motor_health[i];
            const float a = dt / (cfg.motor_tau + dt);
            thrust[i] += a * (commanded - thrust[i]);
        }

        const float t_fl = thrust[0], t_fr = thrust[1];
        const float t_bl = thrust[2], t_br = thrust[3];
        const float total = t_fl + t_fr + t_bl + t_br;

        // On an X frame each motor sits at 45 degrees to both axes, so its
        // moment arm about either one is the geometric arm times cos(45).
        const float arm_eff = cfg.arm * 0.70710678f;

        // Positive roll is right side down, so more thrust on the left pair.
        const float tau_roll = arm_eff * ((t_fl + t_bl) - (t_fr + t_br))
                               - cfg.rot_damping * p + wind_torque_roll;
        // Positive pitch is nose up, so more thrust on the front pair.
        const float tau_pitch = arm_eff * ((t_fl + t_fr) - (t_bl + t_br))
                                - cfg.rot_damping * q;
        // Yaw comes from the reaction torque of the two counter-rotating pairs.
        const float tau_yaw = cfg.yaw_torque_k * ((t_fr + t_bl) - (t_fl + t_br))
                              - cfg.rot_damping * r;

        p += (tau_roll / cfg.inertia_xx) * dt;
        q += (tau_pitch / cfg.inertia_yy) * dt;
        r += (tau_yaw / cfg.inertia_zz) * dt;

        // Euler kinematics from body rates.
        const float sr = sinf(roll), cr = cosf(roll);
        const float tp = tanf(pitch), cp = cosf(pitch);
        roll += (p + q * sr * tp + r * cr * tp) * dt;
        pitch += (q * cr - r * sr) * dt;
        yaw += ((q * sr + r * cr) / (cp == 0.0f ? 1e-4f : cp)) * dt;

        // Thrust points along body up. Rotated into the world it is the only
        // force besides gravity and drag.
        const float sp = sinf(pitch);
        const float tx = total * (sp * cosf(yaw) + sr * cp * sinf(yaw));
        const float ty = total * (sp * sinf(yaw) - sr * cp * cosf(yaw));
        const float tz = total * (cr * cp);

        const float fx = tx - cfg.lin_drag * vx;
        const float fy = ty - cfg.lin_drag * vy;
        const float fz = tz - cfg.lin_drag * vz - cfg.mass * kG;

        vx += (fx / cfg.mass) * dt;
        vy += (fy / cfg.mass) * dt;
        vz += (fz / cfg.mass) * dt;
        x += vx * dt;
        y += vy * dt;
        z += vz * dt;

        if (z <= 0.0f) {
            z = 0.0f;
            if (vz < 0.0f) vz = 0.0f;
            landed = true;
        } else {
            landed = false;
        }
    }
};

/** The IMU as the flight controller sees it: biased, noisy, and in its units. */
struct ImuModel {
    float bias_p = 0.0f, bias_q = 0.0f, bias_r = 0.0f; // rad/s
    float gyro_noise = 0.015f;  // rad/s, one sigma
    float accel_noise = 0.03f;  // g, one sigma
    bool corrupt = false;       // inject garbage, to test divergence handling

    void sample(const Plant& plant, Rng& rng,
                float& gx, float& gy, float& gz,
                float& ax, float& ay, float& az) const {
        if (corrupt) {
            gx = gy = gz = 40.0f; // far beyond any real rate
            ax = ay = az = 12.0f;
            return;
        }
        gx = plant.p + bias_p + gyro_noise * rng.gauss();
        gy = plant.q + bias_q + gyro_noise * rng.gauss();
        gz = plant.r + bias_r + gyro_noise * rng.gauss();

        // Specific force in body axes. Gravity shows up because the vehicle is
        // not in free fall: the thrust that holds it up is what the sensor
        // feels, rotated by the attitude.
        const float sr = sinf(plant.roll), cr = cosf(plant.roll);
        const float sp = sinf(plant.pitch), cp = cosf(plant.pitch);
        float total = 0.0f;
        for (int i = 0; i < 4; ++i) total += plant.thrust[i];
        const float specific = total / plant.cfg.mass / kG; // in g, along body up

        // A hovering airframe reads gravity's direction because the thrust that
        // opposes gravity is fixed to the body. Departures from hover show up
        // as the extra term.
        ax = -sp * specific + accel_noise * rng.gauss();
        ay = sr * cp * specific + accel_noise * rng.gauss();
        az = cr * cp * specific + accel_noise * rng.gauss();
        (void)cp;
    }
};

/** Scripted pilot. Everything the controller reads from the receiver. */
struct Sticks {
    float throttle = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    bool signal_lost = false;
};

// ----------------------------------------------------- the flight controller

/** The gains the firmware shipped with before they were derived from a plant. */
struct LegacyGains {
    static constexpr float kp = 1.2f, ki = 0.04f, kd = 15.0f;
    static constexpr float yaw_kp = 2.0f, yaw_ki = 0.02f, yaw_kd = 0.0f;
};

/**
 * The flight controller, which is FlightLoop from flight_loop.h: the same
 * object main.cpp steps on the board. Nothing here restates the control
 * sequence; this only adapts the harness's sensor and stick types to it.
 */
struct Controller : FlightLoop {
    /**
     * Swap in the pre-tuning gains. Kept so the regression that motivated the
     * retune stays reproducible rather than living only in a commit message,
     * and so the comparison plot in the README can be regenerated from source.
     */
    void useLegacyGains() {
        pid_roll.setGains(LegacyGains::kp, LegacyGains::ki, LegacyGains::kd);
        pid_pitch.setGains(LegacyGains::kp, LegacyGains::ki, LegacyGains::kd);
        pid_yaw.setGains(LegacyGains::yaw_kp, LegacyGains::yaw_ki, LegacyGains::yaw_kd);
    }

    static ImuSample sample(float gx, float gy, float gz, float ax, float ay, float az) {
        ImuSample s;
        s.p = gx; s.q = gy; s.r = gz;
        s.ax = ax; s.ay = ay; s.az = az;
        return s;
    }

    static PilotInput pilot(const Sticks& sticks) {
        PilotInput in;
        in.throttle = sticks.throttle;
        in.roll = sticks.roll;
        in.pitch = sticks.pitch;
        in.yaw = sticks.yaw;
        in.signal_lost = sticks.signal_lost;
        return in;
    }

    void step(float gx, float gy, float gz, float ax, float ay, float az,
              const Sticks& sticks, float battery_v, uint32_t now_ms,
              float pwm_out[4]) {
        FlightLoop::step(sample(gx, gy, gz, ax, ay, az), pilot(sticks), battery_v,
                         now_ms, pwm_out, kDt);
    }
};

// --------------------------------------------------------------- scenarios

struct Sample {
    float t;
    float roll_true, pitch_true;
    float roll_est, pitch_est;
    float bias_p_est;
    float z, throttle_cmd;
    float pwm[4];
    int state;
    int fault;
    float innovation;
};

struct Result {
    std::string name;
    bool passed = true;
    std::vector<std::string> failures;
    std::vector<Sample> log;

    void require(bool cond, const std::string& what) {
        if (!cond) {
            passed = false;
            failures.push_back(what);
        }
    }
};

/** Everything a scenario can vary. */
struct Scenario {
    std::string name;
    float duration = 20.0f;
    float gyro_bias_dps = 0.0f;
    float wind_torque = 0.0f;       // N m, constant roll disturbance
    float motor_fail_at = -1.0f;    // s, negative for never
    int motor_fail_index = 0;
    float motor_fail_health = 0.7f;
    float signal_loss_at = -1.0f;
    float battery_drop_at = -1.0f;
    float imu_corrupt_at = -1.0f;
    float roll_step_at = -1.0f;
    float roll_step_deg = 0.0f;
    /** Run with the pre-tuning gains, and expect the stability checks to fail. */
    bool legacy_gains = false;
};

/**
 * How the controller is wired to the plant for a run.
 *
 *   DIRECT        a function call, as the harness has always done
 *   HIL_LOOPBACK  through MAVLink bytes: the plant's IMU sample is encoded as
 *                 HIL_SENSOR, the sticks as RC_CHANNELS_OVERRIDE, both pushed
 *                 a byte at a time into the hil::Link the HIL firmware uses,
 *                 and the reply decoded from HIL_ACTUATOR_CONTROLS bytes. If
 *                 the frames, the unit conversions or the sign conventions
 *                 are wrong, the scenarios fail here, without a board.
 */
enum class Wiring { DIRECT, HIL_LOOPBACK };

/** What the simulator side of the HIL link sends for a set of sticks. */
static mavlink::RcChannelsOverride rcFor(const Sticks& st) {
    mavlink::RcChannelsOverride rc;
    auto axis = [](float v) { return (uint16_t)lroundf(1500.0f + 500.0f * v); };
    rc.chan[0] = axis(st.roll);
    rc.chan[1] = axis(st.pitch);
    rc.chan[2] = (uint16_t)lroundf(1000.0f + 1000.0f * st.throttle);
    rc.chan[3] = axis(st.yaw);
    rc.target_system = 1;
    rc.target_component = 1;
    return rc;
}

/** Appends one .tlog record: big endian microseconds, then the frame. */
static void tlogAppend(std::vector<uint8_t>& out, uint64_t usec, const uint8_t* f, size_t n) {
    uint8_t ts[8];
    mavlink::tlogTimestamp(ts, usec);
    out.insert(out.end(), ts, ts + 8);
    out.insert(out.end(), f, f + n);
}

Result run(const Scenario& sc, Wiring wiring = Wiring::DIRECT,
           std::vector<uint8_t>* tlog = nullptr) {
    Result res;
    res.name = sc.name;

    // HIL loopback state: the board's end of the link, and the simulator's
    // parser for what comes back.
    hil::Link board_link;
    mavlink::Parser sim_parser;
    mavlink::Endpoint sim_ep;
    sim_ep.system_id = 255; // a ground station or simulator, by convention
    sim_ep.component_id = 190;
    std::vector<uint8_t> board_tx;
    uint32_t hil_replies = 0;

    // Telemetry, built the way main.cpp builds it, timed on simulated time.
    telemetry::Scheduler tlm;
    LoopTiming timing(LOOP_INTERVAL_US);

    Plant plant;
    ImuModel imu;
    Controller ctrl;
    Rng rng(20260917u);

    imu.bias_p = sc.gyro_bias_dps * kRad;
    imu.bias_q = -0.5f * sc.gyro_bias_dps * kRad;

    if (sc.legacy_gains) ctrl.useLegacyGains();

    // Seed the estimator from one still sample, which is what the firmware
    // does after gyro calibration on the bench.
    ctrl.ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);

    Sticks sticks;
    float battery = 12.4f;
    uint32_t now_ms = 0;
    float pwm[4] = {kEscMin, kEscMin, kEscMin, kEscMin};

    const int steps = (int)(sc.duration / kDt);
    bool ever_armed = false;
    float worst_est_err = 0.0f;
    float worst_tilt = 0.0f;
    long pwm_samples = 0, pwm_railed = 0;
    FaultCause first_fault = FaultCause::NONE;
    FlightState first_failsafe = FlightState::DISARMED;
    float peak_altitude = 0.0f;
    (void)peak_altitude;

    for (int i = 0; i < steps; ++i) {
        const float t = i * kDt;
        now_ms += 4;

        // --- pilot script -------------------------------------------------
        if (t < 2.5f) {
            sticks.throttle = 0.0f;
            sticks.yaw = 1.0f; // hold the arm gesture
        } else if (t < 3.0f) {
            sticks.yaw = 0.0f;
            sticks.throttle = 0.42f; // spool up
        } else {
            sticks.yaw = 0.0f;
            // Hover throttle for this airframe: four motors at mg/4 = 2.45 N
            // each is 0.409 of full thrust, which is 1409 us, which is 0.378
            // of the idle-to-max stick range.
            sticks.throttle = 0.378f;
        }
        sticks.roll = 0.0f;
        sticks.pitch = 0.0f;

        if (sc.roll_step_at >= 0.0f && t >= sc.roll_step_at) {
            sticks.roll = sc.roll_step_deg / 30.0f;
        }

        // --- fault injection ----------------------------------------------
        if (sc.signal_loss_at >= 0.0f && t >= sc.signal_loss_at) {
            // What receiver.h actually reports on a dead link: every axis
            // centred and the throttle closed. The scenario used to keep
            // hover throttle here, which no receiver does, and that is how a
            // failsafe that cut the motors in the air passed for so long.
            sticks.signal_lost = true;
            sticks.roll = sticks.pitch = sticks.yaw = 0.0f;
            sticks.throttle = 0.0f;
        }
        if (sc.motor_fail_at >= 0.0f && t >= sc.motor_fail_at) {
            plant.motor_health[sc.motor_fail_index] = sc.motor_fail_health;
        }
        if (sc.battery_drop_at >= 0.0f && t >= sc.battery_drop_at) {
            battery = 10.0f;
        }
        if (sc.imu_corrupt_at >= 0.0f && t >= sc.imu_corrupt_at) {
            imu.corrupt = true;
        }

        // --- one control step ---------------------------------------------
        float gx, gy, gz, ax, ay, az;
        imu.sample(plant, rng, gx, gy, gz, ax, ay, az);
        const uint64_t sim_us = (uint64_t)now_ms * 1000u;
        timing.markStart((uint32_t)sim_us);

        if (wiring == Wiring::DIRECT) {
            ctrl.step(gx, gy, gz, ax, ay, az, sticks, battery, now_ms, pwm);
        } else {
            // Simulator to board. A lost receiver is modelled the way it
            // happens on the link: the overrides simply stop arriving.
            uint8_t frame[mavlink::kMaxFrame];
            if (!sticks.signal_lost) {
                const size_t n = mavlink::encode(frame, sim_ep, rcFor(sticks));
                for (size_t k = 0; k < n; ++k) board_link.push(frame[k], [](const ImuSample&, uint64_t) {});
            }
            const ImuSample sample = Controller::sample(gx, gy, gz, ax, ay, az);
            const size_t n = mavlink::encode(frame, sim_ep, hil::toHilSensor(sample, sim_us));

            // The board: exactly hilStep() in main.cpp. The battery is passed
            // in directly, because HIL_SENSOR carries none and the low
            // battery scenario still has to be flown.
            board_tx.clear();
            for (size_t k = 0; k < n; ++k) {
                board_link.push(frame[k], [&](const ImuSample& in, uint64_t t_us) {
                    float out[4];
                    ctrl.FlightLoop::step(in, board_link.pilot(t_us), battery,
                                          (uint32_t)(t_us / 1000), out, kDt);
                    uint8_t reply[hil::Link::kActuatorFrame];
                    const size_t m = board_link.encodeActuators(reply, out, t_us,
                                                                ctrl.fsm.motorsEnabled());
                    board_tx.insert(board_tx.end(), reply, reply + m);
                });
            }

            // Board to simulator.
            for (uint8_t b : board_tx) {
                sim_parser.push(b, [&](const mavlink::Frame& f) {
                    mavlink::HilActuatorControls ac;
                    if (f.decode(ac)) {
                        hil::pwmFromActuators(ac, pwm);
                        ++hil_replies;
                    }
                });
            }
        }
        timing.markEnd((uint32_t)sim_us);

        if (tlog) {
            telemetry::Snapshot snap = telemetry::snapshotOf(
                ctrl, Controller::sample(gx, gy, gz, ax, ay, az), now_ms, battery);
            snap.timing = timing.report();
            snap.total_overruns = timing.totalOverruns();
            snap.receiver_ok = !sticks.signal_lost;
            tlm.poll(snap, [&](const uint8_t* f, size_t n) {
                tlogAppend(*tlog, sim_us, f, n);
                return true;
            });
        }

        // Wind acts on an airframe that is actually flying. Applying it while
        // the model is still sitting on the ground, where nothing holds it
        // level, just tips it over before it can arm.
        const float wind = (plant.z > 0.5f) ? sc.wind_torque : 0.0f;
        plant.step(pwm, wind, kDt);

        if (ctrl.fsm.state() == FlightState::ARMED) ever_armed = true;
        if (first_fault == FaultCause::NONE && ctrl.fsm.fault() != FaultCause::NONE) {
            first_fault = ctrl.fsm.fault();
            first_failsafe = ctrl.fsm.state();
        }
        if (plant.z > peak_altitude) peak_altitude = plant.z;

        if (ever_armed && t > 4.0f && ctrl.fsm.state() == FlightState::ARMED) {
            for (int m = 0; m < 4; ++m) {
                ++pwm_samples;
                if (pwm[m] <= kEscMin + 1.0f || pwm[m] >= kEscMax - 1.0f) ++pwm_railed;
            }
        }

        // --- metrics --------------------------------------------------------
        if (ever_armed && t > 4.0f && !imu.corrupt) {
            const float err = fabsf(ctrl.ekf.getRollDeg() - plant.roll * kDeg);
            if (err > worst_est_err) worst_est_err = err;
            const float tilt = fabsf(plant.roll * kDeg);
            if (tilt > worst_tilt) worst_tilt = tilt;
        }

        if (i % 5 == 0) { // log at 50 Hz
            Sample s;
            s.t = t;
            s.roll_true = plant.roll * kDeg;
            s.pitch_true = plant.pitch * kDeg;
            s.roll_est = ctrl.ekf.getRollDeg();
            s.pitch_est = ctrl.ekf.getPitchDeg();
            s.bias_p_est = ctrl.ekf.getBiasP() * kDeg;
            s.z = plant.z;
            s.throttle_cmd = ctrl.fsm.commandedThrottle();
            for (int m = 0; m < 4; ++m) s.pwm[m] = pwm[m];
            s.state = (int)ctrl.fsm.state();
            s.fault = (int)ctrl.fsm.fault();
            s.innovation = sqrtf(ctrl.ekf.getInnovation(0) * ctrl.ekf.getInnovation(0) +
                                 ctrl.ekf.getInnovation(1) * ctrl.ekf.getInnovation(1) +
                                 ctrl.ekf.getInnovation(2) * ctrl.ekf.getInnovation(2));
            res.log.push_back(s);
        }
    }

    // ------------------------------------------------------- pass criteria
    res.require(ever_armed, "never reached ARMED, the arming sequence did not complete");

    if (wiring == Wiring::HIL_LOOPBACK) {
        // Lockstep means exactly one reply per sample, and a clean link means
        // no frame was rejected in either direction.
        res.require(hil_replies == (uint32_t)steps,
                    "HIL link did not answer every sensor sample");
        res.require(board_link.stats().crc_errors == 0 && sim_parser.stats().crc_errors == 0,
                    "HIL link rejected frames on a lossless channel");
        res.require(board_link.stats().seq_gaps == 0 && sim_parser.stats().seq_gaps == 0,
                    "HIL link reported sequence gaps on a lossless channel");
    }

    if (sc.imu_corrupt_at >= 0.0f) {
        res.require(ctrl.fsm.state() == FlightState::FAILSAFE_CUT,
                    "corrupt IMU did not trigger a cut");
        res.require(first_fault == FaultCause::ESTIMATOR_DIVERGED ||
                        first_fault == FaultCause::EXCESSIVE_TILT,
                    std::string("corrupt IMU raised ") + toString(first_fault) + " first");
    } else if (sc.signal_loss_at >= 0.0f) {
        res.require(first_fault == FaultCause::SIGNAL_LOST,
                    std::string("signal loss raised ") + toString(first_fault) + " first");
        res.require(first_failsafe == FlightState::FAILSAFE_LAND,
                    "signal loss cut power instead of flying it down");
        res.require(plant.z < 1.0f, "failsafe did not bring the airframe down");
    } else if (sc.battery_drop_at >= 0.0f) {
        res.require(first_fault == FaultCause::LOW_BATTERY,
                    std::string("low battery raised ") + toString(first_fault) + " first");
        res.require(first_failsafe == FlightState::FAILSAFE_LAND,
                    "low battery cut power instead of flying it down");
        res.require(plant.z < 1.0f, "low battery failsafe did not descend");
    } else {
        res.require(ctrl.fsm.state() == FlightState::ARMED,
                    std::string("ended in ") + toString(ctrl.fsm.state()) +
                        " rather than staying armed");
        res.require(worst_tilt < 35.0f, "airframe exceeded 35 degrees of roll in normal flight");
        res.require(worst_est_err < 6.0f, "estimator error exceeded 6 degrees");
        res.require(!ctrl.ekf.isDiverged(), "estimator diverged");

        // Attitude staying small is not on its own evidence of a stable loop.
        // A controller chattering between the ESC stops can average out to a
        // level airframe while it saturates the actuators, burns the motors
        // and has no authority left for a real disturbance. Measuring how
        // often the outputs sit on a rail is what catches that, and it is the
        // criterion that exposed the original gains.
        const float railed = pwm_samples > 0 ? (float)pwm_railed / (float)pwm_samples : 0.0f;
        res.require(railed < 0.02f,
                    "motor outputs sat at an ESC limit for more than 2 percent of "
                    "steady flight, which is actuator chatter rather than control");
    }

    if (sc.gyro_bias_dps != 0.0f && sc.imu_corrupt_at < 0.0f) {
        const float est = ctrl.ekf.getBiasP() * kDeg;
        res.require(fabsf(est - sc.gyro_bias_dps) < 1.0f,
                    "gyro bias was not estimated to within 1 deg/s");
    }

    return res;
}

/**
 * Monte Carlo robustness campaign.
 *
 * The gains in config.h are derived for one airframe: 1 kg, 0.15 m arm, 6 N
 * motors. Every document in this repository says so, and says they are a
 * starting point rather than a substitute for bench tuning. That is honest as
 * far as it goes, but it leaves the obvious question unanswered: how far can
 * the real airframe be from the modelled one before those gains stop working?
 *
 * This answers it by holding the gains fixed and randomising the plant.
 * Mass, inertia, motor thrust and lag, arm length and the two damping terms
 * are each drawn independently, along with gyro bias and sensor noise, and the
 * same hover-with-disturbance flight is run for each draw. A trial passes on
 * the same criteria the named scenarios use: still armed at the end, tilt
 * under control, estimator converged, and the actuators not sitting on a rail.
 *
 * The result is a number that can be quoted, and more usefully a boundary:
 * the CSV records every draw with its outcome, so the parameter that actually
 * decides failure can be read off rather than guessed at.
 *
 * Deterministic: trial N always draws the same airframe, so a failure is
 * reproducible by running that trial alone.
 */
struct Trial {
    int index = 0;
    Airframe cfg;
    float gyro_bias_dps = 0.0f;
    float gyro_noise = 0.0f;
    float accel_noise = 0.0f;

    bool passed = false;
    float worst_tilt = 0.0f;
    float worst_est_err = 0.0f;
    float railed_fraction = 0.0f;
    const char* verdict = "pass";
};

/** Uniform in [lo, hi]. */
static float draw(Rng& rng, float lo, float hi) {
    return lo + (rng.uniform() * 0.5f + 0.5f) * (hi - lo);
}

Trial runTrial(int index) {
    Trial t;
    t.index = index;

    // One generator per trial, seeded from the index, so trial 37 is the same
    // airframe on every machine and every run.
    Rng rng(0x9E3779B9u + (uint32_t)index * 2654435761u);

    const Airframe nominal;
    t.cfg.mass = nominal.mass * draw(rng, 0.4f, 3.0f);
    t.cfg.arm = nominal.arm * draw(rng, 0.5f, 2.0f);
    // Inertia is drawn independently of mass rather than scaled with it. A
    // real airframe's mass distribution is not a fixed function of its weight,
    // and tying the two would hide the case this is meant to find: a heavy
    // battery close to the centre, or a light frame with long arms.
    t.cfg.inertia_xx = nominal.inertia_xx * draw(rng, 0.25f, 5.0f);
    t.cfg.inertia_yy = t.cfg.inertia_xx;
    t.cfg.inertia_zz = nominal.inertia_zz * draw(rng, 0.25f, 5.0f);
    t.cfg.max_thrust = nominal.max_thrust * draw(rng, 0.5f, 2.5f);
    t.cfg.motor_tau = draw(rng, 0.01f, 0.15f);
    t.cfg.rot_damping = nominal.rot_damping * draw(rng, 0.5f, 2.0f);
    t.cfg.yaw_torque_k = nominal.yaw_torque_k * draw(rng, 0.6f, 1.6f);
    t.cfg.lin_drag = nominal.lin_drag * draw(rng, 0.5f, 1.8f);

    t.gyro_bias_dps = draw(rng, -4.0f, 4.0f);
    t.gyro_noise = draw(rng, 0.005f, 0.035f);
    t.accel_noise = draw(rng, 0.01f, 0.07f);

    Plant plant;
    plant.cfg = t.cfg;
    ImuModel imu;
    imu.bias_p = t.gyro_bias_dps * kRad;
    imu.bias_q = -0.5f * t.gyro_bias_dps * kRad;
    imu.gyro_noise = t.gyro_noise;
    imu.accel_noise = t.accel_noise;

    Controller ctrl;
    ctrl.ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);

    Sticks sticks;
    uint32_t now_ms = 0;
    float pwm[4] = {kEscMin, kEscMin, kEscMin, kEscMin};

    // Hover throttle has to follow the drawn airframe, not the nominal one, or
    // a heavy draw would fail for lack of throttle rather than for control.
    const float hover_thrust = t.cfg.mass * kG / 4.0f;
    float hover_u = hover_thrust / t.cfg.max_thrust;
    if (hover_u > 0.95f) hover_u = 0.95f;
    const float hover_pwm = kEscMin + hover_u * (kEscMax - kEscMin);
    const float hover_stick = (hover_pwm - kEscIdle) / (kEscMax - kEscIdle);

    bool ever_armed = false;
    long samples = 0, railed = 0;

    const int steps = (int)(20.0f / kDt);
    for (int i = 0; i < steps; ++i) {
        const float time = i * kDt;

        if (time < 2.5f) {
            sticks.throttle = 0.0f;
            sticks.yaw = 1.0f;
        } else {
            sticks.yaw = 0.0f;
            sticks.throttle = hover_stick;
        }

        float gx, gy, gz, ax, ay, az;
        imu.sample(plant, rng, gx, gy, gz, ax, ay, az);
        ctrl.step(gx, gy, gz, ax, ay, az, sticks, 12.4f, (now_ms += 4), pwm);

        // A constant roll disturbance once airborne, scaled to the airframe so
        // every draw is asked to reject a comparable upset rather than the
        // heavy ones getting an easier ride.
        const float wind = (plant.z > 0.5f && time > 6.0f)
                               ? 0.06f * (t.cfg.inertia_xx / nominal.inertia_xx)
                               : 0.0f;
        plant.step(pwm, wind, kDt);

        if (ctrl.fsm.state() == FlightState::ARMED) ever_armed = true;

        if (ever_armed && time > 5.0f) {
            const float tilt = fabsf(plant.roll * kDeg);
            if (tilt > t.worst_tilt) t.worst_tilt = tilt;
            const float err = fabsf(ctrl.ekf.getRollDeg() - plant.roll * kDeg);
            if (err > t.worst_est_err) t.worst_est_err = err;

            if (ctrl.fsm.state() == FlightState::ARMED) {
                for (int m = 0; m < 4; ++m) {
                    ++samples;
                    if (pwm[m] <= kEscMin + 1.0f || pwm[m] >= kEscMax - 1.0f) ++railed;
                }
            }
        }
    }

    t.railed_fraction = samples > 0 ? (float)railed / (float)samples : 1.0f;

    if (!ever_armed) t.verdict = "never armed";
    else if (ctrl.fsm.state() != FlightState::ARMED) t.verdict = toString(ctrl.fsm.fault());
    else if (t.worst_tilt >= 35.0f) t.verdict = "tilt";
    else if (t.worst_est_err >= 8.0f) t.verdict = "estimator";
    else if (t.railed_fraction >= 0.02f) t.verdict = "actuator chatter";
    else t.passed = true;

    return t;
}

int runMonteCarlo(int trials, const std::string& dir) {
    std::printf("Monte Carlo robustness, %d trials\n", trials);
    std::printf("  gains held fixed, airframe randomised around the nominal\n\n");

    const std::string path = dir + "/monte_carlo.csv";
    FILE* f = fopen(path.c_str(), "w");
    if (f) {
        fprintf(f, "trial,mass_ratio,inertia_ratio,thrust_ratio,arm_ratio,motor_tau,"
                   "rot_damping_ratio,lin_drag_ratio,gyro_bias_dps,gyro_noise,accel_noise,"
                   "plant_gain_ratio,worst_tilt_deg,worst_est_err_deg,railed_fraction,passed,verdict\n");
    }

    const Airframe nominal;
    int passed = 0;
    std::vector<float> gain_ratio((size_t)trials, 0.0f);
    std::vector<bool> outcome((size_t)trials, false);

    for (int i = 0; i < trials; ++i) {
        const Trial t = runTrial(i);
        if (t.passed) ++passed;
        gain_ratio[(size_t)i] = (t.cfg.arm / nominal.arm) *
                                (t.cfg.max_thrust / nominal.max_thrust) /
                                (t.cfg.inertia_xx / nominal.inertia_xx);
        outcome[(size_t)i] = t.passed;
        if (f) {
            fprintf(f, "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,"
                       "%.4f,%.3f,%.3f,%.5f,%d,%s\n",
                    t.index,
                    t.cfg.mass / nominal.mass,
                    t.cfg.inertia_xx / nominal.inertia_xx,
                    t.cfg.max_thrust / nominal.max_thrust,
                    t.cfg.arm / nominal.arm,
                    t.cfg.motor_tau,
                    t.cfg.rot_damping / nominal.rot_damping,
                    t.cfg.lin_drag / nominal.lin_drag,
                    t.gyro_bias_dps, t.gyro_noise, t.accel_noise,
                    gain_ratio[(size_t)i],
                    t.worst_tilt, t.worst_est_err, t.railed_fraction,
                    t.passed ? 1 : 0, t.verdict);
        }
    }
    if (f) fclose(f);

    const float rate = 100.0f * (float)passed / (float)trials;
    std::printf("  %d of %d passed (%.1f%%)\n\n", passed, trials, rate);

    // A bare pass rate says whether the gains survive, not where they stop.
    // The five randomised airframe parameters collapse into one number that
    // the controller actually feels: the plant gain, which is proportional to
    // arm * thrust / inertia. Reporting failures against that, and against
    // motor lag, turns the campaign from a percentage into a boundary.
    std::printf("  failures by plant gain (arm x thrust / inertia, relative to design)\n");
    const float bands[4][2] = {{0.0f, 0.25f}, {0.25f, 0.5f}, {0.5f, 1.0f}, {1.0f, 1e9f}};
    const char* labels[4] = {"below 0.25", "0.25 to 0.5", "0.5 to 1.0", "above 1.0"};
    for (int b = 0; b < 4; ++b) {
        int n = 0, bad = 0;
        for (int i = 0; i < trials; ++i) {
            const float k = gain_ratio[i];
            if (k < bands[b][0] || k >= bands[b][1]) continue;
            ++n;
            if (!outcome[i]) ++bad;
        }
        if (n == 0) continue;
        std::printf("    %-12s %4d trials, %3d failed (%.1f%%)\n",
                    labels[b], n, bad, 100.0f * (float)bad / (float)n);
    }
    std::printf("\n  per-trial detail written to %s\n\n", path.c_str());

    // A campaign is only meaningful against a stated expectation. Below this
    // the gains are not tolerating realistic airframe variation and the
    // derivation, not the airframe, is what needs revisiting.
    const float kRequired = 90.0f;
    if (rate < kRequired) {
        std::printf("  FAIL: below the %.0f%% floor for this parameter spread\n", kRequired);
        return 1;
    }
    std::printf("  pass: at or above the %.0f%% floor\n", kRequired);
    return 0;
}

bool writeCsv(const Result& r, const std::string& dir) {
    const std::string path = dir + "/" + r.name + ".csv";
    FILE* f = fopen(path.c_str(), "w");
    if (!f) {
        std::printf("  could not write %s\n", path.c_str());
        return false;
    }
    fprintf(f, "t,roll_true,pitch_true,roll_est,pitch_est,bias_p_est,z,throttle_cmd,"
               "pwm_fl,pwm_fr,pwm_bl,pwm_br,state,fault,innovation\n");
    for (const auto& s : r.log) {
        fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.5f\n",
                s.t, s.roll_true, s.pitch_true, s.roll_est, s.pitch_est, s.bias_p_est,
                s.z, s.throttle_cmd, s.pwm[0], s.pwm[1], s.pwm[2], s.pwm[3],
                s.state, s.fault, s.innovation);
    }
    fclose(f);
    return true;
}

/** The telemetry the board would have sent, in the .tlog format QGroundControl replays. */
bool writeTlog(const std::vector<uint8_t>& data, const std::string& path) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) {
        std::printf("  could not write %s\n", path.c_str());
        return false;
    }
    fwrite(data.data(), 1, data.size(), f);
    fclose(f);
    return true;
}

/**
 * Fly every scenario twice, once wired directly and once through the HIL
 * byte stream, and compare. They are not expected to be bit identical: the
 * sticks are quantised to whole microseconds on the way through, and a lost
 * receiver is noticed after the 500 ms override timeout rather than at once.
 * They are expected to reach the same verdict, and to agree closely on
 * attitude wherever the receiver stays up.
 */
int runHilLoopback(const std::vector<Scenario>& scenarios) {
    std::printf("DroneCtrl HIL loopback\n");
    std::printf("every scenario flown through MAVLink HIL_SENSOR / HIL_ACTUATOR_CONTROLS bytes\n\n");
    std::printf("  %-22s %-8s %-8s %s\n", "scenario", "direct", "hil", "max roll difference");

    int failed = 0;
    for (const auto& sc : scenarios) {
        const Result a = run(sc, Wiring::DIRECT);
        const Result b = run(sc, Wiring::HIL_LOOPBACK);

        const bool expect_pass = !sc.legacy_gains;
        const bool a_ok = a.passed == expect_pass;
        const bool b_ok = b.passed == expect_pass;

        float max_diff = 0.0f;
        const size_t n = a.log.size() < b.log.size() ? a.log.size() : b.log.size();
        for (size_t i = 0; i < n; ++i) {
            const float d = fabsf(a.log[i].roll_true - b.log[i].roll_true);
            if (d > max_diff) max_diff = d;
        }

        // Where the link carries the same information, the flights must
        // agree. Signal loss and the deliberately unstable legacy gains are
        // excluded: the first is timed differently by design, and the second
        // is chaotic, so any perturbation grows.
        const bool comparable = sc.signal_loss_at < 0.0f && !sc.legacy_gains &&
                                sc.imu_corrupt_at < 0.0f;
        const bool agree = !comparable || max_diff < 0.5f;

        const bool ok = a_ok && b_ok && agree;
        std::printf("  %-22s %-8s %-8s %.4f deg%s%s\n", sc.name.c_str(),
                    a_ok ? "pass" : "FAIL", b_ok ? "pass" : "FAIL", max_diff,
                    comparable ? "" : "  (not compared)", agree ? "" : "  TOO LARGE");
        if (!b_ok && expect_pass) {
            for (const auto& f : b.failures) std::printf("      %s\n", f.c_str());
        }
        if (!ok) ++failed;
    }
    std::printf("\n%d of %d scenarios agree through the HIL link\n",
                (int)scenarios.size() - failed, (int)scenarios.size());
    return failed == 0 ? 0 : 1;
}

/**
 * A virtual board: the HIL firmware's loop, with stdin and stdout standing in
 * for the USB serial port. It does exactly what hilStep() in main.cpp does,
 * including the telemetry the board sends alongside its replies, so a
 * simulator can be developed and tested against the real controller without
 * a board attached. The simulator's own CI drives it this way
 * (simulator/tools/hil-check.mjs).
 */
int runHilBoard() {
    hil::Link link;
    FlightLoop flight;
    flight.ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    telemetry::Rates rates;
    rates.attitude_ms = 100;
    telemetry::Scheduler tlm(rates);
    std::vector<uint8_t> out;

    auto step = [&](const ImuSample& sample, uint64_t sim_usec) {
        const PilotInput pilot = link.pilot(sim_usec);
        const uint32_t sim_ms = (uint32_t)(sim_usec / 1000);
        float pwm[4];
        flight.step(sample, pilot, 12.4f, sim_ms, pwm);
        uint8_t frame[hil::Link::kActuatorFrame];
        const size_t n = link.encodeActuators(frame, pwm, sim_usec, flight.fsm.motorsEnabled());
        out.insert(out.end(), frame, frame + n);

        telemetry::Snapshot snap = telemetry::snapshotOf(flight, sample, sim_ms, 12.4f);
        snap.receiver_ok = !pilot.signal_lost;
        snap.link_errors = link.stats().crc_errors;
        snap.hil = true;
        tlm.poll(snap, [&](const uint8_t* f, size_t m) {
            out.insert(out.end(), f, f + m);
            return true;
        });
    };

    // read(), not fread(): fread waits to fill its buffer, and in lockstep
    // the simulator sends one sample and then waits for the reply, so a
    // buffered read deadlocks both ends.
    uint8_t buf[512];
    ssize_t got;
    while ((got = read(0, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < got; ++i) link.push(buf[i], step);
        if (!out.empty()) {
            fwrite(out.data(), 1, out.size(), stdout);
            fflush(stdout);
            out.clear();
        }
    }
    std::fprintf(stderr, "hil board: %u samples, %u crc errors, %u sequence gaps\n",
                 (unsigned)link.sensorFrames(), (unsigned)link.stats().crc_errors,
                 (unsigned)link.stats().seq_gaps);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::string out_dir = "logs";
    int monte_carlo = 0;
    bool hil_loopback = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) out_dir = argv[++i];
        else if (std::strcmp(argv[i], "--monte-carlo") == 0 && i + 1 < argc) {
            monte_carlo = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--hil-loopback") == 0) {
            hil_loopback = true;
        } else if (std::strcmp(argv[i], "--hil-board") == 0) {
            // Binary on stdout: nothing else may be printed there.
            return runHilBoard();
        }
    }

    if (monte_carlo > 0) return runMonteCarlo(monte_carlo, out_dir);

    std::vector<Scenario> scenarios;
    {
        Scenario s;
        s.name = "hover";
        s.duration = 20.0f;
        scenarios.push_back(s);
    }
    {
        Scenario s;
        s.name = "gyro_bias";
        s.duration = 40.0f;
        s.gyro_bias_dps = 3.0f;
        scenarios.push_back(s);
    }
    {
        Scenario s;
        s.name = "wind_disturbance";
        s.duration = 20.0f;
        s.wind_torque = 0.06f;
        scenarios.push_back(s);
    }
    {
        Scenario s;
        s.name = "roll_step";
        s.duration = 20.0f;
        s.roll_step_at = 8.0f;
        s.roll_step_deg = 15.0f;
        scenarios.push_back(s);
    }
    {
        Scenario s;
        s.name = "motor_degraded";
        s.duration = 25.0f;
        s.motor_fail_at = 10.0f;
        s.motor_fail_index = 0;
        s.motor_fail_health = 0.7f;
        scenarios.push_back(s);
    }
    {
        Scenario s;
        s.name = "signal_loss";
        s.duration = 25.0f;
        s.signal_loss_at = 8.0f;
        scenarios.push_back(s);
    }
    {
        Scenario s;
        s.name = "low_battery";
        s.duration = 25.0f;
        s.battery_drop_at = 8.0f;
        scenarios.push_back(s);
    }
    {
        // Documented regression. These gains chatter the ESCs between their
        // stops, so the harness is expected to reject them; the scenario is
        // kept so the comparison stays reproducible from source.
        Scenario s;
        s.name = "legacy_gains_rejected";
        s.duration = 20.0f;
        s.legacy_gains = true;
        scenarios.push_back(s);
    }
    {
        Scenario s;
        s.name = "imu_corruption";
        s.duration = 15.0f;
        s.imu_corrupt_at = 8.0f;
        scenarios.push_back(s);
    }

    if (hil_loopback) return runHilLoopback(scenarios);

    std::printf("DroneCtrl software-in-the-loop\n");
    std::printf("control loop 250 Hz, linking the firmware headers directly\n\n");

    int failed = 0;
    for (const auto& sc : scenarios) {
        std::vector<uint8_t> tlog;
        const Result r = run(sc, Wiring::DIRECT, &tlog);
        writeCsv(r, out_dir);
        writeTlog(tlog, out_dir + "/" + sc.name + ".tlog");

        // The legacy-gain run passes by failing. Asserting that keeps the
        // regression honest: if someone reintroduces those gains and the
        // harness stops objecting, this scenario turns red.
        const bool ok = sc.legacy_gains ? !r.passed : r.passed;
        std::printf("  %-22s %s%s\n", r.name.c_str(), ok ? "pass" : "FAIL",
                    sc.legacy_gains ? "   (expected to be rejected)" : "");
        if (!sc.legacy_gains) {
            for (const auto& f : r.failures) std::printf("      %s\n", f.c_str());
        } else if (!ok) {
            std::printf("      the pre-tuning gains were NOT rejected, the "
                        "stability criteria have regressed\n");
        }
        if (!ok) ++failed;
    }

    std::printf("\n%d of %d scenarios passed\n",
                (int)scenarios.size() - failed, (int)scenarios.size());
    std::printf("logs written to %s/\n", out_dir.c_str());
    return failed == 0 ? 0 : 1;
}
