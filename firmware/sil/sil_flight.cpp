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
 * Adding hardware in the loop means running this same loop on the board with
 * the sensor reads replaced by injected samples over the serial link, and the
 * seam for that is already here, in SensorSource.
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

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "attitude_ekf.h"
#include "pid_controller.h"
#include "mixer_math.h"
#include "flight_state.h"

// config.h is Arduino-free apart from one uint32_t, so the harness can take the
// real gains rather than a copy that drifts.
#include <stdint.h>
#include "config.h"

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

/**
 * The same sequence main.cpp runs, against the same headers.
 *
 * Keep this in step with firmware/src/main.cpp. The point of a SIL harness is
 * lost the moment the two orders of operations diverge.
 */
/** The gains the firmware shipped with before they were derived from a plant. */
struct LegacyGains {
    static constexpr float kp = 1.2f, ki = 0.04f, kd = 15.0f;
    static constexpr float yaw_kp = 2.0f, yaw_ki = 0.02f, yaw_kd = 0.0f;
};

struct Controller {
    AttitudeEKF ekf;
    // Straight from config.h, so the harness cannot silently test gains the
    // firmware does not use.
    PIDController<float> pid_roll{PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 100.0f, 400.0f};
    PIDController<float> pid_pitch{PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 100.0f, 400.0f};
    PIDController<float> pid_yaw{PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 100.0f, 400.0f};
    FlightStateMachine fsm;

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

    float out_roll = 0.0f, out_pitch = 0.0f, out_yaw = 0.0f;
    bool authority_limited = false;

    void step(float gx, float gy, float gz, float ax, float ay, float az,
              const Sticks& sticks, float battery_v, uint32_t now_ms,
              float pwm_out[4]) {
        ekf.predict(gx, gy, gz, kDt);
        ekf.updateAccel(ax, ay, az);

        FlightInputs in;
        in.throttle_norm = sticks.throttle;
        in.yaw_norm = sticks.yaw;
        in.roll_deg = ekf.getRollDeg();
        in.pitch_deg = ekf.getPitchDeg();
        in.battery_volts = battery_v;
        in.signal_lost = sticks.signal_lost;
        in.estimator_diverged = ekf.isDiverged();
        fsm.update(in, now_ms, kDt);

        if (!fsm.motorsEnabled()) {
            pid_roll.reset();
            pid_pitch.reset();
            pid_yaw.reset();
            out_roll = out_pitch = out_yaw = 0.0f;
            for (int i = 0; i < 4; ++i) pwm_out[i] = kEscMin;
            return;
        }

        // In failsafe the pilot's sticks are gone, so hold level and let the
        // state machine's throttle ramp fly it down.
        const bool failsafe = (fsm.state() == FlightState::FAILSAFE_LAND);
        const float sp_roll = failsafe ? 0.0f : sticks.roll * 30.0f;
        const float sp_pitch = failsafe ? 0.0f : sticks.pitch * 30.0f;
        const float sp_yaw_rate = failsafe ? 0.0f : sticks.yaw * 150.0f;

        // Hold the integrators whenever the mixer could not deliver what was
        // asked for last step, so they do not wind up against an actuator that
        // is already at its limit.
        const bool hold = authority_limited;

        out_roll = pid_roll.compute(sp_roll, ekf.getRollDeg(), kDt, hold);
        out_pitch = pid_pitch.compute(sp_pitch, ekf.getPitchDeg(), kDt, hold);
        out_yaw = pid_yaw.compute(sp_yaw_rate, gz * kDeg, kDt, hold);

        const float base = kEscIdle + fsm.commandedThrottle() * (kEscMax - kEscIdle);
        const auto mix = mixer::mixX(base, out_pitch, out_roll, out_yaw, kEscMin, kEscMax);
        authority_limited = mix.authority_limited;

        pwm_out[0] = mix.fl;
        pwm_out[1] = mix.fr;
        pwm_out[2] = mix.bl;
        pwm_out[3] = mix.br;
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

Result run(const Scenario& sc) {
    Result res;
    res.name = sc.name;

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
            sticks.signal_lost = true;
            sticks.roll = sticks.pitch = sticks.yaw = 0.0f;
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
        ctrl.step(gx, gy, gz, ax, ay, az, sticks, battery, now_ms, pwm);

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

} // namespace

int main(int argc, char** argv) {
    std::string out_dir = "logs";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) out_dir = argv[++i];
    }

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

    std::printf("DroneCtrl software-in-the-loop\n");
    std::printf("control loop 250 Hz, linking the firmware headers directly\n\n");

    int failed = 0;
    for (const auto& sc : scenarios) {
        const Result r = run(sc);
        writeCsv(r, out_dir);

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
