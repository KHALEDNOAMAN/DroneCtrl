#pragma once

#include <stdint.h>

#include "config.h"
#include "attitude_ekf.h"
#include "flight_state.h"
#include "mixer_math.h"
#include "pid_controller.h"

/**
 * One control step: estimate, decide, control, mix.
 *
 * Before this header existed the sequence lived twice, once in
 * firmware/src/main.cpp and once in the SIL harness's Controller, with a
 * comment in each asking that they be kept in step. Two copies kept in step
 * by a comment drift; it is only a question of when. Now both call this, and
 * so does the hardware-in-the-loop build, which means the three ways the
 * controller is exercised (bench, SIL, HIL) run one implementation.
 *
 * The extraction was checked by running the SIL suite and the 800 trial Monte
 * Carlo campaign before and after: every CSV is byte identical.
 *
 * Inputs are in the units the estimator wants, not raw sensor counts. Turning
 * MPU6050 counts into these is imu.h's job on the board and hil_link.h's job
 * when the samples come from a simulator.
 */

/** One IMU sample. Body rates in rad/s, specific force in g. */
struct ImuSample {
    float p = 0.0f, q = 0.0f, r = 0.0f;
    /** Accelerometer, the firmware's convention: +1 g on z at rest. */
    float ax = 0.0f, ay = 0.0f, az = 1.0f;
};

/** Pilot commands, normalised as the receiver produces them. */
struct PilotInput {
    float throttle = 0.0f; ///< 0..1
    float roll = 0.0f;     ///< -1..1
    float pitch = 0.0f;    ///< -1..1
    float yaw = 0.0f;      ///< -1..1, also the arming gesture axis
    bool signal_lost = false;
};

class FlightLoop {
public:
    AttitudeEKF ekf;
    PIDController<float> pid_roll{PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 100.0f, 400.0f};
    PIDController<float> pid_pitch{PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 100.0f, 400.0f};
    PIDController<float> pid_yaw{PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 100.0f, 400.0f};
    FlightStateMachine fsm;

    FlightLoop() {
        FlightLimits limits;
        limits.max_tilt_deg = MAX_TILT_ANGLE;
        limits.min_battery_volts = MIN_BATTERY_VOLTAGE;
        fsm.setLimits(limits);
    }

    /** Last step's PID outputs, for telemetry and logging. */
    float out_roll = 0.0f, out_pitch = 0.0f, out_yaw = 0.0f;
    /** Gyro yaw rate the yaw loop flew on, deg/s. */
    float yaw_rate_dps = 0.0f;
    /** The last mix could not deliver the commanded torque. */
    bool authority_limited = false;

    /**
     * @param pwm_out FL, FR, BL, BR in ESC microseconds. Written on every
     *        call, ESC_MIN on all four whenever motors are not enabled.
     */
    void step(const ImuSample& imu, const PilotInput& pilot, float battery_v,
              uint32_t now_ms, float pwm_out[4], float dt = DT) {
        // --- sense --------------------------------------------------------
        ekf.predict(imu.p, imu.q, imu.r, dt);
        ekf.updateAccel(imu.ax, imu.ay, imu.az);
        yaw_rate_dps = imu.r * 57.2957795f;

        // --- decide -------------------------------------------------------
        FlightInputs in;
        in.throttle_norm = pilot.throttle;
        in.yaw_norm = pilot.yaw;
        in.roll_deg = ekf.getRollDeg();
        in.pitch_deg = ekf.getPitchDeg();
        in.battery_volts = battery_v;
        in.signal_lost = pilot.signal_lost;
        in.estimator_diverged = ekf.isDiverged();
        fsm.update(in, now_ms, dt);

        // --- control and act ----------------------------------------------
        if (!fsm.motorsEnabled()) {
            pid_roll.reset();
            pid_pitch.reset();
            pid_yaw.reset();
            out_roll = out_pitch = out_yaw = 0.0f;
            authority_limited = false;
            for (int i = 0; i < 4; ++i) pwm_out[i] = ESC_MIN;
            return;
        }

        // In a failsafe descent the receiver is either gone or not to be
        // trusted, so the setpoint is level and the state machine owns the
        // throttle.
        const bool failsafe = (fsm.state() == FlightState::FAILSAFE_LAND);
        const float sp_roll = failsafe ? 0.0f : pilot.roll * MAX_TILT_SETPOINT;
        const float sp_pitch = failsafe ? 0.0f : pilot.pitch * MAX_TILT_SETPOINT;
        const float sp_yaw_rate = failsafe ? 0.0f : pilot.yaw * MAX_YAW_RATE;

        // Hold the integrators whenever the mixer could not deliver last
        // step's torque. Integrating against an actuator that is already at
        // its limit only builds a correction that has to be unwound
        // afterwards, which is what turns a brief saturation into an
        // overshoot once it clears.
        const bool hold = authority_limited;

        out_roll = pid_roll.compute(sp_roll, ekf.getRollDeg(), dt, hold);
        out_pitch = pid_pitch.compute(sp_pitch, ekf.getPitchDeg(), dt, hold);
        out_yaw = pid_yaw.compute(sp_yaw_rate, yaw_rate_dps, dt, hold);

        const float esc_min = ESC_MIN, esc_max = ESC_MAX, esc_idle = ESC_IDLE;
        const float base = esc_idle + fsm.commandedThrottle() * (esc_max - esc_idle);
        const auto mix = mixer::mixX(base, out_pitch, out_roll, out_yaw, esc_min, esc_max);
        authority_limited = mix.authority_limited;

        pwm_out[0] = mix.fl;
        pwm_out[1] = mix.fr;
        pwm_out[2] = mix.bl;
        pwm_out[3] = mix.br;
    }
};
