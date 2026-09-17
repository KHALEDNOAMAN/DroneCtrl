/**
 * DroneCtrl flight controller.
 *
 * Loop structure, 250 Hz fixed rate:
 *
 *   sense   read the IMU and step the attitude EKF
 *   decide  feed the estimate and the receiver into the flight state machine
 *   control run the attitude PIDs when, and only when, that machine allows it
 *   act     mix to four ESC outputs
 *
 * Everything in the middle three steps is implemented in headers that contain
 * no Arduino dependency: attitude_ekf.h, flight_state.h, pid_controller.h and
 * mixer_math.h. That is what lets firmware/test exercise them on the host and
 * firmware/sil fly this exact code against a rigid body model. This file is
 * the wiring, and should stay that way: logic that lands here is logic that
 * cannot be tested without a board and a flight.
 *
 * Keep the order below in step with Controller::step in
 * firmware/sil/sil_flight.cpp. A SIL harness that runs a different sequence
 * from the aircraft is testing something the aircraft does not do.
 */

#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "receiver.h"
#include "motor_mixer.h"
#include "pid_controller.h"
#include "battery.h"
#include "flight_state.h"

IMU imu;
Receiver rx;
// Named motors, not mixer: mixer_math.h already owns the `mixer` namespace
// and a global of that name shadows it, which does not compile.
MotorMixer motors;
Battery battery;
FlightStateMachine fsm;

PIDController<float> pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 100.0f, 400.0f);
PIDController<float> pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 100.0f, 400.0f);
PIDController<float> pidYaw(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 100.0f, 400.0f);

unsigned long prev_loop_time = 0;

void setup() {
    Serial.begin(115200);

    if (!imu.init()) {
        Serial.println(F("IMU init failed"));
        // Halting is the right response. Continuing without an attitude
        // estimate would let the airframe arm with no idea which way is up.
        while (true) { delay(10); }
    }

    Serial.println(F("Calibrating gyro, keep the airframe still and level."));
    imu.calibrate();
    Serial.println(F("Calibration done."));

    rx.init();
    motors.init();
    battery.init();

    FlightLimits limits;
    limits.max_tilt_deg = MAX_TILT_ANGLE;
    limits.min_battery_volts = MIN_BATTERY_VOLTAGE;
    fsm.setLimits(limits);

    prev_loop_time = micros();
}

void loop() {
    const unsigned long now_us = micros();
    // Unsigned subtraction, so this stays correct across the micros() rollover
    // at roughly 71 minutes rather than stalling the loop for one cycle.
    if (now_us - prev_loop_time < LOOP_INTERVAL_US) return;
    prev_loop_time = now_us;

    // --- sense ------------------------------------------------------------
    imu.update(DT);
    rx.readChannels();

    // The battery divider is slow and noisy, and nothing downstream needs it
    // at 250 Hz. Every 50th cycle is 5 Hz, which is far faster than a pack
    // discharges.
    static uint8_t battery_divider = 0;
    if (++battery_divider >= 50) {
        battery.update();
        battery_divider = 0;
    }

    // --- decide -----------------------------------------------------------
    FlightInputs in;
    in.throttle_norm = rx.getThrottle();
    in.yaw_norm = rx.getYaw();
    in.roll_deg = imu.getRoll();
    in.pitch_deg = imu.getPitch();
    in.battery_volts = battery.getVoltage();
    in.signal_lost = rx.isSignalLost();
    in.estimator_diverged = imu.isDiverged();

    fsm.update(in, millis(), DT);

    // --- control and act --------------------------------------------------
    if (!fsm.motorsEnabled()) {
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
        motors.stop();
        return;
    }

    // In a failsafe descent the receiver is either gone or not to be trusted,
    // so the setpoint is level and the state machine owns the throttle.
    const bool failsafe = (fsm.state() == FlightState::FAILSAFE_LAND);
    const float sp_roll = failsafe ? 0.0f : rx.getRoll() * MAX_TILT_SETPOINT;
    const float sp_pitch = failsafe ? 0.0f : rx.getPitch() * MAX_TILT_SETPOINT;
    const float sp_yaw_rate = failsafe ? 0.0f : rx.getYaw() * MAX_YAW_RATE;

    // Hold the integrators whenever the mixer could not deliver last step's
    // torque. Integrating against an actuator that is already at its limit
    // only builds a correction that has to be unwound afterwards, which is
    // what turns a brief saturation into an overshoot once it clears.
    const bool hold = motors.authorityLimited();

    const float out_roll = pidRoll.compute(sp_roll, imu.getRoll(), DT, hold);
    const float out_pitch = pidPitch.compute(sp_pitch, imu.getPitch(), DT, hold);
    const float out_yaw = pidYaw.compute(sp_yaw_rate, imu.getYawRate(), DT, hold);

    motors.setOutputs(fsm.commandedThrottle(), out_pitch, out_roll, out_yaw);
}
