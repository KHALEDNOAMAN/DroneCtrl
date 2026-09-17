#pragma once

#include <stdint.h>

// Configuration for DroneCtrl.
//
// Split into two halves on purpose. Everything below the pin block is plain
// arithmetic, so the host tests and the SIL harness include this file and take
// the real gains and limits rather than a copy that drifts out of step with
// the firmware. The pin assignments are the one part that needs the Arduino
// core's own constants, so they are guarded and simply absent on the host,
// where there are no pins to assign.

// Loop timing
constexpr float LOOP_FREQUENCY_HZ = 250.0f;
constexpr float DT = 1.0f / LOOP_FREQUENCY_HZ;
constexpr uint32_t LOOP_INTERVAL_US = 1000000 / LOOP_FREQUENCY_HZ;

// I2C Config
constexpr uint8_t MPU6050_ADDRESS = 0x68;

#if defined(ARDUINO)
// Motor Pins (X-config)
constexpr uint8_t PIN_MOTOR_FL = 3;  // Front Left (D3)
constexpr uint8_t PIN_MOTOR_FR = 5;  // Front Right (D5)
constexpr uint8_t PIN_MOTOR_BL = 6;  // Back Left (D6)
constexpr uint8_t PIN_MOTOR_BR = 9;  // Back Right (D9)

// Receiver Pins
constexpr uint8_t PIN_RX_ROLL = 2;   // CH1 (D2)
constexpr uint8_t PIN_RX_PITCH = 4;  // CH2 (D4)
constexpr uint8_t PIN_RX_THROTTLE = 7; // CH3 (D7)
constexpr uint8_t PIN_RX_YAW = 8;    // CH4 (D8)

// Battery Pin. A0 comes from the Arduino core, which is why this block is
// guarded: the host build has no such symbol and no pins to name.
constexpr uint8_t PIN_BATTERY = A0;
#endif // ARDUINO

// PID gains.
//
// Units: the controller takes an angle error in degrees (a rate error, for
// yaw) and returns a differential motor command in ESC microseconds.
//
// Where these came from
// ---------------------
// The previous values (Kp 1.2, Ki 0.04, Kd 15) were never validated against a
// plant. Running them through firmware/sil showed why that mattered: with the
// derivative term that large, the loop chased its own measurement noise and
// the four outputs chattered between 1000 and 2000 us continuously. Mean
// attitude still looked acceptable, which is exactly what makes the failure
// mode easy to miss, but every actuator was saturated and there was no
// authority left for a disturbance. A 30 percent loss on one motor put the
// airframe past 45 degrees and into a failsafe cut within six seconds.
//
// These are derived from the airframe model in firmware/sil/sil_flight.cpp
// rather than guessed. For that model, one microsecond of roll command
// produces about 13.3 deg/s^2 of angular acceleration, and the airframe's own
// aerodynamic damping contributes about 1.8 1/s. Treating the inner loop as
// second order:
//
//     omega_n^2   = 13.3 * Kp
//     2*zeta*omega_n = 13.3 * Kd + 1.8
//
// Solving for omega_n = 6 rad/s and zeta = 0.75, which is a normal bandwidth
// for a quadcopter of this size and leaves the 0.03 s motor lag pole an order
// of magnitude above the crossover, gives Kp = 2.7 and Kd = 0.55. Ki follows
// from an integral time of about 1.5 s, which is fast enough to trim out a
// degraded motor and slow enough not to interact with the attitude loop.
//
// IMPORTANT: these are tuned for the simulated airframe, which is a 1 kg quad
// with a 0.15 m arm and 6 N motors. They are a defensible starting point, not
// a substitute for bench tuning on real hardware. Change the airframe and the
// derivation above has to be redone with its numbers.

// Roll
constexpr float PID_ROLL_KP = 2.7f;
constexpr float PID_ROLL_KI = 1.8f;
constexpr float PID_ROLL_KD = 0.55f;

// Pitch. Symmetric airframe, so the same as roll.
constexpr float PID_PITCH_KP = 2.7f;
constexpr float PID_PITCH_KI = 1.8f;
constexpr float PID_PITCH_KD = 0.55f;

// Yaw runs in rate mode, so its setpoint is already a rate and its derivative
// term would see angular acceleration. That responds to the controller's own
// output within a single step, which is why Kd stays at zero here.
constexpr float PID_YAW_KP = 4.0f;
constexpr float PID_YAW_KI = 0.5f;
constexpr float PID_YAW_KD = 0.0f;

// Limits & Safety
constexpr float MAX_TILT_ANGLE = 45.0f; // degrees
constexpr float MIN_BATTERY_VOLTAGE = 10.5f; // volts (3S LiPo)
constexpr float MAX_YAW_RATE = 150.0f; // degrees/sec
// Full stick deflection in angle mode. Kept below MAX_TILT_ANGLE so a pilot
// holding the stick at its stop cannot by itself trip the tilt failsafe.
constexpr float MAX_TILT_SETPOINT = 30.0f; // degrees

// ESC Range
constexpr uint16_t ESC_MIN = 1000;
constexpr uint16_t ESC_MAX = 2000;
constexpr uint16_t ESC_IDLE = 1050; // Idle spin when armed
