#pragma once

// Configuration for DroneCtrl

// Loop timing
constexpr float LOOP_FREQUENCY_HZ = 250.0f;
constexpr float DT = 1.0f / LOOP_FREQUENCY_HZ;
constexpr uint32_t LOOP_INTERVAL_US = 1000000 / LOOP_FREQUENCY_HZ;

// I2C Config
constexpr uint8_t MPU6050_ADDRESS = 0x68;

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

// Battery Pin
constexpr uint8_t PIN_BATTERY = A0;

// PID Gains
// Roll
constexpr float PID_ROLL_KP = 1.2f;
constexpr float PID_ROLL_KI = 0.04f;
constexpr float PID_ROLL_KD = 15.0f;

// Pitch
constexpr float PID_PITCH_KP = 1.2f;
constexpr float PID_PITCH_KI = 0.04f;
constexpr float PID_PITCH_KD = 15.0f;

// Yaw
constexpr float PID_YAW_KP = 2.0f;
constexpr float PID_YAW_KI = 0.02f;
constexpr float PID_YAW_KD = 0.0f;

// Limits & Safety
constexpr float MAX_TILT_ANGLE = 45.0f; // degrees
constexpr float MIN_BATTERY_VOLTAGE = 10.5f; // volts (3S LiPo)
constexpr float MAX_YAW_RATE = 150.0f; // degrees/sec

// ESC Range
constexpr uint16_t ESC_MIN = 1000;
constexpr uint16_t ESC_MAX = 2000;
constexpr uint16_t ESC_IDLE = 1050; // Idle spin when armed
