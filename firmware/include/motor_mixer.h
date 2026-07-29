#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "config.h"

class MotorMixer {
public:
    MotorMixer() : armed_(false) {}

    void init() {
        motorFL.attach(PIN_MOTOR_FL, ESC_MIN, ESC_MAX);
        motorFR.attach(PIN_MOTOR_FR, ESC_MIN, ESC_MAX);
        motorBL.attach(PIN_MOTOR_BL, ESC_MIN, ESC_MAX);
        motorBR.attach(PIN_MOTOR_BR, ESC_MIN, ESC_MAX);
        disarm();
    }

    void arm() {
        armed_ = true;
    }

    void disarm() {
        armed_ = false;
        writeMotors(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
    }

    bool isArmed() const { return armed_; }

    void setThrottle(float throttle_norm, float pitch_pid, float roll_pid, float yaw_pid) {
        if (!armed_) {
            disarm();
            return;
        }

        // Map normalized throttle (0.0 to 1.0) to PWM (ESC_IDLE to ESC_MAX)
        float base_pwm = ESC_IDLE + throttle_norm * (ESC_MAX - ESC_IDLE);

        // Mix for X configuration
        float fl = base_pwm + pitch_pid - roll_pid - yaw_pid;
        float fr = base_pwm + pitch_pid + roll_pid + yaw_pid;
        float bl = base_pwm - pitch_pid - roll_pid + yaw_pid;
        float br = base_pwm - pitch_pid + roll_pid - yaw_pid;

        writeMotors(
            constrain((int)fl, ESC_MIN, ESC_MAX),
            constrain((int)fr, ESC_MIN, ESC_MAX),
            constrain((int)bl, ESC_MIN, ESC_MAX),
            constrain((int)br, ESC_MIN, ESC_MAX)
        );
    }

private:
    Servo motorFL, motorFR, motorBL, motorBR;
    bool armed_;

    void writeMotors(int fl, int fr, int bl, int br) {
        motorFL.writeMicroseconds(fl);
        motorFR.writeMicroseconds(fr);
        motorBL.writeMicroseconds(bl);
        motorBR.writeMicroseconds(br);
    }
};
