#pragma once
#include <Arduino.h>
#include "battery.h"
#include "receiver.h"
#include "mpu6050.h"
#include "motor_mixer.h"

enum class SafetyStatus {
    OK,
    LOW_BATTERY,
    MAX_TILT_EXCEEDED,
    SIGNAL_LOST
};

class Safety {
public:
    Safety(Battery& b, Receiver& r, MPU6050& m, MotorMixer& mx)
        : bat(b), rx(r), mpu(m), mixer(mx), arm_timer(0), disarm_timer(0) {}

    SafetyStatus checkAll() {
        if (rx.isSignalLost()) {
            mixer.disarm();
            return SafetyStatus::SIGNAL_LOST;
        }

        if (bat.isLow()) {
            return SafetyStatus::LOW_BATTERY;
        }

        if (abs(mpu.getRoll()) > MAX_TILT_ANGLE || abs(mpu.getPitch()) > MAX_TILT_ANGLE) {
            mixer.disarm();
            return SafetyStatus::MAX_TILT_EXCEEDED;
        }

        handleArmingLogic();

        return SafetyStatus::OK;
    }

private:
    Battery& bat;
    Receiver& rx;
    MPU6050& mpu;
    MotorMixer& mixer;

    unsigned long arm_timer;
    unsigned long disarm_timer;

    void handleArmingLogic() {
        // Throttle low
        if (rx.getThrottle() < 0.05f) {
            // Yaw right -> Arm
            if (rx.getYaw() > 0.8f) {
                if (arm_timer == 0) arm_timer = millis();
                else if (millis() - arm_timer > 2000) {
                    mixer.arm();
                }
            } else {
                arm_timer = 0;
            }

            // Yaw left -> Disarm
            if (rx.getYaw() < -0.8f) {
                if (disarm_timer == 0) disarm_timer = millis();
                else if (millis() - disarm_timer > 2000) {
                    mixer.disarm();
                }
            } else {
                disarm_timer = 0;
            }
        } else {
            arm_timer = 0;
            disarm_timer = 0;
        }
    }
};
