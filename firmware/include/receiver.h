#pragma once
#include <Arduino.h>
#include "config.h"

class Receiver {
public:
    void init() {
        pinMode(PIN_RX_ROLL, INPUT);
        pinMode(PIN_RX_PITCH, INPUT);
        pinMode(PIN_RX_THROTTLE, INPUT);
        pinMode(PIN_RX_YAW, INPUT);
        last_update_ms = millis();
    }

    void readChannels() {
        // Using pulseIn for simplicity; timeout 25000us
        unsigned long r = pulseIn(PIN_RX_ROLL, HIGH, 25000);
        unsigned long p = pulseIn(PIN_RX_PITCH, HIGH, 25000);
        unsigned long t = pulseIn(PIN_RX_THROTTLE, HIGH, 25000);
        unsigned long y = pulseIn(PIN_RX_YAW, HIGH, 25000);

        if (r > 900 && p > 900 && t > 900 && y > 900) {
            roll_ = mapFloat(r, 1000, 2000, -1.0f, 1.0f);
            pitch_ = mapFloat(p, 1000, 2000, -1.0f, 1.0f);
            throttle_ = mapFloat(t, 1000, 2000, 0.0f, 1.0f);
            yaw_ = mapFloat(y, 1000, 2000, -1.0f, 1.0f);
            
            // Constrain
            roll_ = constrain(roll_, -1.0f, 1.0f);
            pitch_ = constrain(pitch_, -1.0f, 1.0f);
            throttle_ = constrain(throttle_, 0.0f, 1.0f);
            yaw_ = constrain(yaw_, -1.0f, 1.0f);

            last_update_ms = millis();
            signal_lost_ = false;
        } else {
            // Signal lost timeout (500ms)
            if (millis() - last_update_ms > 500) {
                signal_lost_ = true;
                throttle_ = 0.0f;
                roll_ = 0.0f;
                pitch_ = 0.0f;
                yaw_ = 0.0f;
            }
        }
    }

    float getRoll() const { return roll_; }
    float getPitch() const { return pitch_; }
    float getThrottle() const { return throttle_; }
    float getYaw() const { return yaw_; }
    bool isSignalLost() const { return signal_lost_; }

private:
    float roll_ = 0, pitch_ = 0, throttle_ = 0, yaw_ = 0;
    unsigned long last_update_ms = 0;
    bool signal_lost_ = true;

    float mapFloat(unsigned long x, unsigned long in_min, unsigned long in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
    }
};
