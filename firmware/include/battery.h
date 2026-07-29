#pragma once
#include <Arduino.h>
#include "config.h"

class Battery {
public:
    void init() {
        pinMode(PIN_BATTERY, INPUT);
        voltage_ = readRawVoltage();
    }

    void update() {
        float raw = readRawVoltage();
        // Simple low-pass filter
        voltage_ = 0.95f * voltage_ + 0.05f * raw;
    }

    float getVoltage() const { return voltage_; }

    float getLevelPercent() const {
        // Assume 3S LiPo: 12.6V max, 10.5V min
        float pct = (voltage_ - MIN_BATTERY_VOLTAGE) / (12.6f - MIN_BATTERY_VOLTAGE) * 100.0f;
        return constrain(pct, 0.0f, 100.0f);
    }

    bool isLow() const {
        return voltage_ < MIN_BATTERY_VOLTAGE;
    }

private:
    float voltage_ = 0;

    float readRawVoltage() {
        // Voltage divider: R1=10k, R2=2k -> scaling factor 6.0
        // ADC 5V ref, 10-bit
        int val = analogRead(PIN_BATTERY);
        return val * (5.0f / 1023.0f) * 6.0f; 
    }
};
