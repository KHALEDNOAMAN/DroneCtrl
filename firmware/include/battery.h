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
        // The ADC's full scale and resolution differ between the boards (5 V
        // and 10 bits on the Nano, about 3.3 V and 12 bits on the ESP32), so
        // both come from config.h. Hard coding the Nano's made the ESP32 read
        // roughly six times the real voltage, which is a low battery
        // failsafe that could never fire.
        int val = analogRead(PIN_BATTERY);
        return val * (ADC_FULL_SCALE_VOLTS / ADC_MAX_COUNTS) * BATTERY_DIVIDER_RATIO;
    }
};
