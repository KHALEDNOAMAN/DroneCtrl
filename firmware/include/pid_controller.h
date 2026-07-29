#pragma once
#include <Arduino.h>

template <typename T>
class PIDController {
public:
    PIDController(T kp, T ki, T kd, T max_integral, T max_output)
        : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), max_output_(max_output),
          integral_(0), prev_measurement_(0), prev_derivative_(0) {}

    void setGains(T kp, T ki, T kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    T compute(T setpoint, T measurement, T dt) {
        T error = setpoint - measurement;
        
        integral_ += error * dt;
        // Anti-windup
        if (integral_ > max_integral_) integral_ = max_integral_;
        else if (integral_ < -max_integral_) integral_ = -max_integral_;
        
        // Derivative on measurement to avoid setpoint kick
        T derivative = (measurement - prev_measurement_) / dt;
        
        // Simple low-pass filter on derivative term
        constexpr T alpha = 0.5f;
        T filtered_derivative = prev_derivative_ + alpha * (derivative - prev_derivative_);
        
        prev_measurement_ = measurement;
        prev_derivative_ = filtered_derivative;

        T output = (kp_ * error) + (ki_ * integral_) - (kd_ * filtered_derivative);
        
        if (output > max_output_) return max_output_;
        if (output < -max_output_) return -max_output_;
        return output;
    }

    void reset() {
        integral_ = 0;
        prev_measurement_ = 0;
        prev_derivative_ = 0;
    }

private:
    T kp_, ki_, kd_;
    T max_integral_;
    T max_output_;
    T integral_;
    T prev_measurement_;
    T prev_derivative_;
};
