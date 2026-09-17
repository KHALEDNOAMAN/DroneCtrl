#pragma once

/**
 * PID with derivative on measurement, a filtered derivative, and anti-windup.
 *
 * No Arduino.h: nothing here touches hardware, and keeping it free of that
 * include is what lets the host tests and the SIL harness link the same file
 * the flight controller runs. A control law that is tested in a different
 * build from the one that flies is not really the tested one.
 */

template <typename T>
class PIDController {
public:
    /**
     * @param max_integral clamp on the integral term's contribution
     * @param max_output   clamp on the total output
     * @param derivative_tau time constant of the derivative low-pass, seconds.
     *        Expressed as a time rather than as a raw smoothing factor so the
     *        filter keeps the same cutoff if the loop rate changes. The old
     *        fixed factor silently retuned itself whenever dt moved.
     */
    PIDController(T kp, T ki, T kd, T max_integral, T max_output,
                  T derivative_tau = T(0.02))
        : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral),
          max_output_(max_output), derivative_tau_(derivative_tau),
          integral_(0), prev_measurement_(0), filtered_derivative_(0),
          primed_(false), saturated_(false) {}

    void setGains(T kp, T ki, T kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setDerivativeTau(T tau) { derivative_tau_ = tau; }

    /**
     * One control step.
     *
     * @param hold_integral stop integrating this step. The mixer raises this
     *        when it had to clip or rescale, because integrating against an
     *        actuator that is already at its limit only builds up a correction
     *        that has to be unwound later, which is what turns a brief
     *        saturation into an overshoot after it clears.
     */
    T compute(T setpoint, T measurement, T dt, bool hold_integral = false) {
        if (dt <= T(0)) return T(0);

        const T error = setpoint - measurement;

        if (!hold_integral) {
            integral_ += error * dt;
            if (integral_ > max_integral_) integral_ = max_integral_;
            else if (integral_ < -max_integral_) integral_ = -max_integral_;
        }

        // Derivative on measurement, not on error, so a step in the setpoint
        // does not produce a spike in the output.
        T derivative = T(0);
        if (primed_) {
            derivative = (measurement - prev_measurement_) / dt;
        } else {
            // First call after construction or reset. prev_measurement_ is not
            // a real previous sample yet, so differencing against it would
            // produce a large spike from a standing start.
            primed_ = true;
        }

        if (derivative_tau_ > T(0)) {
            const T alpha = dt / (derivative_tau_ + dt);
            filtered_derivative_ += alpha * (derivative - filtered_derivative_);
        } else {
            filtered_derivative_ = derivative;
        }

        prev_measurement_ = measurement;

        const T output = (kp_ * error) + (ki_ * integral_) - (kd_ * filtered_derivative_);

        if (output > max_output_) {
            saturated_ = true;
            return max_output_;
        }
        if (output < -max_output_) {
            saturated_ = true;
            return -max_output_;
        }
        saturated_ = false;
        return output;
    }

    /** True if the last compute() hit the output clamp. */
    bool isSaturated() const { return saturated_; }

    T getIntegral() const { return integral_; }

    void reset() {
        integral_ = 0;
        prev_measurement_ = 0;
        filtered_derivative_ = 0;
        primed_ = false;
        saturated_ = false;
    }

private:
    T kp_, ki_, kd_;
    T max_integral_;
    T max_output_;
    T derivative_tau_;
    T integral_;
    T prev_measurement_;
    T filtered_derivative_;
    bool primed_;
    bool saturated_;
};
