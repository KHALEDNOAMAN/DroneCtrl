#pragma once

#include <math.h>
#include <stdint.h>

/**
 * Control loop timing: period jitter, execution time and missed deadlines.
 *
 * Why this exists
 * ---------------
 * Every gain in config.h, every filter constant in the EKF and the derivative
 * term of every PID assume the loop runs at exactly DT. When it does not, the
 * controller is quietly a different controller: a late cycle integrates the
 * gyro over the wrong interval and the derivative sees a rate that never
 * happened. The firmware asserted 250 Hz without ever measuring it. This
 * measures it, on the board, and reports it over telemetry so the claim can
 * be checked in flight rather than taken on trust.
 *
 * What it records, per reporting window
 * -------------------------------------
 *   period       start-to-start interval: min, max, mean and standard deviation
 *   jitter       the largest |period - nominal| seen, which is the number that
 *                actually bounds the control error
 *   execution    start-to-end time of the control step, mean and max
 *   utilisation  mean execution time over the nominal period
 *   overruns     cycles that started more than half a period late, which is a
 *                missed deadline rather than ordinary jitter
 *
 * Timestamps are uint32_t microseconds, as micros() returns them on both the
 * AVR and the ESP32. All interval arithmetic is unsigned subtraction, so the
 * rollover every 71.6 minutes costs nothing, and it is tested.
 *
 * No Arduino dependency.
 */
class LoopTiming {
public:
    struct Report {
        uint32_t cycles = 0;
        uint32_t period_min_us = 0;
        uint32_t period_max_us = 0;
        float period_mean_us = 0.0f;
        float period_stddev_us = 0.0f;
        uint32_t jitter_max_us = 0;
        float exec_mean_us = 0.0f;
        uint32_t exec_max_us = 0;
        /** Mean execution time as a fraction of the nominal period, 0..1+. */
        float utilisation = 0.0f;
        uint32_t overruns = 0;
    };

    explicit LoopTiming(uint32_t nominal_us) : nominal_us_(nominal_us) { resetWindow(); }

    /** Call at the very start of each control step. */
    void markStart(uint32_t now_us) {
        if (have_prev_) {
            const uint32_t period = now_us - prev_start_us_; // rollover safe
            addPeriod(period);
        }
        prev_start_us_ = now_us;
        have_prev_ = true;
        in_cycle_ = true;
    }

    /** Call when the control step has written its outputs. */
    void markEnd(uint32_t now_us) {
        if (!in_cycle_) return;
        in_cycle_ = false;
        const uint32_t exec = now_us - prev_start_us_;
        exec_sum_ += exec;
        ++exec_n_;
        if (exec > exec_max_) exec_max_ = exec;
    }

    /** Statistics since the last resetWindow(). */
    Report report() const {
        Report r;
        r.cycles = n_;
        r.overruns = overruns_;
        r.exec_max_us = exec_max_;
        r.exec_mean_us = exec_n_ ? static_cast<float>(exec_sum_) / exec_n_ : 0.0f;
        r.utilisation = r.exec_mean_us / static_cast<float>(nominal_us_);
        if (n_ == 0) return r;
        r.period_min_us = min_;
        r.period_max_us = max_;
        r.period_mean_us = static_cast<float>(mean_);
        r.period_stddev_us = n_ > 1 ? static_cast<float>(sqrt(m2_ / (n_ - 1))) : 0.0f;
        r.jitter_max_us = jitter_max_;
        return r;
    }

    /**
     * Start a new window. The previous start time is kept, so the first period
     * of the new window is measured rather than lost.
     */
    void resetWindow() {
        n_ = 0;
        mean_ = 0.0;
        m2_ = 0.0;
        min_ = UINT32_MAX;
        max_ = 0;
        jitter_max_ = 0;
        overruns_ = 0;
        exec_sum_ = 0;
        exec_n_ = 0;
        exec_max_ = 0;
    }

    /** Missed deadlines since boot. Never reset, so a single one is not lost
     *  between two telemetry reports. */
    uint32_t totalOverruns() const { return total_overruns_; }
    uint32_t nominalUs() const { return nominal_us_; }

private:
    uint32_t nominal_us_;
    uint32_t prev_start_us_ = 0;
    bool have_prev_ = false;
    bool in_cycle_ = false;

    // Welford's running mean and variance. A sum of squares in float loses
    // everything after the first few thousand samples of a 4000 us period;
    // this stays exact enough over a whole flight. Double is 4 bytes on AVR,
    // which is still fine here because Welford never forms the large sum.
    uint32_t n_ = 0;
    double mean_ = 0.0;
    double m2_ = 0.0;
    uint32_t min_ = UINT32_MAX;
    uint32_t max_ = 0;
    uint32_t jitter_max_ = 0;
    uint32_t overruns_ = 0;
    uint32_t total_overruns_ = 0;
    uint64_t exec_sum_ = 0;
    uint32_t exec_n_ = 0;
    uint32_t exec_max_ = 0;

    void addPeriod(uint32_t period) {
        ++n_;
        const double x = static_cast<double>(period);
        const double delta = x - mean_;
        mean_ += delta / n_;
        m2_ += delta * (x - mean_);

        if (period < min_) min_ = period;
        if (period > max_) max_ = period;

        const uint32_t dev = period > nominal_us_ ? period - nominal_us_ : nominal_us_ - period;
        if (dev > jitter_max_) jitter_max_ = dev;

        if (period > nominal_us_ + nominal_us_ / 2) {
            ++overruns_;
            ++total_overruns_;
        }
    }
};
