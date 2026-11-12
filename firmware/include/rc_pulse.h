#pragma once

#include <stdint.h>

/**
 * RC PWM decoding from pin edges, with no hardware dependency.
 *
 * Why this replaced pulseIn()
 * ---------------------------
 * The receiver used to call pulseIn() on each of four channels in turn.
 * pulseIn() busy-waits for the next rising edge and then for the falling one,
 * and a standard receiver repeats every channel once per 20 ms frame. Reading
 * four channels that way blocks for anything from a few milliseconds to most
 * of a frame per channel, every cycle. The loop was configured for 250 Hz,
 * which is a 4 ms budget; it could not have been running at that rate, and
 * the gains, the EKF and every PID derivative assume it does. Nothing measured
 * the loop, so nothing noticed. loop_timing.h now measures it.
 *
 * The fix is the standard one: timestamp edges in a pin interrupt and do the
 * arithmetic here. The control loop then reads four widths in constant time.
 *
 * ISR side: edge(). Loop side: everything else. On the AVR a uint32_t is not
 * read atomically, so receiver.h copies a decoder with interrupts masked
 * before reading it; nothing in here assumes otherwise.
 */
class RcPulseChannel {
public:
    /** Accept widths in this window as a real servo pulse. */
    static constexpr uint16_t kMinUs = 900;
    static constexpr uint16_t kMaxUs = 2100;

    /** Call from the pin interrupt with the new pin level and micros(). */
    void edge(bool high, uint32_t now_us) {
        if (high) {
            rise_us_ = now_us;
            have_rise_ = true;
            return;
        }
        if (!have_rise_) return; // first edge after boot was a falling one
        have_rise_ = false;
        const uint32_t width = now_us - rise_us_; // rollover safe
        if (width >= kMinUs && width <= kMaxUs) {
            width_us_ = static_cast<uint16_t>(width);
            last_good_us_ = now_us;
            have_good_ = true;
        } else {
            ++rejected_;
        }
    }

    /**
     * Most recent valid width, if it arrived within max_age_us of now_us.
     * Stale means the receiver stopped driving this pin.
     */
    bool read(uint32_t now_us, uint32_t max_age_us, uint16_t& width_us) const {
        if (!have_good_) return false;
        if (now_us - last_good_us_ > max_age_us) return false;
        width_us = width_us_;
        return true;
    }

    uint32_t rejected() const { return rejected_; }

private:
    uint32_t rise_us_ = 0;
    uint32_t last_good_us_ = 0;
    uint16_t width_us_ = 0;
    bool have_rise_ = false;
    bool have_good_ = false;
    uint32_t rejected_ = 0;
};

/**
 * Four channels to the normalised sticks the flight loop wants, with the same
 * signal-loss policy the pulseIn() receiver had: keep the last good sticks for
 * up to 500 ms of bad frames, then declare the link lost and centre everything
 * with the throttle closed.
 */
class RcDecoder {
public:
    enum Channel { ROLL = 0, PITCH = 1, THROTTLE = 2, YAW = 3 };

    /** A channel counts as live if it pulsed within the last five frames. */
    static constexpr uint32_t kChannelStaleUs = 100000;
    static constexpr uint32_t kSignalLostUs = 500000;

    /** @param ch a copy of the four ISR channels, taken atomically. */
    void update(const RcPulseChannel ch[4], uint32_t now_us) {
        uint16_t w[4];
        bool all = true;
        for (int i = 0; i < 4; ++i) all = ch[i].read(now_us, kChannelStaleUs, w[i]) && all;

        if (all) {
            roll_ = axis(w[ROLL]);
            pitch_ = axis(w[PITCH]);
            throttle_ = throttle(w[THROTTLE]);
            yaw_ = axis(w[YAW]);
            last_good_us_ = now_us;
            have_good_ = true;
            signal_lost_ = false;
            return;
        }
        if (!have_good_ || now_us - last_good_us_ > kSignalLostUs) {
            signal_lost_ = true;
            roll_ = pitch_ = throttle_ = yaw_ = 0.0f;
        }
    }

    float roll() const { return roll_; }
    float pitch() const { return pitch_; }
    float throttle() const { return throttle_; }
    float yaw() const { return yaw_; }
    bool signalLost() const { return signal_lost_; }

    static float axis(uint16_t us) {
        const float v = (static_cast<float>(us) - 1000.0f) * 2.0f / 1000.0f - 1.0f;
        return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
    }
    static float throttle(uint16_t us) {
        const float v = (static_cast<float>(us) - 1000.0f) / 1000.0f;
        return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    }

private:
    float roll_ = 0.0f, pitch_ = 0.0f, throttle_ = 0.0f, yaw_ = 0.0f;
    uint32_t last_good_us_ = 0;
    bool have_good_ = false;
    bool signal_lost_ = true;
};
