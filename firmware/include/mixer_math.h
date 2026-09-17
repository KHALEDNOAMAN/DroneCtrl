#pragma once

/**
 * Motor mixing for an X-configuration quadcopter, with no hardware dependency.
 *
 * This is split out of MotorMixer so the mixing law can be unit tested on the
 * host and linked into the SIL harness. MotorMixer keeps the Servo objects and
 * the armed flag, and calls in here for the arithmetic.
 *
 * Saturation
 * ----------
 * The naive mix is base + pitch +- roll +- yaw per motor, each clamped to the
 * ESC range independently. That clamp is where attitude authority quietly goes
 * missing: once a motor hits the ceiling, the extra command that should have
 * gone into it is discarded, the four outputs no longer differ by the torque
 * the controller asked for, and the airframe stops responding in roll or pitch
 * exactly at the high throttle where it is least forgiving. The controller
 * cannot see this happen, so its integrator winds up against a limit it does
 * not know about.
 *
 * What this does instead is treat the differential part as the thing worth
 * protecting and the common throttle as the thing that can give way:
 *
 *   1. Mix the differential torques with no common throttle.
 *   2. Work out the widest common throttle that keeps every motor inside the
 *      ESC range once those torques are added, and move the requested throttle
 *      into that window.
 *   3. Only if the torques alone are too wide to fit at any throttle is the
 *      differential itself scaled down, equally on all axes so the commanded
 *      torque direction is preserved even when its magnitude cannot be.
 *
 * This is the same trade PX4 and Betaflight make. Sacrificing altitude to hold
 * attitude is nearly always right, because a quadcopter that has lost attitude
 * control cannot recover altitude either.
 *
 * The caller is told which of the three cases happened, so the condition can be
 * logged and used to hold the PID integrators.
 */

namespace mixer {

struct Outputs {
    float fl; ///< front left
    float fr; ///< front right
    float bl; ///< back left
    float br; ///< back right

    /** True when the requested common throttle had to be moved to fit. */
    bool throttle_adjusted;
    /** True when the differential torques themselves had to be scaled down. */
    bool authority_limited;
    /** Factor the torques were scaled by, 1.0 when untouched. */
    float authority_scale;
};

/**
 * Mix one control step.
 *
 * @param base_pwm   requested common throttle in ESC microseconds
 * @param pitch_cmd  pitch torque in the same units
 * @param roll_cmd   roll torque
 * @param yaw_cmd    yaw torque
 * @param esc_min    lower ESC limit, microseconds
 * @param esc_max    upper ESC limit, microseconds
 */
inline Outputs mixX(float base_pwm, float pitch_cmd, float roll_cmd, float yaw_cmd,
                    float esc_min, float esc_max) {
    Outputs out{};
    out.authority_scale = 1.0f;

    // Differential part only.
    //
    // Sign convention: a positive command produces a positive response in the
    // axis it names, using the same body frame the estimator reports in (x
    // forward, y right, positive roll is right side down, positive pitch is
    // nose up).
    //
    //   pitch: nose up needs more thrust at the front, so the front pair rises.
    //   roll:  right side down needs more thrust on the LEFT, so the left pair
    //          rises. This is the opposite of what the original mixer did, and
    //          the discrepancy was real: paired with an estimator that reports
    //          positive roll as right side down, the old signs closed the roll
    //          loop with positive feedback.
    //   yaw:   the diagonals take opposite signs, because that is where the
    //          reaction torque of two counter-rotating pairs comes from.
    float d[4];
    d[0] = pitch_cmd + roll_cmd - yaw_cmd; // FL
    d[1] = pitch_cmd - roll_cmd + yaw_cmd; // FR
    d[2] = -pitch_cmd + roll_cmd + yaw_cmd; // BL
    d[3] = -pitch_cmd - roll_cmd - yaw_cmd; // BR

    float d_min = d[0], d_max = d[0];
    for (int i = 1; i < 4; ++i) {
        if (d[i] < d_min) d_min = d[i];
        if (d[i] > d_max) d_max = d[i];
    }

    const float span = d_max - d_min;
    const float range = esc_max - esc_min;

    if (span > range) {
        // The torques alone are wider than the ESC range. Nothing can fit them,
        // so shrink all axes by one factor and keep the torque direction.
        out.authority_scale = range / span;
        out.authority_limited = true;
        for (int i = 0; i < 4; ++i) d[i] *= out.authority_scale;
        d_min *= out.authority_scale;
        d_max *= out.authority_scale;
    }

    // Common throttle now has a window it can live in without clipping anything.
    const float lo = esc_min - d_min;
    const float hi = esc_max - d_max;
    float base = base_pwm;
    if (base < lo) {
        base = lo;
        out.throttle_adjusted = true;
    } else if (base > hi) {
        base = hi;
        out.throttle_adjusted = true;
    }

    out.fl = base + d[0];
    out.fr = base + d[1];
    out.bl = base + d[2];
    out.br = base + d[3];
    return out;
}

/** Clamp a value into [lo, hi]. Present so the header needs no Arduino.h. */
inline float clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

} // namespace mixer
