/**
 * Host tests for the control law and the motor mixer.
 *
 * These build for the host with plain g++, so they run in CI with no board
 * attached and no toolchain for the target. They link the exact headers the
 * flight controller compiles, which is the point: a control law verified in a
 * separate reimplementation has only been verified as a reimplementation.
 */

#include "../test_harness.h"
#include <math.h>
#include <initializer_list>

#include "pid_controller.h"
#include "mixer_math.h"

static constexpr float kDt = 1.0f / 250.0f;
static constexpr float kEscMin = 1000.0f;
static constexpr float kEscMax = 2000.0f;

// ---------------------------------------------------------------- PID

/** A pure proportional controller should return Kp times the error exactly. */
void test_pid_proportional_is_exact() {
    PIDController<float> pid(2.0f, 0.0f, 0.0f, 100.0f, 400.0f);
    const float out = pid.compute(10.0f, 4.0f, kDt);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 12.0f, out);
}

/**
 * The first call must not produce a derivative kick.
 *
 * This is a regression test for a real defect: prev_measurement_ starts at
 * zero, so differencing the first real sample against it produced a spike of
 * measurement/dt. At 250 Hz and a 5 degree starting tilt that was 1250 deg/s
 * of phantom rate, which with Kd = 15 saturated the output on the very first
 * step after arming.
 */
void test_pid_no_derivative_kick_on_first_call() {
    PIDController<float> pid(0.0f, 0.0f, 15.0f, 100.0f, 400.0f);
    const float first = pid.compute(0.0f, 5.0f, kDt);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, first);
}

/** Holding a constant measurement should drive the derivative term to zero. */
void test_pid_derivative_settles_on_constant_measurement() {
    PIDController<float> pid(0.0f, 0.0f, 15.0f, 100.0f, 400.0f);
    float out = 0.0f;
    for (int i = 0; i < 500; ++i) out = pid.compute(0.0f, 5.0f, kDt);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out);
}

/** A sustained error must not integrate past the configured clamp. */
void test_pid_integral_clamps() {
    PIDController<float> pid(0.0f, 1.0f, 0.0f, 3.0f, 400.0f);
    for (int i = 0; i < 5000; ++i) pid.compute(10.0f, 0.0f, kDt);
    TEST_ASSERT_TRUE(pid.getIntegral() <= 3.0f + 1e-4f);
    TEST_ASSERT_TRUE(pid.getIntegral() >= -3.0f - 1e-4f);
}

/** hold_integral must freeze the integrator, not merely slow it. */
void test_pid_integral_hold_freezes_accumulation() {
    PIDController<float> pid(0.0f, 1.0f, 0.0f, 100.0f, 400.0f);
    for (int i = 0; i < 100; ++i) pid.compute(10.0f, 0.0f, kDt);
    const float held = pid.getIntegral();
    for (int i = 0; i < 100; ++i) pid.compute(10.0f, 0.0f, kDt, true);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, held, pid.getIntegral());
}

/** Output clamping must be reported, so the caller can hold the integrator. */
void test_pid_reports_saturation() {
    PIDController<float> pid(100.0f, 0.0f, 0.0f, 100.0f, 50.0f);
    const float out = pid.compute(10.0f, 0.0f, kDt);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 50.0f, out);
    TEST_ASSERT_TRUE(pid.isSaturated());
}

/** reset() must clear the integrator and re-arm the derivative guard. */
void test_pid_reset_clears_state() {
    PIDController<float> pid(0.0f, 1.0f, 15.0f, 100.0f, 400.0f);
    for (int i = 0; i < 100; ++i) pid.compute(10.0f, 3.0f, kDt);
    pid.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.getIntegral());

    // The derivative guard has to be re-armed too, so check it on a pure D
    // controller. Checking the total output here instead would fail for the
    // wrong reason: with Ki set, one step of error legitimately puts
    // error * dt into the integral, and that is the controller working.
    PIDController<float> d_only(0.0f, 0.0f, 15.0f, 100.0f, 400.0f);
    for (int i = 0; i < 100; ++i) d_only.compute(0.0f, 3.0f, kDt);
    d_only.reset();
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, d_only.compute(0.0f, 5.0f, kDt));
}

/** A non-positive dt must be rejected rather than dividing by zero. */
void test_pid_rejects_non_positive_dt() {
    PIDController<float> pid(2.0f, 1.0f, 15.0f, 100.0f, 400.0f);
    const float out = pid.compute(10.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out);
    TEST_ASSERT_TRUE(isfinite(pid.getIntegral()));
}

/**
 * The derivative cutoff must not move with the loop rate.
 *
 * The old implementation used a fixed smoothing factor, so halving dt halved
 * the filter's time constant and silently retuned the D term. Driving the same
 * ramp at 250 Hz and 500 Hz should now give the same steady derivative.
 */
void test_pid_derivative_filter_is_rate_independent() {
    auto steady_derivative = [](float dt) {
        PIDController<float> pid(0.0f, 0.0f, 1.0f, 100.0f, 400.0f);
        float measurement = 0.0f;
        float out = 0.0f;
        for (float t = 0.0f; t < 2.0f; t += dt) {
            measurement += 10.0f * dt; // 10 units per second ramp
            out = pid.compute(0.0f, measurement, dt);
        }
        return out;
    };
    TEST_ASSERT_FLOAT_WITHIN(0.05f, steady_derivative(1.0f / 250.0f),
                             steady_derivative(1.0f / 500.0f));
}

// -------------------------------------------------------------- Mixer

/** With no torque commanded, all four motors sit at the common throttle. */
void test_mix_neutral_is_uniform() {
    const auto o = mixer::mixX(1500.0f, 0.0f, 0.0f, 0.0f, kEscMin, kEscMax);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1500.0f, o.fl);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1500.0f, o.fr);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1500.0f, o.bl);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1500.0f, o.br);
    TEST_ASSERT_FALSE(o.throttle_adjusted);
    TEST_ASSERT_FALSE(o.authority_limited);
}

/**
 * A positive roll command means "roll right side down", so it must raise the
 * LEFT pair and lower the right pair, equally.
 *
 * This is the regression test for a sign error worth naming. The estimator
 * reports positive roll as right side down, while the old mixer raised the
 * right pair for a positive roll command. Those two together close the roll
 * loop with positive feedback: a drone leaning right is commanded to lean
 * further right. It never showed up in the previous code only because the old
 * complementary filter had roll and pitch transposed as well, and the two
 * mistakes cancelled.
 */
void test_mix_roll_is_antisymmetric() {
    const auto o = mixer::mixX(1500.0f, 0.0f, 100.0f, 0.0f, kEscMin, kEscMax);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1600.0f, o.fl);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1600.0f, o.bl);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1400.0f, o.fr);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1400.0f, o.br);
}

/** Yaw must act on the diagonals, which is what distinguishes it from roll. */
void test_mix_yaw_acts_on_diagonals() {
    const auto o = mixer::mixX(1500.0f, 0.0f, 0.0f, 50.0f, kEscMin, kEscMax);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, o.fr, o.bl); // one diagonal up together
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, o.fl, o.br); // the other down together
    TEST_ASSERT_TRUE(o.fr > o.fl);
}

/** Pitch is unchanged by the roll fix: nose up still means front pair up. */
void test_mix_pitch_raises_front_pair() {
    const auto o = mixer::mixX(1500.0f, 100.0f, 0.0f, 0.0f, kEscMin, kEscMax);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1600.0f, o.fl);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1600.0f, o.fr);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1400.0f, o.bl);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1400.0f, o.br);
}

/**
 * The central claim of the new mixer: at full throttle a roll command still
 * produces the full commanded differential.
 *
 * The old mixer clamped each motor on its own, so at 2000 us base the two
 * rising motors clipped and the differential collapsed to half of what was
 * asked for, losing roll authority precisely at full power.
 */
void test_mix_preserves_authority_at_full_throttle() {
    const auto o = mixer::mixX(2000.0f, 0.0f, 100.0f, 0.0f, kEscMin, kEscMax);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 200.0f, o.fl - o.fr);
    TEST_ASSERT_TRUE(o.throttle_adjusted);
    TEST_ASSERT_FALSE(o.authority_limited);
}

/** The same must hold at the bottom of the range. */
void test_mix_preserves_authority_at_zero_throttle() {
    const auto o = mixer::mixX(1000.0f, 80.0f, 0.0f, 0.0f, kEscMin, kEscMax);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 160.0f, o.fl - o.bl);
    TEST_ASSERT_TRUE(o.throttle_adjusted);
}

/** No output may ever leave the ESC range, whatever is commanded. */
void test_mix_never_exceeds_esc_range() {
    const float cmds[] = {-5000.0f, -600.0f, -50.0f, 0.0f, 50.0f, 600.0f, 5000.0f};
    for (float base : {900.0f, 1000.0f, 1500.0f, 2000.0f, 2500.0f}) {
        for (float p : cmds) {
            for (float r : cmds) {
                for (float y : cmds) {
                    const auto o = mixer::mixX(base, p, r, y, kEscMin, kEscMax);
                    const float m[4] = {o.fl, o.fr, o.bl, o.br};
                    for (int i = 0; i < 4; ++i) {
                        TEST_ASSERT_TRUE(m[i] >= kEscMin - 1e-3f);
                        TEST_ASSERT_TRUE(m[i] <= kEscMax + 1e-3f);
                    }
                }
            }
        }
    }
}

/**
 * When the torques alone are wider than the ESC range, scaling them down must
 * keep their direction. Losing magnitude is unavoidable, but a mixer that
 * distorted the torque direction under saturation would push the airframe
 * somewhere the controller never asked to go.
 */
void test_mix_scaling_preserves_torque_direction() {
    const auto small = mixer::mixX(1500.0f, 100.0f, 60.0f, 20.0f, kEscMin, kEscMax);
    const auto huge = mixer::mixX(1500.0f, 1000.0f, 600.0f, 200.0f, kEscMin, kEscMax);
    TEST_ASSERT_TRUE(huge.authority_limited);

    // Compare the two as differential patterns about their own mean.
    const float sm[4] = {small.fl, small.fr, small.bl, small.br};
    const float hm[4] = {huge.fl, huge.fr, huge.bl, huge.br};
    float s_mean = 0.0f, h_mean = 0.0f;
    for (int i = 0; i < 4; ++i) { s_mean += sm[i] / 4.0f; h_mean += hm[i] / 4.0f; }

    const float ratio = (hm[0] - h_mean) / (sm[0] - s_mean);
    for (int i = 1; i < 4; ++i) {
        const float r = (hm[i] - h_mean) / (sm[i] - s_mean);
        TEST_ASSERT_FLOAT_WITHIN(1e-2f, ratio, r);
    }
}

/** Saturation must be reported so the controller can hold its integrators. */
void test_mix_reports_authority_limit() {
    const auto ok = mixer::mixX(1500.0f, 100.0f, 0.0f, 0.0f, kEscMin, kEscMax);
    TEST_ASSERT_FALSE(ok.authority_limited);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, ok.authority_scale);

    const auto over = mixer::mixX(1500.0f, 2000.0f, 0.0f, 0.0f, kEscMin, kEscMax);
    TEST_ASSERT_TRUE(over.authority_limited);
    TEST_ASSERT_TRUE(over.authority_scale < 1.0f);
}

int main(int, char**) {
    UNITY_BEGIN();

    RUN_TEST(test_pid_proportional_is_exact);
    RUN_TEST(test_pid_no_derivative_kick_on_first_call);
    RUN_TEST(test_pid_derivative_settles_on_constant_measurement);
    RUN_TEST(test_pid_integral_clamps);
    RUN_TEST(test_pid_integral_hold_freezes_accumulation);
    RUN_TEST(test_pid_reports_saturation);
    RUN_TEST(test_pid_reset_clears_state);
    RUN_TEST(test_pid_rejects_non_positive_dt);
    RUN_TEST(test_pid_derivative_filter_is_rate_independent);

    RUN_TEST(test_mix_neutral_is_uniform);
    RUN_TEST(test_mix_roll_is_antisymmetric);
    RUN_TEST(test_mix_yaw_acts_on_diagonals);
    RUN_TEST(test_mix_pitch_raises_front_pair);
    RUN_TEST(test_mix_preserves_authority_at_full_throttle);
    RUN_TEST(test_mix_preserves_authority_at_zero_throttle);
    RUN_TEST(test_mix_never_exceeds_esc_range);
    RUN_TEST(test_mix_scaling_preserves_torque_direction);
    RUN_TEST(test_mix_reports_authority_limit);

    return UNITY_END();
}
