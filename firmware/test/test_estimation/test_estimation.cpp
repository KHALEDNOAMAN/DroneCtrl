/**
 * Host tests for the attitude EKF.
 *
 * Every test drives the filter from a synthetic truth trajectory, so there is
 * a known answer to compare against. The generator uses the exact Euler
 * kinematics rather than the filter's linearised copy of them, which is what
 * makes these tests capable of catching an error in the Jacobian instead of
 * just reproducing it.
 *
 * Noise comes from a fixed-seed linear congruential generator rather than
 * <random>, so a failure here reproduces byte for byte on any machine. A
 * statistical test that only fails sometimes is worse than no test.
 */

#include "../test_harness.h"
#include <math.h>

#include "attitude_ekf.h"

static constexpr float kDt = 1.0f / 250.0f;
static constexpr float kDeg = 57.2957795f;

// ------------------------------------------------------- truth generator

class Rng {
public:
    explicit Rng(uint32_t seed) : s_(seed) {}
    /** Uniform in [-1, 1]. */
    float uniform() {
        s_ = s_ * 1664525u + 1013904223u;
        return (float)((s_ >> 8) & 0xFFFFFF) / 8388608.0f - 1.0f;
    }
    /** Approximately Gaussian, unit variance, by summing three uniforms. */
    float gauss() { return (uniform() + uniform() + uniform()) * 1.0f; }

private:
    uint32_t s_;
};

struct Truth {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    /** Advance the truth attitude by commanded Euler rates. */
    void step(float roll_dot, float pitch_dot, float yaw_dot, float dt) {
        roll += roll_dot * dt;
        pitch += pitch_dot * dt;
        yaw += yaw_dot * dt;
    }

    /** Body rates that produce those Euler rates at the current attitude. */
    void bodyRates(float roll_dot, float pitch_dot, float yaw_dot,
                   float& p, float& q, float& r) const {
        const float sr = sinf(roll), cr = cosf(roll);
        const float sp = sinf(pitch), cp = cosf(pitch);
        p = roll_dot - yaw_dot * sp;
        q = pitch_dot * cr + yaw_dot * cp * sr;
        r = -pitch_dot * sr + yaw_dot * cp * cr;
    }

    /** Gravity as the accelerometer sees it, in g, with no manoeuvre. */
    void accel(float& ax, float& ay, float& az) const {
        ax = -sinf(pitch);
        ay = sinf(roll) * cosf(pitch);
        az = cosf(roll) * cosf(pitch);
    }
};

// ------------------------------------------------------------------ tests

/** A level, still airframe must settle at zero and stay there. */
void test_ekf_converges_when_level() {
    AttitudeEKF ekf;
    for (int i = 0; i < 2500; ++i) { // 10 seconds
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 1.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ekf.getRollDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ekf.getPitchDeg());
}

/** Held at a constant tilt, it must find that tilt from the accelerometer. */
void test_ekf_converges_to_static_tilt() {
    Truth t;
    t.roll = 20.0f / kDeg;
    t.pitch = -12.0f / kDeg;

    AttitudeEKF ekf;
    float ax, ay, az;
    t.accel(ax, ay, az);
    for (int i = 0; i < 2500; ++i) {
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(ax, ay, az);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, ekf.getRollDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -12.0f, ekf.getPitchDeg());
}

/** Seeding from one accelerometer sample must start it near the answer. */
void test_ekf_initialization_from_accel() {
    Truth t;
    t.roll = 30.0f / kDeg;
    t.pitch = 15.0f / kDeg;
    float ax, ay, az;
    t.accel(ax, ay, az);

    AttitudeEKF ekf;
    ekf.initializeFromAccel(ax, ay, az);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, ekf.getRollDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 15.0f, ekf.getPitchDeg());
}

/**
 * The reason the filter carries bias states at all.
 *
 * A gyro with a 2 deg/s offset integrates into 120 degrees of error in a
 * minute. The filter must both estimate the offset and keep attitude correct
 * despite it.
 */
void test_ekf_estimates_gyro_bias() {
    const float bias_p = 2.0f / kDeg; // rad/s
    const float bias_q = -1.5f / kDeg;

    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 250 * 60; ++i) { // 60 seconds
        ekf.predict(bias_p, bias_q, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 1.0f);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.3f, 2.0f, ekf.getBiasP() * kDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, -1.5f, ekf.getBiasQ() * kDeg);
    // Attitude must be unharmed by the bias it just absorbed.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, ekf.getRollDeg());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, ekf.getPitchDeg());
}

/**
 * The direct comparison against the filter this replaces.
 *
 * The honest framing matters here. On a level airframe with a clean
 * accelerometer, a complementary filter at alpha = 0.98 and 250 Hz settles at
 * only about 0.4 degrees of error under a 2 deg/s bias, because the
 * accelerometer is pulling it back on every single step. Claiming the EKF
 * rescues it from a disaster in that case would be false.
 *
 * The difference shows up when the accelerometer stops being usable, which on
 * a quadcopter is any aggressive manoeuvre, because the sensor then measures
 * thrust rather than gravity. The complementary filter has no memory of the
 * bias, so the moment its only correction is gone it integrates raw gyro and
 * drifts at the full bias rate. The EKF carries the bias as a state, so it
 * keeps propagating correctly through the same gap.
 *
 * Ten seconds of blind propagation under a 2 deg/s bias is 20 degrees of drift
 * for the old filter. That is the gap this test measures.
 */
void test_ekf_holds_attitude_through_accel_dropout() {
    const float bias = 2.0f / kDeg;
    const float alpha = 0.98f;

    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    float comp_roll = 0.0f;

    // Phase 1: quiet flight. Both filters see a good accelerometer, and the
    // EKF gets the chance to learn the bias.
    for (int i = 0; i < 250 * 30; ++i) {
        ekf.predict(bias, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 1.0f);
        comp_roll = alpha * (comp_roll + bias * kDt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 2.0f, ekf.getBiasP() * kDeg);

    // Phase 2: the accelerometer is unusable, so neither filter gets an
    // update. The airframe is in fact still level.
    for (int i = 0; i < 250 * 10; ++i) {
        ekf.predict(bias, 0.0f, 0.0f, kDt);
        comp_roll += bias * kDt; // nothing left to pull it back
    }

    const float ekf_err = fabsf(ekf.getRollDeg());
    const float comp_err = fabsf(comp_roll * kDeg);

    TEST_ASSERT_TRUE(comp_err > 15.0f);  // the old filter has walked away
    TEST_ASSERT_TRUE(ekf_err < 3.0f);    // the new one has not
    TEST_ASSERT_TRUE(ekf_err < comp_err * 0.25f);
}

/**
 * With a clean accelerometer the two should be close, and the EKF should still
 * be the better of them. Stated separately from the test above so the modest
 * size of this win is on the record rather than implied to be larger.
 */
void test_ekf_at_least_matches_complementary_filter_when_static() {
    const float bias = 2.0f / kDeg;
    const float alpha = 0.98f;

    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    float comp_roll = 0.0f;

    for (int i = 0; i < 250 * 30; ++i) {
        ekf.predict(bias, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 1.0f);
        comp_roll = alpha * (comp_roll + bias * kDt);
    }

    TEST_ASSERT_TRUE(fabsf(ekf.getRollDeg()) <= fabsf(comp_roll * kDeg) + 1e-3f);
}

/**
 * A lateral acceleration is not gravity. The filter must not read it as tilt.
 *
 * This is the failure mode that makes a naive accelerometer update dangerous
 * on a quadcopter: every translation looks like a lean, so the controller
 * corrects an attitude error that was never there and the airframe oscillates.
 */
void test_ekf_rejects_acceleration_transient() {
    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 500; ++i) {
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 1.0f);
    }

    // Half a g sideways for a second, with the gyro correctly reporting no
    // rotation at all.
    for (int i = 0; i < 250; ++i) {
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.5f, 1.0f);
    }

    // Trusting that sample blindly would read as atan2(0.5, 1) = 26.6 degrees.
    TEST_ASSERT_TRUE(fabsf(ekf.getRollDeg()) < 8.0f);
}

/** After the disturbance clears, it must come back rather than stay offset. */
void test_ekf_recovers_after_transient() {
    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 250; ++i) {
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.5f, 1.0f);
    }
    for (int i = 0; i < 1250; ++i) { // 5 quiet seconds
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 1.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, ekf.getRollDeg());
}

/** It must track a real rotation, not merely hold still convincingly. */
void test_ekf_tracks_commanded_rotation() {
    Truth t;
    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);

    const float roll_rate = 30.0f / kDeg; // 30 deg/s for one second
    for (int i = 0; i < 250; ++i) {
        float p, q, r;
        t.bodyRates(roll_rate, 0.0f, 0.0f, p, q, r);
        t.step(roll_rate, 0.0f, 0.0f, kDt);

        float ax, ay, az;
        t.accel(ax, ay, az);
        ekf.predict(p, q, r, kDt);
        ekf.updateAccel(ax, ay, az);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.5f, t.roll * kDeg, ekf.getRollDeg());
}

/** Realistic noise on both sensors must not stop it tracking. */
void test_ekf_tracks_under_sensor_noise() {
    Rng rng(12345u);
    Truth t;
    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);

    const float gyro_sigma = 0.02f; // rad/s, around an MPU6050 at 42 Hz bandwidth
    const float accel_sigma = 0.05f; // g

    float worst = 0.0f;
    for (int i = 0; i < 250 * 10; ++i) {
        // A slow figure the filter has to actually follow.
        const float rate = 20.0f / kDeg * sinf(2.0f * 3.14159f * 0.2f * i * kDt);
        float p, q, r;
        t.bodyRates(rate, 0.0f, 0.0f, p, q, r);
        t.step(rate, 0.0f, 0.0f, kDt);

        float ax, ay, az;
        t.accel(ax, ay, az);

        ekf.predict(p + gyro_sigma * rng.gauss(), q + gyro_sigma * rng.gauss(),
                    r + gyro_sigma * rng.gauss(), kDt);
        ekf.updateAccel(ax + accel_sigma * rng.gauss(), ay + accel_sigma * rng.gauss(),
                        az + accel_sigma * rng.gauss());

        if (i > 250) { // let it settle first
            const float err = fabsf(ekf.getRollDeg() - t.roll * kDeg);
            if (err > worst) worst = err;
        }
    }
    TEST_ASSERT_TRUE(worst < 4.0f);
}

/** Covariance must stay finite and bounded over a long run. */
void test_ekf_stays_numerically_healthy() {
    Rng rng(999u);
    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 250 * 300; ++i) { // 5 minutes
        ekf.predict(0.01f * rng.gauss(), 0.01f * rng.gauss(), 0.01f * rng.gauss(), kDt);
        ekf.updateAccel(0.03f * rng.gauss(), 0.03f * rng.gauss(), 1.0f + 0.03f * rng.gauss());
    }
    TEST_ASSERT_FALSE(ekf.isDiverged());
    TEST_ASSERT_TRUE(ekf.getRollSigma() < 1.0f);
    TEST_ASSERT_TRUE(ekf.getRollSigma() > 0.0f);
}

/** A dead accelerometer reading all zeros must be skipped, not divided by. */
void test_ekf_survives_zero_accel() {
    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 250; ++i) {
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 0.0f);
    }
    TEST_ASSERT_FALSE(ekf.isDiverged());
}

/** A non-positive dt must be a no-op rather than a source of infinities. */
void test_ekf_rejects_non_positive_dt() {
    AttitudeEKF ekf;
    ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
    const float before = ekf.getRollDeg();
    ekf.predict(1.0f, 1.0f, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, before, ekf.getRollDeg());
    TEST_ASSERT_FALSE(ekf.isDiverged());
}

/** Near vertical, tan(pitch) must stay finite instead of blowing the filter up. */
void test_ekf_bounded_near_vertical() {
    AttitudeEKF ekf;
    for (int i = 0; i < 1000; ++i) {
        ekf.predict(0.0f, 2.0f, 0.0f, kDt); // pitch up hard and keep going
        ekf.updateAccel(-1.0f, 0.0f, 0.0f);
    }
    TEST_ASSERT_FALSE(ekf.isDiverged());
    TEST_ASSERT_TRUE(fabsf(ekf.getPitchDeg()) <= 90.0f);
}

/** Uncertainty must shrink once data arrives, which is the filter working. */
void test_ekf_covariance_shrinks_with_data() {
    AttitudeEKF ekf;
    const float sigma0 = ekf.getRollSigma();
    for (int i = 0; i < 1250; ++i) {
        ekf.predict(0.0f, 0.0f, 0.0f, kDt);
        ekf.updateAccel(0.0f, 0.0f, 1.0f);
    }
    TEST_ASSERT_TRUE(ekf.getRollSigma() < sigma0 * 0.2f);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ekf_converges_when_level);
    RUN_TEST(test_ekf_converges_to_static_tilt);
    RUN_TEST(test_ekf_initialization_from_accel);
    RUN_TEST(test_ekf_estimates_gyro_bias);
    RUN_TEST(test_ekf_holds_attitude_through_accel_dropout);
    RUN_TEST(test_ekf_at_least_matches_complementary_filter_when_static);
    RUN_TEST(test_ekf_rejects_acceleration_transient);
    RUN_TEST(test_ekf_recovers_after_transient);
    RUN_TEST(test_ekf_tracks_commanded_rotation);
    RUN_TEST(test_ekf_tracks_under_sensor_noise);
    RUN_TEST(test_ekf_stays_numerically_healthy);
    RUN_TEST(test_ekf_survives_zero_accel);
    RUN_TEST(test_ekf_rejects_non_positive_dt);
    RUN_TEST(test_ekf_bounded_near_vertical);
    RUN_TEST(test_ekf_covariance_shrinks_with_data);
    return UNITY_END();
}
