// Property tests for the X mixer, run over a table of commands.
//
// The hand-rolled suite checks named cases. These check two properties that
// must hold for every command, at every throttle:
//   1. no output ever leaves the ESC range, and
//   2. the torque the mixer delivers points the same way as the torque the
//      controller asked for, even when it had to be scaled down.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <tuple>

#include "mixer_math.h"

namespace {

constexpr float kEscMin = 1000.0f;
constexpr float kEscMax = 2000.0f;

// throttle, pitch, roll, yaw (all in ESC microseconds)
using MixCase = std::tuple<float, float, float, float>;

class MixerProperty : public ::testing::TestWithParam<MixCase> {};

TEST_P(MixerProperty, EveryOutputStaysInsideTheEscRange) {
    const auto [base, pitch, roll, yaw] = GetParam();
    const auto out = mixer::mixX(base, pitch, roll, yaw, kEscMin, kEscMax);

    using ::testing::AllOf;
    using ::testing::Ge;
    using ::testing::Le;
    const auto in_range = AllOf(Ge(kEscMin - 1e-3f), Le(kEscMax + 1e-3f));
    EXPECT_THAT(out.fl, in_range);
    EXPECT_THAT(out.fr, in_range);
    EXPECT_THAT(out.bl, in_range);
    EXPECT_THAT(out.br, in_range);
}

TEST_P(MixerProperty, DeliveredTorqueKeepsTheCommandedDirection) {
    const auto [base, pitch, roll, yaw] = GetParam();
    const auto out = mixer::mixX(base, pitch, roll, yaw, kEscMin, kEscMax);

    // Recover each axis from the four outputs. The common throttle cancels.
    const float pitch_out = (out.fl + out.fr - out.bl - out.br) / 4.0f;
    const float roll_out = (out.fl - out.fr + out.bl - out.br) / 4.0f;
    const float yaw_out = (-out.fl + out.fr + out.bl - out.br) / 4.0f;

    const float s = out.authority_scale;
    EXPECT_GT(s, 0.0f);
    EXPECT_LE(s, 1.0f);
    // Scaled equally on all three axes: the direction is preserved.
    EXPECT_NEAR(pitch_out, pitch * s, 1e-2f);
    EXPECT_NEAR(roll_out, roll * s, 1e-2f);
    EXPECT_NEAR(yaw_out, yaw * s, 1e-2f);
    EXPECT_EQ(out.authority_limited, s < 1.0f);
}

INSTANTIATE_TEST_SUITE_P(
    ThrottleByTorqueGrid, MixerProperty,
    ::testing::Combine(::testing::Values(1000.0f, 1100.0f, 1500.0f, 1900.0f, 2000.0f),
                       ::testing::Values(-600.0f, -150.0f, 0.0f, 150.0f, 600.0f),
                       ::testing::Values(-600.0f, 0.0f, 300.0f),
                       ::testing::Values(-200.0f, 0.0f, 200.0f)));

TEST(Mixer, ZeroTorqueAtHoverPassesThrottleThrough) {
    const auto out = mixer::mixX(1400.0f, 0.0f, 0.0f, 0.0f, kEscMin, kEscMax);
    EXPECT_FLOAT_EQ(out.fl, 1400.0f);
    EXPECT_FLOAT_EQ(out.fr, 1400.0f);
    EXPECT_FLOAT_EQ(out.bl, 1400.0f);
    EXPECT_FLOAT_EQ(out.br, 1400.0f);
    EXPECT_FALSE(out.throttle_adjusted);
    EXPECT_FALSE(out.authority_limited);
}

TEST(Mixer, FullThrottleGivesWayToTorque) {
    // At the ceiling there is no headroom, so the throttle must drop to make
    // room for the roll command rather than clipping it away.
    const auto out = mixer::mixX(2000.0f, 0.0f, 100.0f, 0.0f, kEscMin, kEscMax);
    EXPECT_TRUE(out.throttle_adjusted);
    EXPECT_FALSE(out.authority_limited);
    EXPECT_NEAR((out.fl - out.fr + out.bl - out.br) / 4.0f, 100.0f, 1e-3f);
}

} // namespace
