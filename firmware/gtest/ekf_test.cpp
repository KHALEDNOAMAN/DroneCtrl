// The EKF must find any static attitude from the accelerometer alone, across
// the envelope, not just at level. Each case feeds the gravity vector for one
// roll/pitch pair with the gyros at rest and checks where the estimate lands.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <tuple>

#include "attitude_ekf.h"

namespace {

constexpr float kDegToRad = 0.017453292f;

// roll, pitch in degrees
using Attitude = std::tuple<float, float>;

class EkfStaticAttitude : public ::testing::TestWithParam<Attitude> {};

TEST_P(EkfStaticAttitude, ConvergesFromLevelWithinTwoSeconds) {
    const auto [roll_deg, pitch_deg] = GetParam();
    const float r = roll_deg * kDegToRad;
    const float p = pitch_deg * kDegToRad;
    // h(x) = g * [-sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)]
    const float ax = -std::sin(p);
    const float ay = std::sin(r) * std::cos(p);
    const float az = std::cos(r) * std::cos(p);

    AttitudeEKF ekf; // starts level, deliberately wrong
    for (int i = 0; i < 500; ++i) { // 2 s at 250 Hz
        ekf.predict(0.0f, 0.0f, 0.0f, 0.004f);
        ekf.updateAccel(ax, ay, az);
    }

    EXPECT_NEAR(ekf.getRollDeg(), roll_deg, 0.5f);
    EXPECT_NEAR(ekf.getPitchDeg(), pitch_deg, 0.5f);
    EXPECT_FALSE(ekf.isDiverged());
    EXPECT_LT(ekf.getRollSigma(), 2.0f * kDegToRad);
}

INSTANTIATE_TEST_SUITE_P(
    Envelope, EkfStaticAttitude,
    ::testing::Values(Attitude{0, 0}, Attitude{10, 0}, Attitude{-10, 0}, Attitude{0, 10},
                      Attitude{0, -10}, Attitude{25, 15}, Attitude{-25, -15},
                      Attitude{40, 0}, Attitude{0, 40}),
    [](const ::testing::TestParamInfo<Attitude>& info) {
        auto name = [](float v) {
            return (v < 0 ? std::string("m") : std::string("p")) +
                   std::to_string(static_cast<int>(std::fabs(v)));
        };
        return "roll_" + name(std::get<0>(info.param)) + "_pitch_" +
               name(std::get<1>(info.param));
    });

} // namespace
