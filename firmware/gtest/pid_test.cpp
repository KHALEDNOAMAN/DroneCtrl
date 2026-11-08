// PIDController is a template, so every behaviour here is checked for both
// float (what the flight controller runs) and double (what an analysis tool
// would run). A behaviour that only holds in one precision is a bug.

#include <gtest/gtest.h>

#include "pid_controller.h"

namespace {

template <typename T>
class PidTyped : public ::testing::Test {
protected:
    // Flight gains from config.h, with the output clamp the firmware uses.
    PIDController<T> pid{T(2.7), T(1.8), T(0.55), T(100), T(400)};
    static constexpr T kDt = T(0.004); // 250 Hz
};

using Precisions = ::testing::Types<float, double>;
TYPED_TEST_SUITE(PidTyped, Precisions);

TYPED_TEST(PidTyped, SetpointStepCausesNoDerivativeKick) {
    using T = TypeParam;
    // Settle at zero error so the derivative is primed.
    for (int i = 0; i < 50; ++i) this->pid.compute(T(0), T(0), this->kDt);

    // A 15 degree setpoint step with the measurement unchanged. Derivative on
    // measurement means only the P and I terms may respond to it.
    const T out = this->pid.compute(T(15), T(0), this->kDt);
    const T p_plus_i = T(2.7) * T(15) + T(1.8) * this->pid.getIntegral();
    EXPECT_NEAR(out, p_plus_i, T(1e-3));
}

TYPED_TEST(PidTyped, HoldIntegralFreezesTheIntegrator) {
    using T = TypeParam;
    for (int i = 0; i < 10; ++i) this->pid.compute(T(5), T(0), this->kDt);
    const T before = this->pid.getIntegral();

    for (int i = 0; i < 100; ++i) this->pid.compute(T(5), T(0), this->kDt, true);
    EXPECT_EQ(this->pid.getIntegral(), before);
}

TYPED_TEST(PidTyped, OutputIsClampedAndReportsSaturation) {
    using T = TypeParam;
    const T out = this->pid.compute(T(1000), T(0), this->kDt);
    EXPECT_EQ(out, T(400));
    EXPECT_TRUE(this->pid.isSaturated());
}

TYPED_TEST(PidTyped, NonPositiveDtIsIgnored) {
    using T = TypeParam;
    EXPECT_EQ(this->pid.compute(T(10), T(0), T(0)), T(0));
    EXPECT_EQ(this->pid.compute(T(10), T(0), T(-0.004)), T(0));
    EXPECT_EQ(this->pid.getIntegral(), T(0));
}

TYPED_TEST(PidTyped, IntegralIsClampedBothWays) {
    using T = TypeParam;
    for (int i = 0; i < 100000; ++i) this->pid.compute(T(50), T(0), this->kDt);
    EXPECT_LE(this->pid.getIntegral(), T(100));
    for (int i = 0; i < 200000; ++i) this->pid.compute(T(-50), T(0), this->kDt);
    EXPECT_GE(this->pid.getIntegral(), T(-100));
}

} // namespace
