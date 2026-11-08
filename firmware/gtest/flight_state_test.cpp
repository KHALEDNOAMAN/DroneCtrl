// The failsafe policy as a table: for each fault, what an armed airframe must
// do, and that the same fault on the ground blocks arming instead.

#include <gtest/gtest.h>

#include <cstdint>
#include <ostream>
#include <string>

#include "flight_state.h"

namespace {

struct FaultCase {
    std::string name;
    FlightInputs inputs;   // healthy airframe with exactly one fault injected
    FaultCause cause;
    FlightState in_flight; // what an armed airframe must switch to
};

// Print the case by name, so a failure reads "SignalLost" rather than a dump
// of the struct's bytes.
void PrintTo(const FaultCase& c, std::ostream* os) { *os << c.name; }

FlightInputs healthy() {
    FlightInputs in;
    in.throttle_norm = 0.5f;
    return in;
}

FaultCase signalLost() {
    auto in = healthy();
    in.signal_lost = true;
    return {"SignalLost", in, FaultCause::SIGNAL_LOST, FlightState::FAILSAFE_LAND};
}
FaultCase lowBattery() {
    auto in = healthy();
    in.battery_volts = 10.2f;
    return {"LowBattery", in, FaultCause::LOW_BATTERY, FlightState::FAILSAFE_LAND};
}
FaultCase excessiveTilt() {
    auto in = healthy();
    in.roll_deg = 60.0f;
    return {"ExcessiveTilt", in, FaultCause::EXCESSIVE_TILT, FlightState::FAILSAFE_CUT};
}
FaultCase estimatorDiverged() {
    auto in = healthy();
    in.estimator_diverged = true;
    return {"EstimatorDiverged", in, FaultCause::ESTIMATOR_DIVERGED,
            FlightState::FAILSAFE_CUT};
}

// Hold the arm gesture (throttle down, yaw right) for the full period.
void arm(FlightStateMachine& fsm, uint32_t& now) {
    FlightInputs gesture;
    gesture.throttle_norm = 0.0f;
    gesture.yaw_norm = 1.0f;
    for (int i = 0; i < 600 && fsm.state() != FlightState::ARMED; ++i) {
        fsm.update(gesture, now, 0.004f);
        now += 4;
    }
}

class FailsafePolicy : public ::testing::TestWithParam<FaultCase> {};

TEST_P(FailsafePolicy, ArmedAirframeTakesTheRightFailsafe) {
    const FaultCase& c = GetParam();
    FlightStateMachine fsm;
    uint32_t now = 0;
    arm(fsm, now);
    ASSERT_EQ(fsm.state(), FlightState::ARMED);

    fsm.update(c.inputs, now, 0.004f);
    EXPECT_EQ(fsm.state(), c.in_flight);
    EXPECT_EQ(fsm.fault(), c.cause);
}

TEST_P(FailsafePolicy, FailsafeLatchesAfterTheFaultClears) {
    const FaultCase& c = GetParam();
    FlightStateMachine fsm;
    uint32_t now = 0;
    arm(fsm, now);
    fsm.update(c.inputs, now, 0.004f);

    // The fault goes away. The airframe must not hand control back by itself.
    FlightInputs recovered = healthy();
    for (int i = 0; i < 50; ++i) {
        now += 4;
        fsm.update(recovered, now, 0.004f);
    }
    EXPECT_NE(fsm.state(), FlightState::ARMED);
}

TEST_P(FailsafePolicy, SameFaultOnTheGroundBlocksArming) {
    const FaultCase& c = GetParam();
    FlightStateMachine fsm;
    FlightInputs gesture = c.inputs;
    gesture.throttle_norm = 0.0f;
    gesture.yaw_norm = 1.0f;

    uint32_t now = 0;
    for (int i = 0; i < 1000; ++i) { // 4 s, twice the arm hold
        fsm.update(gesture, now, 0.004f);
        now += 4;
    }
    EXPECT_EQ(fsm.state(), FlightState::DISARMED);
    EXPECT_FALSE(fsm.motorsEnabled());
}

INSTANTIATE_TEST_SUITE_P(EveryFault, FailsafePolicy,
                         ::testing::Values(signalLost(), lowBattery(), excessiveTilt(),
                                           estimatorDiverged()),
                         [](const ::testing::TestParamInfo<FaultCase>& info) {
                             return info.param.name;
                         });

} // namespace
