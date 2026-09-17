/**
 * Host tests for the arming and failsafe state machine, and for the CAN
 * telemetry encoding.
 *
 * The failsafe table is the part of this firmware where a mistake breaks
 * something physical, and it is also the part that is impossible to exercise
 * on a bench without deliberately damaging a flight. Making time and sensor
 * health into parameters is what lets every branch below run in milliseconds.
 */

#include "../test_harness.h"
#include <math.h>

#include "flight_state.h"
#include "can_telemetry.h"

static constexpr float kDt = 1.0f / 250.0f;

/** Run the machine for a stretch of wall clock with fixed inputs. */
static void run(FlightStateMachine& fsm, FlightInputs in, uint32_t& now_ms, float seconds) {
    const int steps = (int)(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        now_ms += 4; // 250 Hz
        fsm.update(in, now_ms, kDt);
    }
}

/** A healthy airframe starts disarmed with motors inhibited. */
void test_starts_disarmed() {
    FlightStateMachine fsm;
    TEST_ASSERT_EQUAL(FlightState::DISARMED, fsm.state());
    TEST_ASSERT_FALSE(fsm.motorsEnabled());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fsm.commandedThrottle());
}

/** The arm gesture must be held for the full period before motors enable. */
void test_arming_requires_full_hold() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    FlightInputs in;
    in.throttle_norm = 0.0f;
    in.yaw_norm = 1.0f;

    run(fsm, in, now, 1.0f); // one second, short of the two required
    TEST_ASSERT_EQUAL(FlightState::ARMING, fsm.state());
    TEST_ASSERT_FALSE(fsm.motorsEnabled());

    run(fsm, in, now, 1.5f);
    TEST_ASSERT_EQUAL(FlightState::ARMED, fsm.state());
    TEST_ASSERT_TRUE(fsm.motorsEnabled());
}

/** Releasing the gesture early must restart the count, not bank the progress. */
void test_arming_aborts_when_gesture_released() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    FlightInputs held;
    held.yaw_norm = 1.0f;
    FlightInputs released;

    run(fsm, held, now, 1.8f);
    run(fsm, released, now, 0.1f);
    TEST_ASSERT_EQUAL(FlightState::DISARMED, fsm.state());

    run(fsm, held, now, 1.0f);
    TEST_ASSERT_EQUAL(FlightState::ARMING, fsm.state()); // counting from scratch
}

/** Arming with the throttle up must be refused outright. */
void test_cannot_arm_with_throttle_up() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    FlightInputs in;
    in.throttle_norm = 0.5f;
    in.yaw_norm = 1.0f;
    run(fsm, in, now, 5.0f);
    TEST_ASSERT_EQUAL(FlightState::DISARMED, fsm.state());
}

static void armIt(FlightStateMachine& fsm, uint32_t& now) {
    FlightInputs in;
    in.yaw_norm = 1.0f;
    run(fsm, in, now, 2.5f);
    TEST_ASSERT_EQUAL(FlightState::ARMED, fsm.state());
}

/** Signal loss is recoverable, so it must descend rather than cut power. */
void test_signal_loss_lands_rather_than_cutting() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs in;
    in.throttle_norm = 0.6f;
    in.signal_lost = true;
    now += 4;
    fsm.update(in, now, kDt);

    TEST_ASSERT_EQUAL(FlightState::FAILSAFE_LAND, fsm.state());
    TEST_ASSERT_EQUAL(FaultCause::SIGNAL_LOST, fsm.fault());
    TEST_ASSERT_TRUE(fsm.motorsEnabled()); // still flying it down
}

/** The landing ramp must reach zero and then cut, monotonically. */
void test_failsafe_land_ramps_down_monotonically() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs in;
    in.throttle_norm = 0.6f;
    in.signal_lost = true;
    now += 4;
    fsm.update(in, now, kDt);

    float previous = fsm.commandedThrottle();
    for (int i = 0; i < 250 * 10; ++i) {
        now += 4;
        fsm.update(in, now, kDt);
        TEST_ASSERT_TRUE(fsm.commandedThrottle() <= previous + 1e-6f);
        previous = fsm.commandedThrottle();
    }
    TEST_ASSERT_EQUAL(FlightState::FAILSAFE_CUT, fsm.state());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fsm.commandedThrottle());
}

/** Past the recoverable tilt the controller cannot help, so power must stop. */
void test_excessive_tilt_cuts_immediately() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs in;
    in.throttle_norm = 0.6f;
    in.roll_deg = 70.0f;
    now += 4;
    fsm.update(in, now, kDt);

    TEST_ASSERT_EQUAL(FlightState::FAILSAFE_CUT, fsm.state());
    TEST_ASSERT_EQUAL(FaultCause::EXCESSIVE_TILT, fsm.fault());
    TEST_ASSERT_FALSE(fsm.motorsEnabled());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fsm.commandedThrottle());
}

/**
 * A diverged estimator means attitude is unknown. Every command from that
 * point is guesswork, so a controlled descent is not on the table.
 */
void test_estimator_divergence_cuts_immediately() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs in;
    in.throttle_norm = 0.6f;
    in.estimator_diverged = true;
    now += 4;
    fsm.update(in, now, kDt);

    TEST_ASSERT_EQUAL(FlightState::FAILSAFE_CUT, fsm.state());
    TEST_ASSERT_EQUAL(FaultCause::ESTIMATOR_DIVERGED, fsm.fault());
}

/** A flat battery is known in advance, so it gets a controlled descent. */
void test_low_battery_lands() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs in;
    in.throttle_norm = 0.4f;
    in.battery_volts = 9.8f;
    now += 4;
    fsm.update(in, now, kDt);

    TEST_ASSERT_EQUAL(FlightState::FAILSAFE_LAND, fsm.state());
    TEST_ASSERT_EQUAL(FaultCause::LOW_BATTERY, fsm.fault());
}

/**
 * A tumbling airframe whose link also dropped must be reported and handled as
 * the tilt fault. Severity, not arrival order, has to decide the response.
 */
void test_most_severe_fault_wins() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs in;
    in.throttle_norm = 0.6f;
    in.signal_lost = true;
    in.roll_deg = 80.0f;
    now += 4;
    fsm.update(in, now, kDt);

    TEST_ASSERT_EQUAL(FaultCause::EXCESSIVE_TILT, fsm.fault());
    TEST_ASSERT_EQUAL(FlightState::FAILSAFE_CUT, fsm.state());
}

/**
 * The latch. A receiver that recovers mid-descent must not hand control back
 * on its own, or the airframe climbs again while someone walks towards it.
 */
void test_failsafe_latches_when_fault_clears() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs lost;
    lost.throttle_norm = 0.6f;
    lost.signal_lost = true;
    now += 4;
    fsm.update(lost, now, kDt);
    TEST_ASSERT_EQUAL(FlightState::FAILSAFE_LAND, fsm.state());

    FlightInputs back;
    back.throttle_norm = 0.6f;
    run(fsm, back, now, 1.0f);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::FAILSAFE_LAND ||
                     fsm.state() == FlightState::FAILSAFE_CUT);
}

/** Clearing a latch must be refused while the throttle is still up. */
void test_failsafe_clear_refused_under_power() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs lost;
    lost.throttle_norm = 0.6f;
    lost.signal_lost = true;
    now += 4;
    fsm.update(lost, now, kDt);

    FlightInputs hot;
    hot.throttle_norm = 0.6f;
    TEST_ASSERT_FALSE(fsm.clearFailsafe(hot));

    FlightInputs safe;
    TEST_ASSERT_TRUE(fsm.clearFailsafe(safe));
    TEST_ASSERT_EQUAL(FlightState::DISARMED, fsm.state());
}

/** Clearing must also be refused while the fault is still present. */
void test_failsafe_clear_refused_while_fault_present() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    armIt(fsm, now);

    FlightInputs lost;
    lost.throttle_norm = 0.6f;
    lost.signal_lost = true;
    now += 4;
    fsm.update(lost, now, kDt);

    FlightInputs still_lost;
    still_lost.signal_lost = true;
    TEST_ASSERT_FALSE(fsm.clearFailsafe(still_lost));
}

/** A fault on the ground blocks arming instead of declaring an emergency. */
void test_fault_on_ground_blocks_arming() {
    FlightStateMachine fsm;
    uint32_t now = 1000;
    FlightInputs in;
    in.yaw_norm = 1.0f;
    in.battery_volts = 9.0f;
    run(fsm, in, now, 5.0f);
    TEST_ASSERT_EQUAL(FlightState::DISARMED, fsm.state());
    TEST_ASSERT_FALSE(fsm.motorsEnabled());
}

// ------------------------------------------------------------ CAN encoding

/** Round tripping must preserve every field inside the quantisation step. */
void test_can_attitude_round_trip() {
    can_telemetry::Attitude a;
    a.roll_deg = -12.34f;
    a.pitch_deg = 45.67f;
    a.yaw_rate_dps = -123.45f;
    a.flight_state = (uint8_t)FlightState::FAILSAFE_LAND;
    a.fault = (uint8_t)FaultCause::SIGNAL_LOST;

    const auto f = can_telemetry::packAttitude(a, 42, 7);
    can_telemetry::Attitude out;
    uint8_t transfer = 0;
    TEST_ASSERT_TRUE(can_telemetry::unpackAttitude(f, out, &transfer));

    TEST_ASSERT_FLOAT_WITHIN(0.01f, a.roll_deg, out.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, a.pitch_deg, out.pitch_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, a.yaw_rate_dps, out.yaw_rate_dps);
    TEST_ASSERT_EQUAL(a.flight_state, out.flight_state);
    TEST_ASSERT_EQUAL(a.fault, out.fault);
    TEST_ASSERT_EQUAL(7, transfer);
}

/** The identifier must carry both the message type and the source node. */
void test_can_id_encodes_message_and_node() {
    const uint32_t id = can_telemetry::makeId(can_telemetry::kMsgAttitude, 42);
    TEST_ASSERT_EQUAL_UINT16(can_telemetry::kMsgAttitude, can_telemetry::messageIdOf(id));
    TEST_ASSERT_EQUAL_UINT8(42, can_telemetry::nodeIdOf(id));
    TEST_ASSERT_TRUE(id < (1u << 29)); // must fit an extended identifier
}

/**
 * Out-of-range values must saturate, never wrap. A wrapped 400 degree roll
 * arriving as a small negative angle is the one encoding bug that could make a
 * ground station show a healthy airframe while it tumbles.
 */
void test_can_saturates_instead_of_wrapping() {
    can_telemetry::Attitude a;
    a.roll_deg = 5000.0f;
    a.pitch_deg = -5000.0f;

    const auto f = can_telemetry::packAttitude(a, 1, 0);
    can_telemetry::Attitude out;
    can_telemetry::unpackAttitude(f, out);

    TEST_ASSERT_TRUE(out.roll_deg > 300.0f);
    TEST_ASSERT_TRUE(out.pitch_deg < -300.0f);
}

/** A frame of the wrong type or length must be rejected, not misread. */
void test_can_rejects_wrong_frame() {
    can_telemetry::Estimator e;
    const auto f = can_telemetry::packEstimator(e, 1, 0);
    can_telemetry::Attitude out;
    TEST_ASSERT_FALSE(can_telemetry::unpackAttitude(f, out));

    auto truncated = can_telemetry::packAttitude({}, 1, 0);
    truncated.length = 4;
    TEST_ASSERT_FALSE(can_telemetry::unpackAttitude(truncated, out));
}

/** Estimator health must survive the round trip too. */
void test_can_estimator_round_trip() {
    can_telemetry::Estimator e;
    e.bias_p_dps = 1.234f;
    e.bias_q_dps = -0.567f;
    e.innovation_mg = 4321;

    const auto f = can_telemetry::packEstimator(e, 9, 200);
    can_telemetry::Estimator out;
    TEST_ASSERT_TRUE(can_telemetry::unpackEstimator(f, out));
    TEST_ASSERT_FLOAT_WITHIN(0.002f, e.bias_p_dps, out.bias_p_dps);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, e.bias_q_dps, out.bias_q_dps);
    TEST_ASSERT_EQUAL_UINT16(e.innovation_mg, out.innovation_mg);
}

/** Dropped frames must be counted, including across the counter wrap. */
void test_can_transfer_monitor_counts_gaps() {
    can_telemetry::TransferMonitor mon;
    TEST_ASSERT_EQUAL_UINT8(0, mon.observe(10));
    TEST_ASSERT_EQUAL_UINT8(0, mon.observe(11));
    TEST_ASSERT_EQUAL_UINT8(3, mon.observe(15)); // 12, 13, 14 missing
    TEST_ASSERT_EQUAL_UINT8(0, mon.observe(16));

    mon.reset();
    TEST_ASSERT_EQUAL_UINT8(0, mon.observe(254));
    TEST_ASSERT_EQUAL_UINT8(0, mon.observe(255));
    TEST_ASSERT_EQUAL_UINT8(0, mon.observe(0)); // wrap is not a gap
    TEST_ASSERT_EQUAL_UINT8(1, mon.observe(2));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_disarmed);
    RUN_TEST(test_arming_requires_full_hold);
    RUN_TEST(test_arming_aborts_when_gesture_released);
    RUN_TEST(test_cannot_arm_with_throttle_up);
    RUN_TEST(test_signal_loss_lands_rather_than_cutting);
    RUN_TEST(test_failsafe_land_ramps_down_monotonically);
    RUN_TEST(test_excessive_tilt_cuts_immediately);
    RUN_TEST(test_estimator_divergence_cuts_immediately);
    RUN_TEST(test_low_battery_lands);
    RUN_TEST(test_most_severe_fault_wins);
    RUN_TEST(test_failsafe_latches_when_fault_clears);
    RUN_TEST(test_failsafe_clear_refused_under_power);
    RUN_TEST(test_failsafe_clear_refused_while_fault_present);
    RUN_TEST(test_fault_on_ground_blocks_arming);

    RUN_TEST(test_can_attitude_round_trip);
    RUN_TEST(test_can_id_encodes_message_and_node);
    RUN_TEST(test_can_saturates_instead_of_wrapping);
    RUN_TEST(test_can_rejects_wrong_frame);
    RUN_TEST(test_can_estimator_round_trip);
    RUN_TEST(test_can_transfer_monitor_counts_gaps);
    return UNITY_END();
}
