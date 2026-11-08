/**
 * Known-answer tests for the MAVLink v2 encoder.
 *
 * The expected frames below were produced by pymavlink, which is generated
 * from MAVLink's official message definitions, with the same field values.
 * Matching them byte for byte checks the CRC seed, the wire field order, the
 * sequence counter and the trailing-zero truncation in one go, with nothing
 * installed. tools/verify_mavlink.py checks whole SIL flights the same way.
 */
#include <stdint.h>

#include "../test_harness.h"
#include "mavlink_telemetry.h"

static bool sameBytes(const uint8_t* a, const uint8_t* b, int n) {
    for (int i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            std::printf("    byte %d: got 0x%02X, expected 0x%02X\n", i, a[i], b[i]);
            return false;
        }
    }
    return true;
}

/** HEARTBEAT for an armed quadrotor in FlightState::ARMED, sequence 0. */
void test_heartbeat_matches_pymavlink() {
    static const uint8_t expected[] = {
        0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02,
        0x00, 0x00, 0x00, 0x02, 0x00, 0xD1, 0x04, 0x03, 0x0B, 0x76};
    mavlink::Encoder enc(1, 1);
    mavlink::Snapshot s;
    s.armed = true;
    s.flight_state = 2;
    uint8_t buf[mavlink::kMaxFrame];
    const uint8_t n = enc.heartbeat(s, buf);
    TEST_ASSERT_EQUAL((int)sizeof(expected), (int)n);
    TEST_ASSERT_TRUE(sameBytes(buf, expected, n));
}

/** ATTITUDE at roll 10, pitch -5 degrees, sequence 1: units and field order. */
void test_attitude_matches_pymavlink() {
    static const uint8_t expected[] = {
        0xFD, 0x0C, 0x00, 0x00, 0x01, 0x01, 0x01, 0x1E, 0x00, 0x00, 0xD2, 0x04,
        0x00, 0x00, 0xC2, 0xB8, 0x32, 0x3E, 0xC2, 0xB8, 0xB2, 0xBD, 0x52, 0x29};
    mavlink::Encoder enc(1, 1);
    mavlink::Snapshot s;
    uint8_t buf[mavlink::kMaxFrame];
    enc.heartbeat(s, buf); // advance the sequence to 1

    s.time_boot_ms = 1234;
    s.roll_deg = 10.0f;
    s.pitch_deg = -5.0f;
    const uint8_t n = enc.attitude(s, buf);
    TEST_ASSERT_EQUAL((int)sizeof(expected), (int)n);
    TEST_ASSERT_TRUE(sameBytes(buf, expected, n));
}

/** An all-zero payload truncates to one byte, never to zero. */
void test_all_zero_payload_keeps_one_byte() {
    static const uint8_t expected[] = {
        0xFD, 0x01, 0x00, 0x00, 0x02, 0x01, 0x01, 0x1E, 0x00, 0x00, 0x00, 0x6E, 0xB5};
    mavlink::Encoder enc(1, 1);
    mavlink::Snapshot s;
    uint8_t buf[mavlink::kMaxFrame];
    enc.heartbeat(s, buf);
    enc.heartbeat(s, buf); // sequence 2

    const uint8_t n = enc.attitude(mavlink::Snapshot{}, buf);
    TEST_ASSERT_EQUAL((int)sizeof(expected), (int)n);
    TEST_ASSERT_TRUE(sameBytes(buf, expected, n));
}

/** The sequence wraps at 255, so a ground station can count drops forever. */
void test_sequence_wraps() {
    mavlink::Encoder enc(1, 1);
    mavlink::Snapshot s;
    uint8_t buf[mavlink::kMaxFrame];
    for (int i = 0; i < 256; ++i) enc.heartbeat(s, buf);
    TEST_ASSERT_EQUAL(0, (int)enc.sequence());
    enc.heartbeat(s, buf);
    TEST_ASSERT_EQUAL(0, (int)buf[4]);
}

/** The heartbeat must tell a ground station the airframe is in trouble. */
void test_failsafe_raises_system_status() {
    mavlink::Encoder enc(1, 1);
    mavlink::Snapshot s;
    uint8_t buf[mavlink::kMaxFrame];
    const int status_byte = mavlink::kHeaderLen + 7;

    s.armed = true;
    s.severity = 1;
    enc.heartbeat(s, buf);
    TEST_ASSERT_EQUAL((int)mavlink::kStateCritical, (int)buf[status_byte]);

    s.armed = false;
    s.severity = 2;
    enc.heartbeat(s, buf);
    TEST_ASSERT_EQUAL((int)mavlink::kStateEmergency, (int)buf[status_byte]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_heartbeat_matches_pymavlink);
    RUN_TEST(test_attitude_matches_pymavlink);
    RUN_TEST(test_all_zero_payload_keeps_one_byte);
    RUN_TEST(test_sequence_wraps);
    RUN_TEST(test_failsafe_raises_system_status);
    return UNITY_END();
}
