/**
 * Host tests for the link layer: MAVLink framing and messages, the telemetry
 * scheduler, loop timing, RC pulse decoding and the HIL conversions.
 *
 * The byte-level expectations here were produced by pymavlink, an independent
 * implementation, not by this code. A codec checked only against itself
 * proves it is consistent, not that anything else can read it.
 * tools/mavlink_crosscheck.py repeats the comparison over every frame the SIL
 * harness writes.
 */

#include "../test_harness.h"
#include <math.h>
#include <string.h>
#include <vector>

#include "mavlink_lite.h"
#include "mavlink_telemetry.h"
#include "loop_timing.h"
#include "rc_pulse.h"
#include "hil_link.h"

using namespace mavlink;

static std::vector<Frame> parseAll(const uint8_t* data, size_t n, Parser& p) {
    std::vector<Frame> out;
    for (size_t i = 0; i < n; ++i) {
        p.push(data[i], [&](const Frame& f) { out.push_back(f); });
    }
    return out;
}

// ------------------------------------------------------------- checksum

/** The CRC-16/MCRF4XX check value: "123456789" gives 0x6F91. */
void test_crc_matches_published_check_value() {
    const char* s = "123456789";
    TEST_ASSERT_EQUAL(0x6F91, crcBlock(reinterpret_cast<const uint8_t*>(s), 9));
}

// ---------------------------------------------------- reference frames

/**
 * HEARTBEAT as pymavlink encodes it:
 *   MAVLink(file, srcSystem=1, srcComponent=1).heartbeat_encode(
 *       type=2, autopilot=0, base_mode=81, custom_mode=0x0102,
 *       system_status=3, mavlink_version=3), seq 0.
 */
void test_heartbeat_matches_pymavlink_bytes() {
    const uint8_t expected[] = {0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
                                0x02, 0x01, 0x00, 0x00, 0x02, 0x00, 0x51, 0x03, 0x03,
                                0x9D, 0x9A};
    Heartbeat hb;
    hb.custom_mode = 0x0102;
    hb.type = 2;
    hb.autopilot = 0;
    hb.base_mode = 81;
    hb.system_status = 3;
    Endpoint ep;
    uint8_t out[frameSize(Heartbeat::kLen)] = {};
    const size_t n = encode(out, ep, hb);
    TEST_ASSERT_EQUAL(sizeof(expected), n);
    TEST_ASSERT_TRUE(memcmp(out, expected, sizeof(expected)) == 0);
}

/**
 * ATTITUDE with roll 0.1, pitch -0.2, rates 0.01, 0.02, -0.03, at
 * time_boot_ms 1234, seq 7, from pymavlink. yaw is zero and sits mid
 * payload, so it is not truncated; the trailing bytes are not zero either.
 */
void test_attitude_matches_pymavlink_bytes() {
    const uint8_t expected[] = {
        0xFD, 0x1C, 0x00, 0x00, 0x07, 0x01, 0x01, 0x1E, 0x00, 0x00, 0xD2, 0x04, 0x00, 0x00,
        0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x0A, 0xD7,
        0x23, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x8F, 0xC2, 0xF5, 0xBC, 0x04, 0xB4};
    Attitude a;
    a.time_boot_ms = 1234;
    a.roll = 0.1f;
    a.pitch = -0.2f;
    a.rollspeed = 0.01f;
    a.pitchspeed = 0.02f;
    a.yawspeed = -0.03f;
    Endpoint ep;
    ep.seq = 7;
    uint8_t out[frameSize(Attitude::kLen)] = {};
    const size_t n = encode(out, ep, a);
    TEST_ASSERT_EQUAL(sizeof(expected), n);
    TEST_ASSERT_TRUE(memcmp(out, expected, sizeof(expected)) == 0);
}

/** v2 senders must drop trailing zero bytes, keeping at least one. */
void test_trailing_zeros_are_truncated() {
    NamedValueFloat nv;
    nv.time_boot_ms = 5;
    nv.value = 0.0f;
    nv.setName(""); // everything after the first byte is zero
    Endpoint ep;
    uint8_t out[frameSize(NamedValueFloat::kLen)] = {};
    const size_t n = encode(out, ep, nv);
    TEST_ASSERT_EQUAL(1, out[1]); // payload length on the wire
    TEST_ASSERT_EQUAL(kHeaderLen + 1 + kChecksumLen, n);

    // And the parser zero extends it back to the full message.
    Parser p;
    auto frames = parseAll(out, n, p);
    TEST_ASSERT_EQUAL(1, frames.size());
    NamedValueFloat back;
    TEST_ASSERT_TRUE(frames[0].decode(back));
    TEST_ASSERT_EQUAL(5, back.time_boot_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, back.value);
}

/** An all zero payload still keeps one byte, as the spec requires. */
void test_all_zero_payload_keeps_one_byte() {
    Heartbeat hb;
    hb.type = 0;
    hb.system_status = 0;
    hb.mavlink_version = 0;
    Endpoint ep;
    uint8_t out[frameSize(Heartbeat::kLen)] = {};
    encode(out, ep, hb);
    TEST_ASSERT_EQUAL(1, out[1]);
}

// ----------------------------------------------------------- round trips

void test_every_message_round_trips() {
    Endpoint ep;
    Parser p;
    uint8_t buf[kMaxFrame] = {};

    HilSensor hs;
    hs.time_usec = 123456789012ull;
    hs.xacc = 0.5f; hs.yacc = -0.25f; hs.zacc = -9.81f;
    hs.xgyro = 0.1f; hs.ygyro = -0.2f; hs.zgyro = 0.3f;
    hs.fields_updated = 0x3F;
    hs.id = 2;
    size_t n = encode(buf, ep, hs);
    auto f = parseAll(buf, n, p);
    TEST_ASSERT_EQUAL(1, f.size());
    HilSensor hs2;
    TEST_ASSERT_TRUE(f[0].decode(hs2));
    TEST_ASSERT_TRUE(hs2.time_usec == hs.time_usec);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, -9.81f, hs2.zacc);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.3f, hs2.zgyro);
    TEST_ASSERT_EQUAL(0x3F, hs2.fields_updated);
    TEST_ASSERT_EQUAL(2, hs2.id);

    HilActuatorControls ac;
    ac.time_usec = 42;
    ac.controls[0] = 0.4f; ac.controls[3] = 0.6f;
    ac.mode = kModeFlagHilEnabled | kModeFlagSafetyArmed;
    n = encode(buf, ep, ac);
    f = parseAll(buf, n, p);
    TEST_ASSERT_EQUAL(1, f.size());
    HilActuatorControls ac2;
    TEST_ASSERT_TRUE(f[0].decode(ac2));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.6f, ac2.controls[3]);
    TEST_ASSERT_EQUAL(ac.mode, ac2.mode);

    RcChannelsOverride rc;
    rc.chan[0] = 1500; rc.chan[1] = 1400; rc.chan[2] = 1100; rc.chan[3] = 1900;
    rc.target_system = 1; rc.target_component = 1;
    n = encode(buf, ep, rc);
    f = parseAll(buf, n, p);
    RcChannelsOverride rc2;
    TEST_ASSERT_TRUE(f.size() == 1 && f[0].decode(rc2));
    TEST_ASSERT_EQUAL(1900, rc2.chan[3]);
    TEST_ASSERT_EQUAL(1, rc2.target_component);

    SysStatus st;
    st.voltage_battery = 12400;
    st.load = 333;
    st.battery_remaining = -1;
    st.errors_count[0] = 9;
    n = encode(buf, ep, st);
    f = parseAll(buf, n, p);
    SysStatus st2;
    TEST_ASSERT_TRUE(f.size() == 1 && f[0].decode(st2));
    TEST_ASSERT_EQUAL(12400, st2.voltage_battery);
    TEST_ASSERT_EQUAL(-1, st2.battery_remaining);
    TEST_ASSERT_EQUAL(9, st2.errors_count[0]);

    TEST_ASSERT_EQUAL(0, p.stats().crc_errors);
    TEST_ASSERT_EQUAL(0, p.stats().seq_gaps);
}

// ---------------------------------------------------------------- parser

/** Text before a frame, as boot messages on the same UART produce. */
void test_parser_skips_leading_garbage() {
    std::vector<uint8_t> stream;
    const char* boot = "Calibrating gyro, keep the airframe still and level.\r\n";
    stream.insert(stream.end(), boot, boot + strlen(boot));
    Endpoint ep;
    uint8_t buf[kMaxFrame] = {};
    const size_t n = encode(buf, ep, Heartbeat());
    stream.insert(stream.end(), buf, buf + n);

    Parser p;
    auto f = parseAll(stream.data(), stream.size(), p);
    TEST_ASSERT_EQUAL(1, f.size());
    TEST_ASSERT_EQUAL(0, p.stats().crc_errors);
}

/**
 * A stray STX that claims a long payload must not swallow the real frame
 * inside its declared length. This is the resynchronisation the parser
 * documents, and the one that is easiest to get wrong.
 */
void test_parser_recovers_frame_inside_false_start() {
    std::vector<uint8_t> stream = {0xFD, 200, 0, 0, 0, 1, 1, 0, 0, 0};
    Endpoint ep;
    uint8_t buf[kMaxFrame] = {};
    for (int i = 0; i < 3; ++i) {
        Attitude a;
        a.time_boot_ms = 100 + i;
        a.roll = 0.01f * i + 0.5f;
        const size_t n = encode(buf, ep, a);
        stream.insert(stream.end(), buf, buf + n);
    }
    // Pad so the false start's declared length is satisfied and it fails CRC.
    stream.resize(stream.size() + 200, 0x55);

    Parser p;
    auto f = parseAll(stream.data(), stream.size(), p);
    TEST_ASSERT_EQUAL(3, f.size());
    TEST_ASSERT_EQUAL(1, p.stats().crc_errors);
    Attitude last;
    TEST_ASSERT_TRUE(f.back().decode(last));
    TEST_ASSERT_EQUAL(102, last.time_boot_ms);
}

void test_parser_rejects_corrupted_byte() {
    Endpoint ep;
    uint8_t buf[kMaxFrame] = {};
    Attitude a;
    a.roll = 0.3f;
    const size_t n = encode(buf, ep, a);
    buf[14] ^= 0x01;
    Parser p;
    auto f = parseAll(buf, n, p);
    TEST_ASSERT_EQUAL(0, f.size());
    TEST_ASSERT_EQUAL(1, p.stats().crc_errors);
}

void test_parser_counts_sequence_gaps() {
    Endpoint ep;
    uint8_t buf[kMaxFrame] = {};
    Parser p;
    std::vector<uint8_t> stream;
    for (int i = 0; i < 10; ++i) {
        const size_t n = encode(buf, ep, Heartbeat());
        if (i == 4 || i == 5) continue; // lose two
        stream.insert(stream.end(), buf, buf + n);
    }
    auto f = parseAll(stream.data(), stream.size(), p);
    TEST_ASSERT_EQUAL(8, f.size());
    TEST_ASSERT_EQUAL(2, p.stats().seq_gaps);
}

/** Sequence wraps at 256 and that is not a gap. */
void test_parser_sequence_wrap_is_not_a_gap() {
    Endpoint ep;
    ep.seq = 250;
    uint8_t buf[kMaxFrame] = {};
    Parser p;
    for (int i = 0; i < 12; ++i) {
        const size_t n = encode(buf, ep, Heartbeat());
        parseAll(buf, n, p);
    }
    TEST_ASSERT_EQUAL(12, p.stats().frames_ok);
    TEST_ASSERT_EQUAL(0, p.stats().seq_gaps);
}

/** Unknown ids are consumed whole and a following frame still parses. */
void test_parser_consumes_unknown_messages() {
    // A COMMAND_LONG sized frame with id 76, which we do not implement.
    std::vector<uint8_t> stream = {0xFD, 33, 0, 0, 0, 255, 190, 76, 0, 0};
    for (int i = 0; i < 33; ++i) stream.push_back(0xFD); // payload full of STX
    stream.push_back(0x12);
    stream.push_back(0x34);
    Endpoint ep;
    uint8_t buf[kMaxFrame] = {};
    const size_t n = encode(buf, ep, Heartbeat());
    stream.insert(stream.end(), buf, buf + n);

    Parser p;
    auto f = parseAll(stream.data(), stream.size(), p);
    TEST_ASSERT_EQUAL(1, f.size());
    TEST_ASSERT_EQUAL(1, p.stats().unknown_ids);
    TEST_ASSERT_EQUAL(0, p.stats().crc_errors);
}

void test_parser_drops_signed_frames() {
    Endpoint ep;
    uint8_t buf[kMaxFrame + 13] = {};
    uint8_t hbuf[kMaxFrame] = {};
    size_t n = encode(hbuf, ep, Heartbeat());
    memcpy(buf, hbuf, n);
    buf[2] |= kIncompatSigned;
    for (int i = 0; i < 13; ++i) buf[n + i] = 0xFD; // signature bytes
    n += 13;
    const size_t m = encode(hbuf, ep, Heartbeat());
    memcpy(buf + n, hbuf, m);

    Parser p;
    auto f = parseAll(buf, n + m, p);
    TEST_ASSERT_EQUAL(1, f.size());
    TEST_ASSERT_EQUAL(1, p.stats().signed_dropped);
}

// ------------------------------------------------------------- telemetry

void test_heartbeat_mapping_follows_flight_state() {
    using telemetry::heartbeatFor;
    auto hb = heartbeatFor(FlightState::DISARMED, FaultCause::NONE, false);
    TEST_ASSERT_EQUAL(kMavStateStandby, hb.system_status);
    TEST_ASSERT_FALSE(hb.base_mode & kModeFlagSafetyArmed);

    hb = heartbeatFor(FlightState::ARMED, FaultCause::NONE, false);
    TEST_ASSERT_EQUAL(kMavStateActive, hb.system_status);
    TEST_ASSERT_TRUE(hb.base_mode & kModeFlagSafetyArmed);

    hb = heartbeatFor(FlightState::FAILSAFE_LAND, FaultCause::SIGNAL_LOST, false);
    TEST_ASSERT_EQUAL(kMavStateCritical, hb.system_status);
    TEST_ASSERT_TRUE(hb.base_mode & kModeFlagSafetyArmed);
    TEST_ASSERT_EQUAL((uint32_t)FlightState::FAILSAFE_LAND | ((uint32_t)FaultCause::SIGNAL_LOST << 8),
                      hb.custom_mode);

    hb = heartbeatFor(FlightState::FAILSAFE_CUT, FaultCause::ESTIMATOR_DIVERGED, true);
    TEST_ASSERT_EQUAL(kMavStateEmergency, hb.system_status);
    TEST_ASSERT_FALSE(hb.base_mode & kModeFlagSafetyArmed);
    TEST_ASSERT_TRUE(hb.base_mode & kModeFlagHilEnabled);
}

/** Ten seconds at the default rates: counts per message and the byte budget. */
void test_scheduler_rates_and_bandwidth() {
    telemetry::Scheduler s;
    telemetry::Snapshot snap;
    snap.battery_v = 12.3f;
    int counts[256] = {};
    Parser p;
    for (uint32_t t = 0; t < 10000; t += 4) { // the 250 Hz loop
        snap.time_ms = t;
        s.poll(snap, [&](const uint8_t* f, size_t n) {
            for (size_t i = 0; i < n; ++i) {
                p.push(f[i], [&](const Frame& fr) { ++counts[fr.msgid & 0xFF]; });
            }
            return true;
        });
    }
    TEST_ASSERT_EQUAL(10, counts[kHeartbeat]);
    TEST_ASSERT_EQUAL(20, counts[kSysStatus]);
    TEST_ASSERT_EQUAL(250, counts[kAttitude]);
    TEST_ASSERT_EQUAL(100, counts[kNamedValueFloat]);
    TEST_ASSERT_EQUAL(0, p.stats().crc_errors);
    TEST_ASSERT_EQUAL(0, p.stats().seq_gaps);

    // 115200 baud, 8N1, is 11520 bytes per second. Keep telemetry under
    // half of it so the link has room for anything a GCS sends back.
    const float bytes_per_s = s.bytesSent() / 10.0f;
    TEST_ASSERT_TRUE(bytes_per_s < 11520.0f * 0.5f);
}

/** A full link drops frames without blocking and without faking a gap. */
void test_scheduler_drops_without_sequence_gap() {
    telemetry::Scheduler s;
    telemetry::Snapshot snap;
    Parser p;
    int accepted = 0;
    for (uint32_t t = 0; t < 2000; t += 4) {
        snap.time_ms = t;
        s.poll(snap, [&](const uint8_t* f, size_t n) {
            // Refuse every other frame offered.
            if (((accepted + s.framesDropped()) & 1) == 1) return false;
            ++accepted;
            for (size_t i = 0; i < n; ++i) p.push(f[i], [](const Frame&) {});
            return true;
        });
    }
    TEST_ASSERT_TRUE(s.framesDropped() > 0);
    TEST_ASSERT_EQUAL(0, p.stats().seq_gaps);
}

void test_sys_status_reports_health_and_load() {
    telemetry::Snapshot snap;
    snap.battery_v = 11.1f;
    snap.timing.utilisation = 0.42f;
    snap.total_overruns = 3;
    snap.receiver_ok = false;
    const SysStatus st = telemetry::Scheduler::sysStatus(snap);
    TEST_ASSERT_EQUAL(11100, st.voltage_battery);
    TEST_ASSERT_EQUAL(420, st.load);
    TEST_ASSERT_EQUAL(3, st.errors_count[0]);
    TEST_ASSERT_FALSE(st.sensors_health & kSensorRcReceiver);
    TEST_ASSERT_TRUE(st.sensors_health & kSensorAhrs);
}

void test_named_values_fit_and_cycle() {
    telemetry::Snapshot snap;
    snap.timing.jitter_max_us = 37;
    for (uint8_t i = 0; i < telemetry::Scheduler::kNamedValues; ++i) {
        const NamedValueFloat nv = telemetry::Scheduler::namedValue(snap, i);
        TEST_ASSERT_TRUE(nv.name[0] != '\0');
    }
    const NamedValueFloat jit = telemetry::Scheduler::namedValue(snap, 0);
    TEST_ASSERT_TRUE(strncmp(jit.name, "loop_jit", 10) == 0);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 37.0f, jit.value);
}

// ----------------------------------------------------------- loop timing

void test_timing_perfect_loop_has_no_jitter() {
    LoopTiming t(4000);
    uint32_t now = 1000;
    for (int i = 0; i < 250; ++i) {
        t.markStart(now);
        t.markEnd(now + 800);
        now += 4000;
    }
    const auto r = t.report();
    TEST_ASSERT_EQUAL(249, r.cycles);
    TEST_ASSERT_EQUAL(0, r.jitter_max_us);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 4000.0, r.period_mean_us);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0, r.period_stddev_us);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.2, r.utilisation);
    TEST_ASSERT_EQUAL(800, r.exec_max_us);
    TEST_ASSERT_EQUAL(0, r.overruns);
}

void test_timing_measures_jitter_and_overruns() {
    LoopTiming t(4000);
    uint32_t now = 0;
    const int32_t pattern[] = {0, 30, -30, 0, 2500 /* missed deadline */, -2500};
    for (int i = 0; i < 6; ++i) {
        now += 4000 + pattern[i];
        t.markStart(now);
    }
    const auto r = t.report();
    TEST_ASSERT_EQUAL(5, r.cycles);
    TEST_ASSERT_EQUAL(2500, r.jitter_max_us);
    TEST_ASSERT_EQUAL(6500, r.period_max_us);
    TEST_ASSERT_EQUAL(1500, r.period_min_us);
    TEST_ASSERT_EQUAL(1, r.overruns);
    TEST_ASSERT_EQUAL(1, t.totalOverruns());
}

/** micros() wraps every 71.6 minutes; a period across it is still 4000. */
void test_timing_survives_micros_rollover() {
    LoopTiming t(4000);
    uint32_t now = 0xFFFFFFFFu - 6000u;
    for (int i = 0; i < 5; ++i) {
        t.markStart(now);
        now += 4000; // wraps on the second step
    }
    const auto r = t.report();
    TEST_ASSERT_EQUAL(4000, r.period_max_us);
    TEST_ASSERT_EQUAL(4000, r.period_min_us);
    TEST_ASSERT_EQUAL(0, r.overruns);
}

/**
 * What the old receiver did to the loop: four pulseIn() calls cost several
 * milliseconds each. The monitor has to call that what it is.
 */
void test_timing_flags_a_blocking_receiver() {
    LoopTiming t(4000);
    uint32_t now = 0;
    for (int i = 0; i < 50; ++i) {
        t.markStart(now);
        t.markEnd(now + 18000); // pulseIn x4 on a 50 Hz receiver
        now += 18000;
    }
    const auto r = t.report();
    TEST_ASSERT_EQUAL(49, r.overruns);
    TEST_ASSERT_TRUE(r.utilisation > 4.0f);
}

void test_timing_window_reset_keeps_continuity() {
    LoopTiming t(4000);
    t.markStart(0);
    t.markStart(4000);
    t.resetWindow();
    t.markStart(8100);
    const auto r = t.report();
    TEST_ASSERT_EQUAL(1, r.cycles);
    TEST_ASSERT_EQUAL(4100, r.period_max_us);
}

// ------------------------------------------------------------- RC pulses

void test_rc_pulse_measures_width() {
    RcPulseChannel ch;
    ch.edge(true, 1000);
    ch.edge(false, 2500);
    uint16_t w = 0;
    TEST_ASSERT_TRUE(ch.read(3000, 100000, w));
    TEST_ASSERT_EQUAL(1500, w);
}

void test_rc_pulse_rejects_glitches_and_stale() {
    RcPulseChannel ch;
    ch.edge(true, 0);
    ch.edge(false, 300); // 300 us glitch
    uint16_t w = 0;
    TEST_ASSERT_FALSE(ch.read(400, 100000, w));
    TEST_ASSERT_EQUAL(1, ch.rejected());

    ch.edge(true, 1000);
    ch.edge(false, 2200);
    TEST_ASSERT_TRUE(ch.read(5000, 100000, w));
    TEST_ASSERT_FALSE(ch.read(2200 + 100001, 100000, w));
}

void test_rc_pulse_ignores_falling_edge_first() {
    RcPulseChannel ch;
    ch.edge(false, 500); // boot mid pulse
    uint16_t w = 0;
    TEST_ASSERT_FALSE(ch.read(600, 100000, w));
    TEST_ASSERT_EQUAL(0, ch.rejected());
}

void test_rc_pulse_width_across_rollover() {
    RcPulseChannel ch;
    ch.edge(true, 0xFFFFFFFFu - 499u);
    ch.edge(false, 1000); // 1500 us later, after the wrap
    uint16_t w = 0;
    TEST_ASSERT_TRUE(ch.read(1100, 100000, w));
    TEST_ASSERT_EQUAL(1500, w);
}

/** Same signal-loss policy as the pulseIn() receiver: hold 500 ms, then cut. */
void test_rc_decoder_signal_loss_policy() {
    RcPulseChannel ch[4];
    RcDecoder dec;
    const uint16_t widths[4] = {1750, 1500, 1300, 1000};
    uint32_t t = 0;
    for (int i = 0; i < 4; ++i) {
        ch[i].edge(true, t);
        ch[i].edge(false, t + widths[i]);
    }
    t = 3000;
    dec.update(ch, t);
    TEST_ASSERT_FALSE(dec.signalLost());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5, dec.roll());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.3, dec.throttle());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -1.0, dec.yaw());

    // Receiver goes quiet. Sticks hold until the 500 ms window expires.
    dec.update(ch, 300000);
    TEST_ASSERT_FALSE(dec.signalLost());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5, dec.roll());
    dec.update(ch, 504000);
    TEST_ASSERT_TRUE(dec.signalLost());
    TEST_ASSERT_FLOAT_WITHIN(0.0, 0.0, dec.throttle());
    TEST_ASSERT_FLOAT_WITHIN(0.0, 0.0, dec.roll());
}

void test_rc_decoder_starts_lost() {
    RcPulseChannel ch[4];
    RcDecoder dec;
    dec.update(ch, 0);
    TEST_ASSERT_TRUE(dec.signalLost());
}

// ------------------------------------------------------------------- HIL

/** At rest, level: FRD reads -g on z, the firmware wants +1 g. */
void test_hil_conversion_at_rest() {
    HilSensor m;
    m.zacc = -hil::kStandardGravity;
    const ImuSample s = hil::fromHilSensor(m);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, s.az);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, s.ax);
}

/**
 * Nose up by 10 degrees. Physically an FRD accelerometer then reads
 * +g sin(10) on x. The SIL model, which the estimator was built against,
 * reports -sin(pitch) on x for nose up. The conversion has to reconcile the
 * two, and the EKF has to come out reading nose up.
 */
void test_hil_pitch_sign_reaches_the_estimator() {
    const float th = 10.0f * 0.01745329252f;
    HilSensor m;
    m.xacc = hil::kStandardGravity * sinf(th);
    m.zacc = -hil::kStandardGravity * cosf(th);
    const ImuSample s = hil::fromHilSensor(m);
    AttitudeEKF ekf;
    ekf.initializeFromAccel(s.ax, s.ay, s.az);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 10.0, ekf.getPitchDeg());
}

/** Right side down by 10 degrees: FRD y reads -g sin(10). */
void test_hil_roll_sign_reaches_the_estimator() {
    const float ph = 10.0f * 0.01745329252f;
    HilSensor m;
    m.yacc = -hil::kStandardGravity * sinf(ph);
    m.zacc = -hil::kStandardGravity * cosf(ph);
    const ImuSample s = hil::fromHilSensor(m);
    AttitudeEKF ekf;
    ekf.initializeFromAccel(s.ax, s.ay, s.az);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 10.0, ekf.getRollDeg());
}

void test_hil_rc_mapping_and_timeout() {
    hil::Link link;
    Endpoint sim;
    uint8_t buf[kMaxFrame] = {};

    HilSensor hs;
    hs.time_usec = 1000000;
    hs.zacc = -9.80665f;
    int samples = 0;
    auto on = [&](const ImuSample&, uint64_t) { ++samples; };

    RcChannelsOverride rc;
    rc.chan[0] = 2000; rc.chan[1] = 1000; rc.chan[2] = 1500; rc.chan[3] = 1500;
    size_t n = encode(buf, sim, hs);
    for (size_t i = 0; i < n; ++i) link.push(buf[i], on);
    n = encode(buf, sim, rc);
    for (size_t i = 0; i < n; ++i) link.push(buf[i], on);

    TEST_ASSERT_EQUAL(1, samples);
    PilotInput in = link.pilot(1000000);
    TEST_ASSERT_FALSE(in.signal_lost);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, in.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -1.0, in.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5, in.throttle);

    in = link.pilot(1000000 + hil::kRcTimeoutUs + 1);
    TEST_ASSERT_TRUE(in.signal_lost);
    TEST_ASSERT_FLOAT_WITHIN(0.0, 0.0, in.throttle);
}

void test_hil_starts_with_signal_lost() {
    hil::Link link;
    TEST_ASSERT_TRUE(link.pilot(0).signal_lost);
}

void test_hil_actuators_scale_and_flag() {
    const float pwm[4] = {1000.0f, 1500.0f, 2000.0f, 2500.0f};
    const HilActuatorControls m = hil::toActuators(pwm, 7, true);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, m.controls[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5, m.controls[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, m.controls[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, m.controls[3]); // clamped
    TEST_ASSERT_TRUE(m.mode & kModeFlagSafetyArmed);
    TEST_ASSERT_TRUE(m.mode & kModeFlagHilEnabled);
    float back[4];
    hil::pwmFromActuators(m, back);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1500.0, back[1]);
}

int main(int, char**) {
    UNITY_BEGIN();

    RUN_TEST(test_crc_matches_published_check_value);
    RUN_TEST(test_heartbeat_matches_pymavlink_bytes);
    RUN_TEST(test_attitude_matches_pymavlink_bytes);
    RUN_TEST(test_trailing_zeros_are_truncated);
    RUN_TEST(test_all_zero_payload_keeps_one_byte);
    RUN_TEST(test_every_message_round_trips);

    RUN_TEST(test_parser_skips_leading_garbage);
    RUN_TEST(test_parser_recovers_frame_inside_false_start);
    RUN_TEST(test_parser_rejects_corrupted_byte);
    RUN_TEST(test_parser_counts_sequence_gaps);
    RUN_TEST(test_parser_sequence_wrap_is_not_a_gap);
    RUN_TEST(test_parser_consumes_unknown_messages);
    RUN_TEST(test_parser_drops_signed_frames);

    RUN_TEST(test_heartbeat_mapping_follows_flight_state);
    RUN_TEST(test_scheduler_rates_and_bandwidth);
    RUN_TEST(test_scheduler_drops_without_sequence_gap);
    RUN_TEST(test_sys_status_reports_health_and_load);
    RUN_TEST(test_named_values_fit_and_cycle);

    RUN_TEST(test_timing_perfect_loop_has_no_jitter);
    RUN_TEST(test_timing_measures_jitter_and_overruns);
    RUN_TEST(test_timing_survives_micros_rollover);
    RUN_TEST(test_timing_flags_a_blocking_receiver);
    RUN_TEST(test_timing_window_reset_keeps_continuity);

    RUN_TEST(test_rc_pulse_measures_width);
    RUN_TEST(test_rc_pulse_rejects_glitches_and_stale);
    RUN_TEST(test_rc_pulse_ignores_falling_edge_first);
    RUN_TEST(test_rc_pulse_width_across_rollover);
    RUN_TEST(test_rc_decoder_signal_loss_policy);
    RUN_TEST(test_rc_decoder_starts_lost);

    RUN_TEST(test_hil_conversion_at_rest);
    RUN_TEST(test_hil_pitch_sign_reaches_the_estimator);
    RUN_TEST(test_hil_roll_sign_reaches_the_estimator);
    RUN_TEST(test_hil_rc_mapping_and_timeout);
    RUN_TEST(test_hil_starts_with_signal_lost);
    RUN_TEST(test_hil_actuators_scale_and_flag);

    return UNITY_END();
}
