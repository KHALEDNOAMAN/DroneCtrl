#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

/**
 * MAVLink v2 telemetry, encode only, with no library and no Arduino dependency.
 *
 * MAVLink is what ground stations speak: QGroundControl, Mission Planner and
 * every PX4 or ArduPilot vehicle. Emitting it means DroneCtrl's attitude,
 * state and battery show up in the same tools a real autopilot uses, and the
 * SIL harness can write a .tlog file that QGroundControl replays as if it were
 * a recorded flight.
 *
 * Scope, stated plainly: four outgoing messages, nothing incoming, no
 * parameters, no missions, no signing. Enough to be watched by a ground
 * station, not enough to be commanded by one.
 *
 * Why hand-rolled rather than the generated C headers: the generated library
 * is tens of thousands of lines and pulls in every dialect, which is a lot to
 * add to an ATmega328 build for four messages. The framing is small, and the
 * one thing easy to get wrong, the per-message CRC_EXTRA seed and the field
 * order, is checked in CI by decoding these exact bytes with pymavlink
 * (tools/verify_mavlink.py), which is generated from the official XML.
 *
 * Frame layout (v2)
 * -----------------
 *   0xFD | len | incompat | compat | seq | sysid | compid | msgid x3 | payload | crc x2
 *
 * The checksum is CRC-16/MCRF4XX over everything after 0xFD, then over the
 * message's CRC_EXTRA byte. Payload fields go on the wire largest type first,
 * not in the order the XML lists them, and trailing zero bytes are truncated.
 */

namespace mavlink {

constexpr uint8_t kStx = 0xFD;
constexpr uint8_t kHeaderLen = 10;
constexpr uint8_t kChecksumLen = 2;
/** Largest payload of the messages below (SYS_STATUS). */
constexpr uint8_t kMaxPayload = 31;
constexpr uint8_t kMaxFrame = kHeaderLen + kMaxPayload + kChecksumLen;

// Message ids and CRC_EXTRA seeds, from common.xml.
constexpr uint32_t kMsgHeartbeat = 0;
constexpr uint32_t kMsgSysStatus = 1;
constexpr uint32_t kMsgAttitude = 30;
constexpr uint32_t kMsgVfrHud = 74;
constexpr uint8_t kCrcHeartbeat = 50;
constexpr uint8_t kCrcSysStatus = 124;
constexpr uint8_t kCrcAttitude = 39;
constexpr uint8_t kCrcVfrHud = 20;

// Enum values used in HEARTBEAT, from common.xml and minimal.xml.
constexpr uint8_t kTypeQuadrotor = 2;
constexpr uint8_t kAutopilotGeneric = 0;
constexpr uint8_t kModeFlagCustom = 1;
constexpr uint8_t kModeFlagStabilize = 16;
constexpr uint8_t kModeFlagManualInput = 64;
constexpr uint8_t kModeFlagArmed = 128;
constexpr uint8_t kStateStandby = 3;
constexpr uint8_t kStateActive = 4;
constexpr uint8_t kStateCritical = 5;
constexpr uint8_t kStateEmergency = 6;

/** CRC-16/MCRF4XX, the X.25 variant MAVLink uses. */
inline void crcAccumulate(uint8_t b, uint16_t& crc) {
    uint8_t tmp = b ^ static_cast<uint8_t>(crc & 0xFF);
    tmp ^= static_cast<uint8_t>(tmp << 4);
    crc = static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
}

/** Little-endian field writers. MAVLink is little-endian on the wire. */
inline uint8_t* putU8(uint8_t* p, uint8_t v) { *p = v; return p + 1; }
inline uint8_t* putU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    return p + 2;
}
inline uint8_t* putU32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
    return p + 4;
}
inline uint8_t* putF32(uint8_t* p, float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    return putU32(p, bits);
}

/** Flight controller state, in the units it already uses. */
struct Snapshot {
    uint32_t time_boot_ms = 0;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float roll_rate_dps = 0.0f;
    float pitch_rate_dps = 0.0f;
    float yaw_rate_dps = 0.0f;
    float altitude_m = 0.0f;
    float climb_mps = 0.0f;
    float throttle_norm = 0.0f; ///< 0..1
    float battery_volts = 0.0f;
    bool armed = false;
    /** FlightState as an integer, sent as the custom mode. */
    uint8_t flight_state = 0;
    /** 0 normal, 1 controlled descent, 2 motors cut. */
    uint8_t severity = 0;
};

/**
 * Builds frames into a caller-owned buffer and keeps the sequence counter.
 * One instance per link, so each link's sequence numbers are contiguous and a
 * ground station can count dropped packets.
 */
class Encoder {
public:
    explicit Encoder(uint8_t system_id = 1, uint8_t component_id = 1)
        : sysid_(system_id), compid_(component_id) {}

    uint8_t sequence() const { return seq_; }

    /** HEARTBEAT, sent at 1 Hz. Ground stations drop a vehicle without it. */
    uint8_t heartbeat(const Snapshot& s, uint8_t* out) {
        uint8_t pl[9];
        uint8_t* p = pl;
        p = putU32(p, s.flight_state);                       // custom_mode
        p = putU8(p, kTypeQuadrotor);                        // type
        p = putU8(p, kAutopilotGeneric);                     // autopilot
        uint8_t base = kModeFlagCustom | kModeFlagStabilize | kModeFlagManualInput;
        if (s.armed) base |= kModeFlagArmed;
        p = putU8(p, base);                                  // base_mode
        p = putU8(p, systemStatus(s));                       // system_status
        putU8(p, 3);                                         // mavlink_version
        return frame(kMsgHeartbeat, kCrcHeartbeat, pl, sizeof(pl), out);
    }

    /** ATTITUDE in radians and radians per second. */
    uint8_t attitude(const Snapshot& s, uint8_t* out) {
        constexpr float k = 0.017453292f;
        uint8_t pl[28];
        uint8_t* p = pl;
        p = putU32(p, s.time_boot_ms);
        p = putF32(p, s.roll_deg * k);
        p = putF32(p, s.pitch_deg * k);
        p = putF32(p, s.yaw_deg * k);
        p = putF32(p, s.roll_rate_dps * k);
        p = putF32(p, s.pitch_rate_dps * k);
        putF32(p, s.yaw_rate_dps * k);
        return frame(kMsgAttitude, kCrcAttitude, pl, sizeof(pl), out);
    }

    /** SYS_STATUS, for the battery voltage. Sensor bitmasks are left zero. */
    uint8_t sysStatus(const Snapshot& s, uint8_t* out) {
        uint8_t pl[31];
        uint8_t* p = pl;
        p = putU32(p, 0);                                    // sensors present
        p = putU32(p, 0);                                    // sensors enabled
        p = putU32(p, 0);                                    // sensors health
        p = putU16(p, 0);                                    // load
        float mv = s.battery_volts * 1000.0f;
        if (mv < 0.0f) mv = 0.0f;
        if (mv > 65534.0f) mv = 65534.0f;
        p = putU16(p, static_cast<uint16_t>(mv + 0.5f));     // voltage_battery, mV
        p = putU16(p, static_cast<uint16_t>(-1));            // current_battery: unknown
        p = putU16(p, 0);                                    // drop_rate_comm
        p = putU16(p, 0);                                    // errors_comm
        for (int i = 0; i < 4; ++i) p = putU16(p, 0);        // errors_count1..4
        putU8(p, static_cast<uint8_t>(-1));                  // battery_remaining: unknown
        return frame(kMsgSysStatus, kCrcSysStatus, pl, sizeof(pl), out);
    }

    /** VFR_HUD, which drives the altitude, climb and throttle instruments. */
    uint8_t vfrHud(const Snapshot& s, uint8_t* out) {
        uint8_t pl[20];
        uint8_t* p = pl;
        p = putF32(p, 0.0f);                                 // airspeed
        p = putF32(p, 0.0f);                                 // groundspeed
        p = putF32(p, s.altitude_m);                         // alt
        p = putF32(p, s.climb_mps);                          // climb
        float hdg = fmodf(s.yaw_deg, 360.0f);
        if (hdg < 0.0f) hdg += 360.0f;
        p = putU16(p, static_cast<uint16_t>(static_cast<int>(hdg + 0.5f) % 360)); // heading
        float thr = s.throttle_norm * 100.0f;
        if (thr < 0.0f) thr = 0.0f;
        if (thr > 100.0f) thr = 100.0f;
        putU16(p, static_cast<uint16_t>(thr + 0.5f));         // throttle, percent
        return frame(kMsgVfrHud, kCrcVfrHud, pl, sizeof(pl), out);
    }

private:
    uint8_t sysid_;
    uint8_t compid_;
    uint8_t seq_ = 0;

    static uint8_t systemStatus(const Snapshot& s) {
        if (s.severity == 2) return kStateEmergency;
        if (s.severity == 1) return kStateCritical;
        return s.armed ? kStateActive : kStateStandby;
    }

    /** Wrap a payload. Returns the number of bytes written to out. */
    uint8_t frame(uint32_t msgid, uint8_t crc_extra, const uint8_t* payload,
                  uint8_t len, uint8_t* out) {
        // v2 truncates trailing zero bytes, but always keeps at least one.
        while (len > 1 && payload[len - 1] == 0) --len;

        out[0] = kStx;
        out[1] = len;
        out[2] = 0; // incompat_flags: unsigned
        out[3] = 0; // compat_flags
        out[4] = seq_++;
        out[5] = sysid_;
        out[6] = compid_;
        out[7] = static_cast<uint8_t>(msgid);
        out[8] = static_cast<uint8_t>(msgid >> 8);
        out[9] = static_cast<uint8_t>(msgid >> 16);
        memcpy(out + kHeaderLen, payload, len);

        uint16_t crc = 0xFFFF;
        for (uint8_t i = 1; i < kHeaderLen + len; ++i) crcAccumulate(out[i], crc);
        crcAccumulate(crc_extra, crc);
        out[kHeaderLen + len] = static_cast<uint8_t>(crc);
        out[kHeaderLen + len + 1] = static_cast<uint8_t>(crc >> 8);
        return static_cast<uint8_t>(kHeaderLen + len + kChecksumLen);
    }
};

} // namespace mavlink
