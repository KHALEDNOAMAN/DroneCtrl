#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/**
 * MAVLink v2 framing and the handful of messages DroneCtrl speaks.
 *
 * Scope, stated plainly
 * ---------------------
 * This is not a general MAVLink library. It implements the v2 wire format
 * (framing, the X.25 checksum with CRC_EXTRA, trailing zero truncation, and a
 * resynchronising parser) and seven messages from the common dialect:
 *
 *   HEARTBEAT              0   out   who we are and whether we are armed
 *   SYS_STATUS             1   out   battery, loop load, link errors
 *   ATTITUDE              30   out   estimated roll, pitch and body rates
 *   RC_CHANNELS_OVERRIDE  70   in    sticks from a ground station (HIL only)
 *   HIL_ACTUATOR_CONTROLS 93   out   motor commands to a simulator (HIL only)
 *   HIL_SENSOR           107   in    IMU samples from a simulator (HIL only)
 *   NAMED_VALUE_FLOAT    251   out   loop timing and estimator health
 *
 * That is enough for QGroundControl to recognise the vehicle, show attitude
 * and battery, and for a simulator to fly the firmware in the loop. Signed
 * frames are dropped rather than accepted unverified, and v1 frames are
 * skipped. Both are deliberate: a parser that half-supports signing gives a
 * false sense of an authenticated link.
 *
 * Why write it instead of including the generated C headers: the generated
 * library is several hundred kilobytes of headers built around a global
 * channel state, which costs RAM the Nano does not have, and it cannot be
 * linked by the host tests without vendoring it. Seven messages written out
 * by hand, checked byte for byte against pymavlink in CI (see
 * tools/mavlink_crosscheck.py), is the smaller and more checkable trade.
 *
 * No Arduino dependency, like every other header the host tests link.
 */

namespace mavlink {

constexpr uint8_t kStx = 0xFD;
constexpr size_t kHeaderLen = 10; ///< STX through the 3 byte message id
constexpr size_t kChecksumLen = 2;
constexpr size_t kMaxPayload = 255;
constexpr size_t kMaxFrame = kHeaderLen + kMaxPayload + kChecksumLen;
constexpr uint8_t kIncompatSigned = 0x01;

// Message ids, CRC_EXTRA seeds and full payload lengths, extensions included.
// The seeds come from the message definitions and are what make two sides
// with different ideas of a message's layout reject each other's frames.
constexpr uint32_t kHeartbeat = 0;
constexpr uint32_t kSysStatus = 1;
constexpr uint32_t kAttitude = 30;
constexpr uint32_t kRcChannelsOverride = 70;
constexpr uint32_t kHilActuatorControls = 93;
constexpr uint32_t kHilSensor = 107;
constexpr uint32_t kNamedValueFloat = 251;

struct MessageInfo {
    uint32_t id;
    uint8_t crc_extra;
    uint8_t length; ///< full payload length, extensions included
};

constexpr MessageInfo kMessages[] = {
    {kHeartbeat, 50, 9},
    // 31 base bytes, plus 12 of extended sensor flags we always send as zero.
    // Truncation drops them on the wire, so this costs nothing.
    {kSysStatus, 124, 43},
    {kAttitude, 39, 28},
    {kRcChannelsOverride, 124, 38},
    {kHilActuatorControls, 47, 81},
    {kHilSensor, 108, 65},
    {kNamedValueFloat, 170, 18},
};

inline const MessageInfo* findMessage(uint32_t id) {
    for (const auto& m : kMessages) {
        if (m.id == id) return &m;
    }
    return nullptr;
}

// --------------------------------------------------------------- checksum

/** CRC-16/MCRF4XX, the X.25 variant MAVLink uses, one byte at a time. */
inline uint16_t crcAccumulate(uint8_t byte, uint16_t crc) {
    uint8_t tmp = byte ^ static_cast<uint8_t>(crc & 0xFF);
    tmp ^= static_cast<uint8_t>(tmp << 4);
    return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                 (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
}

inline uint16_t crcBlock(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < len; ++i) crc = crcAccumulate(data[i], crc);
    return crc;
}

// ------------------------------------------------- little endian payloads

/**
 * Payloads are written byte by byte rather than through a packed struct.
 * MAVLink is little endian and so are the ESP32, the AVR and every host this
 * builds on, but a reinterpret_cast onto a packed struct is undefined
 * behaviour on any of them, and an unaligned float load faults on some cores.
 */
class Writer {
public:
    explicit Writer(uint8_t* buf) : buf_(buf) {}
    void u8(uint8_t v) { buf_[pos_++] = v; }
    void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
    void u16(uint16_t v) {
        u8(static_cast<uint8_t>(v));
        u8(static_cast<uint8_t>(v >> 8));
    }
    void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
    void u32(uint32_t v) {
        u16(static_cast<uint16_t>(v));
        u16(static_cast<uint16_t>(v >> 16));
    }
    void u64(uint64_t v) {
        u32(static_cast<uint32_t>(v));
        u32(static_cast<uint32_t>(v >> 32));
    }
    void f32(float v) {
        static_assert(sizeof(float) == 4, "MAVLink floats are IEEE 754 binary32");
        uint32_t bits;
        memcpy(&bits, &v, 4);
        u32(bits);
    }
    void chars(const char* s, size_t n) {
        // Fixed width, zero padded, not necessarily terminated: the MAVLink
        // definition of char[n].
        size_t i = 0;
        for (; i < n && s[i] != '\0'; ++i) u8(static_cast<uint8_t>(s[i]));
        for (; i < n; ++i) u8(0);
    }
    size_t size() const { return pos_; }

private:
    uint8_t* buf_;
    size_t pos_ = 0;
};

class Reader {
public:
    explicit Reader(const uint8_t* buf) : buf_(buf) {}
    uint8_t u8() { return buf_[pos_++]; }
    int8_t i8() { return static_cast<int8_t>(u8()); }
    uint16_t u16() {
        const uint16_t lo = u8();
        return static_cast<uint16_t>(lo | (static_cast<uint16_t>(u8()) << 8));
    }
    int16_t i16() { return static_cast<int16_t>(u16()); }
    uint32_t u32() {
        const uint32_t lo = u16();
        return lo | (static_cast<uint32_t>(u16()) << 16);
    }
    uint64_t u64() {
        const uint64_t lo = u32();
        return lo | (static_cast<uint64_t>(u32()) << 32);
    }
    float f32() {
        const uint32_t bits = u32();
        float v;
        memcpy(&v, &bits, 4);
        return v;
    }
    void chars(char* out, size_t n) {
        for (size_t i = 0; i < n; ++i) out[i] = static_cast<char>(u8());
    }

private:
    const uint8_t* buf_;
    size_t pos_ = 0;
};

// ---------------------------------------------------------------- framing

/** Who is sending. One autopilot on a link needs nothing more. */
struct Endpoint {
    uint8_t system_id = 1;
    uint8_t component_id = 1; ///< MAV_COMP_ID_AUTOPILOT1
    uint8_t seq = 0;
};

/**
 * Wrap a payload into a complete v2 frame.
 *
 * Trailing zero bytes are truncated, as the v2 specification requires a
 * sender to do, keeping at least one byte. The checksum is computed over the
 * truncated payload, which is what a receiver will see.
 *
 * @return frame length in bytes, or 0 if the message id is unknown or the
 *         payload is longer than that message allows.
 */
inline size_t finalize(uint8_t* out, Endpoint& ep, uint32_t msgid,
                       const uint8_t* payload, size_t len) {
    const MessageInfo* info = findMessage(msgid);
    if (!info || len > info->length) return 0;

    while (len > 1 && payload[len - 1] == 0) --len;

    out[0] = kStx;
    out[1] = static_cast<uint8_t>(len);
    out[2] = 0; // incompat flags: unsigned
    out[3] = 0; // compat flags
    out[4] = ep.seq++;
    out[5] = ep.system_id;
    out[6] = ep.component_id;
    out[7] = static_cast<uint8_t>(msgid);
    out[8] = static_cast<uint8_t>(msgid >> 8);
    out[9] = static_cast<uint8_t>(msgid >> 16);
    if (payload != out + kHeaderLen) memmove(out + kHeaderLen, payload, len);

    uint16_t crc = crcBlock(out + 1, kHeaderLen - 1 + len);
    crc = crcAccumulate(info->crc_extra, crc);
    out[kHeaderLen + len] = static_cast<uint8_t>(crc);
    out[kHeaderLen + len + 1] = static_cast<uint8_t>(crc >> 8);
    return kHeaderLen + len + kChecksumLen;
}

// --------------------------------------------------------------- messages
//
// Each message is a plain struct with pack() and unpack(). Fields are written
// in wire order, which MAVLink defines as the base fields sorted by type size,
// largest first, followed by extension fields in declaration order. Getting
// that order wrong produces frames that pass our own round-trip tests and fail
// every other implementation, which is why the cross-check against pymavlink
// exists.

/** MAV_TYPE, MAV_AUTOPILOT, MAV_STATE and MAV_MODE_FLAG values we use. */
constexpr uint8_t kMavTypeQuadrotor = 2;
constexpr uint8_t kMavAutopilotGeneric = 0;
constexpr uint8_t kMavStateBoot = 1;
constexpr uint8_t kMavStateStandby = 3;
constexpr uint8_t kMavStateActive = 4;
constexpr uint8_t kMavStateCritical = 5;
constexpr uint8_t kMavStateEmergency = 6;
constexpr uint8_t kModeFlagCustomEnabled = 1;
constexpr uint8_t kModeFlagStabilizeEnabled = 16;
constexpr uint8_t kModeFlagHilEnabled = 32;
constexpr uint8_t kModeFlagManualInput = 64;
constexpr uint8_t kModeFlagSafetyArmed = 128;

/** MAV_SYS_STATUS_SENSOR bits. */
constexpr uint32_t kSensorGyro = 1u << 0;
constexpr uint32_t kSensorAccel = 1u << 1;
constexpr uint32_t kSensorRateControl = 1u << 10;
constexpr uint32_t kSensorAttitudeStabilization = 1u << 11;
constexpr uint32_t kSensorMotorOutputs = 1u << 15;
constexpr uint32_t kSensorRcReceiver = 1u << 16;
constexpr uint32_t kSensorAhrs = 1u << 21;
constexpr uint32_t kSensorBattery = 1u << 25;

struct Heartbeat {
    static constexpr uint8_t kLen = 9;
    static constexpr uint32_t kId = kHeartbeat;
    uint32_t custom_mode = 0;
    uint8_t type = kMavTypeQuadrotor;
    uint8_t autopilot = kMavAutopilotGeneric;
    uint8_t base_mode = 0;
    uint8_t system_status = kMavStateStandby;
    uint8_t mavlink_version = 3;

    size_t pack(uint8_t* p) const {
        Writer w(p);
        w.u32(custom_mode);
        w.u8(type);
        w.u8(autopilot);
        w.u8(base_mode);
        w.u8(system_status);
        w.u8(mavlink_version);
        return w.size();
    }
    void unpack(const uint8_t* p) {
        Reader r(p);
        custom_mode = r.u32();
        type = r.u8();
        autopilot = r.u8();
        base_mode = r.u8();
        system_status = r.u8();
        mavlink_version = r.u8();
    }
};

struct SysStatus {
    static constexpr uint8_t kLen = 43;
    static constexpr uint32_t kId = kSysStatus;
    uint32_t sensors_present = 0;
    uint32_t sensors_enabled = 0;
    uint32_t sensors_health = 0;
    uint16_t load = 0;            ///< main loop load, 0.1 percent
    uint16_t voltage_battery = 0; ///< mV, UINT16_MAX if unknown
    int16_t current_battery = -1; ///< cA, -1 if unknown
    uint16_t drop_rate_comm = 0;  ///< 0.01 percent
    uint16_t errors_comm = 0;
    uint16_t errors_count[4] = {0, 0, 0, 0};
    int8_t battery_remaining = -1; ///< percent, -1 if unknown

    size_t pack(uint8_t* p) const {
        Writer w(p);
        w.u32(sensors_present);
        w.u32(sensors_enabled);
        w.u32(sensors_health);
        w.u16(load);
        w.u16(voltage_battery);
        w.i16(current_battery);
        w.u16(drop_rate_comm);
        w.u16(errors_comm);
        for (uint16_t e : errors_count) w.u16(e);
        w.i8(battery_remaining);
        // Extended sensor flags, unused.
        w.u32(0);
        w.u32(0);
        w.u32(0);
        return w.size();
    }
    void unpack(const uint8_t* p) {
        Reader r(p);
        sensors_present = r.u32();
        sensors_enabled = r.u32();
        sensors_health = r.u32();
        load = r.u16();
        voltage_battery = r.u16();
        current_battery = r.i16();
        drop_rate_comm = r.u16();
        errors_comm = r.u16();
        for (uint16_t& e : errors_count) e = r.u16();
        battery_remaining = r.i8();
    }
};

struct Attitude {
    static constexpr uint8_t kLen = 28;
    static constexpr uint32_t kId = kAttitude;
    uint32_t time_boot_ms = 0;
    float roll = 0, pitch = 0, yaw = 0;                ///< rad
    float rollspeed = 0, pitchspeed = 0, yawspeed = 0; ///< rad/s

    size_t pack(uint8_t* p) const {
        Writer w(p);
        w.u32(time_boot_ms);
        w.f32(roll);
        w.f32(pitch);
        w.f32(yaw);
        w.f32(rollspeed);
        w.f32(pitchspeed);
        w.f32(yawspeed);
        return w.size();
    }
    void unpack(const uint8_t* p) {
        Reader r(p);
        time_boot_ms = r.u32();
        roll = r.f32();
        pitch = r.f32();
        yaw = r.f32();
        rollspeed = r.f32();
        pitchspeed = r.f32();
        yawspeed = r.f32();
    }
};

struct RcChannelsOverride {
    static constexpr uint8_t kLen = 38;
    static constexpr uint32_t kId = kRcChannelsOverride;
    /** Microseconds. 0 and UINT16_MAX both mean "not overridden". */
    uint16_t chan[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t target_system = 0;
    uint8_t target_component = 0;

    size_t pack(uint8_t* p) const {
        Writer w(p);
        for (uint16_t c : chan) w.u16(c);
        w.u8(target_system);
        w.u8(target_component);
        for (int i = 0; i < 10; ++i) w.u16(0); // channels 9 to 18, unused
        return w.size();
    }
    void unpack(const uint8_t* p) {
        Reader r(p);
        for (uint16_t& c : chan) c = r.u16();
        target_system = r.u8();
        target_component = r.u8();
    }
};

struct HilActuatorControls {
    static constexpr uint8_t kLen = 81;
    static constexpr uint32_t kId = kHilActuatorControls;
    uint64_t time_usec = 0;
    uint64_t flags = 0;
    /** Normalised 0..1 for motors. DroneCtrl order: FL, FR, BL, BR. */
    float controls[16] = {};
    uint8_t mode = 0; ///< MAV_MODE_FLAG bits

    size_t pack(uint8_t* p) const {
        Writer w(p);
        w.u64(time_usec);
        w.u64(flags);
        for (float c : controls) w.f32(c);
        w.u8(mode);
        return w.size();
    }
    void unpack(const uint8_t* p) {
        Reader r(p);
        time_usec = r.u64();
        flags = r.u64();
        for (float& c : controls) c = r.f32();
        mode = r.u8();
    }
};

struct HilSensor {
    static constexpr uint8_t kLen = 65;
    static constexpr uint32_t kId = kHilSensor;
    uint64_t time_usec = 0;
    /** Specific force, m/s^2, body frame FRD. */
    float xacc = 0, yacc = 0, zacc = 0;
    /** Body rates, rad/s, FRD. */
    float xgyro = 0, ygyro = 0, zgyro = 0;
    float xmag = 0, ymag = 0, zmag = 0;
    float abs_pressure = 0, diff_pressure = 0, pressure_alt = 0, temperature = 0;
    uint32_t fields_updated = 0;
    uint8_t id = 0;

    size_t pack(uint8_t* p) const {
        Writer w(p);
        w.u64(time_usec);
        w.f32(xacc);
        w.f32(yacc);
        w.f32(zacc);
        w.f32(xgyro);
        w.f32(ygyro);
        w.f32(zgyro);
        w.f32(xmag);
        w.f32(ymag);
        w.f32(zmag);
        w.f32(abs_pressure);
        w.f32(diff_pressure);
        w.f32(pressure_alt);
        w.f32(temperature);
        w.u32(fields_updated);
        w.u8(id);
        return w.size();
    }
    void unpack(const uint8_t* p) {
        Reader r(p);
        time_usec = r.u64();
        xacc = r.f32();
        yacc = r.f32();
        zacc = r.f32();
        xgyro = r.f32();
        ygyro = r.f32();
        zgyro = r.f32();
        xmag = r.f32();
        ymag = r.f32();
        zmag = r.f32();
        abs_pressure = r.f32();
        diff_pressure = r.f32();
        pressure_alt = r.f32();
        temperature = r.f32();
        fields_updated = r.u32();
        id = r.u8();
    }
};

struct NamedValueFloat {
    static constexpr uint8_t kLen = 18;
    static constexpr uint32_t kId = kNamedValueFloat;
    uint32_t time_boot_ms = 0;
    float value = 0.0f;
    char name[10] = {};

    size_t pack(uint8_t* p) const {
        Writer w(p);
        w.u32(time_boot_ms);
        w.f32(value);
        w.chars(name, 10);
        return w.size();
    }
    void unpack(const uint8_t* p) {
        Reader r(p);
        time_boot_ms = r.u32();
        value = r.f32();
        r.chars(name, 10);
    }
    void setName(const char* s) {
        memset(name, 0, sizeof(name));
        for (size_t i = 0; i < sizeof(name) && s[i] != '\0'; ++i) name[i] = s[i];
    }
};

/**
 * Pack a message straight into a frame buffer and finalise it in place.
 *
 * One buffer, sized at compile time for the message, rather than a 255 byte
 * scratch payload plus a frame: on the Nano a telemetry send then costs about
 * 55 bytes of stack instead of over 500.
 */
template <typename Msg, size_t N>
inline size_t encode(uint8_t (&out)[N], Endpoint& ep, const Msg& msg) {
    static_assert(N >= kHeaderLen + Msg::kLen + kChecksumLen, "frame buffer too small for this message");
    const size_t len = msg.pack(out + kHeaderLen);
    return finalize(out, ep, Msg::kId, out + kHeaderLen, len);
}

/** Frame size for the largest of the given payload lengths. */
constexpr size_t frameSize(size_t payload_len) { return kHeaderLen + payload_len + kChecksumLen; }

// ----------------------------------------------------------------- parser

/** A frame that passed its checksum, payload zero extended to full length. */
struct Frame {
    uint32_t msgid = 0;
    uint8_t seq = 0;
    uint8_t system_id = 0;
    uint8_t component_id = 0;
    uint8_t wire_length = 0;
    uint8_t payload[kMaxPayload] = {};

    template <typename Msg>
    bool decode(Msg& msg) const {
        if (msgid != Msg::kId) return false;
        msg.unpack(payload);
        return true;
    }
};

struct ParserStats {
    uint32_t frames_ok = 0;
    uint32_t crc_errors = 0;
    /** Well formed frames whose id we do not know, so cannot verify. */
    uint32_t unknown_ids = 0;
    uint32_t signed_dropped = 0;
    /** Frames inferred missing from gaps in each sender's sequence number. */
    uint32_t seq_gaps = 0;
};

/**
 * Byte at a time v2 parser.
 *
 * Feed it whatever arrives on the link and it calls back once per verified
 * frame. It hunts for STX, collects one frame, and checks the CRC. On a bad
 * checksum it drops only the STX it started on and rescans the bytes it has
 * already buffered, rather than discarding the whole would-be frame. That
 * matters on a serial line that carries boot text or a half frame from before
 * a reset: a stray 0xFD must not swallow the real frame that follows it.
 *
 * Frames whose id we do not know, and signed frames, are consumed whole. A
 * ground station sends plenty of messages this firmware ignores (parameter
 * and mission requests), and treating each as a false STX would make the
 * parser rescan inside every one of them.
 *
 * Iterative, no recursion, and no allocation. Holds one frame of state, about
 * 600 bytes, which is why the firmware only instantiates it in the HIL build.
 */
class Parser {
public:
    template <typename OnFrame>
    void push(uint8_t byte, OnFrame&& on_frame) {
        if (pos_ == 0 && byte != kStx) return; // hunting, nothing buffered
        buf_[pos_++] = byte;
        process(on_frame);
    }

    const ParserStats& stats() const { return stats_; }

private:
    uint8_t buf_[kMaxFrame + 13] = {};
    size_t pos_ = 0;
    Frame frame_;
    ParserStats stats_;
    /** Last sequence number per system id, bit 8 set once one has been seen. */
    uint16_t last_seq_[256] = {};

    void consume(size_t n) {
        memmove(buf_, buf_ + n, pos_ - n);
        pos_ -= n;
    }

    template <typename OnFrame>
    void process(OnFrame& on_frame) {
        while (pos_ > 0) {
            if (buf_[0] != kStx) {
                consume(1);
                continue;
            }
            if (pos_ < 3) return;
            const uint8_t len = buf_[1];
            const bool is_signed = (buf_[2] & kIncompatSigned) != 0;
            const size_t need = kHeaderLen + len + kChecksumLen + (is_signed ? 13 : 0);
            if (pos_ < need) return;

            const uint32_t msgid = static_cast<uint32_t>(buf_[7]) |
                                   (static_cast<uint32_t>(buf_[8]) << 8) |
                                   (static_cast<uint32_t>(buf_[9]) << 16);
            const MessageInfo* info = findMessage(msgid);
            if (is_signed) {
                ++stats_.signed_dropped;
                consume(need);
                continue;
            }
            if (!info) {
                ++stats_.unknown_ids;
                consume(need);
                continue;
            }

            uint16_t crc = crcBlock(buf_ + 1, kHeaderLen - 1 + len);
            crc = crcAccumulate(info->crc_extra, crc);
            const uint16_t got = static_cast<uint16_t>(buf_[kHeaderLen + len]) |
                                 (static_cast<uint16_t>(buf_[kHeaderLen + len + 1]) << 8);
            if (crc != got || len > info->length) {
                ++stats_.crc_errors;
                consume(1); // drop this STX only, rescan what follows it
                continue;
            }

            frame_.msgid = msgid;
            frame_.seq = buf_[4];
            frame_.system_id = buf_[5];
            frame_.component_id = buf_[6];
            frame_.wire_length = len;
            memset(frame_.payload, 0, info->length);
            memcpy(frame_.payload, buf_ + kHeaderLen, len);

            uint16_t& last = last_seq_[frame_.system_id];
            if (last & 0x100) {
                const uint8_t expected = static_cast<uint8_t>((last & 0xFF) + 1);
                stats_.seq_gaps += static_cast<uint8_t>(frame_.seq - expected);
            }
            last = static_cast<uint16_t>(0x100 | frame_.seq);

            ++stats_.frames_ok;
            consume(need);
            on_frame(frame_);
        }
    }
};

/** Little helper for writers of .tlog files: 8 byte big endian microseconds. */
inline void tlogTimestamp(uint8_t out[8], uint64_t usec) {
    for (int i = 7; i >= 0; --i) {
        out[i] = static_cast<uint8_t>(usec);
        usec >>= 8;
    }
}

} // namespace mavlink
