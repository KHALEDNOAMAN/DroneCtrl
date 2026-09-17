#pragma once

#include <stdint.h>
#include <string.h>

/**
 * Telemetry frame encoding for a CAN 2.0B bus.
 *
 * Scope, stated plainly: this is the encoding layer only. It packs and unpacks
 * the flight controller's telemetry into 8 byte CAN payloads and is covered by
 * host unit tests. It is not a DroneCAN or UAVCAN implementation and does not
 * claim interoperability with one. It borrows two conventions from DroneCAN
 * because they are the right ones and cost nothing to follow: a message type
 * and a source node id in the extended identifier, and a transfer counter so a
 * receiver can spot a dropped frame.
 *
 * Nothing here talks to a peripheral. The ESP32's TWAI controller and an
 * MCP2515 on the Nano's SPI bus present completely different register
 * interfaces, but both take the same (id, payload, length) triple, so keeping
 * the encoding separate means one tested implementation serves both and the
 * host build needs no CAN hardware at all.
 *
 * Identifier layout, 29 bits
 * --------------------------
 *   bits 28..24   reserved, zero
 *   bits 23..8    message id
 *   bits  7..0    source node id
 *
 * Why fixed point rather than floats: a float costs four of the eight payload
 * bytes, so a naive layout fits two numbers per frame. Scaled integers put
 * attitude, rates and battery into one frame each at a resolution well inside
 * the sensors' own noise, which is what keeps a 250 Hz controller's telemetry
 * from saturating a 500 kbit bus.
 */

namespace can_telemetry {

constexpr uint16_t kMsgAttitude = 0x0100;  ///< roll, pitch, yaw rate, flags
constexpr uint16_t kMsgRates = 0x0101;     ///< body rates p, q, r and loop load
constexpr uint16_t kMsgPower = 0x0102;     ///< battery voltage, current, mAh used
constexpr uint16_t kMsgEstimator = 0x0103; ///< gyro bias and innovation health

/** Attitude and rate are sent in centi-degrees: +-327.67 deg at 0.01 deg. */
constexpr float kAngleScale = 100.0f;
/** Rates in centi-degrees per second, +-327.67 deg/s. */
constexpr float kRateScale = 100.0f;
/** Battery volts in millivolts. */
constexpr float kVoltScale = 1000.0f;
/** Gyro bias in milli-degrees per second, which resolves far below the noise. */
constexpr float kBiasScale = 1000.0f;

struct Frame {
    uint32_t id = 0;      ///< 29 bit extended identifier
    uint8_t data[8] = {}; ///< payload
    uint8_t length = 0;   ///< payload bytes in use, 0..8
};

struct Attitude {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_rate_dps = 0.0f;
    uint8_t flight_state = 0; ///< FlightState as a byte
    uint8_t fault = 0;        ///< FaultCause as a byte
};

struct Estimator {
    float bias_p_dps = 0.0f;
    float bias_q_dps = 0.0f;
    /** Accelerometer residual magnitude in milli-g. */
    uint16_t innovation_mg = 0;
};

inline uint32_t makeId(uint16_t message_id, uint8_t node_id) {
    return (static_cast<uint32_t>(message_id) << 8) | node_id;
}

inline uint16_t messageIdOf(uint32_t can_id) {
    return static_cast<uint16_t>((can_id >> 8) & 0xFFFF);
}

inline uint8_t nodeIdOf(uint32_t can_id) {
    return static_cast<uint8_t>(can_id & 0xFF);
}

/** Saturating float to int16 conversion. Wrapping would turn a large tilt into
 *  a small one of the opposite sign, which is the worst possible failure for a
 *  telemetry field a ground station might act on. */
inline int16_t toI16(float value, float scale) {
    const float scaled = value * scale;
    if (scaled >= 32767.0f) return 32767;
    if (scaled <= -32768.0f) return -32768;
    return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

inline float fromI16(int16_t raw, float scale) {
    return static_cast<float>(raw) / scale;
}

/** Little endian, matching every MCU this firmware targets, so pack and unpack
 *  on the same family are a memcpy in practice and the byte order is still
 *  explicit for anyone reading a bus capture. */
inline void put16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline uint16_t get16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline Frame packAttitude(const Attitude& a, uint8_t node_id, uint8_t transfer) {
    Frame f;
    f.id = makeId(kMsgAttitude, node_id);
    put16(&f.data[0], static_cast<uint16_t>(toI16(a.roll_deg, kAngleScale)));
    put16(&f.data[2], static_cast<uint16_t>(toI16(a.pitch_deg, kAngleScale)));
    put16(&f.data[4], static_cast<uint16_t>(toI16(a.yaw_rate_dps, kRateScale)));
    f.data[6] = static_cast<uint8_t>((a.flight_state & 0x0F) | ((a.fault & 0x0F) << 4));
    f.data[7] = transfer;
    f.length = 8;
    return f;
}

inline bool unpackAttitude(const Frame& f, Attitude& out, uint8_t* transfer = nullptr) {
    if (messageIdOf(f.id) != kMsgAttitude || f.length != 8) return false;
    out.roll_deg = fromI16(static_cast<int16_t>(get16(&f.data[0])), kAngleScale);
    out.pitch_deg = fromI16(static_cast<int16_t>(get16(&f.data[2])), kAngleScale);
    out.yaw_rate_dps = fromI16(static_cast<int16_t>(get16(&f.data[4])), kRateScale);
    out.flight_state = static_cast<uint8_t>(f.data[6] & 0x0F);
    out.fault = static_cast<uint8_t>((f.data[6] >> 4) & 0x0F);
    if (transfer) *transfer = f.data[7];
    return true;
}

inline Frame packEstimator(const Estimator& e, uint8_t node_id, uint8_t transfer) {
    Frame f;
    f.id = makeId(kMsgEstimator, node_id);
    put16(&f.data[0], static_cast<uint16_t>(toI16(e.bias_p_dps, kBiasScale)));
    put16(&f.data[2], static_cast<uint16_t>(toI16(e.bias_q_dps, kBiasScale)));
    put16(&f.data[4], e.innovation_mg);
    f.data[6] = 0;
    f.data[7] = transfer;
    f.length = 8;
    return f;
}

inline bool unpackEstimator(const Frame& f, Estimator& out, uint8_t* transfer = nullptr) {
    if (messageIdOf(f.id) != kMsgEstimator || f.length != 8) return false;
    out.bias_p_dps = fromI16(static_cast<int16_t>(get16(&f.data[0])), kBiasScale);
    out.bias_q_dps = fromI16(static_cast<int16_t>(get16(&f.data[2])), kBiasScale);
    out.innovation_mg = get16(&f.data[4]);
    if (transfer) *transfer = f.data[7];
    return true;
}

/**
 * Tracks the transfer counter of an incoming stream and reports gaps.
 * A receiver that does not do this cannot tell a stale reading from a fresh
 * one, which matters when the thing being read is the airframe's attitude.
 */
class TransferMonitor {
public:
    /** @return frames lost between the previous accepted one and this one. */
    uint8_t observe(uint8_t transfer) {
        if (!started_) {
            started_ = true;
            last_ = transfer;
            return 0;
        }
        const uint8_t expected = static_cast<uint8_t>(last_ + 1);
        const uint8_t gap = static_cast<uint8_t>(transfer - expected);
        last_ = transfer;
        return gap;
    }

    void reset() { started_ = false; last_ = 0; }

private:
    bool started_ = false;
    uint8_t last_ = 0;
};

} // namespace can_telemetry
