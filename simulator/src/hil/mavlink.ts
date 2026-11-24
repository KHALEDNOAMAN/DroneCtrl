/**
 * MAVLink v2 for the simulator's side of the HIL link.
 *
 * The TypeScript twin of firmware/include/mavlink_lite.h: the same seven
 * messages, the same truncation rule, the same resynchronising parser. Kept
 * deliberately parallel to the C++ so a reader can check one against the
 * other line by line. tools/hil-check.mjs flies the real firmware loop
 * (through the SIL binary's --hil-board mode) with this code on the other
 * end, so a disagreement between the two shows up as a failed flight.
 */

export const MSG = {
  HEARTBEAT: 0,
  SYS_STATUS: 1,
  ATTITUDE: 30,
  RC_CHANNELS_OVERRIDE: 70,
  HIL_ACTUATOR_CONTROLS: 93,
  HIL_SENSOR: 107,
  NAMED_VALUE_FLOAT: 251,
} as const;

/** CRC_EXTRA seed and full payload length, extensions included. */
const INFO: Record<number, { crcExtra: number; length: number }> = {
  [MSG.HEARTBEAT]: { crcExtra: 50, length: 9 },
  [MSG.SYS_STATUS]: { crcExtra: 124, length: 43 },
  [MSG.ATTITUDE]: { crcExtra: 39, length: 28 },
  [MSG.RC_CHANNELS_OVERRIDE]: { crcExtra: 124, length: 38 },
  [MSG.HIL_ACTUATOR_CONTROLS]: { crcExtra: 47, length: 81 },
  [MSG.HIL_SENSOR]: { crcExtra: 108, length: 65 },
  [MSG.NAMED_VALUE_FLOAT]: { crcExtra: 170, length: 18 },
};

const STX = 0xfd;
const HEADER = 10;

export function crcAccumulate(byte: number, crc: number): number {
  let tmp = (byte ^ (crc & 0xff)) & 0xff;
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}

function crcBlock(data: Uint8Array, start: number, end: number, crc = 0xffff): number {
  for (let i = start; i < end; i++) crc = crcAccumulate(data[i], crc);
  return crc;
}

export class Endpoint {
  seq = 0;
  constructor(public systemId = 255, public componentId = 190) {}

  /** Frame a payload, truncating trailing zeros as v2 requires. */
  frame(msgid: number, payload: Uint8Array): Uint8Array {
    const info = INFO[msgid];
    if (!info || payload.length > info.length) throw new Error(`cannot frame message ${msgid}`);
    let len = payload.length;
    while (len > 1 && payload[len - 1] === 0) len--;

    const out = new Uint8Array(HEADER + len + 2);
    out[0] = STX;
    out[1] = len;
    out[2] = 0;
    out[3] = 0;
    out[4] = this.seq;
    this.seq = (this.seq + 1) & 0xff;
    out[5] = this.systemId;
    out[6] = this.componentId;
    out[7] = msgid & 0xff;
    out[8] = (msgid >> 8) & 0xff;
    out[9] = (msgid >> 16) & 0xff;
    out.set(payload.subarray(0, len), HEADER);
    let crc = crcBlock(out, 1, HEADER + len);
    crc = crcAccumulate(info.crcExtra, crc);
    out[HEADER + len] = crc & 0xff;
    out[HEADER + len + 1] = crc >> 8;
    return out;
  }
}

// ----------------------------------------------------------------- messages

export interface HilSensor {
  timeUsec: bigint;
  /** Specific force, m/s^2, body FRD. */
  xacc: number; yacc: number; zacc: number;
  /** Body rates, rad/s, FRD. */
  xgyro: number; ygyro: number; zgyro: number;
}

export function packHilSensor(m: HilSensor): Uint8Array {
  const b = new Uint8Array(INFO[MSG.HIL_SENSOR].length);
  const v = new DataView(b.buffer);
  v.setBigUint64(0, m.timeUsec, true);
  const f = [m.xacc, m.yacc, m.zacc, m.xgyro, m.ygyro, m.zgyro];
  f.forEach((x, i) => v.setFloat32(8 + 4 * i, x, true));
  // mag, pressures and temperature stay zero: no such sensors in the model
  v.setUint32(60, 0x3f, true); // fields_updated: accel and gyro
  v.setUint8(64, 0);
  return b;
}

/** Channels in microseconds, AETR. */
export function packRcOverride(chan: number[], targetSystem = 1, targetComponent = 1): Uint8Array {
  const b = new Uint8Array(INFO[MSG.RC_CHANNELS_OVERRIDE].length);
  const v = new DataView(b.buffer);
  for (let i = 0; i < 8; i++) v.setUint16(2 * i, Math.round(chan[i] ?? 0), true);
  v.setUint8(16, targetSystem);
  v.setUint8(17, targetComponent);
  return b;
}

export interface HilActuatorControls {
  timeUsec: bigint;
  /** FL, FR, BL, BR, normalised 0..1. */
  controls: [number, number, number, number];
  mode: number;
}

export function unpackHilActuatorControls(p: Uint8Array): HilActuatorControls {
  const v = new DataView(p.buffer, p.byteOffset, p.byteLength);
  return {
    timeUsec: v.getBigUint64(0, true),
    controls: [v.getFloat32(16, true), v.getFloat32(20, true), v.getFloat32(24, true), v.getFloat32(28, true)],
    mode: v.getUint8(80),
  };
}

export interface Heartbeat {
  customMode: number;
  baseMode: number;
  systemStatus: number;
}

export function unpackHeartbeat(p: Uint8Array): Heartbeat {
  const v = new DataView(p.buffer, p.byteOffset, p.byteLength);
  return { customMode: v.getUint32(0, true), baseMode: v.getUint8(6), systemStatus: v.getUint8(7) };
}

export interface NamedValueFloat {
  name: string;
  value: number;
}

export function unpackNamedValueFloat(p: Uint8Array): NamedValueFloat {
  const v = new DataView(p.buffer, p.byteOffset, p.byteLength);
  let name = '';
  for (let i = 8; i < 18 && p[i] !== 0; i++) name += String.fromCharCode(p[i]);
  return { name, value: v.getFloat32(4, true) };
}

// ------------------------------------------------------------------- parser

export interface Frame {
  msgid: number;
  seq: number;
  systemId: number;
  /** Zero extended to the message's full length. */
  payload: Uint8Array;
}

export interface ParserStats {
  framesOk: number;
  crcErrors: number;
  unknownIds: number;
}

/**
 * Same policy as the C++ parser: on a bad checksum drop only the STX and
 * rescan what follows it; consume unknown and signed frames whole.
 */
export class Parser {
  private buf = new Uint8Array(512);
  private pos = 0;
  readonly stats: ParserStats = { framesOk: 0, crcErrors: 0, unknownIds: 0 };

  push(data: Uint8Array, onFrame: (f: Frame) => void): void {
    for (const byte of data) {
      if (this.pos === 0 && byte !== STX) continue;
      this.buf[this.pos++] = byte;
      this.process(onFrame);
    }
  }

  private consume(n: number) {
    this.buf.copyWithin(0, n, this.pos);
    this.pos -= n;
  }

  private process(onFrame: (f: Frame) => void) {
    while (this.pos > 0) {
      if (this.buf[0] !== STX) { this.consume(1); continue; }
      if (this.pos < 3) return;
      const len = this.buf[1];
      const signed = (this.buf[2] & 0x01) !== 0;
      const need = HEADER + len + 2 + (signed ? 13 : 0);
      if (this.pos < need) return;
      const msgid = this.buf[7] | (this.buf[8] << 8) | (this.buf[9] << 16);
      const info = INFO[msgid];
      if (signed || !info) {
        if (!info) this.stats.unknownIds++;
        this.consume(need);
        continue;
      }
      let crc = crcBlock(this.buf, 1, HEADER + len);
      crc = crcAccumulate(info.crcExtra, crc);
      const got = this.buf[HEADER + len] | (this.buf[HEADER + len + 1] << 8);
      if (crc !== got || len > info.length) {
        this.stats.crcErrors++;
        this.consume(1);
        continue;
      }
      const payload = new Uint8Array(info.length);
      payload.set(this.buf.subarray(HEADER, HEADER + len));
      const frame: Frame = { msgid, seq: this.buf[4], systemId: this.buf[5], payload };
      this.stats.framesOk++;
      this.consume(need);
      onFrame(frame);
    }
  }
}
