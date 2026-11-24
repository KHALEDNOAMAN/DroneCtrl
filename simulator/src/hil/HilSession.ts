import {
  Endpoint,
  MSG,
  Parser,
  packHilSensor,
  packRcOverride,
  unpackHeartbeat,
  unpackHilActuatorControls,
  unpackNamedValueFloat,
} from './mavlink';
import { HilPlant, toHilSensorFields } from './HilPlant';

/**
 * The simulator's end of a hardware-in-the-loop session.
 *
 * Lockstep, the same contract firmware/include/hil_link.h documents: send the
 * sticks and one IMU sample, wait for the board's HIL_ACTUATOR_CONTROLS, then
 * and only then advance the plant by one 4 ms step. A slow link makes the
 * simulation run slower than real time; it never hands the controller a
 * sample it has not answered.
 *
 * Transport agnostic. The browser passes a WebSerial port (webserial.ts); the
 * CI check passes the stdin and stdout of the SIL binary in --hil-board mode,
 * which runs the firmware's HIL loop on the host.
 */

export interface Transport {
  write(bytes: Uint8Array): Promise<void> | void;
  /** Register the callback for incoming bytes. Called once. */
  onData(cb: (bytes: Uint8Array) => void): void;
}

/** Normalised sticks, as the receiver reports them. */
export interface Sticks {
  throttle: number; // 0..1
  roll: number; // -1..1
  pitch: number; // -1..1
  yaw: number; // -1..1
}

export const FLIGHT_STATES = ['DISARMED', 'ARMING', 'ARMED', 'FAILSAFE_LAND', 'FAILSAFE_CUT'] as const;
export const FAULTS = ['NONE', 'SIGNAL_LOST', 'LOW_BATTERY', 'EXCESSIVE_TILT', 'ESTIMATOR_DIVERGED'] as const;

export interface BoardStatus {
  state: (typeof FLIGHT_STATES)[number] | 'UNKNOWN';
  fault: (typeof FAULTS)[number] | 'UNKNOWN';
  armed: boolean;
  /** Latest NAMED_VALUE_FLOAT values from the board, by name. */
  values: Record<string, number>;
}

export interface SessionStats {
  steps: number;
  simTime: number; // s
  /** Mean board round trip over the last second of steps, ms. */
  roundTripMs: number;
  timeouts: number;
  crcErrors: number;
}

const DT = 0.004; // 250 Hz, the firmware's loop rate
const REPLY_TIMEOUT_MS = 250;

export class HilSession {
  readonly plant = new HilPlant();
  readonly board: BoardStatus = { state: 'UNKNOWN', fault: 'UNKNOWN', armed: false, values: {} };
  readonly stats: SessionStats = { steps: 0, simTime: 0, roundTripMs: 0, timeouts: 0, crcErrors: 0 };

  /** When false the sticks stop being sent, exactly like a dead receiver. */
  linkUp = true;
  /** Constant roll disturbance on the airframe once airborne, N m. */
  windTorque = 0;

  private ep = new Endpoint(255, 190);
  private parser = new Parser();
  private pwm = [1000, 1000, 1000, 1000];
  private waiting: ((ok: boolean) => void) | null = null;
  private rtts: number[] = [];

  constructor(private transport: Transport) {
    transport.onData((bytes) => this.parser.push(bytes, (frame) => this.onFrame(frame)));
  }

  private onFrame(frame: { msgid: number; payload: Uint8Array }) {
    switch (frame.msgid) {
      case MSG.HIL_ACTUATOR_CONTROLS: {
        const ac = unpackHilActuatorControls(frame.payload);
        for (let i = 0; i < 4; i++) this.pwm[i] = 1000 + ac.controls[i] * 1000;
        this.board.armed = (ac.mode & 128) !== 0;
        const w = this.waiting;
        this.waiting = null;
        w?.(true);
        break;
      }
      case MSG.HEARTBEAT: {
        const hb = unpackHeartbeat(frame.payload);
        this.board.state = FLIGHT_STATES[hb.customMode & 0xff] ?? 'UNKNOWN';
        this.board.fault = FAULTS[(hb.customMode >> 8) & 0xff] ?? 'UNKNOWN';
        break;
      }
      case MSG.NAMED_VALUE_FLOAT: {
        const nv = unpackNamedValueFloat(frame.payload);
        this.board.values[nv.name] = nv.value;
        break;
      }
    }
    this.stats.crcErrors = this.parser.stats.crcErrors;
  }

  /** One lockstep cycle. Resolves false if the board did not answer in time. */
  async step(sticks: Sticks): Promise<boolean> {
    const timeUsec = BigInt(Math.round((this.stats.steps + 1) * DT * 1e6));
    const parts: Uint8Array[] = [];
    if (this.linkUp) {
      const axis = (v: number) => 1500 + 500 * Math.max(-1, Math.min(1, v));
      parts.push(this.ep.frame(MSG.RC_CHANNELS_OVERRIDE, packRcOverride([
        axis(sticks.roll), axis(sticks.pitch),
        1000 + 1000 * Math.max(0, Math.min(1, sticks.throttle)), axis(sticks.yaw),
      ])));
    }
    const imu = this.plant.imu();
    parts.push(this.ep.frame(MSG.HIL_SENSOR, packHilSensor({ timeUsec, ...toHilSensorFields(imu) })));

    const bytes = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
    let o = 0;
    for (const p of parts) { bytes.set(p, o); o += p.length; }

    const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
    const replied = new Promise<boolean>((resolve) => {
      this.waiting = resolve;
      setTimeout(() => {
        if (this.waiting === resolve) { this.waiting = null; resolve(false); }
      }, REPLY_TIMEOUT_MS);
    });
    await this.transport.write(bytes);
    const ok = await replied;
    const t1 = typeof performance !== 'undefined' ? performance.now() : Date.now();

    if (!ok) {
      this.stats.timeouts++;
      return false;
    }
    this.rtts.push(t1 - t0);
    if (this.rtts.length > 250) this.rtts.shift();
    this.stats.roundTripMs = this.rtts.reduce((a, b) => a + b, 0) / this.rtts.length;

    const wind = this.plant.z > 0.5 ? this.windTorque : 0;
    this.plant.step(this.pwm, wind, DT);
    this.stats.steps++;
    this.stats.simTime = this.stats.steps * DT;
    return true;
  }
}
