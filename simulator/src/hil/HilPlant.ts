/**
 * The airframe the HIL board flies against.
 *
 * A line-for-line port of Plant and ImuModel in firmware/sil/sil_flight.cpp,
 * not the simulator's own DronePhysics. That choice is the important one. The
 * firmware's gains are derived for the SIL airframe (1 kg, 0.15 m arm, 6 N
 * motors, see config.h); DronePhysics is a different, heavier aircraft with
 * its own controller. Flying the board against DronePhysics would test the
 * firmware's gains on an airframe they were never designed for and blame the
 * firmware for the result. The board here gets the plant it was tuned on, and
 * the 3D scene just draws it.
 *
 * Conventions are the SIL model's: body rates FRD (positive roll right side
 * down, positive pitch nose up), and an accelerometer that reads +1 g on z at
 * rest. toHilSensor() converts that to MAVLink's FRD specific force, which is
 * the same conversion firmware/include/hil_link.h undoes on the board.
 */

const ESC_MIN = 1000;
const ESC_MAX = 2000;
const G = 9.81;
const STANDARD_GRAVITY = 9.80665;

export interface Airframe {
  mass: number; arm: number;
  inertiaXX: number; inertiaYY: number; inertiaZZ: number;
  maxThrust: number; motorTau: number;
  rotDamping: number; yawTorqueK: number; linDrag: number;
}

export const NOMINAL_AIRFRAME: Airframe = {
  mass: 1.0, arm: 0.15,
  inertiaXX: 0.011, inertiaYY: 0.011, inertiaZZ: 0.021,
  maxThrust: 6.0, motorTau: 0.03,
  rotDamping: 0.02, yawTorqueK: 0.02, linDrag: 0.35,
};

/** The SIL harness's generator, so noise sequences can be compared. */
class Rng {
  constructor(private s: number) {}
  uniform(): number {
    this.s = (Math.imul(this.s, 1664525) + 1013904223) >>> 0;
    return ((this.s >>> 8) & 0xffffff) / 8388608 - 1;
  }
  gauss(): number {
    return this.uniform() + this.uniform() + this.uniform();
  }
}

/** Single precision, as the C++ model computes. */
const f = Math.fround;

export class HilPlant {
  cfg: Airframe;
  roll = 0; pitch = 0; yaw = 0;
  p = 0; q = 0; r = 0;
  x = 0; y = 0; z = 0;
  vx = 0; vy = 0; vz = 0;
  thrust: [number, number, number, number] = [0, 0, 0, 0];
  motorHealth: [number, number, number, number] = [1, 1, 1, 1];
  gyroNoise = 0.015;
  accelNoise = 0.03;
  private rng = new Rng(20260917);

  constructor(cfg: Airframe = NOMINAL_AIRFRAME) {
    this.cfg = { ...cfg };
  }

  /** @param pwm FL, FR, BL, BR in microseconds. */
  step(pwm: readonly number[], windTorqueRoll: number, dt: number): void {
    const c = this.cfg;
    for (let i = 0; i < 4; i++) {
      const u = (pwm[i] - ESC_MIN) / (ESC_MAX - ESC_MIN);
      const commanded = c.maxThrust * (u < 0 ? 0 : u) * this.motorHealth[i];
      const a = dt / (c.motorTau + dt);
      this.thrust[i] = f(this.thrust[i] + a * (commanded - this.thrust[i]));
    }
    const [tFL, tFR, tBL, tBR] = this.thrust;
    const total = tFL + tFR + tBL + tBR;
    const armEff = c.arm * 0.70710678;

    const tauRoll = armEff * (tFL + tBL - (tFR + tBR)) - c.rotDamping * this.p + windTorqueRoll;
    const tauPitch = armEff * (tFL + tFR - (tBL + tBR)) - c.rotDamping * this.q;
    const tauYaw = c.yawTorqueK * (tFR + tBL - (tFL + tBR)) - c.rotDamping * this.r;

    this.p = f(this.p + (tauRoll / c.inertiaXX) * dt);
    this.q = f(this.q + (tauPitch / c.inertiaYY) * dt);
    this.r = f(this.r + (tauYaw / c.inertiaZZ) * dt);

    const sr = Math.sin(this.roll), cr = Math.cos(this.roll);
    const tp = Math.tan(this.pitch), cp = Math.cos(this.pitch);
    const roll = this.roll + (this.p + this.q * sr * tp + this.r * cr * tp) * dt;
    const pitch = this.pitch + (this.q * cr - this.r * sr) * dt;
    const yaw = this.yaw + ((this.q * sr + this.r * cr) / (cp === 0 ? 1e-4 : cp)) * dt;
    this.roll = f(roll); this.pitch = f(pitch); this.yaw = f(yaw);

    const sp = Math.sin(this.pitch);
    const tx = total * (sp * Math.cos(this.yaw) + sr * cp * Math.sin(this.yaw));
    const ty = total * (sp * Math.sin(this.yaw) - sr * cp * Math.cos(this.yaw));
    const tz = total * (cr * cp);

    this.vx = f(this.vx + ((tx - c.linDrag * this.vx) / c.mass) * dt);
    this.vy = f(this.vy + ((ty - c.linDrag * this.vy) / c.mass) * dt);
    this.vz = f(this.vz + ((tz - c.linDrag * this.vz - c.mass * G) / c.mass) * dt);
    this.x = f(this.x + this.vx * dt);
    this.y = f(this.y + this.vy * dt);
    this.z = f(this.z + this.vz * dt);

    if (this.z <= 0) {
      this.z = 0;
      if (this.vz < 0) this.vz = 0;
    }
  }

  /** What the IMU reads, in the firmware's convention: rad/s and g. */
  imu(): { p: number; q: number; r: number; ax: number; ay: number; az: number } {
    const sr = Math.sin(this.roll), cr = Math.cos(this.roll);
    const sp = Math.sin(this.pitch), cp = Math.cos(this.pitch);
    const total = this.thrust[0] + this.thrust[1] + this.thrust[2] + this.thrust[3];
    const specific = total / this.cfg.mass / G;
    return {
      p: this.p + this.gyroNoise * this.rng.gauss(),
      q: this.q + this.gyroNoise * this.rng.gauss(),
      r: this.r + this.gyroNoise * this.rng.gauss(),
      ax: -sp * specific + this.accelNoise * this.rng.gauss(),
      ay: sr * cp * specific + this.accelNoise * this.rng.gauss(),
      az: cr * cp * specific + this.accelNoise * this.rng.gauss(),
    };
  }

  /**
   * Position and attitude for three.js, which is y up with -z forward.
   *
   * The SIL model's horizontal translation is mirrored relative to its own
   * attitude convention (nose up moves it along +x). That has never mattered
   * to the controller, because the IMU model derives specific force from
   * attitude and thrust rather than from translation, so it is corrected here
   * for display only rather than in a model every test depends on.
   */
  sceneTransform(): { position: [number, number, number]; rotation: [number, number, number] } {
    return {
      position: [-this.y, this.z + 0.1, this.x],
      // three.js rotation order YXZ: x pitch (nose up positive), y yaw (nose
      // left positive), z roll (left side down positive).
      rotation: [this.pitch, -this.yaw, -this.roll],
    };
  }
}

/** Firmware-convention IMU sample to MAVLink HIL_SENSOR fields (FRD, SI). */
export function toHilSensorFields(s: { p: number; q: number; r: number; ax: number; ay: number; az: number }) {
  return {
    xgyro: s.p, ygyro: s.q, zgyro: s.r,
    xacc: -s.ax * STANDARD_GRAVITY,
    yacc: -s.ay * STANDARD_GRAVITY,
    zacc: -s.az * STANDARD_GRAVITY,
  };
}
