import { Vector3, Euler } from 'three';
import { PIDController } from './PIDController';
import { ControlInputs, DroneState, FlightMode, TelemetryData, PIDGains } from '../types';
import { ANGULAR_DAMPING, PHYSICS } from '../utils/constants';

/**
 * Rigid-body quadcopter model plus the inner attitude loop.
 *
 * The three attitude PIDs here are the direct port of the C++ controller in
 * firmware/, so the gains you tune in the browser are the gains you flash to
 * the board. Roll and pitch are angle-mode (setpoint is an angle), yaw is
 * rate-mode (setpoint is a rate), matching the firmware's mode split.
 */
export class DroneSimulator {
  position = new Vector3(0, 0.1, 0);
  velocity = new Vector3(0, 0, 0);
  rotation = new Vector3(0, 0, 0); // pitch (x), yaw (y), roll (z)
  angularVelocity = new Vector3(0, 0, 0);

  motorThrusts: [number, number, number, number] = [0, 0, 0, 0];

  /** Remaining pack energy, in watt-hours. A 4S 2200 mAh pack is ~32.6 Wh. */
  private energyRemaining = 32.6;
  private readonly packCapacity = 32.6;

  rollPID: PIDController;
  pitchPID: PIDController;
  yawPID: PIDController;

  constructor(rollGains: PIDGains, pitchGains: PIDGains, yawGains: PIDGains) {
    this.rollPID = new PIDController(rollGains.kp, rollGains.ki, rollGains.kd, -2, 2, 1);
    this.pitchPID = new PIDController(pitchGains.kp, pitchGains.ki, pitchGains.kd, -2, 2, 1);
    this.yawPID = new PIDController(yawGains.kp, yawGains.ki, yawGains.kd, -1, 1, 0.5);
  }

  reset() {
    this.position.set(0, 0.1, 0);
    this.velocity.set(0, 0, 0);
    this.rotation.set(0, 0, 0);
    this.angularVelocity.set(0, 0, 0);
    this.motorThrusts = [0, 0, 0, 0];
    this.energyRemaining = this.packCapacity;
    this.rollPID.reset();
    this.pitchPID.reset();
    this.yawPID.reset();
  }

  update(
    dt: number,
    inputs: ControlInputs,
    windForce: Vector3,
    mode: FlightMode = 'MANUAL',
  ): { droneState: DroneState; telemetry: TelemetryData } {
    if (dt > 0.1) dt = 0.1; // Cap dt for stability
    if (dt <= 0) dt = 0.016; // default 60fps

    const previousVerticalSpeed = this.velocity.y;

    // 1. Attitude loop: angle setpoints for roll/pitch, rate setpoint for yaw
    const rollTorque = this.rollPID.compute(inputs.roll, this.rotation.z, dt);
    const pitchTorque = this.pitchPID.compute(inputs.pitch, this.rotation.x, dt);
    const yawTorque = this.yawPID.compute(inputs.yaw, this.angularVelocity.y, dt);

    // 2. Motor Mixing (X configuration)
    // FL: 0, FR: 1, RL: 2, RR: 3
    const baseThrust = inputs.throttle;

    let tFL = baseThrust + pitchTorque + rollTorque + yawTorque;
    let tFR = baseThrust + pitchTorque - rollTorque - yawTorque;
    let tRL = baseThrust - pitchTorque + rollTorque - yawTorque;
    let tRR = baseThrust - pitchTorque - rollTorque + yawTorque;

    // Clamp thrusts
    tFL = Math.max(0, Math.min(PHYSICS.maxThrustPerMotor, tFL));
    tFR = Math.max(0, Math.min(PHYSICS.maxThrustPerMotor, tFR));
    tRL = Math.max(0, Math.min(PHYSICS.maxThrustPerMotor, tRL));
    tRR = Math.max(0, Math.min(PHYSICS.maxThrustPerMotor, tRR));

    this.motorThrusts = [tFL, tFR, tRL, tRR];

    // 3. Physics Simulation
    const totalThrust = tFL + tFR + tRL + tRR;

    // Torques applied to body
    const L = PHYSICS.armLength;
    const b = PHYSICS.torqueCoefficient;
    const tx = L * (tFL + tFR - tRL - tRR); // Pitch
    const tz = L * (tFL - tFR + tRL - tRR); // Roll
    const ty = b * (tFL - tFR - tRL + tRR); // Yaw

    // Update angular velocity
    // Assuming simple diagonal inertia tensor for now
    const Ixx = 0.01;
    const Iyy = 0.02;
    const Izz = 0.01;

    this.angularVelocity.x += (tx / Ixx) * dt;
    this.angularVelocity.y += (ty / Iyy) * dt;
    this.angularVelocity.z += (tz / Izz) * dt;

    // Aerodynamic damping on body rates. Exponential in dt rather than a flat
    // per-step factor, so the airframe behaves identically at any timestep.
    this.angularVelocity.multiplyScalar(Math.exp(-ANGULAR_DAMPING * dt));

    // Update rotation
    this.rotation.x += this.angularVelocity.x * dt;
    this.rotation.y += this.angularVelocity.y * dt;
    this.rotation.z += this.angularVelocity.z * dt;

    // Calculate linear acceleration in world frame
    const euler = new Euler(this.rotation.x, this.rotation.y, this.rotation.z, 'YXZ');
    const thrustVector = new Vector3(0, totalThrust / PHYSICS.mass, 0).applyEuler(euler);

    const gravityForce = new Vector3(0, -PHYSICS.gravity, 0);
    const dragForce = this.velocity.clone().multiplyScalar(-PHYSICS.drag);
    const acceleration = new Vector3()
      .add(thrustVector)
      .add(gravityForce)
      .add(dragForce)
      .add(windForce.clone().divideScalar(PHYSICS.mass));

    // Ground collision
    if (this.position.y <= 0.1 && acceleration.y < 0) {
      acceleration.y = 0;
      this.velocity.y = 0;
      this.velocity.x *= 0.8;
      this.velocity.z *= 0.8;
      this.position.y = 0.1;
      this.rotation.x *= 0.9;
      this.rotation.z *= 0.9;
    }

    this.velocity.add(acceleration.clone().multiplyScalar(dt));
    this.position.add(this.velocity.clone().multiplyScalar(dt));

    // 4. Battery model. Electrical power scales roughly with thrust^1.5 for a
    // propeller (momentum theory), so hovering is cheap and hard manoeuvring is
    // not. Integrated over time rather than sampled, so the pack actually
    // drains instead of reading a constant.
    const electricalPower = 0.55 * Math.pow(Math.max(totalThrust, 0), 1.5) + 6; // W
    this.energyRemaining = Math.max(0, this.energyRemaining - (electricalPower * dt) / 3600);
    const battery = (this.energyRemaining / this.packCapacity) * 100;

    // Motor RPM from thrust: T = kt * omega^2, so omega scales with sqrt(T).
    const toRPM = (thrust: number) => Math.sqrt(Math.max(thrust, 0) / PHYSICS.maxThrustPerMotor) * 11000;

    return {
      droneState: {
        position: this.position.clone(),
        rotation: this.rotation.clone(),
        velocity: this.velocity.clone(),
        motorSpeeds: [toRPM(tFL), toRPM(tFR), toRPM(tRL), toRPM(tRR)],
        armed: baseThrust > 0,
        mode,
      },
      telemetry: {
        altitude: this.position.y,
        verticalSpeed: (this.velocity.y + previousVerticalSpeed) / 2,
        speed: this.velocity.length(),
        heading: this.rotation.y * (180 / Math.PI),
        battery,
        roll: this.rotation.z,
        pitch: this.rotation.x,
        motorRPMs: [toRPM(tFL), toRPM(tFR), toRPM(tRL), toRPM(tRR)],
        pidOutputs: { roll: rollTorque, pitch: pitchTorque, yaw: yawTorque },
      },
    };
  }
}
