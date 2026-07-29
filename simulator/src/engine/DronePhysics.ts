import { Vector3, Euler } from 'three';
import { PIDController } from './PIDController';
import { DroneState, TelemetryData, PIDGains } from '../types';
import { PHYSICS } from '../utils/constants';

export class DroneSimulator {
  position = new Vector3(0, 0, 0);
  velocity = new Vector3(0, 0, 0);
  rotation = new Vector3(0, 0, 0); // pitch (x), yaw (y), roll (z)
  angularVelocity = new Vector3(0, 0, 0);
  
  motorThrusts: [number, number, number, number] = [0, 0, 0, 0];
  
  rollPID: PIDController;
  pitchPID: PIDController;
  yawPID: PIDController;

  constructor(rollGains: PIDGains, pitchGains: PIDGains, yawGains: PIDGains) {
    this.rollPID = new PIDController(rollGains.kp, rollGains.ki, rollGains.kd, -2, 2, 1);
    this.pitchPID = new PIDController(pitchGains.kp, pitchGains.ki, pitchGains.kd, -2, 2, 1);
    this.yawPID = new PIDController(yawGains.kp, yawGains.ki, yawGains.kd, -1, 1, 0.5);
  }

  update(dt: number, inputs: {throttle: number, roll: number, pitch: number, yaw: number}, windForce: Vector3): {droneState: DroneState, telemetry: TelemetryData} {
    if (dt > 0.1) dt = 0.1; // Cap dt for stability
    if (dt <= 0) dt = 0.016; // default 60fps

    // 1. PID Control to calculate required torques
    const rollTorque = this.rollPID.compute(inputs.roll, this.rotation.z, dt);
    const pitchTorque = this.pitchPID.compute(inputs.pitch, this.rotation.x, dt);
    const yawTorque = this.yawPID.compute(inputs.yaw, this.angularVelocity.y, dt); // Rate mode for yaw

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
    
    // Damping
    this.angularVelocity.multiplyScalar(0.95);

    // Update rotation
    this.rotation.x += this.angularVelocity.x * dt;
    this.rotation.y += this.angularVelocity.y * dt;
    this.rotation.z += this.angularVelocity.z * dt;
    
    // Calculate linear acceleration in world frame
    const euler = new Euler(this.rotation.x, this.rotation.y, this.rotation.z, 'YXZ');
    const thrustVector = new Vector3(0, totalThrust / PHYSICS.mass, 0).applyEuler(euler);
    
    const gravityForce = new Vector3(0, -PHYSICS.gravity, 0);
    const dragForce = this.velocity.clone().multiplyScalar(-PHYSICS.drag);
    const acceleration = new Vector3().add(thrustVector).add(gravityForce).add(dragForce).add(windForce.clone().divideScalar(PHYSICS.mass));
    
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

    return {
      droneState: {
        position: this.position.clone(),
        rotation: this.rotation.clone(),
        velocity: this.velocity.clone(),
        motorSpeeds: [tFL * 100, tFR * 100, tRL * 100, tRR * 100],
        armed: baseThrust > 0,
        mode: 'manual'
      },
      telemetry: {
        altitude: this.position.y,
        speed: this.velocity.length(),
        heading: this.rotation.y * (180/Math.PI),
        battery: Math.max(0, 100 - (totalThrust * 0.01)),
        motorRPMs: [tFL * 1000, tFR * 1000, tRL * 1000, tRR * 1000],
        pidOutputs: { roll: rollTorque, pitch: pitchTorque, yaw: yawTorque }
      }
    };
  }
}
