import { Vector3 } from 'three';
import { PIDGains } from '../types';

/**
 * Inner attitude loop. Shared by manual flight and the autopilot.
 *
 * Roll and pitch run in angle mode: the setpoint is a bank angle and the
 * derivative term sees angular rate, so a large Kd is the right damping.
 *
 * Yaw runs in rate mode: the setpoint is already a rate, so its derivative term
 * sees angular acceleration, which responds to the controller's own output
 * within a single step. A large Kd there makes the loop chase its own tail and
 * the airframe simply refuses to turn, so yaw is P-heavy with almost no D.
 */
export const DEFAULT_ROLL_PID: PIDGains = { kp: 1.5, ki: 0.05, kd: 0.8 };
export const DEFAULT_PITCH_PID: PIDGains = { kp: 1.5, ki: 0.05, kd: 0.8 };
export const DEFAULT_YAW_PID: PIDGains = { kp: 4.0, ki: 0.2, kd: 0.04 };

/** Angular-rate damping, expressed as a decay rate so it is timestep independent. */
export const ANGULAR_DAMPING = 3.08; // 1/s, matches the original 0.95 per 60 Hz step

export const PHYSICS = {
  mass: 1.5, // kg
  armLength: 0.225, // m
  gravity: 9.81, // m/s^2
  drag: 0.1,
  maxThrustPerMotor: 10, // N
  torqueCoefficient: 0.05,
};

/** Thrust each motor must hold to hover, in newtons. */
export const HOVER_THRUST_PER_MOTOR = (PHYSICS.mass * PHYSICS.gravity) / 4;

/** Outer guidance and velocity loops. See engine/Autopilot.ts. */
export const AUTOPILOT = {
  /** Climb straight up to this altitude before accepting horizontal guidance. */
  takeoffAltitude: 6, // m

  /** Target ground speed on the straight sections of a leg. */
  cruiseSpeed: 12, // m/s
  /** Start easing off cruise speed once inside this distance from the gate. */
  approachRadius: 20, // m
  /** A gate counts as captured inside this radius, widened by current speed. */
  captureRadius: 4, // m
  captureSpeedGain: 0.2, // s

  /** Velocity error -> acceleration command. */
  velKp: 1.2,
  /** Ceiling on commanded horizontal acceleration (tan(maxTilt) * g). */
  maxHorizontalAccel: 6, // m/s^2
  /** Ceiling on commanded bank angle, about 34 degrees. */
  maxTilt: 0.6, // rad
  /** How fast the commanded bank angle may change, so the inner loop sees a ramp. */
  attitudeSlewRate: 2.5, // rad/s

  /** Altitude hold: metres of error -> commanded vertical acceleration. */
  altKp: 2.0,
  altKi: 0.4,
  altKd: 2.2,
  altIntegralMax: 4,
  maxVerticalAccel: 7, // m/s^2

  /** Heading error -> commanded yaw rate. */
  yawKp: 2.0,
  maxYawRate: 1.8, // rad/s
  /** Inside this radius the bearing to the gate gets noisy, so hold heading. */
  headingHoldRadius: 8, // m
};

/**
 * Race circuit. A closed loop, so the autopilot demo flies laps indefinitely
 * instead of parking on the last gate. Altitude rises and falls around the lap
 * so the climb and descent behaviour of the altitude hold is visible.
 */
export const CHECKPOINTS: Vector3[] = [
  new Vector3(52, 9, 2),
  new Vector3(42, 14, -30),
  new Vector3(16, 19, -46),
  new Vector3(-14, 21, -44),
  new Vector3(-43, 16, -31),
  new Vector3(-54, 11, 2),
  new Vector3(-41, 8, 31),
  new Vector3(-14, 12, 47),
  new Vector3(18, 18, 44),
  new Vector3(44, 14, 28),
];

/** Outer radius of a gate torus, used for both rendering and the HUD. */
export const GATE_RADIUS = 3.5;

/**
 * Camera offsets from the airframe, in metres.
 *
 * The airframe is only ~0.7 m across and the world it flies through is hundreds
 * of metres wide, so these have to be tight or the drone reads as a speck. At
 * the 2 m the chase view used to sit at, once follow lag was added, it filled
 * under 6% of the frame.
 */
export const CAMERA_OFFSETS = {
  // Slightly off the centreline: a camera directly behind the drone hides its
  // own motion trail behind the airframe.
  chase: new Vector3(0.4, 0.28, 1.05),
  fpv: new Vector3(0, 0.12, -0.22),
  topDown: new Vector3(0, 30, 0.01),
};

/** Slow orbit used by the cinematic camera. */
export const CINEMATIC = {
  radius: 2.1,
  height: 0.75,
  orbitSpeed: 0.22, // rad/s
};
