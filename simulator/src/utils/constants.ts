import { Vector3 } from 'three';
import { PIDGains } from '../types';

export const DEFAULT_ROLL_PID: PIDGains = { kp: 1.5, ki: 0.05, kd: 0.8 };
export const DEFAULT_PITCH_PID: PIDGains = { kp: 1.5, ki: 0.05, kd: 0.8 };
export const DEFAULT_YAW_PID: PIDGains = { kp: 2.0, ki: 0.1, kd: 1.0 };

export const PHYSICS = {
  mass: 1.5, // kg
  armLength: 0.225, // m
  gravity: 9.81, // m/s^2
  drag: 0.1,
  maxThrustPerMotor: 10, // N
  torqueCoefficient: 0.05,
};

export const CHECKPOINTS: Vector3[] = [
  new Vector3(0, 5, -20),
  new Vector3(15, 10, -40),
  new Vector3(30, 8, -60),
  new Vector3(10, 15, -80),
  new Vector3(-20, 20, -70),
  new Vector3(-40, 12, -40),
  new Vector3(-20, 5, -10),
  new Vector3(10, 10, 10),
  new Vector3(30, 15, 30),
  new Vector3(0, 25, 50),
];

export const CAMERA_OFFSETS = {
  chase: new Vector3(0, 2, 5),
  fpv: new Vector3(0, 0.1, -0.2),
  topDown: new Vector3(0, 15, 0),
};
