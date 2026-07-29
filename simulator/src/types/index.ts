import { Vector3 } from 'three';

export interface DroneState {
  position: Vector3;
  rotation: Vector3; // Euler angles in radians (pitch, yaw, roll)
  velocity: Vector3;
  motorSpeeds: [number, number, number, number]; // FL, FR, RL, RR
  armed: boolean;
  mode: 'manual' | 'hover' | 'waypoint';
}

export interface PIDGains {
  kp: number;
  ki: number;
  kd: number;
}

export interface PIDState {
  integral: number;
  prevError: number;
  output: number;
}

export interface TelemetryData {
  altitude: number;
  speed: number;
  heading: number;
  battery: number;
  motorRPMs: [number, number, number, number];
  pidOutputs: {
    roll: number;
    pitch: number;
    yaw: number;
  };
}

export interface GameState {
  score: number;
  checkpointsHit: number;
  crashCount: number;
  windEnabled: boolean;
  showTelemetry: boolean;
  showPID: boolean;
  cameraMode: 'chase' | 'top-down' | 'fpv';
}

export type KeyState = Record<string, boolean>;
