import { Vector3 } from 'three';

export type FlightMode = 'MANUAL' | 'AUTO';

/**
 * Stick/controller inputs. `roll` and `pitch` are attitude setpoints in
 * radians, `yaw` is a rate setpoint in rad/s and `throttle` is per-motor
 * thrust in newtons. Manual input and the autopilot both produce this shape,
 * so the physics engine cannot tell which one is flying.
 */
export interface ControlInputs {
  throttle: number;
  roll: number;
  pitch: number;
  yaw: number;
}

export interface DroneState {
  position: Vector3;
  rotation: Vector3; // Euler angles in radians (pitch, yaw, roll)
  velocity: Vector3;
  motorSpeeds: [number, number, number, number]; // FL, FR, RL, RR
  armed: boolean;
  mode: FlightMode;
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

export interface AutopilotStatus {
  engaged: boolean;
  waypointIndex: number;
  lap: number;
  distanceToWaypoint: number;
  altitudeError: number;
  targetAltitude: number;
  phase: 'IDLE' | 'CLIMB' | 'CRUISE' | 'APPROACH' | 'LOITER';
}

export interface TelemetryData {
  altitude: number;
  verticalSpeed: number;
  speed: number;
  heading: number;
  battery: number;
  roll: number;
  pitch: number;
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
  lap: number;
  crashCount: number;
  windEnabled: boolean;
  showTelemetry: boolean;
  showPID: boolean;
  showHelp: boolean;
  flightMode: FlightMode;
  cameraMode: 'chase' | 'top-down' | 'fpv' | 'cinematic';
}

export type KeyState = Record<string, boolean>;
