import { Vector3 } from 'three';
import { PIDController } from './PIDController';
import { ControlInputs, AutopilotStatus } from '../types';
import { PHYSICS, AUTOPILOT } from '../utils/constants';
import { clamp, wrapAngle } from '../utils/helpers';

/**
 * Autopilot: cascaded guidance and control, the same structure real flight
 * stacks (PX4 / ArduPilot) use.
 *
 *   waypoint  ->  position error  ->  velocity setpoint     (guidance)
 *   velocity setpoint -> velocity error -> acceleration cmd (velocity loop)
 *   acceleration cmd  -> roll / pitch / thrust setpoints    (attitude mapping)
 *   attitude setpoints -> [existing attitude PIDs] -> motors
 *
 * Only the outer two loops live here. The attitude loop is the same PID stack
 * the pilot flies through in manual mode, so AUTO and MANUAL share one
 * controller and the tuning panel affects both.
 */
export class Autopilot {
  /** Index of the waypoint currently being chased. */
  waypointIndex = 0;
  /** Completed laps of the circuit. The route loops, so the demo never ends. */
  lap = 0;
  /** False until the vertical takeoff has cleared the ground. */
  private airborne = false;

  private altPID: PIDController;
  private route: Vector3[];

  // Smoothed attitude setpoints. Real autopilots rate-limit their attitude
  // commands so the airframe never sees a step input.
  private rollCmd = 0;
  private pitchCmd = 0;

  private lastStatus: AutopilotStatus = {
    engaged: false,
    waypointIndex: 0,
    lap: 0,
    distanceToWaypoint: 0,
    altitudeError: 0,
    targetAltitude: 0,
    phase: 'IDLE',
  };

  constructor(route: Vector3[]) {
    this.route = route;
    // Altitude hold: error in metres -> commanded vertical acceleration.
    this.altPID = new PIDController(
      AUTOPILOT.altKp,
      AUTOPILOT.altKi,
      AUTOPILOT.altKd,
      -AUTOPILOT.maxVerticalAccel,
      AUTOPILOT.maxVerticalAccel,
      AUTOPILOT.altIntegralMax,
    );
  }

  reset() {
    this.waypointIndex = 0;
    this.lap = 0;
    this.airborne = false;
    this.rollCmd = 0;
    this.pitchCmd = 0;
    this.altPID.reset();
  }

  /** Waypoint the autopilot is currently flying to. */
  get target(): Vector3 {
    const i = Math.min(this.waypointIndex, this.route.length - 1);
    return this.route[i];
  }

  get status(): AutopilotStatus {
    return this.lastStatus;
  }

  /**
   * Run one control step.
   *
   * @param position current world position
   * @param velocity current world velocity
   * @param yaw      current heading (rad)
   * @param dt       timestep (s)
   * @returns control inputs in exactly the form the manual stick mapping
   *          produces, so the physics engine cannot tell the two apart.
   */
  update(position: Vector3, velocity: Vector3, yaw: number, dt: number): ControlInputs {
    // --- Takeoff -------------------------------------------------------------
    // Climb straight up before accepting any horizontal guidance. Without this
    // the autopilot banks hard toward a gate 50 m away while still sitting on
    // its skids, and just scrapes along the ground.
    if (!this.airborne) {
      if (position.y >= AUTOPILOT.takeoffAltitude) {
        this.airborne = true;
      } else {
        const climbAccel = this.altPID.compute(AUTOPILOT.takeoffAltitude, position.y, dt);
        this.lastStatus = {
          engaged: true,
          waypointIndex: this.waypointIndex,
          lap: this.lap,
          distanceToWaypoint: position.distanceTo(this.target),
          altitudeError: AUTOPILOT.takeoffAltitude - position.y,
          targetAltitude: AUTOPILOT.takeoffAltitude,
          phase: 'CLIMB',
        };
        return {
          throttle: clamp(
            (PHYSICS.mass * (PHYSICS.gravity + climbAccel)) / 4,
            0,
            PHYSICS.maxThrustPerMotor,
          ),
          roll: 0,
          pitch: 0,
          yaw: 0,
        };
      }
    }

    const target = this.target;

    const toTarget = target.clone().sub(position);
    const horizontalError = new Vector3(toTarget.x, 0, toTarget.z);
    const horizontalDistance = horizontalError.length();
    const distance = toTarget.length();

    // --- Waypoint sequencing -------------------------------------------------
    // Capture on a sphere around the waypoint, and open that sphere up as speed
    // rises so a fast pass still counts instead of sending the drone into an
    // orbit it can never close.
    const captureRadius = AUTOPILOT.captureRadius + velocity.length() * AUTOPILOT.captureSpeedGain;
    if (distance < captureRadius) {
      this.waypointIndex += 1;
      if (this.waypointIndex >= this.route.length) {
        this.waypointIndex = 0;
        this.lap += 1;
      }
    }

    // --- Guidance: position error -> velocity setpoint ------------------------
    // Proportional guidance with a trapezoidal speed profile: full cruise speed
    // out on the leg, easing off inside the approach radius so the drone
    // arrives slow instead of overshooting and having to come back.
    // Never bleed all the way to zero: the route loops, so the drone should
    // carry speed through each gate rather than stopping on top of it.
    const approach = clamp(horizontalDistance / AUTOPILOT.approachRadius, 0.35, 1);
    const speedLimit = AUTOPILOT.cruiseSpeed * approach;

    const velocitySetpoint = horizontalDistance > 0.001
      ? horizontalError.clone().normalize().multiplyScalar(speedLimit)
      : new Vector3();

    // --- Velocity loop: velocity error -> acceleration command ----------------
    const velocityError = velocitySetpoint.sub(new Vector3(velocity.x, 0, velocity.z));
    const accelCommand = velocityError.multiplyScalar(AUTOPILOT.velKp);

    const accelMagnitude = accelCommand.length();
    if (accelMagnitude > AUTOPILOT.maxHorizontalAccel) {
      accelCommand.multiplyScalar(AUTOPILOT.maxHorizontalAccel / accelMagnitude);
    }

    // --- Attitude mapping: world acceleration -> body tilt --------------------
    // Rotate the commanded acceleration from world axes into the drone's
    // yaw-aligned body frame, then convert to bank angles. A multirotor
    // accelerates by tilting, so a = g * tan(theta); the small-angle form
    // a / g is accurate to under 4% at the 30 deg tilt limit.
    const cosYaw = Math.cos(yaw);
    const sinYaw = Math.sin(yaw);
    const accelForward = accelCommand.x * sinYaw + accelCommand.z * cosYaw;
    const accelRight = accelCommand.x * cosYaw - accelCommand.z * sinYaw;

    const rollTarget = clamp(-accelRight / PHYSICS.gravity, -AUTOPILOT.maxTilt, AUTOPILOT.maxTilt);
    const pitchTarget = clamp(accelForward / PHYSICS.gravity, -AUTOPILOT.maxTilt, AUTOPILOT.maxTilt);

    // Rate-limit the attitude setpoints so the inner loop never sees a step.
    const maxStep = AUTOPILOT.attitudeSlewRate * dt;
    this.rollCmd += clamp(rollTarget - this.rollCmd, -maxStep, maxStep);
    this.pitchCmd += clamp(pitchTarget - this.pitchCmd, -maxStep, maxStep);

    // --- Altitude hold --------------------------------------------------------
    const verticalAccel = this.altPID.compute(target.y, position.y, dt);

    // Thrust needed to hold altitude while banked: the vertical component of
    // thrust falls off as cos(roll) * cos(pitch), so divide it back out. Without
    // this the drone sinks every time it banks into a turn.
    const tiltCompensation = Math.max(
      0.5,
      Math.cos(this.rollCmd) * Math.cos(this.pitchCmd),
    );
    const throttle = clamp(
      (PHYSICS.mass * (PHYSICS.gravity + verticalAccel)) / 4 / tiltCompensation,
      0,
      PHYSICS.maxThrustPerMotor,
    );

    // --- Heading: point the nose down the track -------------------------------
    // Nose is -Z in body frame, so the heading that faces (dx, dz) is
    // atan2(-dx, -dz). Hold heading when close in, otherwise the bearing goes
    // wild as the horizontal error shrinks toward zero.
    let yawRate = 0;
    if (horizontalDistance > AUTOPILOT.headingHoldRadius) {
      const desiredYaw = Math.atan2(-horizontalError.x, -horizontalError.z);
      const yawError = wrapAngle(desiredYaw - yaw);
      yawRate = clamp(yawError * AUTOPILOT.yawKp, -AUTOPILOT.maxYawRate, AUTOPILOT.maxYawRate);
    }

    // --- Status for the HUD ---------------------------------------------------
    let phase: AutopilotStatus['phase'] = 'CRUISE';
    if (position.y < target.y - 3) phase = 'CLIMB';
    else if (horizontalDistance < AUTOPILOT.approachRadius) phase = 'APPROACH';

    this.lastStatus = {
      engaged: true,
      waypointIndex: this.waypointIndex,
      lap: this.lap,
      distanceToWaypoint: distance,
      altitudeError: target.y - position.y,
      targetAltitude: target.y,
      phase,
    };

    return {
      throttle,
      roll: this.rollCmd,
      pitch: this.pitchCmd,
      yaw: yawRate,
    };
  }

  /** Called when the pilot takes over, so the HUD stops showing AUTO data. */
  disengage() {
    this.lastStatus = { ...this.lastStatus, engaged: false, phase: 'IDLE' };
  }
}
