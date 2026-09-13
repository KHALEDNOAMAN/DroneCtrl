# Changelog
All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- **Waypoint autopilot** for the simulator. Cascaded guidance, velocity and
  attitude-mapping loops feed the existing attitude PIDs, so AUTO and MANUAL
  share one controller. Includes vertical takeoff, altitude hold with
  bank-angle thrust compensation, heading tracking and a looping race circuit.
  The simulator now starts flying on load; touching any flight control hands
  over to the pilot.
- **Headless flight check** (`npm run check` in `simulator/`). Runs the real
  physics and autopilot at a fixed timestep with no browser and no renderer,
  and fails on laps not completed, ground contact, runaway speed or tilt,
  heading not tracking the route, or results that change with the timestep.
  Wired into CI alongside a typecheck and build.
- Artificial horizon, autopilot status and mission readouts, live PID output
  bars, and a cinematic orbit camera in the HUD.

### Fixed
- **Yaw would not track.** The yaw loop runs on rate, so its derivative term
  saw angular *acceleration*, which responds to the controller's own output
  within one step. With the old `Kd = 1.0` the loop fought itself and the
  airframe barely turned at all. Retuned for rate mode and added a low-pass
  filter on the derivative term, which is what the firmware does for the same
  reason.
- **Flight behaviour depended on frame rate.** Physics ran on the raw frame
  delta, so the control loops went unstable on a slow machine and the drone
  fell out of the sky. Physics now runs on a fixed timestep with an
  accumulator.
- **Angular damping depended on the timestep**, applying a flat factor per step
  rather than decaying over elapsed time. Now exponential in `dt`.
- **Battery never drained.** It was computed from instantaneous thrust rather
  than integrated, so it sat at ~100% forever. Now models pack energy with
  propeller power scaling.
- **Motor RPM was linear in thrust.** Thrust goes with the square of rotor
  speed, so RPM now scales with its square root.
- Arm booms rendered as vertical posts: three.js cylinders run along +Y, so
  yawing one leaves it standing upright.
- Derivative term no longer spikes on the first call, when there is no previous
  measurement to difference against.
- Keyboard input is case-insensitive, so controls keep working with caps lock
  on or shift held.

### Changed
- Simulator re-rendered: dusk gradient sky, reworked lighting and terrain,
  oriented race gates, motion trail, and a rebuilt telemetry HUD.
- The 3D scene is now driven from refs rather than React state, and HUD updates
  are throttled, so a 60 fps sim no longer forces 60 renders a second of the
  whole component tree.
- README documents the live demo, the autopilot, and the new controls.
