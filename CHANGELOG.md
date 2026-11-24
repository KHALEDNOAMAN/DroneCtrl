# Changelog
All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- **Hardware in the loop over MAVLink.** An `esp32_hil` build runs the flight
  loop on the board against a simulator's IMU and motors (`HIL_SENSOR` in,
  `HIL_ACTUATOR_CONTROLS` out, sticks as `RC_CHANNELS_OVERRIDE`), in
  lockstep. The browser simulator connects over WebSerial and flies a port of
  the SIL airframe. `sil_flight --hil-board` is a virtual board over a pipe,
  and `npm run hil-check` flies the simulator against it in CI.
  `make hil-loopback` flies every SIL scenario through the encoded byte
  stream and compares it with the direct run. See `docs/hil.md`.
- **MAVLink v2 telemetry** (`mavlink_lite.h`, `mavlink_telemetry.h`):
  HEARTBEAT, ATTITUDE, SYS_STATUS and NAMED_VALUE_FLOAT, readable by
  QGroundControl. Checked byte for byte against pymavlink in the unit tests,
  and in both directions over every SIL log by `tools/mavlink_crosscheck.py`.
  The SIL harness writes a `.tlog` per scenario. See `docs/mavlink.md`.
- **Loop timing measurement** (`loop_timing.h`): period, jitter, execution
  time and missed deadlines, reported over MAVLink.
- **FreeRTOS control task on the ESP32**, released every 4 ms by
  `vTaskDelayUntil` above the telemetry task's priority.
- **Go ground station** (`groundstation/`): MAVLink over UDP or `.tlog`
  replay, live dashboard, JSON, Server-Sent Events, Prometheus metrics.
  Distroless container, Kubernetes manifests under the restricted Pod
  Security Standard, deployed to a kind cluster in CI.
- `flight_loop.h`: the control sequence, now shared by `main.cpp`, the SIL
  harness and the HIL build instead of being kept in step by comments. All
  SIL and Monte Carlo outputs are byte identical before and after.
- A C++11 check on every header the board compiles, because the AVR
  toolchain is C++11 and the test suites are C++17.
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
- **Signal loss in flight cut the motors instead of landing.** The failsafe
  descent started from the throttle on the failing step, and the receiver
  reports zero throttle with a lost signal, so the ramp began at zero and the
  state machine went straight to `FAILSAFE_CUT`. Now starts from the last
  healthy throttle. Found by the HIL loopback; the SIL scenario and the unit
  tests had kept hover throttle on the lost step, which no receiver does.
- **The control loop could not run at 250 Hz.** The receiver read four
  channels with sequential `pulseIn()` calls, which block for most of a 20 ms
  receiver frame. Replaced with pin-change interrupts and a host-tested
  decoder (`rc_pulse.h`) with the same signal-loss policy.
- **The ESP32 build used the Nano's pin numbers**, including GPIO 6 and 9,
  which are the module's SPI flash. It now has its own pin map.
- **Battery voltage read about six times high on the ESP32**, because the ADC
  was scaled for the Nano's 5 V, 10 bit converter. A low battery would never
  have tripped the failsafe there.
- `docs/failsafe_modes.md` described return to launch, GPS altitude hold and a
  geofence, none of which exist. Rewritten to match `flight_state.h`.
- `docs/wiring_diagram.md` showed a 10K/4.7K battery divider that disagreed
  with the code and would put about 4 V on an ESP32 ADC pin.
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
