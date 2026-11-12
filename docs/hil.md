# Hardware in the loop

**Motor order is DroneCtrl's own, not PX4's.** `HIL_ACTUATOR_CONTROLS`
carries front left, front right, back left, back right in `controls[0..3]`,
normalised so 0 is `ESC_MIN` and 1 is `ESC_MAX`. A simulator other than the
ones in this repository has to map that.

## What it is

In the HIL build the flight controller runs on the real ESP32, but its IMU
and its motors belong to a simulator. Every 4 ms of simulated time the
simulator sends one `HIL_SENSOR` sample and the sticks as
`RC_CHANNELS_OVERRIDE`; the board runs one `FlightLoop` step on them and
replies with `HIL_ACTUATOR_CONTROLS`; only then does the simulator advance
its physics. That is lockstep: a slow link slows the simulation down instead
of starving the controller of samples.

What HIL adds over SIL is the board: its clock, its floating point unit, its
serial driver, and the time the step really takes on it. The controller code
is identical in SIL, HIL and flight, by construction: all three call
`FlightLoop` in `firmware/include/flight_loop.h`.

```text
 simulator (browser or host)                    ESP32, esp32_hil build
 ---------------------------                    ----------------------
 plant state
   -> IMU model -> HIL_SENSOR ---------------->  hil::Link (parser)
   sticks -> RC_CHANNELS_OVERRIDE ----------->     |
                                                  FlightLoop::step
 plant.step(pwm) <- HIL_ACTUATOR_CONTROLS <----    (EKF, state machine,
                                                    PIDs, mixer)
```

## Running it

1. Flash the HIL build. The ESC pins are never attached in it, so a board on
   the bench with propellers fitted cannot spin them whatever the simulator
   sends.

   ```
   cd firmware
   pio run -e esp32_hil -t upload
   ```

2. Open the simulator in Chrome or Edge (WebSerial is Chromium only), at
   [drone-ctrl.vercel.app](https://drone-ctrl.vercel.app) or `npm run dev`,
   and press **Connect ESP32** in the panel at the bottom of the screen.
   Pick the board's port.

3. Hold **X** for two seconds to arm (the firmware's gesture: throttle closed,
   yaw right). **Space** and **Shift** move the throttle stick, which stays
   where you leave it; **W A S D** command attitude; **Q E** yaw rate.
   Press **L** to cut the RC link and watch the failsafe fly it down.

The panel shows the board's flight state and fault, the round trip time per
step, timeouts, rejected frames, and the loop jitter the board measures on
itself.

## Without a board

`firmware/sil/sil_flight --hil-board` is a virtual board: it runs exactly what
`hilStep()` in `main.cpp` runs, with stdin and stdout in place of the serial
port. The simulator's CI drives it:

```
make -C firmware/sil sil_flight
cd simulator && npm run hil-check
```

`hil-check` arms the board from RC overrides, holds a level hover, tracks a
15 degree roll command, then stops sending sticks and requires the board to
report `FAILSAFE_LAND` and fly the airframe down rather than drop it.

## Which airframe the board flies

Not the simulator's own `DronePhysics`. The firmware's gains are derived for
the SIL airframe (1 kg, 0.15 m arm, 6 N motors; see `config.h`), and
`DronePhysics` is a different, heavier aircraft with its own controller.
Flying the board against it would test the gains on an airframe they were
never designed for. `simulator/src/hil/HilPlant.ts` is a port of the SIL
plant and IMU model, and the 3D scene draws it.

## Frames and units

MAVLink is body FRD (x forward, y right, z down) in SI units. The firmware's
rates already follow FRD: positive roll is right side down, positive pitch
nose up. Its accelerometer convention is the reaction one, +1 g on z at rest,
where an FRD accelerometer reads -9.81 m/s². So `hil_link.h` passes rates
straight through and negates and scales specific force. Two host tests pin
the signs by feeding a 10 degree pitch and a 10 degree roll through the
conversion and the EKF.

## Evidence

- `make -C firmware/sil hil-loopback` flies all nine SIL scenarios through the
  encoded byte stream as well as directly. Wherever the receiver stays up, the
  two agree to better than 0.0001 degrees of roll for the whole flight.
- **It found a real defect.** Losing the receiver in flight cut the motors
  instead of landing. See [verification.md](verification.md#what-hil-found).

## Limits

- The simulator has no battery model, so the board flies with a fixed
  12.4 V in HIL and the low battery failsafe does not trigger there. SIL
  covers it.
- `HIL_SENSOR` carries no magnetometer, barometer or GPS data from this
  simulator, because the firmware does not use them yet.
- HIL timing is the board's own, but paced by the simulator. The loop jitter
  the board reports in HIL measures the link as much as the board; for the
  control loop's own timing, read it from a normal build.
