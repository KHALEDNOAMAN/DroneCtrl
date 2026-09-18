# Verification

How this firmware is checked, and what each layer can and cannot catch.

The organising idea is that **the control, estimation and safety logic contains
no Arduino dependency**. `attitude_ekf.h`, `pid_controller.h`, `mixer_math.h`,
`flight_state.h` and `can_telemetry.h` are arithmetic and state; hardware
access lives in thin wrappers that include them. That separation is what lets
the tests and the simulator link *the same source the flight controller runs*,
rather than a reimplementation of it. A control law verified in a separate
implementation has only been verified as a separate implementation, and the two
always drift.

## The layers

| Layer | What it runs | Catches | Runtime |
| --- | --- | --- | --- |
| Host unit tests | `firmware/test` | arithmetic, boundaries, state transitions | ~2 s |
| SIL | `firmware/sil` | closed-loop behaviour, stability, failsafes | ~3 s |
| Target build | `pio run` | anything that only fails on AVR or ESP32 | ~1 min |
| Bench | a person | everything the model does not contain | not automated |

All three automated layers run on every push.

## Host unit tests

```
cd firmware/test && make run
```

Plain `g++`, no dependencies. That is deliberate: a suite that needs a package
registry to be reachable stops running the first time that registry is down or
blocked, and a safety test nobody can run is not a safety test. The harness in
`test_harness.h` uses Unity's macro names, so the files drop into a PlatformIO
`pio test` setup unchanged if that is ever wanted.

Three suites, 53 tests. `make run` prints the count and the per-suite pass/fail, so the number here is checkable rather than asserted:

- **`test_control`** covers the PID and the mixer. Includes regression tests for
  a derivative kick on the first call after arming, a derivative filter whose
  cutoff moved with the loop rate, and an exhaustive sweep asserting no mix of
  any commands can put an output outside the ESC range.
- **`test_estimation`** covers the EKF against a synthetic truth trajectory.
  See [state_estimation.md](state_estimation.md).
- **`test_safety`** covers the arming gesture, every failsafe branch, the
  latch, and the CAN telemetry encoding.

**This layer found a real defect.** `test_fault_on_ground_blocks_arming` failed
on first run: clearing the state on a fault was not enough on its own, because
the arm gesture was still being held and the next line walked straight back
into `ARMING`. An airframe with a flat battery would have armed. The fix is in
`flight_state.h` and the test is named after it.

## Software in the loop

```
cd firmware/sil && make run
```

Flies the control loop against a rigid body model: three-axis rotation with
inertia and aerodynamic damping, first-order motor lag, translation with linear
drag, per-motor thrust scaling, gyro bias, white noise on both sensors, and
receiver dropout. Nine scenarios, each with pass criteria:

| Scenario | Checks |
| --- | --- |
| `hover` | stays armed and level, actuators not on a rail |
| `gyro_bias` | 3 deg/s of bias is estimated to within 1 deg/s |
| `wind_disturbance` | a constant roll torque is rejected |
| `roll_step` | a 15 degree command is tracked |
| `motor_degraded` | one motor at 70 percent, airframe stays controlled |
| `signal_loss` | descends under control, does not cut power |
| `low_battery` | same, with the right fault reported |
| `imu_corruption` | garbage sensor data is detected and power is cut |
| `legacy_gains_rejected` | the pre-tuning gains are still rejected |

That last row is a scenario that **passes by failing**. If someone reintroduces
the old gains and the harness stops objecting, it turns red.

### What the model does not contain

Blade flapping, ground effect, propeller inflow, battery sag under load, ESC
nonlinearity, structural flex, wind gradients. This is a model good enough to
catch a sign error, an unstable gain, a windup bug or a failsafe that does not
fire. It is not good enough to predict flight time or to tune gains for a
specific airframe, and nothing here should be read as claiming otherwise.

### Hardware in the loop

Not yet built. The step is to run this same loop on the board with the sensor
reads replaced by samples injected over serial, which is why the harness keeps
its sensor source behind one boundary. Saying what is missing is more useful
than implying it is there.

## What SIL found

The harness was written against the gains the firmware already shipped:
`Kp 1.2, Ki 0.04, Kd 15`. It rejected them.

Mean attitude looked acceptable, which is exactly what makes this failure mode
easy to miss. The actuators told the real story: with that derivative gain the
loop chased its own measurement noise and all four outputs chattered between
1000 and 2000 microseconds continuously, leaving no authority for a real
disturbance. A 30 percent loss on one motor put the airframe past 45 degrees
and into a failsafe cut within six seconds.

![ESC commands before and after retuning](../assets/sil-gain-fix.png)

The criterion that caught it is in the harness: **more than 2 percent of steady
flight with any output sitting at an ESC limit is a failure**, however good the
attitude looks. Attitude staying small is not on its own evidence of a stable
loop.

The replacement gains are derived rather than guessed. For the modelled
airframe, one microsecond of roll command produces about 13.3 deg/s² of angular
acceleration, and the airframe contributes about 1.8 1/s of its own damping.
Treating the inner loop as second order:

```
omega_n^2      = 13.3 * Kp
2*zeta*omega_n = 13.3 * Kd + 1.8
```

Solving for `omega_n = 6 rad/s` and `zeta = 0.75` gives `Kp = 2.7` and
`Kd = 0.55`. `Ki = 1.8` follows from an integral time of about 1.5 s.

Measured results after the change:

| | Before | After |
| --- | --- | --- |
| Hover ESC range | 1000 to 2000 us | 1398 to 1419 us |
| Steady flight on a rail | continuous | 0 of 2796 samples |
| Roll RMS in hover | n/a, unstable | 0.11 deg |
| Roll step, 15 deg | not tracked | 15.0 deg, 15 percent overshoot, 0.38 s to 90 percent |
| One motor at 70 percent | past 45 deg in 6 s | peak 0.7 deg |

**These gains are tuned for the simulated airframe**, a 1 kg quad with a 0.15 m
arm and 6 N motors. They are a defensible starting point, not a substitute for
bench tuning on real hardware. Change the airframe and the derivation has to be
redone with its numbers.

## Target build

`pio run` builds both environments. It is the only layer that catches
AVR-specific and ESP32-specific problems: a missing `lib_deps` entry, a header
that exists on one core and not the other, or a global whose name collides with
something a core header declares.

That last one is not hypothetical. Splitting the mixer arithmetic into a
`mixer` namespace collided with a global named `mixer` in `main.cpp`, and the
host tests could not see it because they never compile that file. The global is
now `motors`.

## Flight logs

Every SIL scenario writes a CSV to `firmware/sil/logs/`, and CI keeps them as
build artifacts. `tools/plot_flight_logs.py` turns them into the figures in the
README: estimate against truth, estimator health, the failsafe timeline, and
the actuator traces above.

Reading a flight log is most of what post-flight analysis is, so the logged
fields are chosen to support it rather than to look complete: truth and
estimate side by side, the bias estimate, the accelerometer innovation, all
four ESC commands, and the state machine's state and fault as integers.

## Running everything

```
cd firmware/test && make run     # unit tests
cd firmware/sil  && make run     # SIL scenarios
cd firmware      && pio run      # both target builds
python3 tools/plot_flight_logs.py  # regenerate the figures
```
