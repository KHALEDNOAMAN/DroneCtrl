# State estimation

Attitude comes from a four-state extended Kalman filter in
[`firmware/include/attitude_ekf.h`](../firmware/include/attitude_ekf.h). This
page covers what it estimates, why it is built the way it is, and what it does
not do.

## Why not a complementary filter

The previous implementation blended gyro and accelerometer with one fixed
weight, `alpha = 0.98`. That weight is a permanent compromise. High enough to
reject accelerometer noise means slow correction of gyro drift; low enough to
track drift means every bump in the airframe leaks into the attitude. There is
also nowhere to put gyro bias, so a warm, drifting gyro is only corrected
indirectly through the same fixed blend.

Two honest numbers, both measured by
[`test_estimation.cpp`](../firmware/test/test_estimation/test_estimation.cpp):

- **Static, clean accelerometer, 2 deg/s of gyro bias.** The complementary
  filter settles about 0.4 degrees off. That is not a disaster, and the test
  says so rather than overstating the case.
- **Ten seconds with the accelerometer unusable.** On a quadcopter that is any
  aggressive manoeuvre, because the sensor then measures thrust rather than
  gravity. The complementary filter has no memory of the bias, so it integrates
  raw gyro and walks off by about 20 degrees. The EKF, carrying bias as a
  state, stays inside 3.

The second case is the one that matters, and it is the reason for the change.

## State vector

```
x = [ roll, pitch, bias_p, bias_q ]        radians, radians/second
```

Bias is modelled as a random walk.

**Yaw is deliberately absent.** A single accelerometer observes gravity, and
gravity says nothing about heading, so a yaw state here would be unobservable
and would drift without bound. Yaw on this airframe is flown as a rate, taken
straight from the gyro. Adding a magnetometer is what would make a yaw state
observable, and that is the honest prerequisite for adding one.

## Process model

Euler-angle kinematics driven by bias-corrected body rates:

```
roll_dot  = p' + q' sin(roll) tan(pitch) + r cos(roll) tan(pitch)
pitch_dot = q' cos(roll) - r sin(roll)
bias_dot  = 0
```

with `p' = p - bias_p` and `q' = q - bias_q`. The transition matrix is
`Phi = I + F dt`, first order. At 250 Hz the neglected term is of order
`dt^2 = 1.6e-5`, far below the process noise, so the second-order term would be
arithmetic without effect.

`tan(pitch)` is unbounded at 90 degrees, so pitch is clamped just short of it.
That clamp is what stops a tumbling airframe from taking the filter to
infinity, and it is covered by `test_ekf_bounded_near_vertical`.

## Measurement model

The accelerometer is treated as an observation of the gravity vector in body
axes rather than being pre-converted into angles:

```
h(x) = g * [ -sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch) ]
```

Computing `atan2` angles first and feeding those in would bake in a
linearisation the filter never sees, and it throws away the vector's length,
which is exactly what the next section is built on.

## Accelerometer trust

The measurement model assumes the only specific force is gravity. On a
quadcopter that is false most of the time. The length of the measured vector is
the evidence available: when it departs from 1 g, something other than gravity
is being measured, so `R` is inflated in proportion:

```
R = accel_noise * (1 + k * abs(norm - 1g))^2
```

The update is de-weighted rather than dropped. Dropping it outright would throw
away the partial information in a mildly disturbed sample and would make the
filter's behaviour discontinuous at the threshold.

`test_ekf_rejects_acceleration_transient` holds the airframe level, applies
half a g sideways for a second with the gyro correctly reporting no rotation,
and requires the estimate to stay inside 8 degrees. Trusting that sample
blindly would read as `atan2(0.5, 1)`, or 26.6 degrees of phantom tilt, and the
controller would dutifully correct an attitude error that was never there.

## Numerical choices

- **Joseph form** for the covariance update. The textbook `(I - KH)P` is
  cheaper but loses symmetry and positive-definiteness to rounding, which on
  32-bit floats at 250 Hz shows up as a filter that quietly stops correcting
  after a few minutes. `test_ekf_stays_numerically_healthy` runs five simulated
  minutes and checks the covariance is still finite and bounded.
- **`P` is forced symmetric** every step, because rounding makes the two
  triangles drift apart.
- **No dynamic allocation, no library.** Every matrix is a fixed-size float
  array, so the same file compiles for the ATmega328 and the ESP32 and links
  into the host tests and the SIL harness unchanged. The largest temporary is
  the 4x3 gain, so an update runs in roughly 200 bytes of stack.

## Health signals

The filter exposes three things the flight code and the logs both use:

| Signal | Used for |
| --- | --- |
| `getRollSigma()` | the filter's own uncertainty, which should shrink once data arrives |
| `getInnovation(axis)` | the accelerometer residual, the standard way to tell a tuning problem from a sensor problem |
| `isDiverged()` | any non-finite state or covariance entry |

`isDiverged()` is wired straight into the flight state machine, which treats it
as unrecoverable and cuts power. If attitude is unknown, every actuator command
after that point is a guess, and a controlled descent is not available.

## Two sign errors that cancelled

Worth recording, because it is the clearest argument in this repository for
having tests at all.

The original code assigned `atan2(-ax, ...)` to roll. That expression is pitch.
The gyro integration had the same transposition, propagating pitch with the
roll rate. Separately, the motor mixer raised the right pair for a positive
roll command, which is the opposite of the convention where positive roll is
right side down.

Either mistake alone would have made the airframe uncontrollable. Together they
cancelled, so the code looked correct. Both are fixed, and each now has a unit
test of its own so they cannot drift back independently:
`test_mix_roll_is_antisymmetric` and the axis convention stated at the top of
[`imu.h`](../firmware/include/imu.h).

## Verification

Fourteen host tests in
[`test_estimation.cpp`](../firmware/test/test_estimation/test_estimation.cpp),
all driven from a synthetic truth trajectory so there is a known answer to
compare against. The generator uses the exact Euler kinematics rather than the
filter's linearised copy of them, which is what makes the tests capable of
catching an error in the Jacobian instead of reproducing it. Noise comes from a
fixed-seed generator, so a failure reproduces byte for byte.

Beyond that, the filter is flown closed-loop in
[`firmware/sil`](../firmware/sil), where its output drives the real controller
rather than being scored in isolation.
