# MATLAB model

A second implementation of the same airframe, estimator, controller and mixer,
in MATLAB. Runs unmodified in [GNU Octave](https://octave.org), which is free,
so the checks below run in CI without a MATLAB licence.

## Why have it twice

The firmware is written for an ATmega328 with 2 kB of RAM and no matrix
library, so `attitude_ekf.h` hand-rolls every product into fixed-size arrays.
That is the right call for the target and the wrong form for reading a
derivation. This version is the readable one: the EKF is four lines of matrix
algebra, a pole can be moved in one edit, and the gain derivation is a script
you can run rather than a comment you have to trust.

Having both only pays off if they are checked against each other, which is
what `verify_mil_sil.m` does.

## The three scripts

```bash
octave-cli tune_gains.m       # derive the PID gains from the airframe
octave-cli verify_mil_sil.m   # check this model against the C++ flight code
```

### `tune_gains.m`

Identifies the plant numerically, then places the closed-loop poles.

Identification is measured from `quad_step.m` rather than derived on paper, so
an error in the model shows up here instead of being reproduced by an equation
written to match it. Two things have to be kept out of the measurement:

- **Motor lag.** The ESC and propeller take about 30 ms to reach a commanded
  thrust, so the first few steps measure the actuator, not the airframe. The
  motors are pre-loaded with the thrust the command implies.
- **Aerodynamic damping.** The torque is `K*cmd - damping*p`, so any
  measurement taken after the rate has built up reads low. Taking it at
  `p = 0` removes the term.

Getting that wrong is not hypothetical: measuring the slope a second into the
response gives 2.5 instead of 13.3, because by then the airframe has rolled
145 degrees and left the linear regime entirely. The script cross-checks the
measured gain against the closed form and errors out if they disagree by more
than 2 percent.

Output for the airframe in `quad_init.m`:

```
  roll command gain   13.26 deg/s^2 per microsecond (measured)
  closed form         13.26 deg/s^2 per microsecond
  aerodynamic damping 1.82 1/s

  PID_ROLL_KP = 2.72f
  PID_ROLL_KI = 1.81f
  PID_ROLL_KD = 0.54f
```

Those are the numbers in `firmware/include/config.h`, reached independently.

### `run_mil.m`

Model in the loop: plant, estimator, controller and mixer all in MATLAB, run
as one closed loop with the same pilot script the C++ harness uses.

### `verify_mil_sil.m`

The equivalence check, and the reason the rest of this directory exists.

Both sides are driven by **the same pseudo-random generator**, reimplemented
in `lcg_uniform.m` to match the C++ one bit for bit, so they see identical
noise sample for sample. Any divergence is then a difference in the models
rather than a difference in the dice.

```
  hover        pass
      true roll  rms 0.0340 deg, max 0.2521 deg
      estimate   rms 0.0139 deg, max 0.1584 deg
  gyro_bias    pass
      true roll  rms 0.0225 deg, max 0.2437 deg
      estimate   rms 0.0163 deg, max 0.3379 deg
  roll_step    pass
      true roll  rms 0.0340 deg, max 0.2521 deg
      estimate   rms 0.0139 deg, max 0.1584 deg
```

**It found a real error on its first run.** The MATLAB model had the yaw
setpoint hardcoded to zero instead of taking the stick. That matters because
the arming gesture holds full yaw for the first two and a half seconds, so the
model behaved differently from the aircraft during exactly the moment the
motors come up. Under a gyro bias the two diverged by 8 degrees; after the
fix, by 0.34.

Worth noting that `hover` and `roll_step` passed with that bug present, at
0.31 degrees. One scenario would not have caught it.

### What this check cannot do

It catches a transcription error in either direction, a sign fixed on one side
only, a gain changed in `config.h` but not in the model, a Jacobian term wrong
in one implementation. It cannot catch an error both implementations share:
two copies of the same misunderstanding agree perfectly.

This checks consistency. Correctness is what the unit tests in
`firmware/test` check, by driving the estimator from a synthetic truth
trajectory where the right answer is known independently.

## Files

| File | Purpose |
| :--- | :--- |
| `quad_init.m`, `quad_step.m` | rigid body plant, same parameters as the C++ harness |
| `ekf_init.m`, `ekf_seed.m`, `ekf_predict.m`, `ekf_update_accel.m` | the attitude filter in matrix form |
| `pid_init.m`, `pid_step.m` | the control law |
| `mix_x.m` | saturation-aware mixer |
| `lcg_init.m`, `lcg_uniform.m`, `lcg_gauss.m` | the C++ harness's generator, reimplemented |
| `run_mil.m` | the closed loop |
| `tune_gains.m` | plant identification and pole placement |
| `verify_mil_sil.m` | the equivalence check |

## Scope

No Simulink model here. Everything is plain `.m`, so it runs in Octave and
stays reviewable in a diff, which a binary `.slx` would not be. The same plant
and controller port to Simulink blocks directly if that is wanted.

The fault-injection scenarios live on the C++ side alone: the failsafe table
is discrete logic, and the unit tests already cover it branch by branch.
