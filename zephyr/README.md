# DroneCtrl on Zephyr

The same control core that runs on the ESP32, running on an STM32 Cortex-M4F under
Zephyr. One copy of the algorithm, built twice.

This was possible in a weekend because of a decision already made in this repository:
the control, estimation and safety code is header-only and carries no Arduino
dependency. That is what lets the host tests link it with no board attached, and it is
the same property that lets Zephyr compile it untouched. Most firmware cannot be ported
this way.

## What runs where

| Class | Header | In the Zephyr build |
|---|---|---|
| `AttitudeEKF` | `attitude_ekf.h` | used as-is |
| `PIDController` | `pid_controller.h` | used as-is |
| `mixer::mixX` | `mixer_math.h` | used as-is |
| `FlightStateMachine` | `flight_state.h` | used as-is |
| `IMU` | `imu.h` | replaced. It owns Wire register reads *and* wraps the EKF, so Zephyr reads the MPU6050 over `i2c_dt_spec` and drives `AttitudeEKF` directly |
| `MotorMixer` | `motor_mixer.h` | replaced. Only its `writeMotors` is Arduino-bound, and `mixer_math.h` already holds the arithmetic, so this build calls `mixX` and writes PWM itself |

Four of six used unchanged. The two that are replaced are replaced because they mix
platform and logic, and in both cases the logic was already separated out into a pure
header, which is why the replacement is small.

## One thing to fix upstream

`firmware/include/config.h` cannot be included from a Zephyr build as it stands, because
of one line:

```cpp
constexpr uint8_t PIN_BATTERY = A0;   // A0 is an Arduino macro
```

Everything else in that file is pure tuning data. The nine PID gains and the ESC limits
are currently **mirrored** in `src/control_binding.cpp`, and mirrored constants are a
liability: change a gain in `config.h` and you must change it there in the same commit.

The clean fix is to split the file:

- `config.h` keeps the tuning constants and stays platform-free
- `board_pins_arduino.h` takes `PIN_MOTOR_*`, `PIN_RX_*` and `PIN_BATTERY`

After that `control_binding.cpp` includes `config.h` like any other consumer and the
duplicated block is deleted.

## Hardware

- **Nucleo-F411RE**, around 15 to 20 USD. The ST-LINK is on the board, so SWD needs no
  separate probe and `west debug` works out of the box.
- The **MPU6050** you already use, on I2C1: SCL to PB8, SDA to PB9.
- Four ESC signal lines on PA6, PA7, PB0, PB1. **Bench test with the propellers off.**

## Build

```sh
west init -l .          # once, if this is not already a west workspace
west update
west build -b nucleo_f411re zephyr
west flash
west debug              # OpenOCD over the on-board ST-LINK
```

## State of this port

Done and working: the 250 Hz drift-free timer, the MPU6050 driver over Zephyr I2C with a
single 14-byte burst read, the estimator and PID and mixer wiring, four PWM channels on
one timer, the IWDG fed only after a complete step, and overrun counting.

Not wired yet, and deliberately safe until it is: the **receiver** and the **battery
ADC**. `control_binding.cpp` holds `in.signal_lost = true` while they are missing, so the
state machine cannot leave `DISARMED` and a bench board cannot spin a motor. Both TODOs
are marked in that file.

## How to know the port is right, not just compiling

A port that merely builds is worth nothing as a claim. Two checks make it evidence:

1. **Run the 53 host tests against this build's core.** They already link the pure
   headers with no board. If they pass for both builds, the two platforms are provably
   running the same control law.
2. **Watch the overrun counter.** `main.c` logs it once per second, and a clean loop
   reports zero. Any non-zero count on an idle bench means a step is not finishing inside
   4 ms, and gains derived for a 4 ms period no longer describe what is flying.
