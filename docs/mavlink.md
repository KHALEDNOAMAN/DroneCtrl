# MAVLink telemetry

The flight controller speaks MAVLink v2 on its USB serial port, so
QGroundControl, MAVProxy, pymavlink or the ground station in
[`groundstation/`](../groundstation) can see it without anything DroneCtrl
specific.

The codec is `firmware/include/mavlink_lite.h`: framing, the X.25 checksum
with CRC_EXTRA, trailing zero truncation, a resynchronising parser, and seven
messages from the common dialect. It has no Arduino dependency, so the host
tests and the SIL harness link the same code the board runs.

## What the board sends

| Message | Rate | Contents |
| --- | --- | --- |
| `HEARTBEAT` | 1 Hz | flight state and fault, armed flag, HIL flag |
| `ATTITUDE` | 25 Hz Nano, 50 Hz ESP32 | estimated roll and pitch, body rates |
| `SYS_STATUS` | 2 Hz | battery voltage, loop load, sensor health, missed deadlines |
| `NAMED_VALUE_FLOAT` | 10 Hz, one value per frame, cycling | loop timing and estimator health |

At the default rates the stream needs under half of a 115200 baud UART, and
`test_scheduler_rates_and_bandwidth` checks that arithmetic. Telemetry never
blocks the control loop: a frame the UART buffer cannot take right now is
dropped and counted, and its sequence number is given back so the receiver
does not count a frame that never left as lost.

### HEARTBEAT mapping

| `FlightState` | `system_status` | armed flag |
| --- | --- | --- |
| `DISARMED`, `ARMING` | `MAV_STATE_STANDBY` | no |
| `ARMED` | `MAV_STATE_ACTIVE` | yes |
| `FAILSAFE_LAND` | `MAV_STATE_CRITICAL` | yes, still flying |
| `FAILSAFE_CUT` | `MAV_STATE_EMERGENCY` | no |

`custom_mode` carries DroneCtrl's own state in bits 0 to 7 and the fault
cause in bits 8 to 15, in the order of the enums in `flight_state.h`, so a
ground station that knows DroneCtrl can show them exactly.

### Named values

| Name | Meaning |
| --- | --- |
| `loop_jit` | largest deviation from the 4000 us period in the last second, us |
| `loop_max` | longest period in the last second, us |
| `loop_sd` | standard deviation of the period in the last second, us |
| `loop_ovr` | missed deadlines since boot (a cycle more than half a period late) |
| `ekf_bp`, `ekf_bq` | estimated gyro bias, roll and pitch axes, deg/s |
| `ekf_inn` | accelerometer innovation magnitude, g |

The loop timing values come from `loop_timing.h`. Until it existed nothing
measured the loop, and the receiver driver at the time (`pulseIn()` on four
channels) could not have let it run at 250 Hz. See
[verification.md](verification.md#what-measuring-the-loop-found).

## Connecting QGroundControl

Plug the board in, open QGroundControl, and add a serial comm link at 115200
baud (921600 for the `esp32_hil` build). The vehicle shows up as a generic
quadrotor with attitude, battery and armed state. It does not answer
parameter or mission requests: those messages are consumed and counted as
unknown, which is why QGroundControl will report that parameters did not
load.

## Logs

The SIL harness writes the telemetry the board would have sent for every
scenario to `firmware/sil/logs/<scenario>.tlog`, in the same format
QGroundControl records. Open one in QGroundControl's log replay, or replay it
into the ground station:

```
cd groundstation
go run ./cmd/groundstation -replay ../firmware/sil/logs/signal_loss.tlog
```

## How it is checked

A codec checked only against itself proves it is self-consistent, not that
anything else can read it. So:

- `firmware/test/test_telemetry` compares encoded frames byte for byte with
  frames produced by pymavlink, and tests the parser on boot text before a
  frame, a corrupted byte, a false start that claims a long payload, sequence
  wraps and gaps, unknown messages and signed frames.
- `tools/mavlink_crosscheck.py` has pymavlink read every frame of every SIL
  log and checks the decoded attitude against the flight's CSV, then has
  pymavlink encode 300 frames our parser must decode field for field.
- The Go ground station is a third implementation and replays every SIL log
  in CI.
