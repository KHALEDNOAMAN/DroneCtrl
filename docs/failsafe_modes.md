# Failsafe modes

What `firmware/include/flight_state.h` actually does. Every row below has a
host test in `firmware/test/test_safety` and, where it involves flight, a SIL
scenario.

| Trigger | Response | Why |
| --- | --- | --- |
| Receiver signal lost (no valid frame for 500 ms) | `FAILSAFE_LAND`: level setpoint, throttle ramped down from the last healthy throttle at 0.15 per second, then `FAILSAFE_CUT` | recoverable; the airframe is still controllable |
| Battery below 10.5 V | `FAILSAFE_LAND`, ramped from the current throttle | recoverable; the link is still up |
| Tilt beyond 45 degrees | `FAILSAFE_CUT` at once | past what the controller can recover |
| Estimator non-finite | `FAILSAFE_CUT` at once | attitude unknown, so every command is a guess |
| Any fault before arming | stays `DISARMED`, arming refused | a fault on the ground is not an emergency |

Faults are ranked most severe first, so a tumbling airframe whose link also
dropped is handled as the tilt fault. Failsafes latch: a receiver that comes
back mid-descent does not hand control back, and clearing the latch is
refused while the throttle is up or the fault is still present.

## Where a signal-loss descent starts

From the last throttle the pilot commanded while the link was healthy, not
from what the receiver reports on the failing step. Every RC receiver driver,
including `receiver.h`, reports zero throttle together with `signal_lost`, so
starting the ramp from the current input started it at zero: the next step
went straight to `FAILSAFE_CUT` and the motors stopped in the air. The HIL
loopback found this; [verification.md](verification.md#what-hil-found) has
the details.

## What it does not do

No return to launch, no GPS altitude hold, no geofence: the firmware has no
position estimate to do them with. The descent is open loop, a fixed throttle
decrement with no barometer in the loop, so it lands firmly; from 2.6 m in
the HIL check it touches down at a little over 4 m/s. Closing the descent on
altitude is the obvious next step.
