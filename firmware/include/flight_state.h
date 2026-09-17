#pragma once

#include <stdint.h>

/**
 * Arming and failsafe state machine, with no hardware dependency.
 *
 * The original arming logic lived inside Safety and read millis() directly,
 * which made it impossible to test: verifying a two second arm hold meant
 * actually waiting two seconds, and verifying a failsafe meant unplugging a
 * receiver. Time is a parameter here instead, so the whole table below is
 * exercised in milliseconds of wall clock by the host tests and driven by the
 * SIL harness during fault injection.
 *
 * States
 * ------
 *   DISARMED       motors inhibited, waiting for the arm gesture
 *   ARMING         gesture held, counting down to ARMED
 *   ARMED          normal flight
 *   FAILSAFE_LAND  a recoverable fault: hold attitude, descend under control
 *   FAILSAFE_CUT   an unrecoverable fault: motors off immediately
 *
 * Fault policy
 * ------------
 * The split between LAND and CUT is the whole safety argument of the airframe,
 * so it is stated explicitly rather than left implicit in the order of a few
 * if statements:
 *
 *   signal loss        -> LAND. The airframe is still controllable and still
 *                         knows its attitude. Cutting here drops it on whatever
 *                         is underneath, which is the worse outcome in every
 *                         case except one already covered by the tilt rule.
 *   low battery        -> LAND. Known in advance, and descending under control
 *                         is what the remaining charge is for.
 *   excessive tilt     -> CUT. Past this angle the attitude controller cannot
 *                         recover the airframe, and spinning props that are
 *                         going to hit the ground anyway should not be powered.
 *   estimator diverged -> CUT. Attitude is unknown, so every actuator command
 *                         from here is guesswork. This is the one fault that
 *                         makes a controlled descent impossible by definition.
 *
 * Latching
 * --------
 * Failsafes latch. A receiver that recovers mid-descent, or a battery that
 * reads higher once the load drops, must not silently hand control back: the
 * pilot re-arms deliberately, from DISARMED, with throttle down. Unlatching on
 * its own is how an airframe ends up climbing again while someone is walking
 * towards it.
 */

enum class FlightState : uint8_t {
    DISARMED = 0,
    ARMING,
    ARMED,
    FAILSAFE_LAND,
    FAILSAFE_CUT,
};

enum class FaultCause : uint8_t {
    NONE = 0,
    SIGNAL_LOST,
    LOW_BATTERY,
    EXCESSIVE_TILT,
    ESTIMATOR_DIVERGED,
};

/** Everything the state machine needs to know about the airframe this step. */
struct FlightInputs {
    float throttle_norm = 0.0f; ///< 0..1 from the receiver
    float yaw_norm = 0.0f;      ///< -1..1, the arming gesture axis
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float battery_volts = 12.6f;
    bool signal_lost = false;
    bool estimator_diverged = false;
};

struct FlightLimits {
    float max_tilt_deg = 45.0f;
    float min_battery_volts = 10.5f;
    float arm_hold_ms = 2000.0f;
    float gesture_threshold = 0.8f;
    float throttle_idle_max = 0.05f;
    /** Descent rate commanded during FAILSAFE_LAND, as throttle per second. */
    float land_throttle_rate = 0.15f;
};

class FlightStateMachine {
public:
    FlightStateMachine() = default;
    explicit FlightStateMachine(const FlightLimits& limits) : limits_(limits) {}

    void setLimits(const FlightLimits& limits) { limits_ = limits; }

    FlightState state() const { return state_; }
    FaultCause fault() const { return fault_; }
    bool motorsEnabled() const {
        return state_ == FlightState::ARMED || state_ == FlightState::FAILSAFE_LAND;
    }
    /** True while the controller should hold its integrators still. */
    bool holdIntegrators() const { return !motorsEnabled(); }

    /**
     * Throttle the mixer should use this step. Equal to the pilot's throttle in
     * ARMED, a controlled ramp to zero in FAILSAFE_LAND, and zero otherwise.
     */
    float commandedThrottle() const { return commanded_throttle_; }

    void reset() {
        state_ = FlightState::DISARMED;
        fault_ = FaultCause::NONE;
        gesture_start_ms_ = 0;
        has_gesture_start_ = false;
        commanded_throttle_ = 0.0f;
        land_throttle_ = 0.0f;
    }

    /**
     * Advance one step.
     *
     * @param in     airframe state this step
     * @param now_ms monotonic milliseconds
     * @param dt     seconds since the previous call
     */
    void update(const FlightInputs& in, uint32_t now_ms, float dt) {
        const FaultCause fault = classify(in);

        if (fault != FaultCause::NONE) {
            enterFailsafe(fault, in);
        }

        switch (state_) {
            case FlightState::DISARMED:
            case FlightState::ARMING:
                commanded_throttle_ = 0.0f;
                // The fault is re-checked here, not just in enterFailsafe.
                // Clearing the state there is not enough on its own: the
                // gesture is still being held, so without this the very next
                // line would walk straight back into ARMING and the airframe
                // would arm on a flat battery. Caught by
                // test_fault_on_ground_blocks_arming.
                if (fault != FaultCause::NONE) {
                    has_gesture_start_ = false;
                    state_ = FlightState::DISARMED;
                    break;
                }
                stepArmingGesture(in, now_ms);
                break;

            case FlightState::ARMED:
                commanded_throttle_ = in.throttle_norm;
                stepDisarmGesture(in, now_ms);
                break;

            case FlightState::FAILSAFE_LAND:
                // Ramp down rather than stepping to zero, so the airframe does
                // not tumble the moment the fault is seen.
                land_throttle_ -= limits_.land_throttle_rate * dt;
                if (land_throttle_ <= 0.0f) {
                    land_throttle_ = 0.0f;
                    state_ = FlightState::FAILSAFE_CUT;
                }
                commanded_throttle_ = land_throttle_;
                break;

            case FlightState::FAILSAFE_CUT:
                commanded_throttle_ = 0.0f;
                break;
        }
    }

    /**
     * Clear a latched failsafe. Refused unless the pilot has throttle down, so
     * a stray call cannot re-enable motors under power.
     */
    bool clearFailsafe(const FlightInputs& in) {
        if (state_ != FlightState::FAILSAFE_LAND && state_ != FlightState::FAILSAFE_CUT) {
            return false;
        }
        if (in.throttle_norm > limits_.throttle_idle_max) return false;
        if (classify(in) != FaultCause::NONE) return false;
        reset();
        return true;
    }

private:
    FlightLimits limits_;
    FlightState state_ = FlightState::DISARMED;
    FaultCause fault_ = FaultCause::NONE;
    uint32_t gesture_start_ms_ = 0;
    bool has_gesture_start_ = false;
    float commanded_throttle_ = 0.0f;
    float land_throttle_ = 0.0f;

    FaultCause classify(const FlightInputs& in) const {
        // Ordered most severe first, so a tumbling airframe with a dead link is
        // reported as the tilt fault that actually decides the response.
        if (in.estimator_diverged) return FaultCause::ESTIMATOR_DIVERGED;
        if (absf(in.roll_deg) > limits_.max_tilt_deg ||
            absf(in.pitch_deg) > limits_.max_tilt_deg) {
            return FaultCause::EXCESSIVE_TILT;
        }
        if (in.signal_lost) return FaultCause::SIGNAL_LOST;
        if (in.battery_volts < limits_.min_battery_volts) return FaultCause::LOW_BATTERY;
        return FaultCause::NONE;
    }

    void enterFailsafe(FaultCause cause, const FlightInputs& in) {
        const bool cut = (cause == FaultCause::EXCESSIVE_TILT ||
                          cause == FaultCause::ESTIMATOR_DIVERGED);

        // On the ground and not yet armed, a fault should stop the airframe
        // arming rather than declare an in-flight emergency.
        if (state_ == FlightState::DISARMED || state_ == FlightState::ARMING) {
            state_ = FlightState::DISARMED;
            fault_ = cause;
            has_gesture_start_ = false;
            return;
        }

        if (cut) {
            state_ = FlightState::FAILSAFE_CUT;
            fault_ = cause;
            return;
        }

        if (state_ == FlightState::ARMED) {
            state_ = FlightState::FAILSAFE_LAND;
            fault_ = cause;
            land_throttle_ = in.throttle_norm;
        }
        // Already landing: keep the first cause, a second recoverable fault
        // changes nothing about what the airframe should now do.
    }

    void stepArmingGesture(const FlightInputs& in, uint32_t now_ms) {
        const bool throttle_down = in.throttle_norm <= limits_.throttle_idle_max;
        const bool gesture = throttle_down && in.yaw_norm > limits_.gesture_threshold;

        if (!gesture) {
            has_gesture_start_ = false;
            state_ = FlightState::DISARMED;
            return;
        }

        if (!has_gesture_start_) {
            has_gesture_start_ = true;
            gesture_start_ms_ = now_ms;
            state_ = FlightState::ARMING;
            return;
        }

        state_ = FlightState::ARMING;
        if ((float)(now_ms - gesture_start_ms_) >= limits_.arm_hold_ms) {
            state_ = FlightState::ARMED;
            fault_ = FaultCause::NONE;
            has_gesture_start_ = false;
        }
    }

    void stepDisarmGesture(const FlightInputs& in, uint32_t now_ms) {
        const bool throttle_down = in.throttle_norm <= limits_.throttle_idle_max;
        const bool gesture = throttle_down && in.yaw_norm < -limits_.gesture_threshold;

        if (!gesture) {
            has_gesture_start_ = false;
            return;
        }
        if (!has_gesture_start_) {
            has_gesture_start_ = true;
            gesture_start_ms_ = now_ms;
            return;
        }
        if ((float)(now_ms - gesture_start_ms_) >= limits_.arm_hold_ms) {
            state_ = FlightState::DISARMED;
            has_gesture_start_ = false;
        }
    }

    static float absf(float v) { return v < 0.0f ? -v : v; }
};

/** Human readable name, for logs and the SIL report. */
inline const char* toString(FlightState s) {
    switch (s) {
        case FlightState::DISARMED: return "DISARMED";
        case FlightState::ARMING: return "ARMING";
        case FlightState::ARMED: return "ARMED";
        case FlightState::FAILSAFE_LAND: return "FAILSAFE_LAND";
        case FlightState::FAILSAFE_CUT: return "FAILSAFE_CUT";
    }
    return "UNKNOWN";
}

inline const char* toString(FaultCause c) {
    switch (c) {
        case FaultCause::NONE: return "NONE";
        case FaultCause::SIGNAL_LOST: return "SIGNAL_LOST";
        case FaultCause::LOW_BATTERY: return "LOW_BATTERY";
        case FaultCause::EXCESSIVE_TILT: return "EXCESSIVE_TILT";
        case FaultCause::ESTIMATOR_DIVERGED: return "ESTIMATOR_DIVERGED";
    }
    return "UNKNOWN";
}
