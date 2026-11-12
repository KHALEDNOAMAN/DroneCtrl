/**
 * DroneCtrl flight controller.
 *
 * Loop structure, 250 Hz fixed rate:
 *
 *   sense   read the IMU and the receiver
 *   step    FlightLoop: EKF, flight state machine, attitude PIDs, mixer
 *   act     write four ESC outputs
 *   report  MAVLink telemetry, never allowed to block the loop
 *
 * Everything in "step" is flight_loop.h, which contains no Arduino dependency
 * and is the same object the host tests, the SIL harness and the HIL build
 * run. This file is the wiring, and should stay that way: logic that lands
 * here is logic that cannot be tested without a board and a flight.
 *
 * Scheduling
 * ----------
 * ESP32: the control step is a FreeRTOS task pinned to core 1 at the highest
 *        application priority, released every 4 ms by vTaskDelayUntil, which
 *        holds the period against the tick rather than drifting by however
 *        long the step took. Telemetry runs in loop(), at the Arduino task's
 *        priority 1, so a slow serial write is preempted by the controller
 *        rather than delaying it.
 * Nano:  no RTOS, so loop() polls micros() for the period and sends telemetry
 *        in whatever time is left, only ever as much as the 64 byte UART
 *        buffer can take without blocking.
 *
 * On both, loop_timing.h measures the period actually achieved and reports it
 * over MAVLink (docs/mavlink.md), so "250 Hz" is something the ground station
 * shows rather than something this comment claims.
 *
 * Build DRONECTRL_HIL (pio run -e esp32_hil) for hardware in the loop: the
 * sensors and motors are a simulator's, over MAVLink on the USB serial port,
 * and the ESC outputs are never attached. See docs/hil.md.
 */

#include <Arduino.h>
#include "config.h"
#include "flight_loop.h"
#include "loop_timing.h"
#include "mavlink_telemetry.h"

#if defined(DRONECTRL_HIL)
#if !defined(ARDUINO_ARCH_ESP32)
#error "The HIL build needs the ESP32: the parser alone is more RAM than the Nano has spare."
#endif
#include "hil_link.h"
constexpr uint32_t SERIAL_BAUD = 921600; // 250 Hz of HIL_SENSOR does not fit in 115200
#else
#include "imu.h"
#include "receiver.h"
#include "motor_mixer.h"
#include "battery.h"
constexpr uint32_t SERIAL_BAUD = 115200;
#endif

// ------------------------------------------------------------------ state

FlightLoop flight;
LoopTiming timing(LOOP_INTERVAL_US);

#if defined(DRONECTRL_HIL)
hil::Link hil_link;
// The simulator has no battery model. A fixed healthy pack keeps the low
// battery failsafe out of HIL runs unless a test wants it.
constexpr float HIL_BATTERY_VOLTS = 12.4f;
#else
IMU imu;
Receiver rx;
// Named motors, not mixer: mixer_math.h already owns the `mixer` namespace
// and a global of that name shadows it, which does not compile.
MotorMixer motors;
Battery battery;
#endif

telemetry::Rates telemetryRates() {
    telemetry::Rates r;
#if defined(ARDUINO_ARCH_ESP32) && !defined(DRONECTRL_HIL)
    r.attitude_ms = 20; // 50 Hz; the ESP32's UART buffer is not the constraint
#elif defined(DRONECTRL_HIL)
    r.attitude_ms = 100; // the simulator already has the truth; this is for QGC
#endif
    return r;
}
telemetry::Scheduler tlm(telemetryRates());

// The control step writes this, telemetry reads it. On the ESP32 they run in
// different tasks, so the copy is guarded. On the Nano both run in loop() and
// no interrupt touches the snapshot, so the guard compiles away rather than
// masking the receiver's edge interrupts for the length of a struct copy.
telemetry::Snapshot shared_snapshot;
#if defined(ARDUINO_ARCH_ESP32)
portMUX_TYPE snapshot_mux = portMUX_INITIALIZER_UNLOCKED;
#define SNAPSHOT_LOCK() portENTER_CRITICAL(&snapshot_mux)
#define SNAPSHOT_UNLOCK() portEXIT_CRITICAL(&snapshot_mux)
#else
#define SNAPSHOT_LOCK() do {} while (0)
#define SNAPSHOT_UNLOCK() do {} while (0)
#endif

/** Timing statistics are published once a second and the window restarted. */
uint16_t timing_window_cycles = 0;
LoopTiming::Report last_timing;

void publishSnapshot(uint32_t now_ms, const ImuSample& s, float battery_v,
                     bool receiver_ok, uint32_t link_errors) {
    if (++timing_window_cycles >= (uint16_t)LOOP_FREQUENCY_HZ) {
        last_timing = timing.report();
        timing.resetWindow();
        timing_window_cycles = 0;
    }

    telemetry::Snapshot snap = telemetry::snapshotOf(flight, s, now_ms, battery_v);
    snap.timing = last_timing;
    snap.total_overruns = timing.totalOverruns();
    snap.link_errors = link_errors;
    snap.receiver_ok = receiver_ok;
#if defined(DRONECTRL_HIL)
    snap.hil = true;
#endif

    SNAPSHOT_LOCK();
    shared_snapshot = snap;
    SNAPSHOT_UNLOCK();
}

/** Never blocks: a frame the UART buffer cannot take right now is dropped. */
bool emitFrame(const uint8_t* frame, size_t len) {
    if ((size_t)Serial.availableForWrite() < len) return false;
    Serial.write(frame, len);
    return true;
}

void sendTelemetry() {
    telemetry::Snapshot snap;
    SNAPSHOT_LOCK();
    snap = shared_snapshot;
    SNAPSHOT_UNLOCK();
    tlm.poll(snap, emitFrame);
}

// ------------------------------------------------------------ normal build

#if !defined(DRONECTRL_HIL)

void controlCycle() {
    timing.markStart(micros());

    // --- sense ------------------------------------------------------------
    ImuSample sample;
    imu.read(sample);
    rx.readChannels();

    // The battery divider is slow and noisy, and nothing downstream needs it
    // at 250 Hz. Every 50th cycle is 5 Hz, which is far faster than a pack
    // discharges.
    static uint8_t battery_divider = 0;
    if (++battery_divider >= 50) {
        battery.update();
        battery_divider = 0;
    }

    PilotInput pilot;
    pilot.throttle = rx.getThrottle();
    pilot.roll = rx.getRoll();
    pilot.pitch = rx.getPitch();
    pilot.yaw = rx.getYaw();
    pilot.signal_lost = rx.isSignalLost();

    // --- step and act -----------------------------------------------------
    float pwm[4];
    const uint32_t now_ms = millis();
    flight.step(sample, pilot, battery.getVoltage(), now_ms, pwm);
    if (flight.fsm.motorsEnabled()) {
        motors.write(pwm);
    } else {
        motors.stop();
    }

    timing.markEnd(micros());
    publishSnapshot(now_ms, sample, battery.getVoltage(), !pilot.signal_lost, 0);
}

#if defined(ARDUINO_ARCH_ESP32)
#if configTICK_RATE_HZ != 1000
#error "The control task's 4 ms period needs the 1 kHz FreeRTOS tick arduino-esp32 ships with."
#endif
static_assert(LOOP_INTERVAL_US % 1000 == 0, "the FreeRTOS period is in whole milliseconds");

void controlTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOOP_INTERVAL_US / 1000);
    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        controlCycle();
    }
}
#endif

void setup() {
    Serial.begin(SERIAL_BAUD);

    if (!imu.init()) {
        Serial.println(F("IMU init failed"));
        // Halting is the right response. Continuing without an attitude
        // estimate would let the airframe arm with no idea which way is up.
        while (true) { delay(10); }
    }

    Serial.println(F("Calibrating gyro, keep the airframe still and level."));
    float ax, ay, az;
    imu.calibrate(ax, ay, az);
    flight.ekf.reset();
    flight.ekf.initializeFromAccel(ax, ay, az);
    Serial.println(F("Calibration done."));

    rx.init();
    motors.init();
    battery.init();

#if defined(ARDUINO_ARCH_ESP32)
    // Core 1, alongside loop(), so core 0 stays free for the radio stack if
    // Wi-Fi telemetry is ever added. 4 kB of stack is several times what the
    // EKF and the mixer use; the high water mark is worth checking on a board.
    xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                            configMAX_PRIORITIES - 1, nullptr, 1);
#endif
}

#if defined(ARDUINO_ARCH_ESP32)
void loop() {
    sendTelemetry();
    vTaskDelay(1); // yield; the control task runs on its own schedule
}
#else
uint32_t prev_loop_time = 0;

void loop() {
    const uint32_t now_us = micros();
    // Unsigned subtraction, so this stays correct across the micros() rollover
    // at roughly 71 minutes rather than stalling the loop for one cycle.
    if (now_us - prev_loop_time >= LOOP_INTERVAL_US) {
        // Advance by the nominal period rather than snapping to now, so one
        // late cycle does not shift every cycle after it. If the loop has
        // fallen a whole period behind, resynchronise instead of bursting.
        prev_loop_time += LOOP_INTERVAL_US;
        if (now_us - prev_loop_time >= LOOP_INTERVAL_US) prev_loop_time = now_us;
        controlCycle();
    }
    sendTelemetry();
}
#endif

// --------------------------------------------------------------- HIL build

#else // DRONECTRL_HIL

/**
 * One lockstep cycle: a simulator sample in, one FlightLoop step, the motor
 * commands straight back. Simulator time drives the state machine's clock,
 * so a HIL run is as repeatable as the simulator that drives it.
 */
void hilStep(const ImuSample& sample, uint64_t sim_usec) {
    timing.markStart(micros());

    const PilotInput pilot = hil_link.pilot(sim_usec);
    const uint32_t sim_ms = (uint32_t)(sim_usec / 1000);

    float pwm[4];
    flight.step(sample, pilot, HIL_BATTERY_VOLTS, sim_ms, pwm);

    uint8_t frame[hil::Link::kActuatorFrame];
    const size_t n = hil_link.encodeActuators(frame, pwm, sim_usec, flight.fsm.motorsEnabled());
    // The reply is what the simulator is waiting on, so it is written even if
    // that means blocking briefly. Telemetry, below, is what gets dropped.
    Serial.write(frame, n);

    timing.markEnd(micros());
    const mavlink::ParserStats& st = hil_link.stats();
    publishSnapshot(sim_ms, sample, HIL_BATTERY_VOLTS, !pilot.signal_lost, st.crc_errors);
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    // No IMU, no receiver and, deliberately, no ESC outputs: the motor pins
    // are never attached in this build, so a board on the bench with props
    // fitted cannot spin them whatever the simulator sends.
    flight.ekf.reset();
    flight.ekf.initializeFromAccel(0.0f, 0.0f, 1.0f);
}

void loop() {
    while (Serial.available() > 0) {
        hil_link.push((uint8_t)Serial.read(), hilStep);
    }
    sendTelemetry();
}

#endif // DRONECTRL_HIL
