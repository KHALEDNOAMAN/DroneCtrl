#pragma once
#include <Arduino.h>
// arduino-esp32 ships no Servo library, so <Servo.h> does not exist there and
// the esp32 environment cannot compile against it. ESP32Servo provides the
// same Servo class under its own header, with matching attach(pin, min, max)
// and writeMicroseconds() signatures, so only the include has to differ.
#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "config.h"
#include "mixer_math.h"

/**
 * The four ESC outputs.
 *
 * All the arithmetic lives in mixer_math.h, which has no Arduino dependency
 * and is unit tested on the host. What is left here is the part that genuinely
 * needs hardware: four Servo objects and the decision to write them at all.
 */
class MotorMixer {
public:
    void init() {
        motorFL.attach(PIN_MOTOR_FL, ESC_MIN, ESC_MAX);
        motorFR.attach(PIN_MOTOR_FR, ESC_MIN, ESC_MAX);
        motorBL.attach(PIN_MOTOR_BL, ESC_MIN, ESC_MAX);
        motorBR.attach(PIN_MOTOR_BR, ESC_MIN, ESC_MAX);
        stop();
    }

    /** Every motor to its minimum. The only path that reaches the ESCs when
     *  the state machine says motors are not enabled. */
    void stop() {
        writeMotors(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
        authority_limited_ = false;
    }

    /**
     * @param throttle_norm 0..1, already decided by the flight state machine
     *        rather than taken straight from the stick, so a failsafe descent
     *        arrives here as an ordinary throttle value.
     */
    void setOutputs(float throttle_norm, float pitch_cmd, float roll_cmd, float yaw_cmd) {
        const float base = ESC_IDLE + throttle_norm * (ESC_MAX - ESC_IDLE);
        const auto out = mixer::mixX(base, pitch_cmd, roll_cmd, yaw_cmd, ESC_MIN, ESC_MAX);
        authority_limited_ = out.authority_limited;
        writeMotors((int)(out.fl + 0.5f), (int)(out.fr + 0.5f),
                    (int)(out.bl + 0.5f), (int)(out.br + 0.5f));
    }

    /** True when the last mix could not deliver the commanded torque. The
     *  control loop uses it to hold the PID integrators. */
    bool authorityLimited() const { return authority_limited_; }

private:
    Servo motorFL, motorFR, motorBL, motorBR;
    bool authority_limited_ = false;

    void writeMotors(int fl, int fr, int bl, int br) {
        motorFL.writeMicroseconds(fl);
        motorFR.writeMicroseconds(fr);
        motorBL.writeMicroseconds(bl);
        motorBR.writeMicroseconds(br);
    }
};
