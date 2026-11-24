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

/**
 * The four ESC outputs.
 *
 * All the arithmetic lives in mixer_math.h, which has no Arduino dependency
 * and is unit tested on the host, and is called from FlightLoop. What is left
 * here is the part that genuinely needs hardware: four Servo objects.
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
    }

    /**
     * Write four ESC commands, FL FR BL BR, in microseconds. The mixing and
     * the saturation handling happen in FlightLoop (via mixer_math.h), so the
     * values arriving here are already the final outputs; this only rounds
     * them to the whole microseconds the Servo driver takes.
     */
    void write(const float pwm[4]) {
        writeMotors((int)(pwm[0] + 0.5f), (int)(pwm[1] + 0.5f),
                    (int)(pwm[2] + 0.5f), (int)(pwm[3] + 0.5f));
    }

private:
    Servo motorFL, motorFR, motorBL, motorBR;

    void writeMotors(int fl, int fr, int bl, int br) {
        motorFL.writeMicroseconds(fl);
        motorFR.writeMicroseconds(fr);
        motorBL.writeMicroseconds(bl);
        motorBR.writeMicroseconds(br);
    }
};
