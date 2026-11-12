#pragma once
#include <Arduino.h>
#include "config.h"
// The edge decoder runs inside the pin interrupt, so on the ESP32 it has to
// live in IRAM like the handler that calls it. rc_pulse.h stays Arduino free
// and only takes the attribute it is given.
#if defined(ARDUINO_ARCH_ESP32)
#define RC_PULSE_ISR_ATTR IRAM_ATTR
#endif
#include "rc_pulse.h"

/**
 * Four channel PWM receiver, interrupt driven.
 *
 * The decoding and the signal-loss policy live in rc_pulse.h, which has no
 * Arduino dependency and is unit tested on the host. What is left here is
 * the part that needs a board: one pin-change interrupt per channel that
 * timestamps edges, and an atomic copy for the loop to read.
 *
 * This replaced four sequential pulseIn() calls, which blocked the control
 * loop for most of each 20 ms receiver frame. See rc_pulse.h for why that
 * mattered.
 *
 * ESP32: attachInterrupt on each pin, CHANGE, with a spinlock for the copy.
 * Nano:  D2, D4 and D7 are on port D and D8 on port B, and only D2 and D3
 *        have a dedicated external interrupt, so all four use the pin change
 *        interrupts PCINT2 (port D) and PCINT0 (port B). The ISR compares
 *        against the previous port state to find which pins moved.
 */

#if defined(ARDUINO_ARCH_ESP32)
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#endif

// File-scope statics rather than C++17 inline variables: the AVR toolchain
// builds as C++11, and main.cpp is the only translation unit that includes
// this header, so there is exactly one copy of each.
namespace rx_isr {
static RcPulseChannel channels[4];

#if defined(ARDUINO_ARCH_ESP32)
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * Pin level straight from the GPIO input registers. digitalRead() is not
 * guaranteed to be in IRAM on arduino-esp32 3.x, and an interrupt handler
 * that calls into flash crashes the first time the flash cache is busy.
 */
static inline bool IRAM_ATTR pinHigh(uint8_t pin) {
    return pin < 32 ? ((REG_READ(GPIO_IN_REG) >> pin) & 1u) != 0
                    : ((REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u) != 0;
}

static void IRAM_ATTR onEdge(int ch, uint8_t pin) {
    const uint32_t now = micros();
    const bool high = pinHigh(pin);
    portENTER_CRITICAL_ISR(&mux);
    channels[ch].edge(high, now);
    portEXIT_CRITICAL_ISR(&mux);
}

// Four plain functions rather than one template. IRAM_ATTR names its section
// from __COUNTER__ at the point it is written, so every instantiation of a
// template shares one section, and the Xtensa linker then places their
// literal pools after the code that loads them ("dangerous relocation: l32r:
// literal placed after use").
static void IRAM_ATTR onRoll() { onEdge(0, PIN_RX_ROLL); }
static void IRAM_ATTR onPitch() { onEdge(1, PIN_RX_PITCH); }
static void IRAM_ATTR onThrottle() { onEdge(2, PIN_RX_THROTTLE); }
static void IRAM_ATTR onYaw() { onEdge(3, PIN_RX_YAW); }
#endif
} // namespace rx_isr

#if defined(ARDUINO_ARCH_AVR)
// Fixed by config.h: roll D2 (PD2), pitch D4 (PD4), throttle D7 (PD7), yaw D8 (PB0).
static_assert(PIN_RX_ROLL == 2 && PIN_RX_PITCH == 4 && PIN_RX_THROTTLE == 7 && PIN_RX_YAW == 8,
              "receiver.h hard codes the Nano's port bits for these pins; update both together");

namespace rx_isr {
static volatile uint8_t prev_pind = 0;
static volatile uint8_t prev_pinb = 0;
} // namespace rx_isr

ISR(PCINT2_vect) {
    const uint32_t now = micros();
    const uint8_t pins = PIND;
    const uint8_t changed = pins ^ rx_isr::prev_pind;
    rx_isr::prev_pind = pins;
    if (changed & _BV(PD2)) rx_isr::channels[0].edge(pins & _BV(PD2), now);
    if (changed & _BV(PD4)) rx_isr::channels[1].edge(pins & _BV(PD4), now);
    if (changed & _BV(PD7)) rx_isr::channels[2].edge(pins & _BV(PD7), now);
}

ISR(PCINT0_vect) {
    const uint32_t now = micros();
    const uint8_t pins = PINB;
    const uint8_t changed = pins ^ rx_isr::prev_pinb;
    rx_isr::prev_pinb = pins;
    if (changed & _BV(PB0)) rx_isr::channels[3].edge(pins & _BV(PB0), now);
}
#endif

class Receiver {
public:
    void init() {
        pinMode(PIN_RX_ROLL, INPUT);
        pinMode(PIN_RX_PITCH, INPUT);
        pinMode(PIN_RX_THROTTLE, INPUT);
        pinMode(PIN_RX_YAW, INPUT);

#if defined(ARDUINO_ARCH_ESP32)
        attachInterrupt(digitalPinToInterrupt(PIN_RX_ROLL), rx_isr::onRoll, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_RX_PITCH), rx_isr::onPitch, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_RX_THROTTLE), rx_isr::onThrottle, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_RX_YAW), rx_isr::onYaw, CHANGE);
#elif defined(ARDUINO_ARCH_AVR)
        noInterrupts();
        rx_isr::prev_pind = PIND;
        rx_isr::prev_pinb = PINB;
        PCMSK2 |= _BV(PCINT18) | _BV(PCINT20) | _BV(PCINT23); // D2, D4, D7
        PCMSK0 |= _BV(PCINT0);                                 // D8
        PCIFR = _BV(PCIF2) | _BV(PCIF0);                       // clear stale flags
        PCICR |= _BV(PCIE2) | _BV(PCIE0);
        interrupts();
#else
#error "receiver.h: no edge capture for this architecture"
#endif
    }

    /** Constant time: copies four timestamps and does arithmetic. */
    void readChannels() {
        RcPulseChannel snapshot[4];
#if defined(ARDUINO_ARCH_ESP32)
        portENTER_CRITICAL(&rx_isr::mux);
        for (int i = 0; i < 4; ++i) snapshot[i] = rx_isr::channels[i];
        portEXIT_CRITICAL(&rx_isr::mux);
#else
        noInterrupts();
        for (int i = 0; i < 4; ++i) snapshot[i] = rx_isr::channels[i];
        interrupts();
#endif
        decoder_.update(snapshot, micros());
    }

    float getRoll() const { return decoder_.roll(); }
    float getPitch() const { return decoder_.pitch(); }
    float getThrottle() const { return decoder_.throttle(); }
    float getYaw() const { return decoder_.yaw(); }
    bool isSignalLost() const { return decoder_.signalLost(); }

private:
    RcDecoder decoder_;
};
