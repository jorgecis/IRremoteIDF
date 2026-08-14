// Copyright 2025 IRremoteIDF contributors
//
// ESP-IDF platform layer for the IR library.
// Everything the protocol code needs from the underlying framework lives here,
// so the ~120 protocol implementations stay framework agnostic.

#ifndef IRPLATFORM_H_
#define IRPLATFORM_H_

#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <stddef.h>
#include <string>
#ifdef UNIT_TEST
#include <iostream>  // Debug output goes to cout when off-target.
#endif  // UNIT_TEST

/// The protocol code builds human readable output with `String`.
/// On ESP-IDF that is just `std::string`.
typedef std::string String;

// The library uses Arduino's F()/PROGMEM flash-string markers in a few places.
// ESP-IDF maps constants into flash transparently, so they are no-ops.
#ifndef F
#define F(x) x
#endif  // F
#ifndef PROGMEM
#define PROGMEM
#endif  // PROGMEM
#ifndef FPSTR
#define FPSTR(x) x
#endif  // FPSTR

#ifndef UNIT_TEST

#include "sdkconfig.h"

/// Logical pin levels.
const uint8_t kIrLow = 0;
const uint8_t kIrHigh = 1;
#ifndef LOW
#define LOW kIrLow
#endif  // LOW
#ifndef HIGH
#define HIGH kIrHigh
#endif  // HIGH

/// Milliseconds since boot. Wraps every ~49.7 days, like Arduino's millis().
uint32_t irMillis(void);
/// Microseconds since boot. Wraps every ~71.6 minutes.
uint32_t irMicros(void);
/// Sleep, yielding the CPU to other FreeRTOS tasks.
void irDelayMs(uint32_t msec);
/// Busy-wait. Accurate, but blocks the core it runs on.
void irDelayUs(uint32_t usec);
/// Feed the task watchdog without giving up the CPU for long.
void irYield(void);

/// Configure a GPIO as a push-pull output.
/// @return true on success.
bool irGpioOutput(uint16_t pin);
/// Configure a GPIO as an input, optionally with the internal pull-up.
/// @return true on success.
bool irGpioInput(uint16_t pin, bool pullup);
/// Set the level of an output GPIO.
void irGpioWrite(uint16_t pin, uint8_t level);
/// Read the level of a GPIO.
uint8_t irGpioRead(uint16_t pin);

/// Reboot the chip. Used when a buffer allocation fails.
void irRestart(void);

#ifdef CONFIG_IRREMOTE_TX_HW_CARRIER
// Optional LEDC-generated carrier. The CPU then only has to time the
// mark/space boundaries instead of every single carrier pulse.

/// Claim an LEDC channel and bind it to `pin`.
/// @return The channel number, or -1 if none was available.
int8_t irCarrierAttach(uint16_t pin, bool inverted);
/// Release a previously claimed channel.
void irCarrierDetach(int8_t channel, uint16_t pin, bool inverted);
/// Set the carrier frequency (Hz) and duty cycle (percent).
void irCarrierConfig(int8_t channel, uint32_t freq, uint8_t duty);
/// Start emitting the carrier.
void irCarrierOn(int8_t channel);
/// Stop emitting the carrier, leaving the pin at its idle level.
void irCarrierOff(int8_t channel);
#endif  // CONFIG_IRREMOTE_TX_HW_CARRIER

#endif  // UNIT_TEST

// Debug output. Enable by defining DEBUG (e.g. -DDEBUG=1).
#ifdef DEBUG
#ifdef UNIT_TEST
#define DPRINT(x) do { \
    std::cout << x; \
  } while (0)
#define DPRINTLN(x) do { \
    std::cout << x << std::endl; \
  } while (0)
#else  // UNIT_TEST
void irDebugPrint(const char *str);
void irDebugPrint(const String &str);
void irDebugPrint(uint64_t value);
void irDebugPrint(int64_t value);
#define DPRINT(x) do { \
    irDebugPrint(x); \
  } while (0)
#define DPRINTLN(x) do { \
    irDebugPrint(x); \
    irDebugPrint("\n"); \
  } while (0)
#endif  // UNIT_TEST
#else  // DEBUG
#define DPRINT(x)
#define DPRINTLN(x)
#endif  // DEBUG

#endif  // IRPLATFORM_H_
