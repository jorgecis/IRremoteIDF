# Porting notes

What changed when IRremoteESP8266 v2.9.0 became an ESP-IDF component, and why.

## What did not change

Every `ir_*.cpp` / `ir_*.h` protocol file, `IRac`, `IRutils`, `IRtext` and the
locale files are the upstream sources. The only edit applied to them was
swapping `#include <Arduino.h>` for `#include "IRplatform.h"`. Bit layouts,
checksums, timings, tolerances and the decode state machine are untouched — the
upstream test suite is the proof, and it runs unmodified except for the timing
constant noted below.

## The platform layer

`include/IRplatform.h` + `src/port/IRplatform.cpp` supply the handful of things
the library used to get from the Arduino core:

| Arduino | ESP-IDF port |
| --- | --- |
| `String` | `std::string` (the library only ever used the shared subset) |
| `millis()` / `micros()` | `esp_timer_get_time()` |
| `delay()` | `vTaskDelay()` for whole ticks + busy-wait for the remainder |
| `delayMicroseconds()` | `esp_rom_delay_us()` |
| `pinMode()` / `digitalWrite()` / `digitalRead()` | `gpio_config()` / `gpio_set_level()` / `gpio_get_level()` |
| `F()` / `PROGMEM` / `FPSTR()` | no-ops; ESP-IDF maps constants to flash already |
| `Serial.print()` in `DPRINT` | `stdout` |
| `ESP.restart()` | `esp_restart()` |

`double_t` (from Arduino's math shims) in `IRsend::calibrate()` became plain
`double`, and `String::substring()` in `dayToString()` became `substr()`. Those
were the only two Arduino-specific API calls outside the platform layer.

## Transmit

The software carrier loop from upstream is kept verbatim: `mark()` toggles the
GPIO for `onTimePeriod` / `offTimePeriod` microseconds, timed by
`esp_rom_delay_us()`.

`CONFIG_IRREMOTE_TX_HW_CARRIER` switches to an LEDC-generated carrier
(`src/port/IRcarrier.cpp`). `IRsend::begin()` then claims one LEDC channel and
timer, `enableIROut()` reprograms them for the requested frequency and duty, and
`mark()` degenerates to "enable the carrier, wait, disable it". The CPU no
longer has to toggle the pin ~40 000 times a second. Call `IRsend::end()` to
release the channel; it is a new method with no upstream equivalent.

`kPeriodOffset` is now unconditionally `-2` (the value upstream used for the
ESP32) instead of the ESP8266's `-5`. This is the only behavioural constant that
moved, and it shifts the software carrier's period by 3 µs. Three tests in
`test/IRsend_test.cpp` assert the exact pulse train and were updated to match;
their arithmetic was re-derived by hand, not copied from the new output.
`IRsend::calibrate()` still exists if your clock configuration wants a different
value.

## Receive

Upstream drove a per-edge GPIO interrupt plus a one-shot hardware timer that the
ISR re-armed on every edge — which on the Arduino ESP32 core required poking
timer registers directly from IRAM because `timerWrite()` is not IRAM-safe.

The port keeps the GPIO ISR (`GPIO_INTR_ANYEDGE`, IRAM-resident, filling the
same `rawbuf` in the same units) and replaces the re-armed timer with a periodic
`esp_timer` that polls "has the line been quiet for longer than the timeout?".
The timer runs at a third of the configured timeout, so a message is closed off
between 1.0x and 1.33x the timeout after its last edge instead of exactly 1.0x.
`rawbuf` content is bit-identical either way; only the latency before `decode()`
can succeed changes. In exchange the ISR calls nothing that is not IRAM-safe,
and no hardware timer group is consumed.

### API differences

* `IRrecv`'s ESP32-only 5th constructor parameter (`timer_num`) is gone, along
  with `kDefaultESP32Timer`. There is no hardware timer to pick any more.
* `IRrecv::enableIRIn()` installs the shared GPIO ISR service if nobody else
  has. `disableIRIn()` only uninstalls it if this library installed it.
* `IRsend::end()` was added (see above).

## Build-time configuration

Upstream selects protocols with `-D` flags. Those still work — the generated
`include/IRconfig.h` only fills in a macro that has not already been defined —
but the same 241 switches are also exposed through `Kconfig` so `idf.py
menuconfig` can drive them.

`Kconfig` and `include/IRconfig.h` are generated. After changing the protocol
list in `include/IRremoteESP8266.h`, regenerate them:

```sh
python3 tools/gen_kconfig.py
```

## Tests

`test/` is the upstream suite. The Makefile was pointed at the split
`include/` + `src/` layout and three rules gained `$(INCLUDES)` so they can find
`IRplatform.h`. `test/IRsend_test.cpp` carries the three updated timing
expectations described above. Nothing else was touched.

## Not verified on hardware

Everything here is compile-verified against ESP-IDF v5.5.5 for esp32, esp32s3,
esp32c3 and esp32c6, and the protocol layer is covered by the host test suite.
The GPIO/LEDC/`esp_timer` paths have not been checked against a real IR
transceiver with a scope. Timing margins in particular deserve a first look on
your own bench.
