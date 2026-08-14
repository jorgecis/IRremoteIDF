# IRremoteIDF

Send and receive infrared (IR) signals from ESP-IDF.

This is an ESP-IDF port of
[IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266) v2.9.0. Every
protocol decoder, encoder and air-conditioner state class is carried over
unchanged, so the protocol behaviour is identical — the same 1669 unit tests
from upstream run against this code and pass. What was rewritten is the layer
underneath: the Arduino core dependency is gone, replaced by native ESP-IDF
GPIO, `esp_timer` and (optionally) LEDC.

* **130 protocols**, most of them both send *and* decode.
* **Full climate control** for 60+ A/C brands via the vendor-neutral `IRac`
  class — mode, temperature, fan, swing, quiet, turbo, econo, sleep, clock.
* **No Arduino compatibility layer to install.** It is a plain ESP-IDF
  component with a `Kconfig`, a manifest and nothing else.
* Builds on ESP32, ESP32-S2/S3, ESP32-C2/C3/C5/C6, ESP32-H2 and ESP32-P4.

## Install

### From the ESP Component Registry

```sh
idf.py add-dependency "jorgecis/IRremoteIDF^1.0.0"
```

### As a git submodule

```sh
git submodule add https://github.com/jorgecis/IRremoteIDF.git \
    components/IRremoteIDF
```

Either way, no further CMake changes are needed. Include the headers and go:

```cpp
#include "IRsend.h"

static IRsend irsend(4);  // GPIO driving the IR LED.

extern "C" void app_main(void) {
  irsend.begin();
  irsend.sendNEC(0x00FFE01FUL);
}
```

## Hardware

| Function | Typical part | Notes |
| --- | --- | --- |
| Receive | TSOP38238, VS1838B, TSOP4838 | 38 kHz demodulator. Output straight to a GPIO. |
| Transmit | 940 nm IR LED + NPN/MOSFET | Do **not** drive the LED from a GPIO directly; the current will exceed the pad rating. |

Pick a GPIO that is free at boot. GPIO 14 is a poor choice on the ESP32-C3
(strapping); GPIO 10 works there.

## Receiving

```cpp
#include "IRrecv.h"
#include "IRutils.h"
#include "IRac.h"

static IRrecv irrecv(/*pin=*/14, /*bufsize=*/1024, /*timeout_ms=*/50,
                     /*save_buffer=*/true);
static decode_results results;

extern "C" void app_main(void) {
  irrecv.enableIRIn();
  while (true) {
    if (irrecv.decode(&results)) {
      printf("%s\n", resultToHumanReadableBasic(&results).c_str());
      String ac = IRAcUtils::resultAcToString(&results);
      if (ac.length()) printf("%s\n", ac.c_str());
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
```

`timeout_ms` is how long the line must stay quiet before a message is
considered finished. 15 ms suits normal remotes; A/C remotes need 50 ms or more.

## Air conditioners

`IRac` exposes one struct that maps onto every supported climate protocol:

```cpp
IRac ac(4);
ac.next.protocol = decode_type_t::COOLIX;
ac.next.power = true;
ac.next.mode = stdAc::opmode_t::kCool;
ac.next.degrees = 24;
ac.next.celsius = true;
ac.next.fanspeed = stdAc::fanspeed_t::kMedium;
ac.sendAc();
```

Switching brands is a one-line change. See
[docs/SupportedProtocols.md](docs/SupportedProtocols.md) for the full list and
which features each protocol carries.

## Configuration (`idf.py menuconfig`)

Under **Component config → IRremote (infrared send/receive)**:

| Option | Default | Effect |
| --- | --- | --- |
| Enable every supported protocol | on | Turn off to pick protocols individually and shrink the build. |
| Protocols → *`<name>`: send / decode* | follows the above | One switch per protocol direction, 241 in total. |
| Generate the transmit carrier in hardware (LEDC) | off | Offloads the 30–60 kHz carrier to an LEDC channel. |
| Language/locale for human readable output | en-AU | 15 locales available, `es-ES` among them. |
| Print library debug output | off | Enables the internal `DPRINT` tracing. |

Trimming is worth real flash: the `ir_receive_dump` example is 318 KB with every
protocol enabled and 188 KB with NEC only.

Command-line `-D` flags still take priority, so an existing recipe such as
`-D_IR_ENABLE_DEFAULT_=false -DDECODE_NEC=true` keeps working:

```cmake
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    _IR_ENABLE_DEFAULT_=false DECODE_NEC=true SEND_NEC=true)
```

## Examples

| Example | What it shows |
| --- | --- |
| [`ir_send_demo`](examples/ir_send_demo) | Sending NEC / Sony / Samsung / RC5 and raw timings. |
| [`ir_receive_dump`](examples/ir_receive_dump) | Decoding, human-readable output and a pasteable raw dump. |
| [`ac_control`](examples/ac_control) | Driving an air conditioner through `IRac`. |
| [`ir_repeater`](examples/ir_repeater) | Capture and rebroadcast, including unknown protocols. |

```sh
cd examples/ir_receive_dump
idf.py set-target esp32
idf.py build flash monitor
```

## Tests

The complete upstream test suite runs on the host against these sources — 83
binaries, 1669 cases:

```sh
git clone -b v1.12.x --depth 1 https://github.com/google/googletest.git lib/googletest
make -C test run
```

## Differences from IRremoteESP8266

The user-facing API is unchanged apart from a few platform-specific corners.
See [docs/PORTING.md](docs/PORTING.md) for the details and how the transmit and
receive paths are implemented on ESP-IDF.

## License

LGPL-2.1, inherited from IRremoteESP8266. The original copyright notices are
kept in every file. See [LICENSE](LICENSE).

Upstream authors: David Conran, Mark Szabo, Sebastien Warin, Ken Shirriff, Roi
Dayan, Massimiliano Pinto, Christian Nilsson and the IRremoteESP8266
contributors.
