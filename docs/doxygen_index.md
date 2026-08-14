# IRremoteIDF

Send and receive infrared signals from ESP-IDF. An ESP-IDF port of
[IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266) v2.9.0.

## Where to start

* `IRsend` — transmit. One method per protocol, plus `sendRaw()`.
* `IRrecv` — receive and decode. `enableIRIn()`, then poll `decode()`.
* `IRac` — vendor-neutral air conditioner control across 60+ brands.
* `IRutils.h` — `typeToString()`, `resultToHumanReadableBasic()`,
  `resultToSourceCode()` and other helpers for printing what you captured.
* `IRplatform.h` — the thin ESP-IDF layer (GPIO, timing, optional LEDC
  carrier) that replaced the Arduino core.

See the [README](https://github.com/jorgecis/IRremoteIDF#readme) for wiring,
installation and `menuconfig` options, and
[PORTING.md](https://github.com/jorgecis/IRremoteIDF/blob/main/docs/PORTING.md)
for what differs from upstream.
