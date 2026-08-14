# Changelog

## 1.0.0

Initial release. ESP-IDF port of
[IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266) v2.9.0
(upstream commit `afd43878`).

* All 130 protocols and every air conditioner class carried over unchanged.
* Arduino core dependency removed; GPIO, timing and logging now go through
  native ESP-IDF APIs (`driver/gpio`, `esp_timer`, `esp_rom_delay_us`).
* Receive uses a GPIO edge ISR plus a periodic `esp_timer` instead of a
  re-armed hardware timer, so no timer group is consumed and the ISR touches
  nothing that is not IRAM-safe.
* Optional LEDC hardware carrier for transmit
  (`CONFIG_IRREMOTE_TX_HW_CARRIER`).
* All 241 protocol switches and the 15 locales exposed through `Kconfig`;
  `-D` build flags still take priority.
* `kPeriodOffset` fixed at `-2` (the ESP32 value) rather than the ESP8266's
  `-5`.
* `IRrecv`'s ESP32-only `timer_num` constructor parameter removed;
  `IRsend::end()` added.
* Upstream's full test suite (83 binaries, 1669 cases) runs on the host and
  passes.
