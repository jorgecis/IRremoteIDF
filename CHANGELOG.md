# Changelog

## 1.0.1

* Fix long inter-frame spaces (over 16 ms) coming out up to one FreeRTOS
  tick short. `vTaskDelay()` wakes on a tick edge, so on the default 10 ms
  tick a 20 ms gap could shrink to 11 ms and protocols with long gaps
  (Kelvinator/Gree, among others) were rejected after the first frame.
  Delays now run against an `esp_timer` deadline: whole ticks are still
  yielded, the remainder is busy-waited.
* Fix the LEDC carrier going silent when two senders share a pin (two
  `IRsend`s, or an `IRsend` plus IRac's per-message objects). The second
  `begin()` reconfigured the pad as a plain GPIO, unrouting the shared
  channel. With the carrier enabled the pad is now left to the LEDC driver.
* Build on ESP-IDF 6: require `esp_driver_gpio`/`esp_driver_ledc` directly
  on IDF 5.3 and later, since `driver` no longer pulls them in. CI now
  builds the examples on IDF 6.0 too.
* Stop logging `gpio_set_level ... GPIO output gpio_num error` when IRac
  describes a received message: senders that never called `begin()` no
  longer touch a pin on destruction.
* Stop logging `GPIO isr service already installed` when the application
  installs the service itself or calls `enableIRIn()` more than once.

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
