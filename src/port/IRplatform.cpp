// Copyright 2025 IRremoteIDF contributors
//
// ESP-IDF implementation of the platform primitives declared in IRplatform.h.

#include "IRplatform.h"

#ifndef UNIT_TEST

#include <inttypes.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *kTag = "irremote";

uint32_t IRAM_ATTR irMicros(void) {
  return static_cast<uint32_t>(esp_timer_get_time());
}

uint32_t irMillis(void) {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

void IRAM_ATTR irDelayUs(uint32_t usec) {
  if (usec) esp_rom_delay_us(usec);
}

void irDelayMs(uint32_t msec) {
  // vTaskDelay(n) wakes on the n-th tick *edge*, not n ticks after the call,
  // so a plain msec/tick conversion can come up short by almost a full tick.
  // Arduino runs a 1 ms tick and never noticed; ESP-IDF defaults to 10 ms,
  // which is enough to truncate the long inter-frame gaps of protocols like
  // Kelvinator/Gree and get the frame rejected. Work against an absolute
  // deadline instead: sleep only the ticks guaranteed to fit, then busy-wait.
  const int64_t deadline =
      esp_timer_get_time() + static_cast<int64_t>(msec) * 1000;
  const int64_t tick_us = static_cast<int64_t>(portTICK_PERIOD_MS) * 1000;
  for (;;) {
    const int64_t remaining = deadline - esp_timer_get_time();
    if (remaining < 2 * tick_us) break;
    // One tick less than would fit, since the first edge may be imminent.
    vTaskDelay(static_cast<TickType_t>(remaining / tick_us - 1));
  }
  const int64_t remaining = deadline - esp_timer_get_time();
  if (remaining > 0) irDelayUs(static_cast<uint32_t>(remaining));
}

void irYield(void) { taskYIELD(); }

bool irGpioOutput(uint16_t pin) {
  if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
    ESP_LOGE(kTag, "GPIO %u cannot be used as an output", pin);
    return false;
  }
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  return gpio_config(&cfg) == ESP_OK;
}

bool irGpioInput(uint16_t pin, bool pullup) {
  if (!GPIO_IS_VALID_GPIO(pin)) {
    ESP_LOGE(kTag, "GPIO %u is not a valid pin", pin);
    return false;
  }
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  return gpio_config(&cfg) == ESP_OK;
}

void IRAM_ATTR irGpioWrite(uint16_t pin, uint8_t level) {
  gpio_set_level(static_cast<gpio_num_t>(pin), level);
}

uint8_t IRAM_ATTR irGpioRead(uint16_t pin) {
  return static_cast<uint8_t>(gpio_get_level(static_cast<gpio_num_t>(pin)));
}

void irRestart(void) { esp_restart(); }

#ifdef DEBUG
void irDebugPrint(const char *str) { fputs(str, stdout); }
void irDebugPrint(const String &str) { fputs(str.c_str(), stdout); }
void irDebugPrint(uint64_t value) { printf("%" PRIu64, value); }
void irDebugPrint(int64_t value) { printf("%" PRId64, value); }
#endif  // DEBUG

#endif  // UNIT_TEST
