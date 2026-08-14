// Copyright 2025 IRremoteIDF contributors
//
// Transmit example. Cycles through a few well known remote codes so you can
// check the wiring with any IR receiver (or a phone camera pointed at the LED).

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "IRremoteESP8266.h"
#include "IRsend.h"

static const char *TAG = "ir_send_demo";

/// GPIO driving the IR LED. Use a transistor; don't drive the LED directly.
static const uint16_t kIrLedPin = 4;

static IRsend irsend(kIrLedPin);

/// A raw NEC "power" frame, in microseconds. mark, space, mark, space, ...
static const uint16_t kRawPower[67] = {
    9000, 4500, 560, 560, 560, 560, 560, 1690, 560, 560, 560, 560, 560, 560,
    560,  560,  560, 560, 560, 1690, 560, 1690, 560, 560, 560, 1690, 560, 1690,
    560,  1690, 560, 1690, 560, 1690, 560, 560, 560, 560, 560, 560, 560, 1690,
    560,  560,  560, 560, 560, 560, 560, 560, 560, 1690, 560, 1690, 560, 1690,
    560,  560,  560, 1690, 560, 1690, 560, 1690, 560, 1690, 560};

extern "C" void app_main(void) {
  irsend.begin();
  ESP_LOGI(TAG, "Transmitting on GPIO %u", kIrLedPin);

  while (true) {
    ESP_LOGI(TAG, "NEC");
    irsend.sendNEC(0x00FFE01FUL);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Sony, 12 bits, 2 extra repeats");
    irsend.sendSony(0xA90, kSony12Bits, 2);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Samsung");
    irsend.sendSAMSUNG(0xE0E09966UL);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "RC5");
    irsend.sendRC5(0x0, kRC5Bits);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Raw timings at 38kHz");
    irsend.sendRaw(kRawPower, 67, 38);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
