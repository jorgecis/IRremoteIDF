// Copyright 2025 IRremoteIDF contributors
//
// Air conditioner example. IRac gives one vendor neutral state struct that
// works across every supported climate protocol, so switching brands is a
// one line change.

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "IRac.h"
#include "IRremoteESP8266.h"
#include "IRutils.h"

static const char *TAG = "ac_control";

/// GPIO driving the IR LED.
static const uint16_t kIrLedPin = 4;

/// Change this to your unit's protocol. `IRac::isProtocolSupported()` tells
/// you whether a given protocol can be driven this way.
static const decode_type_t kProtocol = decode_type_t::COOLIX;

static IRac ac(kIrLedPin);

static void applyState(bool power, float degrees, stdAc::opmode_t mode) {
  ac.next.protocol = kProtocol;
  ac.next.model = 1;  // Most protocols only have one model.
  ac.next.power = power;
  ac.next.mode = mode;
  ac.next.celsius = true;
  ac.next.degrees = degrees;
  ac.next.fanspeed = stdAc::fanspeed_t::kMedium;
  ac.next.swingv = stdAc::swingv_t::kOff;
  ac.next.swingh = stdAc::swingh_t::kOff;
  ac.next.light = true;
  ac.next.beep = false;
  ac.next.econo = false;
  ac.next.filter = false;
  ac.next.turbo = false;
  ac.next.quiet = false;
  ac.next.clean = false;
  ac.next.sleep = -1;
  ac.next.clock = -1;

  if (!ac.sendAc()) ESP_LOGE(TAG, "Protocol is not supported for sending");
}

extern "C" void app_main(void) {
  if (!IRac::isProtocolSupported(kProtocol)) {
    ESP_LOGE(TAG, "%s cannot be controlled via IRac",
             typeToString(kProtocol).c_str());
    return;
  }
  ESP_LOGI(TAG, "Controlling a %s A/C on GPIO %u",
           typeToString(kProtocol).c_str(), kIrLedPin);

  while (true) {
    ESP_LOGI(TAG, "On, cool, 24C");
    applyState(true, 24, stdAc::opmode_t::kCool);
    vTaskDelay(pdMS_TO_TICKS(10000));

    ESP_LOGI(TAG, "On, cool, 20C");
    applyState(true, 20, stdAc::opmode_t::kCool);
    vTaskDelay(pdMS_TO_TICKS(10000));

    ESP_LOGI(TAG, "Off");
    applyState(false, 20, stdAc::opmode_t::kOff);
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
