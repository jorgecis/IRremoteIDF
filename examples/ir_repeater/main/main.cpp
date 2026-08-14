// Copyright 2025 IRremoteIDF contributors
//
// Smart repeater: captures a message, decodes it, and retransmits it in the
// same protocol. Unknown messages are replayed from their raw timings.
// Handy for extending a remote's reach around a corner.

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "IRrecv.h"
#include "IRremoteESP8266.h"
#include "IRsend.h"
#include "IRutils.h"

static const char *TAG = "ir_repeater";

/// GPIO wired to the demodulator's OUT pin.
static const uint16_t kRecvPin = 14;
/// GPIO driving the IR LED.
static const uint16_t kIrLedPin = 4;
/// 1024 entries == ~511 bits, enough for the longest A/C messages.
static const uint16_t kCaptureBufferSize = 1024;
/// Milliseconds of silence that mark the end of a message.
static const uint8_t kTimeout = 50;
/// Carrier used when replaying a message we could not identify.
static const uint16_t kFrequency = 38000;

static IRsend irsend(kIrLedPin);
static IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false);
static decode_results results;

extern "C" void app_main(void) {
  irrecv.enableIRIn();
  irsend.begin();
  ESP_LOGI(TAG, "Repeating GPIO %u -> GPIO %u", kRecvPin, kIrLedPin);

  while (true) {
    if (irrecv.decode(&results)) {
      const decode_type_t protocol = results.decode_type;
      uint16_t size = results.bits;
      bool success = true;

      if (protocol == decode_type_t::UNKNOWN) {
        // Replay the raw timings. resultToRawArray() allocates the array.
        uint16_t *raw_array = resultToRawArray(&results);
        size = getCorrectedRawLength(&results);
#if SEND_RAW
        irsend.sendRaw(raw_array, size, kFrequency);
#else
        success = false;
#endif  // SEND_RAW
        delete[] raw_array;
      } else if (hasACState(protocol)) {
        success = irsend.send(protocol, results.state, size / 8);
      } else {
        success = irsend.send(protocol, results.value, size);
      }

      // Only resume now, so we don't capture our own transmission.
      irrecv.resume();

      ESP_LOGI(TAG, "A %u-bit %s message was %sretransmitted", size,
               typeToString(protocol).c_str(), success ? "" : "NOT ");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
