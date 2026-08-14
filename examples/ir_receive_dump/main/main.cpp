// Copyright 2025 IRremoteIDF contributors
//
// Receive example. Decodes anything it can, prints a human readable summary,
// the A/C settings when the message is a climate protocol, and a ready to
// paste raw/source-code dump.

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "IRac.h"
#include "IRrecv.h"
#include "IRremoteESP8266.h"
#include "IRtext.h"
#include "IRutils.h"

static const char *TAG = "ir_receive_dump";

/// GPIO wired to the demodulator's OUT pin (e.g. a TSOP38238).
static const uint16_t kRecvPin = 14;
/// Big enough for the longest A/C messages.
static const uint16_t kCaptureBufferSize = 1024;
/// Milliseconds of silence that mark the end of a message.
/// 50ms suits most A/C remotes; drop to 15 for plain consumer remotes.
static const uint8_t kTimeout = 50;
/// Ignore "UNKNOWN" messages shorter than this.
static const uint16_t kMinUnknownSize = 12;

static IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
static decode_results results;

extern "C" void app_main(void) {
#if DECODE_HASH
  irrecv.setUnknownThreshold(kMinUnknownSize);
#endif  // DECODE_HASH
  irrecv.setTolerance(kTolerance);
  irrecv.enableIRIn();
  ESP_LOGI(TAG, "Listening on GPIO %u", kRecvPin);

  while (true) {
    if (irrecv.decode(&results)) {
      if (results.overflow)
        ESP_LOGW(TAG, "Buffer overflow: raise kCaptureBufferSize");

      printf("%s\n", resultToHumanReadableBasic(&results).c_str());

      const String description = IRAcUtils::resultAcToString(&results);
      if (description.length()) printf("Mesg Desc.: %s\n", description.c_str());

      printf("%s\n", resultToSourceCode(&results).c_str());
      fflush(stdout);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
