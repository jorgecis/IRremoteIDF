// Copyright 2025 IRremoteIDF contributors
//
// Optional hardware carrier generation for transmit, using the LEDC
// peripheral. Only compiled when CONFIG_IRREMOTE_TX_HW_CARRIER is set.

#include "IRplatform.h"

#if !defined(UNIT_TEST) && defined(CONFIG_IRREMOTE_TX_HW_CARRIER)

#include <inttypes.h>

#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *kTag = "irremote.carrier";

/// Low speed mode exists on every LEDC-equipped target.
static const ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;

namespace {
/// Per-channel bookkeeping. Index == LEDC channel number.
struct CarrierSlot {
  bool used;
  ledc_timer_t timer;
  uint32_t on_duty;  ///< Duty register value that produces the carrier.
};

CarrierSlot slots[LEDC_CHANNEL_MAX];
bool timer_used[LEDC_TIMER_MAX];
portMUX_TYPE alloc_mux = portMUX_INITIALIZER_UNLOCKED;

/// Program `timer` for `freq`, using the highest duty resolution it accepts.
/// @return The resolution in bits, or 0 if the frequency is unreachable.
uint8_t configureTimer(ledc_timer_t timer, uint32_t freq) {
  for (int bits = LEDC_TIMER_BIT_MAX - 1; bits >= 3; bits--) {
    ledc_timer_config_t cfg = {};
    cfg.speed_mode = kMode;
    cfg.duty_resolution = static_cast<ledc_timer_bit_t>(bits);
    cfg.timer_num = timer;
    cfg.freq_hz = freq;
    cfg.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&cfg) == ESP_OK) return static_cast<uint8_t>(bits);
  }
  return 0;
}
}  // namespace

int8_t irCarrierAttach(uint16_t pin, bool inverted) {
  int8_t channel = -1;
  int timer = -1;
  portENTER_CRITICAL(&alloc_mux);
  for (int i = 0; i < LEDC_CHANNEL_MAX && channel < 0; i++)
    if (!slots[i].used) channel = static_cast<int8_t>(i);
  for (int i = 0; i < LEDC_TIMER_MAX && timer < 0; i++)
    if (!timer_used[i]) timer = i;
  if (channel >= 0 && timer >= 0) {
    slots[channel].used = true;
    slots[channel].timer = static_cast<ledc_timer_t>(timer);
    slots[channel].on_duty = 0;
    timer_used[timer] = true;
  }
  portEXIT_CRITICAL(&alloc_mux);

  if (channel < 0 || timer < 0) {
    ESP_LOGE(kTag, "No free LEDC channel/timer for the IR carrier on GPIO %u",
             pin);
    return -1;
  }

  if (!configureTimer(slots[channel].timer, 38000)) {
    ESP_LOGE(kTag, "Unable to configure LEDC timer %d", timer);
    irCarrierDetach(channel, pin, inverted);
    return -1;
  }

  ledc_channel_config_t cfg = {};
  cfg.gpio_num = pin;
  cfg.speed_mode = kMode;
  cfg.channel = static_cast<ledc_channel_t>(channel);
  cfg.intr_type = LEDC_INTR_DISABLE;
  cfg.timer_sel = slots[channel].timer;
  cfg.duty = 0;  // Idle: no carrier.
  cfg.hpoint = 0;
  cfg.flags.output_invert = inverted ? 1 : 0;
  if (ledc_channel_config(&cfg) != ESP_OK) {
    ESP_LOGE(kTag, "Unable to bind LEDC channel %d to GPIO %u", channel, pin);
    irCarrierDetach(channel, pin, inverted);
    return -1;
  }
  return channel;
}

void irCarrierDetach(int8_t channel, uint16_t pin, bool inverted) {
  if (channel < 0 || channel >= LEDC_CHANNEL_MAX) return;
  ledc_stop(kMode, static_cast<ledc_channel_t>(channel), inverted ? 1 : 0);
  portENTER_CRITICAL(&alloc_mux);
  timer_used[slots[channel].timer] = false;
  slots[channel].used = false;
  portEXIT_CRITICAL(&alloc_mux);
  irGpioOutput(pin);
  irGpioWrite(pin, inverted ? kIrHigh : kIrLow);
}

void irCarrierConfig(int8_t channel, uint32_t freq, uint8_t duty) {
  if (channel < 0 || channel >= LEDC_CHANNEL_MAX) return;
  const uint8_t resolution = configureTimer(slots[channel].timer, freq);
  if (!resolution) {
    ESP_LOGW(kTag, "Carrier frequency %" PRIu32 " Hz is out of range", freq);
    return;
  }
  const uint32_t full = 1UL << resolution;
  uint32_t value = (duty >= 100) ? full : (full * duty) / 100;
  if (!value && duty) value = 1;  // Never round a non-zero duty away.
  slots[channel].on_duty = value;
  irCarrierOff(channel);  // The carrier stays off until mark() asks for it.
}

void IRAM_ATTR irCarrierOn(int8_t channel) {
  if (channel < 0) return;
  ledc_set_duty(kMode, static_cast<ledc_channel_t>(channel),
                slots[channel].on_duty);
  ledc_update_duty(kMode, static_cast<ledc_channel_t>(channel));
}

void IRAM_ATTR irCarrierOff(int8_t channel) {
  if (channel < 0) return;
  ledc_set_duty(kMode, static_cast<ledc_channel_t>(channel), 0);
  ledc_update_duty(kMode, static_cast<ledc_channel_t>(channel));
}

#endif  // !UNIT_TEST && CONFIG_IRREMOTE_TX_HW_CARRIER
