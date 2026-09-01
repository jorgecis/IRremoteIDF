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
/// The LEDC driver's own log tag, muted while probing for a resolution.
static const char *kLedcTag = "ledc";

/// Low speed mode exists on every LEDC-equipped target.
static const ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;

namespace {
/// Per-channel bookkeeping. Index == LEDC channel number.
///
/// A slot is keyed by GPIO and reference counted, because more than one
/// IRsend can legitimately share a pin. IRac is the case that forces it: it
/// keeps only a pin number and builds a throw-away protocol object - each
/// with its own IRsend - for every message it sends. Handing each of those a
/// channel of its own would re-route the pad to the newest one, silently
/// killing whichever long-lived IRsend the caller was still using, and would
/// run the eight channels out after eight messages.
struct CarrierSlot {
  uint16_t refs;     ///< How many IRsend instances hold this channel.
  uint16_t pin;      ///< The GPIO it drives. Only meaningful while refs > 0.
  ledc_timer_t timer;
  uint32_t on_duty;  ///< Duty register value that produces the carrier.
  uint32_t freq;     ///< What the timer is currently programmed for, 0 if new.
  uint8_t duty_pct;  ///< The duty irCarrierConfig() was last given.
};

CarrierSlot slots[LEDC_CHANNEL_MAX];
bool timer_used[LEDC_TIMER_MAX];
portMUX_TYPE alloc_mux = portMUX_INITIALIZER_UNLOCKED;

/// Program `timer` for `freq`, using the highest duty resolution it accepts.
/// @return The resolution in bits, or 0 if the frequency is unreachable.
///
/// The search walks down from the maximum because the achievable resolution
/// depends on the clock the driver picks, which is not knowable from here.
/// Each rejected step logs at ESP_LOG_ERROR ("cannot be achieved"), so the
/// driver's own tag is muted for the duration: those lines are a description
/// of the search, not of a fault, and a caller reading the log has no way to
/// tell them from a real failure.
uint8_t configureTimer(ledc_timer_t timer, uint32_t freq) {
  const esp_log_level_t previous = esp_log_level_get(kLedcTag);
  esp_log_level_set(kLedcTag, ESP_LOG_NONE);
  uint8_t found = 0;
  for (int bits = LEDC_TIMER_BIT_MAX - 1; bits >= 3 && !found; bits--) {
    ledc_timer_config_t cfg = {};
    cfg.speed_mode = kMode;
    cfg.duty_resolution = static_cast<ledc_timer_bit_t>(bits);
    cfg.timer_num = timer;
    cfg.freq_hz = freq;
    cfg.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&cfg) == ESP_OK) found = static_cast<uint8_t>(bits);
  }
  esp_log_level_set(kLedcTag, previous);
  return found;
}
}  // namespace

int8_t irCarrierAttach(uint16_t pin, bool inverted) {
  int8_t channel = -1;
  int timer = -1;
  portENTER_CRITICAL(&alloc_mux);
  // An existing channel for this pin is shared rather than duplicated.
  for (int i = 0; i < LEDC_CHANNEL_MAX; i++) {
    if (slots[i].refs && slots[i].pin == pin) {
      slots[i].refs++;
      portEXIT_CRITICAL(&alloc_mux);
      return static_cast<int8_t>(i);
    }
  }
  for (int i = 0; i < LEDC_CHANNEL_MAX && channel < 0; i++)
    if (!slots[i].refs) channel = static_cast<int8_t>(i);
  for (int i = 0; i < LEDC_TIMER_MAX && timer < 0; i++)
    if (!timer_used[i]) timer = i;
  if (channel >= 0 && timer >= 0) {
    slots[channel].refs = 1;
    slots[channel].pin = pin;
    slots[channel].timer = static_cast<ledc_timer_t>(timer);
    slots[channel].on_duty = 0;
    slots[channel].freq = 0;  // Force the next irCarrierConfig() to program it.
    slots[channel].duty_pct = 0;
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
  portENTER_CRITICAL(&alloc_mux);
  const bool last = slots[channel].refs <= 1;
  if (last) {
    timer_used[slots[channel].timer] = false;
    slots[channel].refs = 0;
  } else {
    slots[channel].refs--;
  }
  portEXIT_CRITICAL(&alloc_mux);
  // Another IRsend is still using this pin: leave the peripheral alone.
  if (!last) return;
  ledc_stop(kMode, static_cast<ledc_channel_t>(channel), inverted ? 1 : 0);
  irGpioOutput(pin);
  irGpioWrite(pin, inverted ? kIrHigh : kIrLow);
}

void irCarrierConfig(int8_t channel, uint32_t freq, uint8_t duty) {
  if (channel < 0 || channel >= LEDC_CHANNEL_MAX) return;
  // enableIROut() runs before every transmission and almost always asks for
  // the same 38kHz it asked for last time. Reprogramming the timer for that
  // costs a divider recalculation and a peripheral write per message, for no
  // change; the carrier only has to be put back to its off state.
  if (slots[channel].freq == freq && slots[channel].duty_pct == duty) {
    irCarrierOff(channel);
    return;
  }
  const uint8_t resolution = configureTimer(slots[channel].timer, freq);
  if (!resolution) {
    ESP_LOGW(kTag, "Carrier frequency %" PRIu32 " Hz is out of range", freq);
    return;
  }
  const uint32_t full = 1UL << resolution;
  uint32_t value = (duty >= 100) ? full : (full * duty) / 100;
  if (!value && duty) value = 1;  // Never round a non-zero duty away.
  slots[channel].on_duty = value;
  slots[channel].freq = freq;
  slots[channel].duty_pct = duty;
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
