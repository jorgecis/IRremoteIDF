// Copyright 2025 IRremoteIDF contributors
//
// Optional transmit through the RMT peripheral. Only compiled when
// CONFIG_IRREMOTE_TX_RMT is set.
//
// The protocol code was written for a CPU that drives the pin itself: it
// calls mark() and space() one after another and expects each call to last
// that long. RMT wants the opposite, a whole frame up front, which it then
// times on its own, carrier included, however busy the CPU is. This file
// bridges the two. mark()/space() append symbols to a buffer, and the buffer
// is handed to the hardware when the frame is complete.
//
// Knowing when a frame is complete is the whole problem, because nothing in
// the protocol code says so. Three cues are used:
//
//  * A space of kFlushGapUs or more. Every protocol ends its frames with a
//    gap, and the gaps between the sections of long A/C messages are just as
//    long, so a flush there costs nothing: a few hundred microseconds of
//    encoding added to a space of tens of milliseconds.
//  * ledOff(), which sendRaw() and the GlobalCache sender call because their
//    data may end in a mark.
//  * A timer: a buffer that has not grown for kIdleFlushUs is sent as is.
//    This is the fallback for the few protocols whose closing gap is short
//    (Trotec ends with 1.5 ms), and the only case where send() returns
//    before the frame is on the air.
//
// A frame longer than a chunk streams: a full chunk is transmitted without
// waiting and the next one filled while it plays. The driver queues them
// back to back, so the seam costs an interrupt latency on one space.

#include "IRplatform.h"

#if !defined(UNIT_TEST) && defined(CONFIG_IRREMOTE_TX_RMT)

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

static const char *kTag = "irremote.rmt";

namespace {
/// One tick per microsecond: the unit every protocol is written in.
const uint32_t kResolutionHz = 1000000;
/// Half an RMT symbol holds a 15-bit duration. Longer marks/spaces are split.
const uint32_t kMaxHalfUs = 0x7FFF;
/// Symbol words per chunk. A word holds two edges (a mark and a space), so
/// 512 words take a 1024-edge frame: longer than any A/C state message
/// (Hitachi's 344-bit one is ~700 edges) and the examples' raw captures.
const size_t kChunkWords = 512;
/// Chunks per slot. One more than the driver's queue depth: rmt_transmit()
/// blocks until the oldest transmission finishes once `depth` are in flight,
/// so when it returns the chunk the writer moves on to is no longer in use.
const size_t kChunks = 3;
const size_t kQueueDepth = kChunks - 1;
/// A space at least this long ends a frame or a section, and the buffer is
/// transmitted and waited for. Every in-frame space that has to be precise
/// is shorter; the longest is LG2's 9.9 ms header space.
const uint32_t kFlushGapUs = 10000;
/// A buffer that has not grown for this long is transmitted by the timer.
const uint32_t kIdleFlushUs = 5000;
/// Added to the expected air time when waiting for the hardware.
const int kWaitMarginMs = 100;
/// Slots are keyed by GPIO, one TX channel each. No chip has more than 8,
/// and the driver reports running out on its own, so this is only a bound.
/// (IDF 6 dropped the SOC_RMT_* counts that could have sized it exactly.)
const size_t kSlots = 8;

struct Chunk {
  rmt_symbol_word_t *words;
  size_t halves;  ///< Edges written so far, two per word.
  uint32_t us;    ///< Their total duration.
};

/// Per-pin state. Shared by every IRsend on that pin, reference counted, for
/// the reason given in IRcarrier.cpp: IRac builds a throw-away IRsend per
/// message, and a channel of its own for each would unroute the pad from
/// the long-lived one.
struct Slot {
  uint16_t refs;
  uint16_t pin;
  rmt_channel_handle_t channel;
  rmt_encoder_handle_t encoder;
  /// Serialises claiming a chunk and handing it to the driver, so chunks
  /// never go out of order. The writer holds it while rmt_transmit() blocks
  /// on a full queue; the idle timer only tries it, never waits.
  SemaphoreHandle_t submit_lock;
  esp_timer_handle_t idle_timer;
  Chunk chunks[kChunks];
  size_t fill;          ///< Index of the chunk being written.
  size_t in_flight;     ///< Transmissions handed to the driver and not done.
  int64_t last_append;  ///< esp_timer time of the most recent edge.
  int64_t queue_end;    ///< esp_timer time when the in-flight ones finish.
  TaskHandle_t owner;   ///< Task that last wrote an edge. See irTxLeadUs().
  bool timer_armed;
  uint32_t freq;        ///< Carrier the channel is programmed for, 0 if none.
  uint8_t duty;
};

Slot slots[kSlots];
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/// Driver callback, from the RMT ISR, when a transmission has finished.
bool IRAM_ATTR onTransDone(rmt_channel_handle_t,
                           const rmt_tx_done_event_data_t *, void *arg) {
  Slot *s = static_cast<Slot *>(arg);
  portENTER_CRITICAL_ISR(&mux);
  if (s->in_flight) s->in_flight--;
  portEXIT_CRITICAL_ISR(&mux);
  return false;
}

/// Hand the chunk being written to the driver. The caller holds submit_lock.
/// @param nonblocking Give up instead of waiting for a queue slot. Only the
///   timer uses it; it runs on the esp_timer task, which must not stall.
/// @return false if the chunk was left in place because the queue was full.
bool submit(Slot *s, bool nonblocking) {
  portENTER_CRITICAL(&mux);
  Chunk *c = &s->chunks[s->fill];
  const size_t halves = c->halves;
  const uint32_t us = c->us;
  if (!halves) {
    portEXIT_CRITICAL(&mux);
    return true;
  }
  if (nonblocking && s->in_flight >= kQueueDepth) {
    portEXIT_CRITICAL(&mux);
    return false;
  }
  // Claim it: the writer goes on with the next chunk, which is free (see
  // kChunks), while this one belongs to the hardware until it is done.
  s->fill = (s->fill + 1) % kChunks;
  s->chunks[s->fill].halves = 0;
  s->chunks[s->fill].us = 0;
  s->in_flight++;
  portEXIT_CRITICAL(&mux);

  // An odd number of edges leaves half a word unused. A zero duration is the
  // hardware's end marker, which is where this transmission ends anyway.
  if (halves & 1) {
    c->words[halves / 2].duration1 = 0;
    c->words[halves / 2].level1 = 0;
  }
  rmt_transmit_config_t cfg = {};
  cfg.loop_count = 0;
  cfg.flags.eot_level = 0;  // Idle at the space level.
  cfg.flags.queue_nonblocking = nonblocking ? 1 : 0;
  const esp_err_t err = rmt_transmit(s->channel, s->encoder, c->words,
                                     ((halves + 1) / 2) * sizeof(*c->words),
                                     &cfg);
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&mux);
  if (err == ESP_OK) {
    s->queue_end = std::max(s->queue_end, now) + us;
  } else {
    s->in_flight--;
  }
  portEXIT_CRITICAL(&mux);
  if (err != ESP_OK)
    ESP_LOGE(kTag, "rmt_transmit on GPIO %u failed: %s", s->pin,
             esp_err_to_name(err));
  return true;
}

/// Wait until the hardware has sent everything handed to it.
void waitIdle(Slot *s) {
  portENTER_CRITICAL(&mux);
  const int64_t remaining = s->queue_end - esp_timer_get_time();
  portEXIT_CRITICAL(&mux);
  const int ms = (remaining > 0 ? remaining / 1000 : 0) + kWaitMarginMs;
  if (rmt_tx_wait_all_done(s->channel, ms) != ESP_OK)
    ESP_LOGE(kTag, "Transmission on GPIO %u did not finish in %d ms", s->pin,
             ms);
}

void armTimer(Slot *s);

/// esp_timer callback: send what the writer has left sitting in the buffer.
void onIdle(void *arg) {
  Slot *s = static_cast<Slot *>(arg);
  portENTER_CRITICAL(&mux);
  s->timer_armed = false;
  const bool pending = s->chunks[s->fill].halves > 0;
  const int64_t idle = esp_timer_get_time() - s->last_append;
  portEXIT_CRITICAL(&mux);
  if (!pending) return;
  if (idle >= kIdleFlushUs && xSemaphoreTake(s->submit_lock, 0) == pdTRUE) {
    submit(s, true);
    xSemaphoreGive(s->submit_lock);
  }
  armTimer(s);  // Still pending (a writer stall, a full queue): look again.
}

void armTimer(Slot *s) {
  portENTER_CRITICAL(&mux);
  const bool pending = s->chunks[s->fill].halves > 0;
  const bool arm = pending && !s->timer_armed;
  if (arm) s->timer_armed = true;
  portEXIT_CRITICAL(&mux);
  if (arm) esp_timer_start_once(s->idle_timer, kIdleFlushUs);
}

/// Append one edge. The hot path: a long A/C frame is ~700 of these, and
/// their total is what the gap before the frame grows by.
void append(Slot *s, uint8_t level, uint32_t usec) {
  while (usec) {
    const uint16_t d = static_cast<uint16_t>(std::min(usec, kMaxHalfUs));
    usec -= d;
    // The edge and the count go in together, or the idle timer could claim
    // the chunk between them and the edge would be lost.
    portENTER_CRITICAL(&mux);
    Chunk *c = &s->chunks[s->fill];
    rmt_symbol_word_t *w = &c->words[c->halves / 2];
    if (c->halves & 1) {
      w->duration1 = d;
      w->level1 = level;
    } else {
      w->duration0 = d;
      w->level0 = level;
    }
    c->halves++;
    c->us += d;
    s->last_append = esp_timer_get_time();
    s->owner = xTaskGetCurrentTaskHandle();
    const bool full = c->halves == 2 * kChunkWords;
    portEXIT_CRITICAL(&mux);
    if (full) {
      xSemaphoreTake(s->submit_lock, portMAX_DELAY);
      submit(s, false);
      xSemaphoreGive(s->submit_lock);
    }
  }
  armTimer(s);
}

void release(Slot *s, bool inverted) {
  if (s->idle_timer) {
    esp_timer_stop(s->idle_timer);
    esp_timer_delete(s->idle_timer);
  }
  if (s->channel) {
    rmt_disable(s->channel);
    rmt_del_channel(s->channel);
  }
  if (s->encoder) rmt_del_encoder(s->encoder);
  if (s->submit_lock) vSemaphoreDelete(s->submit_lock);
  free(s->chunks[0].words);
  const uint16_t pin = s->pin;
  portENTER_CRITICAL(&mux);
  memset(s, 0, sizeof(*s));
  portEXIT_CRITICAL(&mux);
  // Leave the pad as a plain output at its idle level, as before begin().
  irGpioOutput(pin);
  irGpioWrite(pin, inverted ? kIrHigh : kIrLow);
}
}  // namespace

int8_t irRmtAttach(uint16_t pin, bool inverted) {
  int8_t index = -1;
  portENTER_CRITICAL(&mux);
  for (size_t i = 0; i < kSlots; i++) {
    if (slots[i].refs && slots[i].pin == pin) {
      slots[i].refs++;
      portEXIT_CRITICAL(&mux);
      return static_cast<int8_t>(i);
    }
  }
  for (size_t i = 0; i < kSlots && index < 0; i++) {
    if (slots[i].refs) continue;
    index = static_cast<int8_t>(i);
    slots[i].refs = 1;  // Reserve it before the slow part below.
    slots[i].pin = pin;
  }
  portEXIT_CRITICAL(&mux);
  if (index < 0) {
    ESP_LOGE(kTag, "No free RMT slot for GPIO %u", pin);
    return -1;
  }
  Slot *s = &slots[index];

  rmt_tx_channel_config_t cfg = {};
  cfg.gpio_num = static_cast<gpio_num_t>(pin);
  cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  cfg.resolution_hz = kResolutionHz;
  cfg.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
  cfg.trans_queue_depth = kQueueDepth;
  cfg.flags.invert_out = inverted ? 1 : 0;
  esp_err_t err = rmt_new_tx_channel(&cfg, &s->channel);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "No RMT TX channel for GPIO %u: %s", pin,
             esp_err_to_name(err));
    release(s, inverted);
    return -1;
  }
  rmt_copy_encoder_config_t ecfg = {};
  err = rmt_new_copy_encoder(&ecfg, &s->encoder);
  if (err == ESP_OK) {
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = onTransDone;
    err = rmt_tx_register_event_callbacks(s->channel, &cbs, s);
  }
  if (err == ESP_OK) err = rmt_enable(s->channel);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "RMT setup for GPIO %u failed: %s", pin,
             esp_err_to_name(err));
    release(s, inverted);
    return -1;
  }

  // The driver insists on a payload in internal RAM.
  rmt_symbol_word_t *words = static_cast<rmt_symbol_word_t *>(heap_caps_malloc(
      kChunks * kChunkWords * sizeof(rmt_symbol_word_t),
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  s->submit_lock = xSemaphoreCreateMutex();
  esp_timer_create_args_t targs = {};
  targs.callback = onIdle;
  targs.arg = s;
  targs.dispatch_method = ESP_TIMER_TASK;
  targs.name = "irremote.rmt";
  if (!words || !s->submit_lock ||
      esp_timer_create(&targs, &s->idle_timer) != ESP_OK) {
    ESP_LOGE(kTag, "Out of memory setting up RMT transmit on GPIO %u", pin);
    free(words);
    release(s, inverted);
    return -1;
  }
  for (size_t i = 0; i < kChunks; i++)
    s->chunks[i].words = words + i * kChunkWords;
  return index;
}

void irRmtDetach(int8_t slot, bool inverted) {
  if (slot < 0 || static_cast<size_t>(slot) >= kSlots) return;
  Slot *s = &slots[slot];
  portENTER_CRITICAL(&mux);
  const bool last = s->refs <= 1;
  if (!last) s->refs--;
  portEXIT_CRITICAL(&mux);
  // Another IRsend still uses this pin: leave the channel to it.
  if (!last) return;
  irRmtFlush(slot);
  release(s, inverted);
}

void irRmtCarrier(int8_t slot, uint32_t freq, uint8_t duty) {
  if (slot < 0 || static_cast<size_t>(slot) >= kSlots) return;
  Slot *s = &slots[slot];
  if (duty >= 100) freq = 0;  // No modulation: a mark is a solid high.
  if (s->freq == freq && s->duty == duty) return;
  // Whatever is still buffered was built for the old carrier.
  irRmtFlush(slot);
  rmt_carrier_config_t cfg = {};
  cfg.frequency_hz = freq;
  cfg.duty_cycle = duty / 100.0f;
  const esp_err_t err = rmt_apply_carrier(s->channel, freq ? &cfg : NULL);
  if (err != ESP_OK) {
    ESP_LOGW(kTag, "Carrier %" PRIu32 " Hz at %u%% rejected: %s", freq, duty,
             esp_err_to_name(err));
    return;
  }
  s->freq = freq;
  s->duty = duty;
}

void irRmtMark(int8_t slot, uint32_t usec) {
  if (slot < 0 || static_cast<size_t>(slot) >= kSlots || !usec) return;
  append(&slots[slot], 1, usec);
}

void irRmtSpace(int8_t slot, uint32_t usec) {
  if (slot < 0 || static_cast<size_t>(slot) >= kSlots || !usec) return;
  append(&slots[slot], 0, usec);
  if (usec >= kFlushGapUs) irRmtFlush(slot);
}

void irRmtFlush(int8_t slot) {
  if (slot < 0 || static_cast<size_t>(slot) >= kSlots) return;
  Slot *s = &slots[slot];
  xSemaphoreTake(s->submit_lock, portMAX_DELAY);
  submit(s, false);
  xSemaphoreGive(s->submit_lock);
  waitIdle(s);
}

uint32_t irTxLeadUs(void) {
  const TaskHandle_t me = xTaskGetCurrentTaskHandle();
  const int64_t now = esp_timer_get_time();
  int64_t lead = 0;
  portENTER_CRITICAL(&mux);
  for (size_t i = 0; i < kSlots; i++) {
    const Slot *s = &slots[i];
    if (!s->refs || s->owner != me) continue;
    lead += s->chunks[s->fill].us;
    if (s->queue_end > now) lead += s->queue_end - now;
  }
  portEXIT_CRITICAL(&mux);
  return static_cast<uint32_t>(lead);
}

#endif  // !UNIT_TEST && CONFIG_IRREMOTE_TX_RMT
