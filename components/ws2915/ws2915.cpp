#include "ws2915.h"

#ifdef USE_ESP32

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cstring>
#include <esp_attr.h>
#include <esp_clk_tree.h>

namespace esphome::ws2915 {

static const char *const TAG = "ws2915";

static constexpr uint32_t RMT_DURATION_MAX = 0x7FFF;  // 15-bit duration field
// Keep the main loop in high-frequency mode this long after the last change, so slow
// transitions (16-bit values that change only every few loops) don't toggle it constantly.
static constexpr uint32_t TRANSITION_HOLD_MS = 250;
// Scheduler ids for the per-group power-on timeouts (POWER_TIMEOUT_ID + group).
static constexpr uint32_t POWER_TIMEOUT_ID = 0x57290000;

static uint32_t rmt_resolution_hz() {
  uint32_t freq = 0;
  esp_clk_tree_src_get_freq_hz((soc_module_clk_t) RMT_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &freq);
  return freq;
}

static uint32_t ns_to_ticks(uint32_t ns, uint32_t res_hz) {
  return static_cast<uint32_t>((static_cast<uint64_t>(ns) * res_hz + 500000000ULL) / 1000000000ULL);
}

static rmt_symbol_word_t make_symbol(uint32_t ticks0, uint32_t level0, uint32_t ticks1, uint32_t level1) {
  rmt_symbol_word_t s{};
  s.duration0 = ticks0;
  s.level0 = level0;
  s.duration1 = ticks1;
  s.level1 = level1;
  return s;
}

static size_t IRAM_ATTR HOT encoder_callback(const void *data, size_t size, size_t symbols_written,
                                             size_t symbols_free, rmt_symbol_word_t *symbols, bool *done,
                                             void *arg) {
  return encode_symbols(*static_cast<const EncoderParams *>(arg), static_cast<const uint8_t *>(data), size,
                        symbols_written, symbols_free, symbols, done);
}

void WS2915Component::set_gain(uint8_t channel, uint8_t gain) {
  if (channel >= CHANNELS_PER_CHIP)
    return;
  if (gain > GAIN_MAX)
    gain = GAIN_MAX;
  if (this->gain_[channel] == gain)
    return;
  this->gain_[channel] = gain;
  this->request_frame_();
}

void WS2915Component::set_header_format(HeaderFormat format) {
  if (this->header_format_ == format)
    return;
  this->header_format_ = format;
  this->request_frame_();
}

void WS2915Component::set_level_(uint16_t chip, uint8_t channel, float state) {
  if (state < 0.0f)
    state = 0.0f;
  if (state > 1.0f)
    state = 1.0f;
  const float scale = static_cast<float>(full_scale(this->chip_type_));
  this->set_value_(chip, channel, static_cast<uint16_t>(state * scale + 0.5f));
}

void WS2915Component::set_value_(uint16_t chip, uint8_t channel, uint16_t value) {
  if (chip >= this->num_chips_ || channel >= CHANNELS_PER_CHIP || this->levels_ == nullptr) {
    if (!this->range_warned_) {
      ESP_LOGW(TAG, "Write to chip %u channel %u ignored (num_chips=%u%s)", chip, channel, this->num_chips_,
               this->levels_ == nullptr ? ", not set up" : "");
      this->range_warned_ = true;
    }
    return;
  }
  const uint16_t max = static_cast<uint16_t>(full_scale(this->chip_type_));
  if (value > max)
    value = max;
  const size_t index = static_cast<size_t>(chip) * CHANNELS_PER_CHIP + channel;
  uint16_t &level = this->levels_[index];
  if (level == value)
    return;
  const bool was_on = level != 0;
  level = value;
  if (was_on != (value != 0)) {
    const uint8_t group = this->group_at_(index);
    if (value != 0) {
      this->groups_[group].active++;
    } else {
      this->groups_[group].active--;
    }
    this->update_group_(group);
  }
  this->mark_changed_();
}

#ifdef USE_POWER_SUPPLY
uint8_t WS2915Component::group_for_(power_supply::PowerSupply *psu, uint32_t keep_on_time_ms) {
  for (size_t i = 1; i < this->groups_.size(); i++) {
    if (this->groups_[i].psu == psu)
      return static_cast<uint8_t>(i);
  }
  PowerGroup g;
  g.psu = psu;
  g.req.set_parent(psu);
  g.keep_on_ms = keep_on_time_ms;
  this->groups_.push_back(g);
  return static_cast<uint8_t>(this->groups_.size() - 1);
}

void WS2915Component::add_channel_power_supply(uint16_t chip, uint8_t channel, power_supply::PowerSupply *psu,
                                               uint32_t keep_on_time_ms) {
  this->channel_supplies_.push_back({chip, channel, this->group_for_(psu, keep_on_time_ms)});
}

void WS2915Component::set_default_power_supply(power_supply::PowerSupply *psu, uint32_t keep_on_time_ms) {
  this->default_group_ = this->group_for_(psu, keep_on_time_ms);
}
#endif

// Hold a group's supply while any of its channels is above 0. After a real switch-on its
// channels are sent as 0 for power_on_delay, so they light up (or fade in) only once the
// boards have power.
void WS2915Component::update_group_(uint8_t group) {
#ifdef USE_POWER_SUPPLY
  PowerGroup &g = this->groups_[group];
  if (g.psu == nullptr)
    return;
  if (g.active > 0 && !g.requested) {
    // Still on if something else holds it, or we released it less than keep_on_time ago
    // (is_enabled() only counts active requests, not the keep-on period).
    const bool on = g.psu->is_enabled() || (g.released && millis() - g.released_ms < g.keep_on_ms);
    g.req.request();
    g.requested = true;
    if (!on)
      this->start_group_wait_(group, this->power_on_delay_ms_);
  } else if (g.active == 0 && g.requested) {
    g.req.unrequest();
    g.requested = false;
    g.released = true;
    g.released_ms = millis();
  }
#endif
}

void WS2915Component::start_group_wait_(uint8_t group, uint32_t ms) {
  if (ms == 0 || this->out_levels_ == nullptr)
    return;
  this->groups_[group].waiting = true;
  ESP_LOGD(TAG, "Power-on (group %u): holding its channels at 0 for %" PRIu32 " ms", group, ms);
  this->set_timeout(POWER_TIMEOUT_ID + group, ms, [this, group]() {
    ESP_LOGD(TAG, "Power ready (group %u)", group);
#ifdef USE_LIGHT
    // Replay fade-ins that ran in the dark, once all supplies of that light are ready. This
    // runs while the group is still held, so no frame with the old levels goes out first.
    for (auto *w : this->lights_) {
      if ((w->groups & (1u << (group & 31))) == 0)
        continue;
      bool other_waiting = false;
      for (size_t k = 0; k < this->groups_.size() && k < 32; k++)
        other_waiting |= k != group && (w->groups & (1u << k)) != 0 && this->groups_[k].waiting;
      if (!other_waiting)
        this->replay_light_(w);
    }
#endif
    this->groups_[group].waiting = false;
    this->dirty_ = true;
    this->enable_loop();
  });
}

#ifdef USE_LIGHT
void WS2915Component::add_light(light::LightState *light, const std::vector<Channel *> &outputs) {
  auto *w = new LightWatch();  // NOLINT(cppcoreguidelines-owning-memory): lives as long as the light
  w->light = light;
  w->outputs = outputs;
  light->add_remote_values_listener(w);
  light->add_target_state_reached_listener(w);
  this->lights_.push_back(w);
}

// The light's transition ran while its boards had no power. If it has already finished,
// run it again from off with the length that call used. A transition still running is
// left alone (the rest of it is visible); a call without transition has nothing to replay.
void WS2915Component::replay_light_(LightWatch *w) {
  light::LightState *l = w->light;
  if (!l->remote_values.is_on() || l->is_transformer_active() || !w->reached || w->length_ms == 0)
    return;
  if (!(l->get_effect_name() == "None"))
    return;
  const light::LightColorValues target = l->remote_values;
  light::LightColorValues start = target;
  start.set_state(0.0f);
  l->current_values = start;
  auto call = l->make_call();
  call.from_light_color_values(target);
  call.set_transition_length(w->length_ms);
  call.perform();
  // Write the transition's start (off) to the outputs now, instead of on the light's next
  // loop, so the line never shows the old target in between.
  l->loop();
  ESP_LOGD(TAG, "Replaying the %" PRIu32 " ms fade-in of '%s'", w->length_ms, l->get_name().c_str());
}
#endif

// A value changed: send a frame, and keep the main loop fast while changes keep coming.
void WS2915Component::mark_changed_() {
  if (this->use_high_freq_) {
    this->last_change_ms_ = millis();
    if (!this->high_freq_active_) {
      this->high_freq_.start();
      this->high_freq_active_ = true;
    }
  }
  this->request_frame_();
}

void WS2915Component::request_frame_() {
  if (!this->dirty_) {
    this->dirty_ = true;
    this->enable_loop();
  }
}

void WS2915Component::setup() {
  const size_t count = this->num_levels_();
  const size_t len = this->get_buffer_size_();
  RAMAllocator<uint16_t> level_alloc(RAMAllocator<uint16_t>::ALLOC_INTERNAL);
  RAMAllocator<uint8_t> tx_alloc(RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  this->levels_ = level_alloc.allocate(count);
  this->tx_buf_ = tx_alloc.allocate(len);
  if (this->levels_ == nullptr || this->tx_buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate frame buffers (%u B)", static_cast<unsigned>(count * 2 + len));
    this->mark_failed();
    return;
  }
  memset(this->levels_, 0, count * sizeof(uint16_t));
  memset(this->tx_buf_, 0, len);
  if (this->groups_.size() > 1) {
    RAMAllocator<uint8_t> group_alloc(RAMAllocator<uint8_t>::ALLOC_INTERNAL);
    this->group_of_ = group_alloc.allocate(count);
    if (this->group_of_ == nullptr) {
      ESP_LOGE(TAG, "Cannot allocate the power supply map (%u B)", static_cast<unsigned>(count));
      this->mark_failed();
      return;
    }
    memset(this->group_of_, this->default_group_, count);
    for (const auto &cs : this->channel_supplies_) {
      if (cs.chip < this->num_chips_ && cs.channel < CHANNELS_PER_CHIP)
        this->group_of_[static_cast<size_t>(cs.chip) * CHANNELS_PER_CHIP + cs.channel] = cs.group;
    }
  }
  if (this->power_on_delay_ms_ != 0) {
    this->out_levels_ = level_alloc.allocate(count);
    if (this->out_levels_ == nullptr)
      ESP_LOGW(TAG, "No RAM for the power-on buffer; power_on_delay disabled");
  }
#ifdef USE_LIGHT
  // Outputs have their chip/channel by now (set after add_light() in codegen).
  for (auto *w : this->lights_) {
    for (auto *o : w->outputs) {
      const size_t idx = static_cast<size_t>(o->get_chip()) * CHANNELS_PER_CHIP + o->get_channel();
      if (idx < count)
        w->groups |= 1u << (this->group_at_(idx) & 31);
    }
  }
#endif

  const uint32_t res = rmt_resolution_hz();
  const uint32_t t0h = ns_to_ticks(this->bit0_high_ns_, res), t0l = ns_to_ticks(this->bit0_low_ns_, res);
  const uint32_t t1h = ns_to_ticks(this->bit1_high_ns_, res), t1l = ns_to_ticks(this->bit1_low_ns_, res);
  const uint32_t rst = ns_to_ticks(this->reset_ns_, res);
  const uint32_t rst0 = rst / 2, rst1 = rst - rst0;
  if (res == 0 || t0h == 0 || t0l == 0 || t1h == 0 || t1l == 0 || t0h > RMT_DURATION_MAX ||
      t0l > RMT_DURATION_MAX || t1h > RMT_DURATION_MAX || t1l > RMT_DURATION_MAX || rst1 > RMT_DURATION_MAX) {
    ESP_LOGE(TAG, "Timing does not fit an RMT symbol at %" PRIu32 " Hz", res);
    this->mark_failed();
    return;
  }
  this->params_.bit0 = make_symbol(t0h, 1, t0l, 0);
  this->params_.bit1 = make_symbol(t1h, 1, t1l, 0);
  // Reset: both halves low, so up to 2 x 32767 ticks (819 us at 80 MHz).
  this->params_.reset = make_symbol(rst0, 0, rst1, 0);

  rmt_tx_channel_config_t ch;
  memset(&ch, 0, sizeof(ch));
  ch.clk_src = RMT_CLK_SRC_DEFAULT;
  ch.resolution_hz = res;
  ch.gpio_num = static_cast<gpio_num_t>(this->pin_);
  ch.mem_block_symbols = this->rmt_symbols_;
  ch.trans_queue_depth = 1;
  ch.flags.invert_out = this->inverted_;
  ch.flags.with_dma = this->use_dma_;
  ch.intr_priority = 0;
  if (rmt_new_tx_channel(&ch, &this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX channel creation failed");
    this->mark_failed();
    return;
  }

  rmt_simple_encoder_config_t enc;
  memset(&enc, 0, sizeof(enc));
  enc.callback = encoder_callback;
  enc.arg = &this->params_;
  enc.min_chunk_size = 1;
  if (rmt_new_simple_encoder(&enc, &this->encoder_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT encoder creation failed");
    this->mark_failed();
    return;
  }

  if (rmt_enable(this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT enable failed");
    this->mark_failed();
    return;
  }

  // Faster than one frame per main loop needs the high-frequency loop during changes.
  this->use_high_freq_ = this->min_frame_us_ != 0 && this->min_frame_us_ < App.get_loop_interval() * 1000UL;
  if (this->refresh_interval_ms_ != 0)
    this->set_interval(this->refresh_interval_ms_, [this]() { this->request_frame_(); });

  // Line is idle low now; guarantee one full reset period before the first frame.
  delayMicroseconds(this->reset_ns_ / 1000 + 1);
  this->dirty_ = true;

  // Boot: channels without a supply were powered together with the ESP (millis() ~ time
  // since power-on); a supply that enable_on_boot switched on was powered just now.
  const uint32_t now = millis();
  if (now < this->power_on_delay_ms_)
    this->start_group_wait_(0, this->power_on_delay_ms_ - now);
#ifdef USE_POWER_SUPPLY
  for (size_t i = 1; i < this->groups_.size(); i++) {
    if (this->groups_[i].psu->is_enabled())
      this->start_group_wait_(static_cast<uint8_t>(i), this->power_on_delay_ms_);
  }
#endif
}

void WS2915Component::loop() {
  if (this->high_freq_active_ && millis() - this->last_change_ms_ > TRANSITION_HOLD_MS) {
    this->high_freq_.stop();
    this->high_freq_active_ = false;
  }
  if (!this->dirty_) {
    if (!this->high_freq_active_)
      this->disable_loop();
    return;
  }
  // Rate limit; a change inside the window is sent when it expires, not dropped.
  if (this->min_frame_us_ != 0 && this->has_sent_ && micros() - this->last_frame_us_ < this->min_frame_us_)
    return;
  // Non-blocking: if the previous frame (incl. its reset) is still on the wire, retry next loop.
  if (rmt_tx_wait_all_done(this->channel_, 0) != ESP_OK)
    return;
  this->transmit_();
}

void WS2915Component::transmit_() {
  // Power-on hold: channels of a group whose boards are still powering up are sent as 0.
  const uint16_t *levels = this->levels_;
  if (this->out_levels_ != nullptr && !this->is_power_ready()) {
    const size_t count = this->num_levels_();
    for (size_t i = 0; i < count; i++)
      this->out_levels_[i] = this->groups_[this->group_at_(i)].waiting ? 0 : this->levels_[i];
    levels = this->out_levels_;
  }

  // The encoder is idle (checked by the caller), so its inputs can be updated safely.
  if (this->has_power_model_) {
    const PowerResult r =
        pack_frame_limited(this->chip_type_, levels, this->num_chips_, this->power_, this->tx_buf_);
    this->requested_current_ = r.requested;
    this->current_ = r.actual;
    if (r.limiting != this->limiting_) {
      this->limiting_ = r.limiting;
      if (r.limiting) {
        ESP_LOGD(TAG, "Power limit active: %.2f A requested, %.2f A sent", r.requested, r.actual);
      } else {
        ESP_LOGD(TAG, "Power limit released (%.2f A)", r.requested);
      }
    }
  } else {
    pack_frame(this->chip_type_, levels, this->num_levels_(), SCALE_ONE, this->tx_buf_);
  }
  this->params_.header = build_header(this->header_format_, this->gain_);
  this->params_.header_bits = header_bits(this->chip_type_, this->header_format_);

  rmt_transmit_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  const esp_err_t err = rmt_transmit(this->channel_, this->encoder_, this->tx_buf_, this->get_buffer_size_(), &cfg);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
    this->status_set_warning();
    return;  // stay dirty, retry next loop
  }
  this->status_clear_warning();
  this->last_frame_us_ = micros();
  this->has_sent_ = true;
  this->dirty_ = false;
}

void WS2915Component::dump_config() {
  const uint32_t period0 = this->bit0_high_ns_ + this->bit0_low_ns_;
  const uint32_t period1 = this->bit1_high_ns_ + this->bit1_low_ns_;
  const uint32_t worst = period0 > period1 ? period0 : period1;
  const uint32_t bits =
      header_bits(this->chip_type_, this->header_format_) + static_cast<uint32_t>(this->get_buffer_size_()) * 8;
  const uint32_t frame_us = static_cast<uint32_t>((static_cast<uint64_t>(bits) * worst) / 1000) + this->reset_ns_ / 1000;
  ESP_LOGCONFIG(TAG,
                "WS2915 line:\n"
                "  Chip: %s (%u-bit), %u chip(s)\n"
                "  Pin: GPIO%u%s\n"
                "  RMT symbols: %" PRIu32 "%s\n"
                "  Bit0: %" PRIu32 "/%" PRIu32 " ns, Bit1: %" PRIu32 "/%" PRIu32 " ns, Reset: %" PRIu32 " us\n"
                "  Frame: %" PRIu32 " us worst case (%" PRIu32 " fps max)",
                this->is_16bit_() ? "WS2915" : "WS2805", this->is_16bit_() ? 16u : 8u, this->num_chips_, this->pin_,
                this->inverted_ ? " (inverted)" : "", this->rmt_symbols_, this->use_dma_ ? " (DMA)" : "",
                this->bit0_high_ns_, this->bit0_low_ns_, this->bit1_high_ns_, this->bit1_low_ns_,
                this->reset_ns_ / 1000, frame_us, frame_us ? 1000000 / frame_us : 0);
  if (this->is_16bit_()) {
    ESP_LOGCONFIG(TAG, "  Gain R/G/B/W1/W2: %u/%u/%u/%u/%u, header %s 0x%08" PRIX32, this->gain_[0], this->gain_[1],
                  this->gain_[2], this->gain_[3], this->gain_[4],
                  this->header_format_ == HEADER_26BIT ? "26-bit" : "32-bit",
                  build_header(this->header_format_, this->gain_));
  }
  if (this->min_frame_us_ != 0) {
    ESP_LOGCONFIG(TAG, "  Transition refresh: %" PRIu32 " us/frame%s", this->min_frame_us_,
                  this->use_high_freq_ ? " (high-frequency loop while changing)" : "");
  } else {
    ESP_LOGCONFIG(TAG, "  Transition refresh: one frame per main loop (%" PRIu32 " ms)", App.get_loop_interval());
  }
  if (this->refresh_interval_ms_ != 0) {
    ESP_LOGCONFIG(TAG, "  Refresh at rest: every %" PRIu32 " ms", this->refresh_interval_ms_);
  } else {
    ESP_LOGCONFIG(TAG, "  Refresh at rest: never");
  }
  if (this->power_on_delay_ms_ != 0)
    ESP_LOGCONFIG(TAG, "  Power-on delay: %" PRIu32 " ms (non-blocking)", this->power_on_delay_ms_);
#ifdef USE_LIGHT
  if (this->power_on_delay_ms_ != 0 && !this->lights_.empty())
    ESP_LOGCONFIG(TAG, "  Fade-in replay after power-on: %u light(s)", static_cast<unsigned>(this->lights_.size()));
#endif
#ifdef USE_POWER_SUPPLY
  for (size_t i = 1; i < this->groups_.size(); i++) {
    size_t channels = 0;
    for (size_t j = 0; this->group_of_ != nullptr && j < this->num_levels_(); j++)
      channels += this->group_of_[j] == i;
    ESP_LOGCONFIG(TAG, "  Power supply group %u: %u channel(s), keep-on %" PRIu32 " ms", static_cast<unsigned>(i),
                  static_cast<unsigned>(channels), this->groups_[i].keep_on_ms);
  }
#endif
  if (this->has_power_model_) {
    const float *c = this->power_.channel_current;
    const float *m = this->power_.max_channel_current;
    ESP_LOGCONFIG(TAG,
                  "  Power limit (CH1..CH5, 0 = no limit):\n"
                  "    current at 100 %%: %.2f/%.2f/%.2f/%.2f/%.2f A\n"
                  "    max per channel:  %.2f/%.2f/%.2f/%.2f/%.2f A\n"
                  "    max per chip: %.2f A, max line: %.2f A",
                  c[0], c[1], c[2], c[3], c[4], m[0], m[1], m[2], m[3], m[4], this->power_.max_chip_current,
                  this->power_.max_current);
  }
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup FAILED");
}

}  // namespace esphome::ws2915

#endif  // USE_ESP32
