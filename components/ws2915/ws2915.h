#pragma once

#ifdef USE_ESP32

#include "esphome/components/output/float_output.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "ws2915_frame.h"

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

namespace esphome::ws2915 {

using EncoderParams = FrameSymbols<rmt_symbol_word_t>;

/// One data line with num_chips WS2915 (16-bit, gain header) or WS2805 (8-bit) chips.
/// Each chip channel is a FloatOutput; the stock light platforms (rgbww, cwww,
/// monochromatic, ...) provide gamma, colour temperature, interlock, transitions and effects.
class WS2915Component final : public Component {
 public:
  class Channel final : public output::FloatOutput {
   public:
    void set_parent(WS2915Component *parent) { this->parent_ = parent; }
    void set_chip(uint16_t chip) { this->chip_ = chip; }
    void set_channel(uint8_t channel) { this->channel_ = channel; }

   protected:
    void write_state(float state) override { this->parent_->set_level_(this->chip_, this->channel_, state); }

    WS2915Component *parent_{nullptr};
    uint16_t chip_{0};
    uint8_t channel_{0};
  };

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // Configuration (codegen)
  void set_chip_type(ChipType type) { this->chip_type_ = type; }
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_num_chips(uint16_t num_chips) { this->num_chips_ = num_chips; }
  void set_rmt_symbols(uint32_t rmt_symbols) { this->rmt_symbols_ = rmt_symbols; }
  void set_use_dma(bool use_dma) { this->use_dma_ = use_dma; }
  void set_timing(uint32_t bit0_high_ns, uint32_t bit0_low_ns, uint32_t bit1_high_ns, uint32_t bit1_low_ns,
                  uint32_t reset_ns) {
    this->bit0_high_ns_ = bit0_high_ns;
    this->bit0_low_ns_ = bit0_low_ns;
    this->bit1_high_ns_ = bit1_high_ns;
    this->bit1_low_ns_ = bit1_low_ns;
    this->reset_ns_ = reset_ns;
  }
  /// Re-send the unchanged frame this often while nothing changes (0 = never).
  void set_refresh_interval(uint32_t ms) { this->refresh_interval_ms_ = ms; }
  /// Minimum time between frames while values change (0 = one frame per main loop, ~16 ms).
  /// Below the main-loop interval, the main loop runs in high-frequency mode during changes.
  void set_transition_refresh_interval(uint32_t us) { this->min_frame_us_ = us; }
  /// Power limiter: LED current per channel at 100 % (same for every chip on the line),
  /// optional cap per chip (board fuse) and for the whole line (supply). 0 = no cap.
  void set_channel_current(uint8_t channel, float amps) {
    if (channel < CHANNELS_PER_CHIP) {
      this->power_.channel_current[channel] = amps;
      this->has_power_model_ = true;
    }
  }
  void set_max_chip_current(float amps) { this->power_.max_chip_current = amps; }
  void set_max_current(float amps) { this->power_.max_current = amps; }

  // Runtime API (lambdas / bench). Gain and header format are chain-wide.
  void set_gain(uint8_t channel, uint8_t gain);
  uint8_t get_gain(uint8_t channel) const { return channel < CHANNELS_PER_CHIP ? this->gain_[channel] : 0; }
  void set_header_format(HeaderFormat format);
  HeaderFormat get_header_format() const { return this->header_format_; }
  /// Write a raw level (0..65535 on WS2915, 0..255 on WS2805), bypassing the light.
  /// The next light update of that channel overwrites it.
  void set_raw(uint16_t chip, uint8_t channel, uint16_t value) { this->set_value_(chip, channel, value); }
  /// Re-send the current frame now.
  void refresh() { this->request_frame_(); }
  /// Estimated LED current of the whole line after power limiting, in A (needs power_limit).
  float get_current() const { return this->current_; }
  /// Current the lights ask for before limiting, in A (needs power_limit).
  float get_requested_current() const { return this->requested_current_; }
  bool is_limiting() const { return this->limiting_; }
  uint16_t get_num_chips() const { return this->num_chips_; }

 protected:
  bool is_16bit_() const { return is_16bit(this->chip_type_); }
  size_t num_levels_() const { return static_cast<size_t>(this->num_chips_) * CHANNELS_PER_CHIP; }
  size_t get_buffer_size_() const { return static_cast<size_t>(this->num_chips_) * bytes_per_chip(this->chip_type_); }

  void set_level_(uint16_t chip, uint8_t channel, float state);
  void set_value_(uint16_t chip, uint8_t channel, uint16_t value);
  void mark_changed_();
  void request_frame_();
  void transmit_();

  ChipType chip_type_{CHIP_WS2915};
  HeaderFormat header_format_{HEADER_32BIT};
  uint8_t pin_{0};
  bool inverted_{false};
  bool use_dma_{false};
  uint16_t num_chips_{1};
  uint32_t rmt_symbols_{64};
  uint32_t bit0_high_ns_{340};
  uint32_t bit0_low_ns_{960};
  uint32_t bit1_high_ns_{700};
  uint32_t bit1_low_ns_{650};
  uint32_t reset_ns_{300000};
  uint8_t gain_[CHANNELS_PER_CHIP]{};

  uint32_t refresh_interval_ms_{0};
  uint32_t min_frame_us_{0};
  bool use_high_freq_{false};
  bool high_freq_active_{false};
  HighFrequencyLoopRequester high_freq_;
  uint32_t last_change_ms_{0};
  uint32_t last_frame_us_{0};
  bool has_sent_{false};

  bool has_power_model_{false};
  PowerModel power_{};
  float current_{0.0f};
  float requested_current_{0.0f};
  bool limiting_{false};

  uint16_t *levels_{nullptr};  // targets, chip-major, before power limiting
  uint8_t *tx_buf_{nullptr};   // wire bytes of the frame on the line
  EncoderParams params_{};
  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};
  bool dirty_{true};
  bool range_warned_{false};
};

}  // namespace esphome::ws2915

#endif  // USE_ESP32
