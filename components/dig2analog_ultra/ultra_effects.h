#pragma once

#ifdef USE_ESP32

#include "esphome/components/light/base_light_effects.h"
#include "esphome/components/light/light_effect.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "ultra_light.h"

#include <vector>

namespace esphome::dig2analog_ultra {

static const char *const EFFECT_TAG = "dig2analog_ultra.effect";

/// An effect that runs on the board: start() sends it once (with its parameters), stop() ends it. The board
/// computes it every 1 ms on the shared clock, so all boards run it in step; nothing is sent while it runs.
class UltraEffect : public light::LightEffect {
 public:
  explicit UltraEffect(const char *name) : LightEffect(name) {}

  void start() override {
    this->light_ = UltraLight::find(this->state_->get_output());
    if (this->light_ == nullptr) {
      ESP_LOGE(EFFECT_TAG, "'%s': %s only runs on a dig2analog_ultra light", this->state_->get_name().c_str(),
               this->name_);
      return;
    }
    this->start_ms_ = millis();
    uint8_t p[ULTRA_FX_PARAMS] = {};
    const size_t n = this->params_(p);
    const std::vector<Step> *steps = this->steps_();
    // A periodic effect starts on the host clock's phase: lights started one after the other (an HA group or scene
    // sends one call per light) run exactly in step, on all data lines.
    const uint8_t fx = this->fx_();
    const int64_t start = phase_start_us(this->light_->get_hub()->start_time_us(), effect_period_ms(fx, p, n, steps));
    this->light_->start_effect(fx, p, n, steps, start);
  }
  void stop() override {
    if (this->light_ != nullptr)
      this->light_->stop_effect();
  }
  void apply() override {}

 protected:
  virtual uint8_t fx_() const = 0;
  virtual size_t params_(uint8_t *p) = 0;
  virtual const std::vector<Step> *steps_() { return nullptr; }
  static uint16_t frac_(float v) { return to16(v); }

  UltraLight *light_{nullptr};
  uint32_t start_ms_{0};
};

/// ESPHome pulse: every update_interval the brightness flips between min and max, with the on / off transitions.
class UltraPulseEffect final : public UltraEffect {
 public:
  explicit UltraPulseEffect(const char *name) : UltraEffect(name) {}
  void set_transition_on_length(uint32_t ms) { this->on_ms_ = ms; }
  void set_transition_off_length(uint32_t ms) { this->off_ms_ = ms; }
  void set_update_interval(uint32_t ms) { this->interval_ms_ = ms; }
  void set_min_max_brightness(float min, float max) {
    this->min_ = min;
    this->max_ = max;
  }

 protected:
  uint8_t fx_() const override { return ULTRA_FX_PULSE; }
  size_t params_(uint8_t *p) override {
    ultra_put24(p, this->on_ms_);
    ultra_put24(p + 3, this->off_ms_);
    ultra_put24(p + 6, this->interval_ms_);
    ultra_put16(p + 9, frac_(this->min_));
    ultra_put16(p + 11, frac_(this->max_));
    return 13;
  }
  uint32_t on_ms_{1000}, off_ms_{1000}, interval_ms_{1000};
  float min_{0.0f}, max_{1.0f};
};

/// ESPHome strobe: a list of colors with duration and transition, repeated (up to 8 colors).
class UltraStrobeEffect final : public UltraEffect {
 public:
  explicit UltraStrobeEffect(const char *name) : UltraEffect(name) {}
  void set_colors(const std::initializer_list<light::StrobeLightEffectColor> &colors) { this->colors_ = colors; }

 protected:
  uint8_t fx_() const override { return ULTRA_FX_STROBE; }
  size_t params_(uint8_t *p) override {
    this->steps_buf_.clear();
    for (const auto &c : this->colors_) {
      if (this->steps_buf_.size() >= ULTRA_MAX_STEPS)
        break;
      Step s{};
      s.duration_ms = static_cast<uint16_t>(c.duration > 65535 ? 65535 : c.duration);
      s.transition_ms = static_cast<uint16_t>(c.transition_length > 65535 ? 65535 : c.transition_length);
      this->light_->map(c.color, s.m, s.a, s.b);
      this->steps_buf_.push_back(s);
    }
    p[0] = static_cast<uint8_t>(this->steps_buf_.size());
    return 1;
  }
  const std::vector<Step> *steps_() override { return &this->steps_buf_; }
  std::vector<light::StrobeLightEffectColor> colors_;
  std::vector<Step> steps_buf_;
};

/// ESPHome flicker: brightness and color wander randomly around the light's state.
class UltraFlickerEffect final : public UltraEffect {
 public:
  explicit UltraFlickerEffect(const char *name) : UltraEffect(name) {}
  void set_alpha(float alpha) { this->alpha_ = alpha; }
  void set_intensity(float intensity) { this->intensity_ = intensity; }
  void set_update_interval(uint32_t ms) { this->interval_ms_ = ms; }

 protected:
  uint8_t fx_() const override { return ULTRA_FX_FLICKER; }
  size_t params_(uint8_t *p) override {
    ultra_put16(p, frac_(this->alpha_));
    ultra_put16(p + 2, frac_(this->intensity_));
    ultra_put16(p + 4, static_cast<uint16_t>(this->interval_ms_ > 65535 ? 65535 : this->interval_ms_));
    return 6;
  }
  float alpha_{0.95f}, intensity_{0.015f};
  uint32_t interval_ms_{16};
};

/// ESPHome random: a new random color (brightness on lights without color) every update_interval.
class UltraRandomEffect final : public UltraEffect {
 public:
  explicit UltraRandomEffect(const char *name) : UltraEffect(name) {}
  void set_transition_length(uint32_t ms) { this->transition_ms_ = ms; }
  void set_update_interval(uint32_t ms) { this->interval_ms_ = ms; }

 protected:
  uint8_t fx_() const override { return ULTRA_FX_RANDOM; }
  size_t params_(uint8_t *p) override {
    ultra_put24(p, this->transition_ms_);
    ultra_put24(p + 3, this->interval_ms_);
    return 6;
  }
  uint32_t transition_ms_{7500}, interval_ms_{10000};
};

/// Candle: a flame-like flicker, only darker than the light's state.
class UltraCandleEffect final : public UltraEffect {
 public:
  explicit UltraCandleEffect(const char *name) : UltraEffect(name) {}
  void set_intensity(float intensity) { this->intensity_ = intensity; }
  void set_speed(float speed) { this->speed_ = speed; }

 protected:
  uint8_t fx_() const override { return ULTRA_FX_CANDLE; }
  size_t params_(uint8_t *p) override {
    ultra_put16(p, frac_(this->intensity_));
    const float pct = this->speed_ * 100.0f + 0.5f;
    ultra_put16(p + 2, static_cast<uint16_t>(pct < 1.0f ? 1.0f : pct > 1000.0f ? 1000.0f : pct));
    return 4;
  }
  float intensity_{0.4f}, speed_{1.0f};
};

/// Breathe: brightness between min and max along a sine, one period.
class UltraBreatheEffect final : public UltraEffect {
 public:
  explicit UltraBreatheEffect(const char *name) : UltraEffect(name) {}
  void set_period(uint32_t ms) { this->period_ms_ = ms; }
  void set_min_max_brightness(float min, float max) {
    this->min_ = min;
    this->max_ = max;
  }

 protected:
  uint8_t fx_() const override { return ULTRA_FX_BREATHE; }
  size_t params_(uint8_t *p) override {
    ultra_put24(p, this->period_ms_);
    ultra_put16(p + 3, frac_(this->min_));
    ultra_put16(p + 5, frac_(this->max_));
    return 7;
  }
  uint32_t period_ms_{4000};
  float min_{0.0f}, max_{1.0f};
};

/// Color loop: the hue goes around once per period (RGB lights; their white channels are off meanwhile).
class UltraColorLoopEffect final : public UltraEffect {
 public:
  explicit UltraColorLoopEffect(const char *name) : UltraEffect(name) {}
  void set_period(uint32_t ms) { this->period_ms_ = ms; }
  void set_saturation(float s) { this->saturation_ = s; }

 protected:
  uint8_t fx_() const override { return ULTRA_FX_COLOR_LOOP; }
  size_t params_(uint8_t *p) override {
    ultra_put24(p, this->period_ms_);
    ultra_put16(p + 3, frac_(this->saturation_));
    return 5;
  }
  uint32_t period_ms_{30000};
  float saturation_{1.0f};
};

/// Sunrise: from a dark, deep orange dawn to the light's state over the duration. Sunset: the reverse, then the
/// light turns off. When done, the light's state in ESPHome follows (effect None / off).
class UltraSunEffect final : public UltraEffect {
 public:
  UltraSunEffect(const char *name, bool rise) : UltraEffect(name), rise_(rise) {}
  void set_duration(uint32_t ms) { this->duration_ms_ = ms; }
  void start() override {
    this->done_ = false;
    UltraEffect::start();
  }
  void apply() override {
    if (this->light_ == nullptr || this->done_ || millis() - this->start_ms_ < this->duration_ms_ + 100)
      return;
    this->done_ = true;
    if (this->rise_) {
      auto call = this->state_->make_call();
      call.set_effect(static_cast<uint32_t>(0));
      call.perform();
    } else {
      auto call = this->state_->turn_off();
      call.set_transition_length(0);
      call.perform();
    }
  }

 protected:
  uint8_t fx_() const override { return this->rise_ ? ULTRA_FX_SUNRISE : ULTRA_FX_SUNSET; }
  size_t params_(uint8_t *p) override {
    ultra_put24(p, this->duration_ms_ > 0xFFFFFF ? 0xFFFFFF : this->duration_ms_);
    return 3;
  }
  bool rise_;
  bool done_{false};
  uint32_t duration_ms_{900000};
};

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
