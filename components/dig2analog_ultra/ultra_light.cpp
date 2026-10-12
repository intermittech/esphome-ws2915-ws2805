#include "ultra_light.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra.light";

std::vector<UltraLight *> UltraLight::registry_;

UltraTransition::~UltraTransition() { this->light_->transition_done_(this); }

void UltraTransition::start() {
  // ESPHome's own start: off -> on keeps the target colors, on -> off keeps the old ones, and a color mode change
  // goes through off. The board gets the end values, the length and whether to go through off.
  light::LightTransitionTransformer::start();
  this->light_->transition_start_(this, this->end_values_, this->length_, this->changing_color_mode_);
}

void UltraTransition::stop() { this->light_->transition_done_(this); }

UltraLight *UltraLight::find(light::LightOutput *output) {
  for (auto *l : registry_) {
    if (static_cast<light::LightOutput *>(l) == output)
      return l;
  }
  return nullptr;
}

bool UltraLight::has_rgb() const {
  for (uint8_t r : this->roles_) {
    if (r == ULTRA_ROLE_RED)
      return true;
  }
  return false;
}

light::LightTraits UltraLight::get_traits() {
  bool rgb = false, white = false, cww = false;
  for (uint8_t r : this->roles_) {
    rgb |= r == ULTRA_ROLE_RED;
    white |= r == ULTRA_ROLE_WHITE;
    cww |= r == ULTRA_ROLE_COLD_WHITE;
  }
  auto traits = light::LightTraits();
  if (rgb && cww) {
    if (this->color_interlock_) {
      traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::COLD_WARM_WHITE});
    } else {
      traits.set_supported_color_modes({light::ColorMode::RGB_COLD_WARM_WHITE});
    }
  } else if (rgb && white) {
    if (this->color_interlock_) {
      traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::WHITE});
    } else {
      traits.set_supported_color_modes({light::ColorMode::RGB_WHITE});
    }
  } else if (rgb) {
    traits.set_supported_color_modes({light::ColorMode::RGB});
  } else if (cww) {
    traits.set_supported_color_modes({light::ColorMode::COLD_WARM_WHITE});
  } else {
    traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
  }
  if (cww) {
    traits.set_min_mireds(this->cold_white_temperature_);
    traits.set_max_mireds(this->warm_white_temperature_);
  }
  return traits;
}

std::unique_ptr<light::LightTransformer> UltraLight::create_default_transition() {
  return std::make_unique<UltraTransition>(this);
}

void UltraLight::setup_state(light::LightState *state) {
  this->state_ = state;
  registry_.push_back(this);
  uint8_t flags = 0;
  if (this->zero_means_zero_)
    flags |= ULTRA_DEF_ZERO_MEANS_ZERO;
  if (this->constant_brightness_)
    flags |= ULTRA_DEF_CONSTANT_BRIGHTNESS;
  const float gamma = state->get_gamma_correct();
  const uint16_t g = static_cast<uint16_t>(std::lround(gamma < 0.0f ? 0.0f : gamma > 65.0f ? 65000.0f : gamma * 1000.0f));
  this->hub_->scheduler().light_def(this->board_, this->index_, this->roles_, g, flags, this->min_, this->max_);
  if (this->dtw_available_)
    this->send_opt_();
  if (this->scene_mode_ == SCENE_NONE) {
    // not in the board's power-on scene (also clears what an earlier configuration stored there)
    static const uint16_t z[CHANNELS] = {0, 0, 0, 0, 0}, one[CHANNELS] = {65535, 65535, 65535, 65535, 65535};
    this->hub_->scheduler().light_scene(this->board_, this->index_, 0, 0, 0, z, one);
  } else {
    state->add_remote_values_listener(this);  // before LightState::setup() restores: the first update is the boot state
  }
  this->hub_->add_light(this);
  this->hub_->wake_();
}

static uint16_t mired10(float mireds) {
  const float v = mireds * 10.0f + 0.5f;
  return static_cast<uint16_t>(v < 0.0f ? 0.0f : v > 65535.0f ? 65535.0f : v);
}

// Dim to warm runs on the board: every 1 ms it moves the CW / WW mix with the brightness (state x brightness, so
// also during transitions and effects), from the light's colour temperature at full brightness to the warm end, or
// along the set points.
void UltraLight::send_opt_() {
  const float assist = this->dtw_assist_ * 65535.0f + 0.5f;
  uint16_t pm[ULTRA_OPT_POINTS_MAX], pt[ULTRA_OPT_POINTS_MAX];
  for (uint8_t i = 0; i < this->dtw_n_; i++) {
    const float m = this->dtw_pb_[i] * 65535.0f + 0.5f;
    pm[i] = static_cast<uint16_t>(m < 1.0f ? 1.0f : m > 65535.0f ? 65535.0f : m);
    pt[i] = mired10(this->dtw_pt_[i]);
  }
  uint8_t flags = this->dtw_on_ ? ULTRA_OPT_DIM_TO_WARM : 0;
  if (this->dtw_n_ > 0)
    flags |= ULTRA_OPT_POINTS;
  this->hub_->scheduler().light_opt(this->board_, this->index_, flags, mired10(this->cold_white_temperature_),
                                    mired10(this->warm_white_temperature_), mired10(this->dtw_warm_),
                                    static_cast<uint16_t>(assist > 65535.0f ? 65535.0f : assist), this->dtw_n_, pm,
                                    pt);
}

void UltraLight::set_dim_to_warm(bool on) {
  if (!this->dtw_available_) {
    ESP_LOGW(TAG, "dim to warm needs dim_to_warm: in the light's configuration");
    return;
  }
  if (on == this->dtw_on_)
    return;
  this->dtw_on_ = on;
  this->send_opt_();
  this->hub_->wake_();
}

// The power-on scene is what ESPHome will restore at its next boot (restore_mode / initial_state); the board stores
// it (writes debounced, small changes at most every 10 min) and shows it from its own power-up until the host is back.
void UltraLight::on_light_remote_values_update() {
  if (this->scene_mode_ == SCENE_NONE || (this->scene_mode_ == SCENE_FIXED && this->scene_sent_))
    return;
  light::LightColorValues v = this->state_->remote_values;
  if (this->scene_mode_ == SCENE_FOLLOW_ON)
    v.set_state(true);
  else if (this->scene_mode_ == SCENE_FOLLOW_INVERTED)
    v.set_state(!v.is_on());
  if (v.is_on() && v.get_brightness() == 0.0f)
    v.set_brightness(1.0f);  // as LightState::setup() does on boot
  uint16_t m, a[CHANNELS], b[CHANNELS];
  this->map(v, m, a, b);
  const uint32_t fade = (this->state_->get_default_transition_length() + 50) / 100;
  const uint8_t f = static_cast<uint8_t>(fade > 255 ? 255 : fade);
  if (this->scene_sent_ && m == this->scene_m_ && f == this->scene_fade_ && memcmp(a, this->scene_a_, sizeof a) == 0 &&
      memcmp(b, this->scene_b_, sizeof b) == 0)
    return;
  this->scene_sent_ = true;
  this->scene_m_ = m;
  this->scene_fade_ = f;
  memcpy(this->scene_a_, a, sizeof a);
  memcpy(this->scene_b_, b, sizeof b);
  this->hub_->scheduler().light_scene(this->board_, this->index_, 1, f, m, a, b);
  this->hub_->wake_();
}

void UltraLight::map(const light::LightColorValues &v, uint16_t &m, uint16_t *a, uint16_t *b) const {
  uint8_t caps = static_cast<uint8_t>(v.get_color_mode());
  if (caps == 0 && this->state_ != nullptr)
    caps = static_cast<uint8_t>(this->state_->remote_values.get_color_mode());
  const ColorValues cv{v.get_state(),      v.get_brightness(),  v.get_color_brightness(),
                       v.get_red(),        v.get_green(),       v.get_blue(),
                       v.get_white(),      v.get_cold_white(),  v.get_warm_white(),
                       caps};
  map_values(this->roles_, cv, m, a, b);
}

// stream: a step of an ESPHome (host-side) effect. It is sent once (the next step replaces a lost one) and glides
// (effect_glide): over the measured step interval, or with auto: small steps over the interval, a jump up at once, a
// jump down over GLIDE_TAIL_MS, so a chase stays crisp and two lights crossing over do not dip.
void UltraLight::send_(const light::LightColorValues &v, uint32_t length, uint8_t flags, bool stream) {
  uint16_t m, a[CHANNELS], b[CHANNELS];
  this->map(v, m, a, b);
  if (length == 0 && this->sent_ && m == this->last_m_ && memcmp(a, this->last_a_, sizeof a) == 0 &&
      memcmp(b, this->last_b_, sizeof b) == 0)
    return;  // the board is there already
  if (stream && this->glide_mode_ != GLIDE_OFF) {
    const uint32_t interval = this->glide_.step(millis());
    if (this->glide_mode_ == GLIDE_ON || !this->sent_) {
      length = interval;
    } else {
      uint16_t was[CHANNELS], now[CHANNELS];  // A x B: the colour as the board weighs it
      for (uint8_t i = 0; i < CHANNELS; i++) {
        was[i] = static_cast<uint16_t>(static_cast<uint32_t>(this->last_a_[i]) * this->last_b_[i] / 65535);
        now[i] = static_cast<uint16_t>(static_cast<uint32_t>(a[i]) * b[i] / 65535);
      }
      length = glide_for(interval, change_of(this->last_m_, was, m, now),
                         value_of(m, now) >= value_of(this->last_m_, was));
    }
  }
  this->sent_ = true;
  this->last_m_ = m;
  memcpy(this->last_a_, a, sizeof a);
  memcpy(this->last_b_, b, sizeof b);
  if (!this->layers_.empty())
    return;  // a strip shows on the light: kept, sent when it is gone
  if (stream) {
    this->hub_->scheduler().light_stream(this->board_, this->index_, m, a, b, length, this->hub_->start_time_us());
  } else {
    this->hub_->scheduler().light_set(this->board_, this->index_, flags, m, a, b, length, this->hub_->start_time_us());
  }
  this->lit_(m, length);
  this->hub_->wake_();
}

void UltraLight::lit_(uint16_t m, uint32_t length_ms) {
  if (this->on_ && m == 0)
    this->off_until_ms_ = millis() + length_ms + this->hub_->start_delay_ms();   // fading out on the board
  this->on_ = m != 0;
}

void UltraLight::send_pixel(uint16_t m, const uint16_t *a, const uint16_t *b, uint32_t length_ms, uint8_t flags,
                            bool stream, int64_t start_us) {
  if (start_us == 0)
    start_us = this->hub_->start_time_us();
  if (stream) {
    this->hub_->scheduler().light_stream(this->board_, this->index_, m, a, b, length_ms, start_us);
  } else {
    this->hub_->scheduler().light_set(this->board_, this->index_, flags, m, a, b, length_ms, start_us);
  }
  this->lit_(m, length_ms);
  this->hub_->wake_();
}

void UltraLight::layer_begin(UltraLayer *layer) {
  for (auto it = this->layers_.begin(); it != this->layers_.end(); ++it) {
    if (*it == layer) {
      this->layers_.erase(it);
      break;
    }
  }
  if (this->layers_.empty())
    this->hub_->scheduler().hold_effect(this->board_, this->index_, this->hub_->start_time_us());
  this->layers_.push_back(layer);
  this->hub_->wake_();
}

void UltraLight::layer_end(UltraLayer *layer, uint32_t fade_ms) {
  const bool top = this->layer_top(layer);
  bool found = false;
  for (auto it = this->layers_.begin(); it != this->layers_.end(); ++it) {
    if (*it == layer) {
      this->layers_.erase(it);
      found = true;
      break;
    }
  }
  if (!found || !top)
    return;
  if (!this->layers_.empty()) {
    this->layers_.back()->layer_shown(this);
    return;
  }
  // Back to the light's own state: its target (or the current step of an ESPHome effect running on it), fading,
  // and its board effect runs again in phase.
  this->hub_->scheduler().release_effect(this->board_, this->index_);
  this->hub_->wake_();
  if (this->state_ == nullptr)
    return;
  const light::LightColorValues &v =
      this->state_->get_current_effect_index() != 0 ? this->state_->current_values : this->state_->remote_values;
  this->sent_ = false;
  this->send_(v, fade_ms, ULTRA_SET_SMOOTH);
}

void UltraLight::transition_start_(UltraTransition *t, const light::LightColorValues &end, uint32_t length,
                                   bool via_off) {
  this->transition_ = t;
  this->send_(end, length, ULTRA_SET_SMOOTH | (via_off ? ULTRA_SET_VIA_OFF : 0));
}

void UltraLight::transition_done_(UltraTransition *t) {
  if (this->transition_ == t)
    this->transition_ = nullptr;
}

// Instant changes (calls without transition, the end of a transition): the board jumps. Steps of ESPHome's own
// effects (flicker, lambda, ...) are streamed and glide (effect_glide); strobe steps and changes during an ultra_*
// board effect stay commands. During a board-run transition ESPHome's intermediate values are not sent.
void UltraLight::write_state(light::LightState *state) {
  if (this->transition_ != nullptr)
    return;
  const uint32_t fx = state->get_current_effect_index();
  if (fx != this->glide_effect_) {
    this->glide_effect_ = fx;
    this->glide_.reset();
  }
  this->send_(state->current_values, 0, 0, fx != 0 && this->effect_kind_(fx) == EFFECT_HOST);
}

void UltraLight::start_effect(uint8_t fx, const uint8_t *params, size_t n, const std::vector<Step> *steps,
                              int64_t start_us) {
  this->hub_->scheduler().light_effect(this->board_, this->index_, fx, params, n,
                                       start_us != 0 ? start_us : this->hub_->start_time_us(), steps);
  this->hub_->wake_();
}

void UltraLight::stop_effect() {
  this->hub_->scheduler().light_effect(this->board_, this->index_, ULTRA_FX_NONE, nullptr, 0,
                                       this->hub_->start_time_us());
  this->hub_->wake_();
}

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
