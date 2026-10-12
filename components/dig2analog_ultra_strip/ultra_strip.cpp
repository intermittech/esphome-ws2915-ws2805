#include "ultra_strip.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <esp_timer.h>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra_strip";

UltraStripTransition::~UltraStripTransition() { this->strip_->transition_done_(this, false); }

void UltraStripTransition::start() {
  // ESPHome's own start: off -> on keeps the target colors, on -> off keeps the old ones (brightness 0), a color mode
  // change goes through off.
  light::LightTransitionTransformer::start();
  this->strip_->transition_start_(this, this->end_values_, this->length_, this->changing_color_mode_);
}

void UltraStripTransition::stop() { this->strip_->transition_done_(this, true); }

void UltraStrip::add_pixel(UltraLight *light, uint32_t roles) {
  Pixel p{};
  p.light = light;
  for (uint8_t i = 0; i < CHANNELS; i++)
    p.roles[i] = static_cast<uint8_t>((roles >> (4 * i)) & 0x0F);
  this->pixels_.push_back(p);
  this->buf_.resize(this->pixels_.size() * 5, 0);
}

void UltraStrip::clear_effect_data() {
  for (size_t i = 0; i < this->pixels_.size(); i++)
    this->buf_[i * 5 + 4] = 0;
}

light::LightTraits UltraStrip::get_traits() {
  auto traits = light::LightTraits();
  if (!this->rgb_) {
    traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});  // effects still draw colours: value is used
  } else if (this->white_) {
    traits.set_supported_color_modes({light::ColorMode::RGB_WHITE});
  } else {
    traits.set_supported_color_modes({light::ColorMode::RGB});
  }
  return traits;
}

light::ESPColorView UltraStrip::get_view_internal(int32_t index) const {
  uint8_t *p = &this->buf_[static_cast<size_t>(index) * 5];
  return {p, p + 1, p + 2, this->white_ ? p + 3 : nullptr, p + 4, &this->correction_};
}

// No gamma table here: the pixels stay as the effects made them, and each board applies its light's own gamma.
void UltraStrip::setup_state(light::LightState *state) { this->state_parent_ = state; }

// The strip's colour into the buffer at full scale (brightness goes to the boards as M), black when off. During an
// addressable effect the effect owns the buffer.
void UltraStrip::update_state(light::LightState *state) {
  if (this->is_effect_active())
    return;
  const auto &v = state->current_values;
  this->all() = v.is_on() ? light::color_from_light_color_values(v) : Color(0, 0, 0, 0);
  this->schedule_show();
}

void UltraStrip::map_(const Pixel &p, float r, float g, float b, float w, float level, uint16_t &m,
                      uint16_t *a) const {
  float cw = 1.0f, ww = 1.0f;
  const light::LightState *st = p.light->get_state();
  if (st != nullptr)
    white_mix(st->remote_values.get_cold_white(), st->remote_values.get_warm_white(), cw, ww);
  uint16_t bb[CHANNELS];
  pixel_to_mab(p.roles, r, g, b, w, level, cw, ww, m, a, bb);
}

void UltraStrip::map_values_(const Pixel &p, const light::LightColorValues &v, float level, uint16_t &m,
                             uint16_t *a) const {
  const float cb = v.get_color_brightness();
  this->map_(p, cb * v.get_red(), cb * v.get_green(), cb * v.get_blue(), this->white_ ? v.get_white() : 0.0f, level,
             m, a);
}

void UltraStrip::send_(Pixel &p, uint16_t m, const uint16_t *a, uint32_t length, uint8_t flags, bool stream) {
  static const uint16_t ONE[CHANNELS] = {65535, 65535, 65535, 65535, 65535};
  p.light->send_pixel(m, a, ONE, length, flags, stream, this->start_us_);
  p.m = m;
  for (uint8_t i = 0; i < CHANNELS; i++)
    p.a[i] = a[i];
  p.sent = true;
}

bool UltraStrip::is_hard_(uint32_t effect) const {
  if (this->glide_mode_ != GLIDE_AUTO)
    return false;
  for (uint32_t e : this->hard_effects_) {
    if (e == effect)
      return true;
  }
  return false;
}

bool UltraStrip::buffer_lit_() const {
  for (size_t i = 0; i < this->pixels_.size(); i++) {
    const uint8_t *px = &this->buf_[i * 5];
    if (px[0] | px[1] | px[2] | (this->white_ ? px[3] : 0))
      return true;
  }
  return false;
}

// When a write starts on the boards: the hub's start delay, plus one frame for every further pixel on the busiest board.
// A board gets one slot per frame (first copies first), so all pixels of a step are there before they start, and a
// chase over several lights of one board stays in step at any chain length.
int64_t UltraStrip::start_now_() {
  if (this->lead_us_ < 0) {
    int64_t lead = 0;
    bool ready = true;
    for (const auto &p : this->pixels_) {
      UltraHub *hub = p.light->get_hub();
      int64_t count = 0;
      for (const auto &q : this->pixels_)
        count += q.light->get_hub() == hub && q.light->get_board() == p.light->get_board();
      ready &= hub->get_frame_us() != 0;  // known after the hub's setup
      const int64_t l = static_cast<int64_t>(hub->start_delay_ms()) * 1000 + (count - 1) * (hub->get_frame_us() + 500);
      lead = l > lead ? l : lead;
    }
    if (!ready)
      return esp_timer_get_time() + lead;
    this->lead_us_ = lead;
  }
  return esp_timer_get_time() + this->lead_us_;
}

// The strip becomes the top layer of its lights: their own output is held back and their board effects pause.
void UltraStrip::begin_layer_() {
  this->layer_ = true;
  for (auto &p : this->pixels_) {
    p.light->layer_begin(this);
    p.sent = false;
  }
}

// Its lights go back to the layer below (another strip that is on) or to their own state, fading over fade_ms.
void UltraStrip::end_layer_(uint32_t fade_ms) {
  this->layer_ = false;
  for (auto &p : this->pixels_) {
    p.light->layer_end(this, fade_ms);
    p.sent = false;
  }
}

void UltraStrip::layer_shown(UltraLight *light) {
  for (auto &p : this->pixels_) {
    if (p.light == light)
      p.sent = false;
  }
  if (this->state_parent_ != nullptr)
    this->schedule_show();
}

// What goes to the boards:
//  - an effect is running: its steps, streamed (sent once). effect_glide auto glides small steps over the measured step
//    interval, jumps up at once and fades jumps down over 50 ms (glide_for); strobe stays hard (commands, with
//    repeats);
//  - the strip is on without an addressable effect: its colour in full precision, as a command;
//  - the strip is off: the buffer as a partition left it. A stream of changes is streamed, a single change or a pixel
//    going dark is a command. Once it is all dark, the strip is no longer a layer over its lights.
void UltraStrip::write_state(light::LightState *state) {
  if (this->transition_ != nullptr)
    return;  // the boards run the transition
  const auto &v = state->current_values;
  const uint32_t now = millis();
  const bool own_on = v.is_on();
  if (!own_on && !this->buffer_lit_()) {
    if (this->layer_)
      this->end_layer_(0);
    return;
  }
  if (!this->layer_)
    this->begin_layer_();
  this->start_us_ = this->start_now_();
  const bool from_values = own_on && !this->is_effect_active();
  const uint32_t fx = state->get_current_effect_index();
  const bool hard = fx != 0 && this->is_hard_(fx);
  if (fx != this->glide_effect_) {
    this->glide_effect_ = fx;
    this->glide_.reset();
  }
  const float level = own_on ? v.get_brightness() * v.get_state() : 1.0f;
  bool first = true, steady = false;
  uint32_t interval = 0;
  for (size_t i = 0; i < this->pixels_.size(); i++) {
    Pixel &p = this->pixels_[i];
    if (!p.light->layer_top(this))
      continue;  // a strip that turned on later is shown on this light
    uint16_t m, a[CHANNELS];
    if (from_values) {
      this->map_values_(p, v, level, m, a);
    } else {
      const uint8_t *px = &this->buf_[i * 5];
      this->map_(p, px[0] / 255.0f, px[1] / 255.0f, px[2] / 255.0f, this->white_ ? px[3] / 255.0f : 0.0f, level, m,
                 a);
    }
    if (p.sent && same_output(p.m, p.a, m, a))
      continue;
    if (first) {
      first = false;
      if (fx != 0 || !own_on) {
        interval = this->glide_.step(now);
        steady = this->glide_.steady();
      }
    }
    const bool stream = fx != 0 ? !hard : (!own_on && steady && m != 0);
    uint32_t glide = 0;
    if (stream && this->glide_mode_ == GLIDE_ON) {
      glide = interval;
    } else if (stream && this->glide_mode_ == GLIDE_AUTO) {
      static const uint16_t DARK[CHANNELS] = {0, 0, 0, 0, 0};
      const uint16_t m0 = p.sent ? p.m : 0;
      const uint16_t *a0 = p.sent ? p.a : DARK;
      glide = glide_for(interval, change_of(m0, a0, m, a), value_of(m, a) >= value_of(m0, a0));
    }
    this->send_(p, m, a, glide, 0, stream);
  }
}

std::unique_ptr<light::LightTransformer> UltraStrip::create_default_transition() {
  return std::make_unique<UltraStripTransition>(this);
}

// A transition without an addressable effect runs on the boards (smoothstep, as ESPHome's): turning on or changing
// colour sends every pixel its target with the length; turning off ends the layer, so the lights fade back to their
// own state (or a strip below) over the length. With an addressable effect running, ESPHome fades the brightness and
// the effect's steps carry it (write_state).
void UltraStrip::transition_start_(UltraStripTransition *t, const light::LightColorValues &end, uint32_t length,
                                   bool via_off) {
  if (this->is_effect_active())
    return;
  this->transition_ = t;
  const float level = end.get_brightness() * end.get_state();
  if (!(level > 0.0f)) {
    if (this->layer_)
      this->end_layer_(length);
    return;
  }
  if (!this->layer_)
    this->begin_layer_();
  this->start_us_ = this->start_now_();
  const uint8_t flags = ULTRA_SET_SMOOTH | (via_off ? ULTRA_SET_VIA_OFF : 0);
  for (auto &p : this->pixels_) {
    if (!p.light->layer_top(this))
      continue;
    uint16_t m, a[CHANNELS];
    this->map_values_(p, end, level, m, a);
    this->send_(p, m, a, length, flags, false);
  }
}

void UltraStrip::transition_done_(UltraStripTransition *t, bool finished) {
  if (this->transition_ != t)
    return;
  this->transition_ = nullptr;
  // The state is at the target now: the buffer follows (black after turning off), the boards are there already.
  if (finished && this->state_parent_ != nullptr && !this->is_effect_active()) {
    const auto &v = this->state_parent_->current_values;
    this->all() = v.is_on() ? light::color_from_light_color_values(v) : Color(0, 0, 0, 0);
  }
}

void UltraStrip::dump_config() {
  static const char *const GLIDE[] = {"off", "always", "auto (small steps glide, jumps up at once, down 50 ms)"};
  ESP_LOGCONFIG(TAG,
                "QuinLED dig2analog Ultra strip:\n"
                "  Pixels: %u (dig2analog_ultra lights)\n"
                "  Colour: %s\n"
                "  Effect glide: %s\n"
                "  Start: %.0f ms after a change (all pixels together)",
                static_cast<unsigned>(this->pixels_.size()),
                !this->rgb_ ? "brightness only" : this->white_ ? "RGBW" : "RGB",
                GLIDE[this->glide_mode_ > GLIDE_AUTO ? static_cast<uint8_t>(GLIDE_AUTO) : this->glide_mode_],
                this->lead_us_ < 0 ? 0.0f : this->lead_us_ / 1000.0f);
}

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
