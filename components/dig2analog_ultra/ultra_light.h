#pragma once

#ifdef USE_ESP32

#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/light/transformers.h"

#include "dig2analog_ultra.h"
#include "ultra_stream.h"

#include <memory>
#include <vector>

namespace esphome::dig2analog_ultra {

class UltraLight;

/// ESPHome's transition, run by the board: start() sends the target and length once (SET); ESPHome still
/// interpolates its own copy for the state it reports, but nothing is written to the line meanwhile.
class UltraTransition final : public light::LightTransitionTransformer {
 public:
  explicit UltraTransition(UltraLight *light) : light_(light) {}
  ~UltraTransition() override;
  void start() override;
  void stop() override;

 protected:
  UltraLight *light_;
};

/// Power-on scene modes (power_on_scene: true), from the light's restore_mode: what ESPHome will restore at its next
/// boot is what the board shows from its own power-up until ESPHome takes over.
enum PowerOnScene : uint8_t {
  SCENE_NONE = 0,             // not in the scene: dark until the host takes over
  SCENE_FIXED = 1,            // ALWAYS_ON: the state after boot (initial_state), sent once
  SCENE_FOLLOW = 2,           // RESTORE_DEFAULT_OFF / _ON: the current state
  SCENE_FOLLOW_ON = 3,        // RESTORE_AND_ON: the current values, on
  SCENE_FOLLOW_INVERTED = 4,  // RESTORE_INVERTED_*: the current values, on / off inverted
};

/// effect_glide: how the steps of ESPHome's own (host-side) effects reach the board.
enum EffectGlide : uint8_t {
  GLIDE_OFF = 0,   // hard steps
  GLIDE_ON = 1,    // every step glides over the measured step interval
  GLIDE_AUTO = 2,  // small steps glide, a jump up is immediate, a jump down fades GLIDE_TAIL_MS (glide_for)
};
/// What an effect of the light is, by index (codegen): its steps glide and are streamed, or they are commands.
enum EffectKind : uint8_t {
  EFFECT_HOST = 0,   // ESPHome's own effect: streamed steps (sent once), gliding
  EFFECT_HARD = 1,   // strobe: commands (sent with repeats), hard
  EFFECT_BOARD = 2,  // ultra_*: runs on the board; changes meanwhile are commands
};

/// A light on 1-5 channels of one board. Roles per channel (red, green, blue, white, cold_white, warm_white or
/// brightness) give the ESPHome light type (monochromatic, cwww, rgb, rgbw, rgbww). Gamma, min / max power,
/// transitions and the ultra_* effects run on the board in 16 bit.
class UltraLight final : public light::LightOutput, public light::LightRemoteValuesListener {
 public:
  // Configuration (codegen)
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }
  void set_light_index(uint8_t index) { this->index_ = index; }
  void set_role(uint8_t channel, uint8_t role) {
    if (channel < CHANNELS)
      this->roles_[channel] = role;
  }
  void set_power_range(uint8_t channel, float min_power, float max_power) {
    if (channel < CHANNELS) {
      this->min_[channel] = to16(min_power);
      this->max_[channel] = to16(max_power);
    }
  }
  void set_zero_means_zero(bool zmz) { this->zero_means_zero_ = zmz; }
  void set_constant_brightness(bool cb) { this->constant_brightness_ = cb; }
  void set_color_interlock(bool ci) { this->color_interlock_ = ci; }
  void set_cold_white_temperature(float mireds) { this->cold_white_temperature_ = mireds; }
  void set_warm_white_temperature(float mireds) { this->warm_white_temperature_ = mireds; }
  void set_power_on_scene(uint8_t mode) { this->scene_mode_ = mode; }
  /// dim_to_warm: in the YAML (warm end in mireds, red assist 0..1); on from boot unless set_dim_to_warm(false)
  void set_dim_to_warm_config(float warm_mireds, float red_assist) {
    this->dtw_available_ = true;
    this->dtw_on_ = true;
    this->dtw_warm_ = warm_mireds;
    this->dtw_assist_ = red_assist;
  }
  /// dim_to_warm points: (brightness 0..1, colour temperature in mireds), up to ULTRA_OPT_POINTS_MAX
  void add_dim_to_warm_point(float brightness, float mireds) {
    if (this->dtw_n_ < ULTRA_OPT_POINTS_MAX) {
      this->dtw_pb_[this->dtw_n_] = brightness;
      this->dtw_pt_[this->dtw_n_] = mireds;
      this->dtw_n_++;
    }
  }
  /// Dim to warm on / off at runtime (lambda, e.g. a template switch; needs dim_to_warm: in the YAML). The board
  /// switches with a smooth change of mix only where the light is dimmed.
  void set_dim_to_warm(bool on);
  bool get_dim_to_warm() const { return this->dtw_on_; }
  void set_effect_glide(uint8_t mode) { this->glide_mode_ = mode; }
  /// Effect index (1 = the first) -> EffectKind; effects not listed are EFFECT_HOST
  void set_effect_kind(uint32_t index, uint8_t kind) {
    if (index >= this->effect_kinds_.size())
      this->effect_kinds_.resize(index + 1, EFFECT_HOST);
    this->effect_kinds_[index] = kind;
  }

  // LightRemoteValuesListener: the state ESPHome would restore changed -> the board's power-on scene
  void on_light_remote_values_update() override;

  // LightOutput
  light::LightTraits get_traits() override;
  std::unique_ptr<light::LightTransformer> create_default_transition() override;
  void setup_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;

  // Board effects (ultra_* effects). start_us 0: the hub's start time.
  void start_effect(uint8_t fx, const uint8_t *params, size_t n, const std::vector<Step> *steps = nullptr,
                    int64_t start_us = 0);
  void stop_effect();
  /// M / A / B of a color (an UNKNOWN color mode means the light's current one)
  void map(const light::LightColorValues &v, uint16_t &m, uint16_t *a, uint16_t *b) const;
  light::LightState *get_state() const { return this->state_; }
  UltraHub *get_hub() const { return this->hub_; }
  uint16_t get_board() const { return this->board_; }

  // dig2analog_ultra_strip
  /// A pixel's value: a streamed step (sent once, linear glide over length_ms) or a command (with repeats, flags as
  /// SET), starting at start_us (0: the hub's start time).
  void send_pixel(uint16_t m, const uint16_t *a, const uint16_t *b, uint32_t length_ms, uint8_t flags, bool stream,
                  int64_t start_us);
  /// A strip that is on is a layer over the light: the newest layer is shown. Meanwhile the light's own changes
  /// are kept (not sent) and its board effect pauses.
  void layer_begin(UltraLayer *layer);
  /// The layer is gone: the next layer sends again, or the light goes back to its own state over fade_ms (and its
  /// board effect runs again, in phase).
  void layer_end(UltraLayer *layer, uint32_t fade_ms);
  bool layer_top(const UltraLayer *layer) const { return !this->layers_.empty() && this->layers_.back() == layer; }
  /// On, or still fading out on the board (power supply request).
  bool is_lit(uint32_t now_ms) const { return this->on_ || static_cast<int32_t>(this->off_until_ms_ - now_ms) > 0; }
  bool has_rgb() const;

  /// The UltraLight behind a light's output, nullptr for other light platforms (no RTTI in ESPHome).
  static UltraLight *find(light::LightOutput *output);

 protected:
  friend class UltraTransition;
  void transition_start_(UltraTransition *t, const light::LightColorValues &end, uint32_t length, bool via_off);
  void transition_done_(UltraTransition *t);
  void send_(const light::LightColorValues &v, uint32_t length, uint8_t flags, bool stream = false);
  void lit_(uint16_t m, uint32_t length_ms);
  uint8_t effect_kind_(uint32_t index) const {
    return index < this->effect_kinds_.size() ? this->effect_kinds_[index] : static_cast<uint8_t>(EFFECT_HOST);
  }

  UltraHub *hub_{nullptr};
  light::LightState *state_{nullptr};
  UltraTransition *transition_{nullptr};  // board-run transition in progress
  uint16_t board_{0};
  uint8_t index_{0};
  uint8_t roles_[CHANNELS]{};
  uint16_t min_[CHANNELS]{};
  uint16_t max_[CHANNELS]{65535, 65535, 65535, 65535, 65535};
  bool zero_means_zero_{false};
  bool constant_brightness_{false};
  bool color_interlock_{false};
  float cold_white_temperature_{0};
  float warm_white_temperature_{0};

  void send_opt_();

  bool dtw_available_{false};
  bool dtw_on_{false};
  float dtw_warm_{0};
  float dtw_assist_{0};
  uint8_t dtw_n_{0};                         // set points (0: the straight line to dtw_warm_)
  float dtw_pb_[ULTRA_OPT_POINTS_MAX]{};     // brightness 0..1
  float dtw_pt_[ULTRA_OPT_POINTS_MAX]{};     // mireds
  uint8_t scene_mode_{SCENE_NONE};
  bool scene_sent_{false};
  uint8_t scene_fade_{0};
  uint16_t scene_m_{0}, scene_a_[CHANNELS]{}, scene_b_[CHANNELS]{};

  bool sent_{false};
  bool on_{false};
  uint32_t off_until_ms_{0};
  uint16_t last_m_{0}, last_a_[CHANNELS]{}, last_b_[CHANNELS]{};

  uint8_t glide_mode_{GLIDE_AUTO};
  std::vector<uint8_t> effect_kinds_;  // by effect index (EffectKind)
  GlideTimer glide_;
  uint32_t glide_effect_{0};           // the effect the timer measures
  std::vector<UltraLayer *> layers_;   // strips shown on this light, the newest last

  static std::vector<UltraLight *> registry_;
};

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
