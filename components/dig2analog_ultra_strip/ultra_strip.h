#pragma once

#ifdef USE_ESP32

#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/transformers.h"
#include "esphome/components/dig2analog_ultra/ultra_light.h"
#include "esphome/components/dig2analog_ultra/ultra_stream.h"

#include <memory>
#include <vector>

namespace esphome::dig2analog_ultra {

class UltraStrip;

/// A transition of the strip without an addressable effect: every pixel gets its target and the length once, and the
/// boards fade in 16 bit. Nothing is sent meanwhile (as UltraTransition does for one light).
class UltraStripTransition final : public light::LightTransitionTransformer {
 public:
  explicit UltraStripTransition(UltraStrip *strip) : strip_(strip) {}
  ~UltraStripTransition() override;
  void start() override;
  void stop() override;

 protected:
  UltraStrip *strip_;
};

/// A virtual addressable strip whose pixels are dig2analog_ultra lights (any boards, any hubs, in the order given).
/// ESPHome's addressable effects run on it unchanged; each changed pixel goes to its board as one SET:
///  - the strip's brightness stays 16 bit (M), the pixel's colour is the 8-bit effect output, and the board applies the
///    light's own gamma, min / max power and dim to warm;
///  - effect steps are streamed (sent once) and glide (effect_glide);
///  - without an effect the strip's colour goes in full precision, and transitions run on the boards.
/// While it is on, the strip is a layer over its lights: it is shown, their own state stays underneath (changes to the
/// lights are kept, their board effects pause). When it turns off, its lights fade back to their own state, or to
/// another strip that is still on.
class UltraStrip final : public light::AddressableLight, public UltraLayer {
 public:
  // Configuration (codegen)
  /// roles: CH1..CH5 roles (ULTRA_ROLE_*) of the light, 4 bits each, CH1 in the lowest nibble
  void add_pixel(UltraLight *light, uint32_t roles);
  /// rgb: some light has RGB (else the strip is brightness-only, e.g. single-channel downlights); white: some light
  /// has white channels (the pixels' W)
  void set_colors(bool rgb, bool white) {
    this->rgb_ = rgb;
    this->white_ = rgb && white;
  }
  void set_glide_mode(uint8_t mode) { this->glide_mode_ = mode; }
  /// Effects (by index, 1 = the first) whose steps stay hard with effect_glide: auto (strobe)
  void add_hard_effect(uint32_t index) { this->hard_effects_.push_back(index); }

  // AddressableLight
  int32_t size() const override { return static_cast<int32_t>(this->pixels_.size()); }
  void clear_effect_data() override;
  light::LightTraits get_traits() override;
  void setup_state(light::LightState *state) override;
  void update_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;
  std::unique_ptr<light::LightTransformer> create_default_transition() override;

  // UltraLayer: the layer above ended on this light, send its pixel again
  void layer_shown(UltraLight *light) override;

  // Component
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  friend class UltraStripTransition;

  struct Pixel {
    UltraLight *light;
    uint8_t roles[CHANNELS];
    uint16_t m;  // what the board was sent last (M, A; B is 1)
    uint16_t a[CHANNELS];
    bool sent;
  };

  light::ESPColorView get_view_internal(int32_t index) const override;
  void map_(const Pixel &p, float r, float g, float b, float w, float level, uint16_t &m, uint16_t *a) const;
  void map_values_(const Pixel &p, const light::LightColorValues &v, float level, uint16_t &m, uint16_t *a) const;
  void send_(Pixel &p, uint16_t m, const uint16_t *a, uint32_t length, uint8_t flags, bool stream);
  bool is_hard_(uint32_t effect) const;
  bool buffer_lit_() const;
  int64_t start_now_();
  void begin_layer_();
  void end_layer_(uint32_t fade_ms);
  void transition_start_(UltraStripTransition *t, const light::LightColorValues &end, uint32_t length, bool via_off);
  void transition_done_(UltraStripTransition *t, bool finished);

  std::vector<Pixel> pixels_;
  mutable std::vector<uint8_t> buf_;  // r, g, b, w, effect data per pixel (views write through it)
  std::vector<uint32_t> hard_effects_;
  UltraStripTransition *transition_{nullptr};
  GlideTimer glide_;
  uint32_t glide_effect_{0};
  uint8_t glide_mode_{GLIDE_AUTO};
  bool rgb_{true};
  bool white_{false};
  bool layer_{false};  // the strip is a layer over its lights
  int64_t lead_us_{-1};  // start of a write after now (start_now_)
  int64_t start_us_{0};  // start of the current write, the same for every pixel
};

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
