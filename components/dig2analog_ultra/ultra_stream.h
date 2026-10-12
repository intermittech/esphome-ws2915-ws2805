#pragma once

// Streamed light values without ESP-IDF / ESPHome dependencies (tests/ultra_stream_host_test.cpp runs it on a PC):
// the glide between the steps of a host-side effect, and how one pixel of a dig2analog_ultra_strip maps onto the
// channels of one light.

#include <cstdint>

#include "ultra_protocol.h"

namespace esphome::dig2analog_ultra {

static constexpr uint32_t GLIDE_MAX_MS = 1000;  // longest glide between two steps
static constexpr uint32_t GLIDE_GAP_MS = 2000;  // a longer pause starts a new measurement
static constexpr uint32_t GLIDE_TAIL_MS = 50;   // effect_glide auto: a light dropping by a jump fades out this long
static constexpr uint32_t GLIDE_JUMP = 16384;   // a change of more than 25 % of full scale is a jump

class UltraLight;

/// A dig2analog_ultra_strip that is on is a layer over its lights: the newest layer is shown, the light's own state
/// stays underneath. When the layer on top ends, the light tells the next one to send its value again.
class UltraLayer {
 public:
  virtual void layer_shown(UltraLight *light) = 0;

 protected:
  ~UltraLayer() = default;
};

/// The step interval of a host-side effect, measured from its steps: the board glides from one step to the next over
/// this long, so it arrives about when the next step comes. A running average of the intervals; the first step after
/// a start or a pause jumps (0).
class GlideTimer {
 public:
  /// A step at now_ms: its glide length in ms.
  uint32_t step(uint32_t now_ms) {
    const uint32_t dt = now_ms - this->last_ms_;
    this->last_ms_ = now_ms;
    if (!this->running_ || dt > GLIDE_GAP_MS) {
      this->running_ = true;
      this->steps_ = 1;
      this->avg_ = 0;
      return 0;
    }
    this->avg_ = this->steps_++ == 1 ? dt : (this->avg_ * 3 + dt + 2) / 4;
    return this->avg_ > GLIDE_MAX_MS ? GLIDE_MAX_MS : this->avg_;
  }
  /// The last step followed another one within GLIDE_GAP_MS: the values are a stream, not a single change.
  bool steady() const { return this->running_ && this->steps_ > 1; }
  void reset() { this->running_ = false; }

 protected:
  uint32_t last_ms_{0};
  uint32_t avg_{0};
  uint32_t steps_{0};
  bool running_{false};
};

inline uint16_t unit16(float v) {
  if (!(v > 0.0f))
    return 0;
  if (v >= 1.0f)
    return 65535;
  return static_cast<uint16_t>(v * 65535.0f + 0.5f);
}

/// One pixel (r, g, b, w: 0..1, the pixel's own brightness included) as M / A / B of a light with these roles.
///  - level: M of the strip (its state x brightness, 0..1). The board applies the light's own gamma, min / max power
///    and dim to warm to the result, in 16 bit.
///  - cw, ww: the white mix for cold / warm white channels (the light's own colour temperature, the larger one 1).
/// Per channel:
///  - red / green / blue take r / g / b. A light with RGB but no white channel adds w to all three (white = R = G = B).
///  - white takes w; cold / warm white take w x their mix.
///  - A light without RGB (CCT, single white) takes the pixel's value max(r, g, b, w): colour is lost, brightness
///    stays. A brightness channel (monochromatic light) gets it through M.
/// Linear in the pixel, so a glide between two pixels is a straight line in RGBW. A dark pixel is M = 0, A = 0.
inline void pixel_to_mab(const uint8_t *roles, float r, float g, float b, float w, float level, float cw, float ww,
                         uint16_t &m, uint16_t *a, uint16_t *bb) {
  bool rgb = false, white = false;
  for (uint8_t i = 0; i < 5; i++) {
    rgb |= roles[i] >= ULTRA_ROLE_RED && roles[i] <= ULTRA_ROLE_BLUE;
    white |= roles[i] >= ULTRA_ROLE_WHITE && roles[i] <= ULTRA_ROLE_WARM_WHITE;
  }
  float value = r > g ? r : g;
  value = value > b ? value : b;
  value = value > w ? value : w;
  const float add = rgb && !white ? w : 0.0f;
  m = unit16(level);
  for (uint8_t i = 0; i < 5; i++) {
    float v = 0.0f;
    bb[i] = 65535;
    switch (roles[i]) {
      case ULTRA_ROLE_BRIGHTNESS:
        v = 1.0f;
        m = unit16(level * value);
        break;
      case ULTRA_ROLE_RED:
        v = r + add;
        break;
      case ULTRA_ROLE_GREEN:
        v = g + add;
        break;
      case ULTRA_ROLE_BLUE:
        v = b + add;
        break;
      case ULTRA_ROLE_WHITE:
        v = rgb ? w : value;
        break;
      case ULTRA_ROLE_COLD_WHITE:
        v = (rgb ? w : value) * cw;
        break;
      case ULTRA_ROLE_WARM_WHITE:
        v = (rgb ? w : value) * ww;
        break;
      default:
        break;
    }
    a[i] = unit16(v);
  }
  if (!(value > 0.0f) || m == 0) {
    m = 0;
    for (uint8_t i = 0; i < 5; i++)
      a[i] = 0;
  }
}

/// The white mix of a light from its cold / warm white values: the larger one 1 (both 1 when there are none).
inline void white_mix(float cold, float warm, float &cw, float &ww) {
  const float top = cold > warm ? cold : warm;
  if (!(top > 0.0f)) {
    cw = ww = 1.0f;
    return;
  }
  cw = cold / top;
  ww = warm / top;
}

/// The largest change of a channel between two outputs (M x A, 0..65535): how big a step is.
inline uint32_t change_of(uint16_t m1, const uint16_t *a1, uint16_t m2, const uint16_t *a2) {
  uint32_t top = 0;
  for (uint8_t i = 0; i < 5; i++) {
    const uint32_t v1 = static_cast<uint32_t>(m1) * a1[i] / 65535, v2 = static_cast<uint32_t>(m2) * a2[i] / 65535;
    const uint32_t d = v1 > v2 ? v1 - v2 : v2 - v1;
    top = d > top ? d : top;
  }
  return top;
}

/// The brightest channel of an output (M x A, 0..65535).
inline uint32_t value_of(uint16_t m, const uint16_t *a) {
  uint32_t top = 0;
  for (uint8_t i = 0; i < 5; i++) {
    const uint32_t v = static_cast<uint32_t>(m) * a[i] / 65535;
    top = v > top ? v : top;
  }
  return top;
}

/// effect_glide auto, per light and step:
///  - a small step (a rainbow moving on, a flicker) glides over the whole step interval, so the light moves without
///    stopping;
///  - a jump up (a chase or scan arriving, a sparkle) is immediate, so it stays crisp;
///  - a jump down (the light the chase just left) fades out over GLIDE_TAIL_MS: a short trail.
/// The board fades before gamma, so two lights crossing over linearly dip halfway (both at 50 % give ~29 % of one at
/// gamma 2.8); jumping up and fading down keeps the total from dipping.
inline uint32_t glide_for(uint32_t interval_ms, uint32_t change, bool rising) {
  if (change <= GLIDE_JUMP)
    return interval_ms;
  if (rising)
    return 0;
  return interval_ms < GLIDE_TAIL_MS ? interval_ms : GLIDE_TAIL_MS;
}

/// Same output on the board: both dark, or the same M and A (B is always 1 for pixels).
inline bool same_output(uint16_t m1, const uint16_t *a1, uint16_t m2, const uint16_t *a2) {
  if (m1 == 0 && m2 == 0)
    return true;
  if (m1 != m2)
    return false;
  for (uint8_t i = 0; i < 5; i++) {
    if (a1[i] != a2[i])
      return false;
  }
  return true;
}

}  // namespace esphome::dig2analog_ultra
