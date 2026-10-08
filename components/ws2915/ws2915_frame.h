#pragma once

// Frame layout, bit encoding and power limiting for the WS2915 / WS2805.
// Pure C++ without ESP-IDF dependencies, so tests/host_test.cpp can exercise it on a PC.

#include <cstddef>
#include <cstdint>

#if defined(__GNUC__)
#define WS2915_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define WS2915_ALWAYS_INLINE inline
#endif

namespace esphome::ws2915 {

enum ChipType : uint8_t { CHIP_WS2915 = 0, CHIP_WS2805 = 1 };
enum HeaderFormat : uint8_t { HEADER_32BIT = 0, HEADER_26BIT = 1 };

static constexpr uint8_t CHANNELS_PER_CHIP = 5;  // R, G, B, W1, W2 (wire order)
static constexpr uint8_t GAIN_MAX = 31;
static constexpr uint32_t SCALE_ONE = 65536;  // Q16 1.0 for pack_frame()

inline bool is_16bit(ChipType chip) { return chip == CHIP_WS2915; }
inline uint32_t full_scale(ChipType chip) { return is_16bit(chip) ? 65535 : 255; }
inline size_t bytes_per_chip(ChipType chip) { return is_16bit(chip) ? 10 : 5; }

inline uint8_t header_bits(ChipType chip, HeaderFormat fmt) {
  if (!is_16bit(chip))
    return 0;
  return fmt == HEADER_26BIT ? 26 : 32;
}

// WS2915 gain header, right-aligned: the low header_bits() bits are sent MSB first.
//   32-bit (datasheet V1.3/V1.5): R5 G5 B5 W1-5 W2-5, then 7 check bits '0'
//   26-bit (datasheet V1.1):      R5 G5 B5 W1-5 W2-5, then 1 check bit '1'
inline uint32_t build_header(HeaderFormat fmt, const uint8_t *gain) {
  uint32_t g = 0;
  for (uint8_t i = 0; i < CHANNELS_PER_CHIP; i++)
    g = (g << 5) | (gain[i] & GAIN_MAX);
  return fmt == HEADER_26BIT ? (g << 1) | 1U : g << 7;
}

template<typename Symbol> struct FrameSymbols {
  Symbol bit0;
  Symbol bit1;
  Symbol reset;
  uint32_t header;
  uint8_t header_bits;
};

// One call of the RMT simple-encoder callback. A frame is header_bits header symbols,
// then size * 8 data symbols (MSB first), then one reset symbol. symbols_written is
// cumulative over the transaction, so it doubles as the bit cursor. Emits one symbol per
// iteration, so a single free slot is enough (min_chunk_size = 1). Always inlined, so it
// lands in the IRAM_ATTR callback that calls it.
template<typename Symbol>
WS2915_ALWAYS_INLINE size_t encode_symbols(const FrameSymbols<Symbol> &p, const uint8_t *bytes, size_t size,
                             size_t symbols_written, size_t symbols_free, Symbol *symbols, bool *done) {
  const size_t hbits = p.header_bits;
  const size_t data_end = hbits + size * 8;
  size_t pos = symbols_written;
  size_t out = 0;
  while (out < symbols_free) {
    if (pos < hbits) {
      symbols[out++] = ((p.header >> (hbits - 1 - pos)) & 1U) ? p.bit1 : p.bit0;
    } else if (pos < data_end) {
      const size_t bit = pos - hbits;
      symbols[out++] = (bytes[bit >> 3] & (0x80U >> (bit & 7U))) ? p.bit1 : p.bit0;
    } else {
      symbols[out++] = p.reset;
      *done = true;
      break;
    }
    pos++;
  }
  return out;
}

// Levels (0..full_scale, chip-major, CHANNELS_PER_CHIP each) -> wire bytes, multiplied by
// scale_q16 / 65536 (rounded down, so a limited frame never exceeds the budget).
inline void pack_frame(ChipType chip, const uint16_t *levels, size_t count, uint32_t scale_q16, uint8_t *out) {
  const bool wide = is_16bit(chip);
  for (size_t i = 0; i < count; i++) {
    uint32_t v = levels[i];
    if (scale_q16 < SCALE_ONE)
      v = (v * scale_q16) >> 16;
    if (wide) {
      *out++ = static_cast<uint8_t>(v >> 8);
      *out++ = static_cast<uint8_t>(v);
    } else {
      *out++ = static_cast<uint8_t>(v);
    }
  }
}

struct PowerModel {
  float channel_current[CHANNELS_PER_CHIP];      // LED current per channel at 100 % duty, A
  float max_channel_current[CHANNELS_PER_CHIP];  // per channel (e.g. its MOSFET rating), A; 0 = no limit
  float max_chip_current;                        // per chip (e.g. its board fuse), A; 0 = no limit
  float max_current;                             // whole line (e.g. the PSU), A; 0 = no limit
};

struct PowerResult {
  float requested;  // what the levels ask for, A
  float actual;     // after limiting (upper bound; rounding only lowers it), A
  bool limiting;
};

inline float chip_current(const uint16_t *levels, const float *channel_current, float inv_full_scale) {
  float sum = 0.0f;
  for (uint8_t ch = 0; ch < CHANNELS_PER_CHIP; ch++)
    sum += channel_current[ch] * static_cast<float>(levels[ch]);
  return sum * inv_full_scale;
}

// Highest level each channel may reach under max_channel_current (full scale if uncapped).
inline void channel_level_caps(const PowerModel &pm, uint32_t full, uint16_t *caps) {
  for (uint8_t ch = 0; ch < CHANNELS_PER_CHIP; ch++) {
    caps[ch] = static_cast<uint16_t>(full);
    const float cap = pm.max_channel_current[ch], cur = pm.channel_current[ch];
    if (cap > 0.0f && cur > cap)
      caps[ch] = static_cast<uint16_t>(cap / cur * static_cast<float>(full));  // rounded down
  }
}

// One chip's levels after the per-channel caps. Returns true if a channel was capped.
inline bool clamp_channels(const uint16_t *levels, const uint16_t *caps, uint16_t *out) {
  bool capped = false;
  for (uint8_t ch = 0; ch < CHANNELS_PER_CHIP; ch++) {
    out[ch] = levels[ch] > caps[ch] ? caps[ch] : levels[ch];
    capped |= levels[ch] > caps[ch];
  }
  return capped;
}

// pack_frame() with power limiting, in three steps:
//   1. a channel above max_channel_current is capped on its own (other channels, and so
//      independent lights on the same chip, are not touched);
//   2. a chip above max_chip_current is scaled down uniformly (its colour mix is kept);
//   3. if the line is still above max_current, every chip is scaled down uniformly.
// The targets in `levels` are not modified.
inline PowerResult pack_frame_limited(ChipType chip, const uint16_t *levels, size_t num_chips, const PowerModel &pm,
                                      uint8_t *out) {
  const uint32_t full = full_scale(chip);
  const float inv_fs = 1.0f / static_cast<float>(full);
  const size_t bpc = bytes_per_chip(chip);
  const bool chip_limit = pm.max_chip_current > 0.0f;
  uint16_t caps[CHANNELS_PER_CHIP];
  channel_level_caps(pm, full, caps);
  uint16_t lv[CHANNELS_PER_CHIP];

  float requested = 0.0f, after_chip = 0.0f;
  for (size_t c = 0; c < num_chips; c++) {
    const uint16_t *raw = levels + c * CHANNELS_PER_CHIP;
    requested += chip_current(raw, pm.channel_current, inv_fs);
    clamp_channels(raw, caps, lv);
    const float i = chip_current(lv, pm.channel_current, inv_fs);
    after_chip += (chip_limit && i > pm.max_chip_current) ? pm.max_chip_current : i;
  }
  const float line = (pm.max_current > 0.0f && after_chip > pm.max_current) ? pm.max_current / after_chip : 1.0f;

  PowerResult r{requested, 0.0f, line < 1.0f};
  for (size_t c = 0; c < num_chips; c++) {
    if (clamp_channels(levels + c * CHANNELS_PER_CHIP, caps, lv))
      r.limiting = true;
    const float i = chip_current(lv, pm.channel_current, inv_fs);
    float s = line;
    if (chip_limit && i > pm.max_chip_current) {
      s *= pm.max_chip_current / i;
      r.limiting = true;
    }
    const uint32_t q = s >= 1.0f ? SCALE_ONE : static_cast<uint32_t>(s * static_cast<float>(SCALE_ONE));
    pack_frame(chip, lv, CHANNELS_PER_CHIP, q, out + c * bpc);
    r.actual += i * static_cast<float>(q) / static_cast<float>(SCALE_ONE);
  }
  return r;
}

}  // namespace esphome::ws2915
