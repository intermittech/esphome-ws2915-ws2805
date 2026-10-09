// Host test for components/ws2915/ws2915_frame.h: header layout, bit stream (fed through
// random RMT chunk sizes and decoded back), wire packing and the power limiter.
// Build: python tests/run_host_test.py   (uses the ziglang pip package as C++ compiler)

#include "../components/ws2915/ws2915_frame.h"

#include <cmath>
#include <initializer_list>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace esphome::ws2915;

static int failures = 0;
#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      failures++; \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__); \
      std::printf("\n"); \
    } \
  } while (0)

enum Sym : uint8_t { S0 = 0, S1 = 1, SR = 2, SX = 0xEE };

// Feed a whole frame through encode_symbols with random free-space chunks, like the RMT
// driver does when it refills its ping-pong memory.
static std::vector<Sym> run_encoder(const FrameSymbols<Sym> &p, const std::vector<uint8_t> &bytes, std::mt19937 &rng,
                                    int max_chunk) {
  std::vector<Sym> out;
  bool done = false;
  std::uniform_int_distribution<int> chunk(1, max_chunk);
  int guard = 0;
  while (!done && guard++ < 1000000) {
    const size_t n = chunk(rng);
    std::vector<Sym> buf(n, SX);
    const size_t got = encode_symbols(p, bytes.data(), bytes.size(), out.size(), n, buf.data(), &done);
    CHECK(got <= n, "encoder wrote %zu > free %zu", got, n);
    CHECK(got > 0 || done, "encoder made no progress");
    out.insert(out.end(), buf.begin(), buf.begin() + got);
  }
  CHECK(done, "encoder never finished");
  return out;
}

static void test_headers() {
  const uint8_t all[5] = {31, 31, 31, 31, 31};
  CHECK(build_header(HEADER_32BIT, all) == 0xFFFFFF80u, "32-bit all-31 = %08X", build_header(HEADER_32BIT, all));
  CHECK(build_header(HEADER_26BIT, all) == 0x03FFFFFFu, "26-bit all-31 = %08X", build_header(HEADER_26BIT, all));

  std::mt19937 rng(1);
  for (int i = 0; i < 1000; i++) {
    uint8_t g[5];
    for (auto &v : g)
      v = rng() % 32;
    // WorldSemi configurator formula (V1.3/V1.5) and datasheet V1.1 layout.
    const uint32_t ws32 = (uint32_t(g[0]) << 27) | (uint32_t(g[1]) << 22) | (uint32_t(g[2]) << 17) |
                          (uint32_t(g[3]) << 12) | (uint32_t(g[4]) << 7);
    const uint32_t ws26 = (uint32_t(g[0]) << 21) | (uint32_t(g[1]) << 16) | (uint32_t(g[2]) << 11) |
                          (uint32_t(g[3]) << 6) | (uint32_t(g[4]) << 1) | 1u;
    CHECK(build_header(HEADER_32BIT, g) == ws32, "32-bit header mismatch");
    CHECK(build_header(HEADER_26BIT, g) == ws26, "26-bit header mismatch");
    CHECK((build_header(HEADER_32BIT, g) & 0x7F) == 0, "32-bit check bits not 0");
  }
  CHECK(header_bits(CHIP_WS2915, HEADER_32BIT) == 32, "hbits 32");
  CHECK(header_bits(CHIP_WS2915, HEADER_26BIT) == 26, "hbits 26");
  CHECK(header_bits(CHIP_WS2805, HEADER_32BIT) == 0, "WS2805 has no header");
}

static void test_stream(ChipType chip, HeaderFormat fmt, int chips, int max_chunk, std::mt19937 &rng) {
  const size_t count = size_t(chips) * CHANNELS_PER_CHIP;
  std::vector<uint16_t> levels(count);
  for (auto &v : levels)
    v = rng() % (full_scale(chip) + 1);
  std::vector<uint8_t> bytes(size_t(chips) * bytes_per_chip(chip));
  pack_frame(chip, levels.data(), count, SCALE_ONE, bytes.data());

  uint8_t gain[5];
  for (auto &g : gain)
    g = rng() % 32;
  FrameSymbols<Sym> p{S0, S1, SR, build_header(fmt, gain), header_bits(chip, fmt)};
  const auto syms = run_encoder(p, bytes, rng, max_chunk);

  const size_t hb = p.header_bits;
  CHECK(syms.size() == hb + bytes.size() * 8 + 1, "symbol count %zu, want %zu", syms.size(),
        hb + bytes.size() * 8 + 1);
  if (syms.size() != hb + bytes.size() * 8 + 1)
    return;
  CHECK(syms.back() == SR, "last symbol is not reset");

  // Decode: header MSB first.
  uint32_t header = 0;
  for (size_t i = 0; i < hb; i++) {
    CHECK(syms[i] == S0 || syms[i] == S1, "bad header symbol");
    header = (header << 1) | (syms[i] == S1);
  }
  if (hb)
    CHECK(header == p.header, "header decoded %08X want %08X", header, p.header);

  // Decode data and compare with levels per chip/channel (R G B W1 W2, MSB first).
  const int width = is_16bit(chip) ? 16 : 8;
  size_t pos = hb;
  for (size_t i = 0; i < count; i++) {
    uint32_t v = 0;
    for (int b = 0; b < width; b++, pos++) {
      CHECK(syms[pos] == S0 || syms[pos] == S1, "bad data symbol at %zu", pos);
      v = (v << 1) | (syms[pos] == S1);
    }
    CHECK(v == levels[i], "chip %zu ch %zu decoded %u want %u", i / 5, i % 5, v, levels[i]);
  }
}

static void test_pack_and_limit() {
  // 16-bit big-endian, channel order R G B W1 W2.
  const uint16_t lv[5] = {0x1234, 0xFFFF, 0x0001, 0x8000, 0x0000};
  uint8_t out[10];
  pack_frame(CHIP_WS2915, lv, 5, SCALE_ONE, out);
  const uint8_t want[10] = {0x12, 0x34, 0xFF, 0xFF, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00};
  for (int i = 0; i < 10; i++)
    CHECK(out[i] == want[i], "pack16 byte %d = %02X want %02X", i, out[i], want[i]);
  pack_frame(CHIP_WS2915, lv, 5, SCALE_ONE / 2, out);
  CHECK(out[0] == 0x09 && out[1] == 0x1A && out[2] == 0x7F && out[3] == 0xFF, "pack16 half scale");

  const uint16_t lv8[5] = {255, 128, 1, 0, 77};
  uint8_t out8[5];
  pack_frame(CHIP_WS2805, lv8, 5, SCALE_ONE, out8);
  for (int i = 0; i < 5; i++)
    CHECK(out8[i] == lv8[i], "pack8 byte %d", i);

}

// Current a packed frame actually draws.
static float packed_current(ChipType chip, const uint8_t *bytes, size_t chips, const float *cur, std::vector<float> *per) {
  float total = 0;
  for (size_t c = 0; c < chips; c++) {
    float i = 0;
    for (int ch = 0; ch < 5; ch++) {
      uint32_t v = is_16bit(chip) ? (uint32_t(bytes[c * 10 + ch * 2]) << 8) | bytes[c * 10 + ch * 2 + 1]
                                  : bytes[c * 5 + ch];
      i += cur[ch] * float(v) / float(full_scale(chip));
    }
    if (per)
      per->push_back(i);
    total += i;
  }
  return total;
}

static PowerModel model(std::initializer_list<float> cur, std::initializer_list<float> limits, float chip, float line) {
  PowerModel pm{};
  int i = 0;
  for (float v : cur)
    pm.channel_current[i++] = v;
  i = 0;
  for (float v : limits)
    pm.max_channel_current[i++] = v;
  pm.max_chip_current = chip;
  pm.max_current = line;
  return pm;
}

static uint32_t level_at(ChipType chip, const uint8_t *bytes, size_t c, int ch) {
  return is_16bit(chip) ? (uint32_t(bytes[c * 10 + ch * 2]) << 8) | bytes[c * 10 + ch * 2 + 1] : bytes[c * 5 + ch];
}

static void test_power_limit(std::mt19937 &rng) {
  const uint16_t full[5] = {65535, 65535, 65535, 65535, 65535};
  uint8_t out[10];

  // Strip draws 3 A per RGB channel and 5 A per white channel at 100 %.
  const PowerModel none = model({3, 3, 3, 5, 5}, {}, 0, 0);
  PowerResult r = pack_frame_limited(CHIP_WS2915, full, 1, none, out);
  CHECK(std::fabs(r.requested - 19.0f) < 1e-3f && !r.limiting, "no-limit model: %f", r.requested);
  CHECK(out[0] == 0xFF && out[9] == 0xFF, "no-limit frame unchanged");

  // Per-chip limit (board fuse) keeps the colour mix.
  const PowerModel fuse = model({3, 3, 3, 5, 5}, {}, 5.0f, 0);
  r = pack_frame_limited(CHIP_WS2915, full, 1, fuse, out);
  const float got = packed_current(CHIP_WS2915, out, 1, fuse.channel_current, nullptr);
  CHECK(r.limiting && got <= 5.0f + 1e-4f && got > 4.99f, "chip limit: sent %f", got);
  CHECK(std::fabs(r.actual - got) < 1e-3f, "reported actual %f vs packed %f", r.actual, got);
  CHECK(out[0] == out[2] && out[2] == out[4] && out[6] == out[8], "chip limit keeps ratios");

  // Per-channel limits (QuinLED dig2analog-5ch: 3 A on CH1-3, 5 A on CH4-5) with a strip that draws
  // 4 A on CH1 and 6 A on CH5: only CH1 and CH5 are held back, the others stay at 100 %.
  const PowerModel chan = model({4, 2, 2, 4, 6}, {3, 3, 3, 5, 5}, 0, 0);
  r = pack_frame_limited(CHIP_WS2915, full, 1, chan, out);
  CHECK(r.limiting, "channel limits should report limiting");
  CHECK(level_at(CHIP_WS2915, out, 0, 0) == uint32_t(0.75f * 65535), "CH1 limited to 75 %%: %u",
        level_at(CHIP_WS2915, out, 0, 0));
  CHECK(level_at(CHIP_WS2915, out, 0, 1) == 65535 && level_at(CHIP_WS2915, out, 0, 2) == 65535 &&
            level_at(CHIP_WS2915, out, 0, 3) == 65535,
        "channels without a limit untouched");
  CHECK(4.0f * level_at(CHIP_WS2915, out, 0, 0) / 65535 <= 3.0f && 6.0f * level_at(CHIP_WS2915, out, 0, 4) / 65535 <= 5.0f,
        "limited channels within their rating");
  // Below its limit a channel is not touched at all.
  const uint16_t half[5] = {32768, 32768, 32768, 32768, 32768};
  r = pack_frame_limited(CHIP_WS2915, half, 1, chan, out);
  CHECK(!r.limiting && level_at(CHIP_WS2915, out, 0, 0) == 32768, "below channel limit: unchanged");

  // Random chains: every channel <= its limit, every chip <= its limit, line <= line limit.
  for (int iter = 0; iter < 2000; iter++) {
    const size_t chips = 1 + rng() % 16;
    const ChipType type = (iter & 1) ? CHIP_WS2805 : CHIP_WS2915;
    std::vector<uint16_t> lv(chips * 5);
    for (auto &v : lv)
      v = (rng() % 4 == 0) ? 0 : rng() % (full_scale(type) + 1);
    PowerModel pm{};
    for (int ch = 0; ch < 5; ch++) {
      pm.channel_current[ch] = float(rng() % 80) / 10;                      // 0..7.9 A
      pm.max_channel_current[ch] = (rng() % 3) ? float(1 + rng() % 60) / 10 : 0.0f;  // 0.1..6 A or none
    }
    pm.max_chip_current = (rng() % 2) ? 4.0f + float(rng() % 100) / 10 : 0.0f;
    pm.max_current = (rng() % 2) ? 2.0f + float(rng() % 400) / 10 : 0.0f;
    std::vector<uint8_t> bytes(chips * bytes_per_chip(type));
    r = pack_frame_limited(type, lv.data(), chips, pm, bytes.data());
    std::vector<float> per;
    const float total = packed_current(type, bytes.data(), chips, pm.channel_current, &per);
    for (size_t c = 0; c < chips; c++) {
      for (int ch = 0; ch < 5; ch++) {
        const uint32_t sent = level_at(type, bytes.data(), c, ch);
        CHECK(sent <= lv[c * 5 + ch], "chip %zu ch %d raised %u > %u", c, ch, sent, lv[c * 5 + ch]);
        if (pm.max_channel_current[ch] > 0) {
          const float i = pm.channel_current[ch] * float(sent) / float(full_scale(type));
          CHECK(i <= pm.max_channel_current[ch] + 1e-3f, "chip %zu ch %d: %f > limit %f", c, ch, i,
                pm.max_channel_current[ch]);
        }
      }
      if (pm.max_chip_current > 0)
        CHECK(per[c] <= pm.max_chip_current + 1e-3f, "chip %zu: %f > %f", c, per[c], pm.max_chip_current);
    }
    if (pm.max_current > 0)
      CHECK(total <= pm.max_current + 1e-3f, "line %f > %f", total, pm.max_current);
    CHECK(total <= r.actual + 1e-3f, "reported actual %f below packed %f", r.actual, total);
    CHECK(r.requested + 1e-3f >= total, "requested below sent");
    if (!r.limiting) {
      std::vector<uint8_t> plain(bytes.size());
      pack_frame(type, lv.data(), lv.size(), SCALE_ONE, plain.data());
      CHECK(plain == bytes, "not limiting but frame changed");
    }
  }
}

int main() {
  test_headers();
  std::mt19937 rng(12345);
  for (int iter = 0; iter < 300; iter++) {
    const int chips = 1 + int(rng() % 12);
    const int chunk = (iter % 3 == 0) ? 1 : 1 + int(rng() % 200);
    test_stream(CHIP_WS2915, HEADER_32BIT, chips, chunk, rng);
    test_stream(CHIP_WS2915, HEADER_26BIT, chips, chunk, rng);
    test_stream(CHIP_WS2805, HEADER_32BIT, chips, chunk, rng);
  }
  test_stream(CHIP_WS2915, HEADER_32BIT, 1024, 64, rng);  // max chain
  test_pack_and_limit();
  test_power_limit(rng);

  if (failures) {
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
  }
  std::printf("host test: all checks passed\n");
  return 0;
}
