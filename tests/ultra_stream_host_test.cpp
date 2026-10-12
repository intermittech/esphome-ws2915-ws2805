// Host test for components/dig2analog_ultra/ultra_stream.h: the pixel -> M / A / B mapping of dig2analog_ultra_strip
// and the glide timer of host-side effect steps. With the board firmware next to this repo, the steps also go
// through the scheduler and the boards' real engine: hard steps versus glides.
// Build: python tests/run_ultra_stream_host_test.py

#include "../components/dig2analog_ultra/ultra_core.h"
#include "../components/dig2analog_ultra/ultra_stream.h"

#ifdef WITH_ENGINE
extern "C" {
#include "ultra.h"
}
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace esphome::dig2analog_ultra;

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

static const uint8_t RGB[5] = {ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, 0, 0};
static const uint8_t RGBW[5] = {ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_WHITE, 0};
static const uint8_t RGBWW[5] = {ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_WARM_WHITE,
                                 ULTRA_ROLE_COLD_WHITE};
static const uint8_t CCT[5] = {ULTRA_ROLE_WARM_WHITE, ULTRA_ROLE_COLD_WHITE, 0, 0, 0};
static const uint8_t MONO[5] = {0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS};
static const uint8_t WHITE1[5] = {0, 0, 0, ULTRA_ROLE_WHITE, 0};

static bool near(uint16_t v, float want) { return std::fabs(v - want * 65535.0f) <= 1.0f; }

static void test_pixel_map() {
  uint16_t m, a[5], b[5];
  // RGB: colour into A, the strip's brightness into M
  pixel_to_mab(RGB, 1.0f, 0.5f, 0.0f, 0.0f, 0.4f, 1, 1, m, a, b);
  CHECK(near(m, 0.4f) && near(a[0], 1.0f) && near(a[1], 0.5f) && a[2] == 0 && b[0] == 65535, "rgb %u %u %u", m, a[0],
        a[1]);
  // RGB light, pixel with white only: white = R = G = B
  pixel_to_mab(RGB, 0.0f, 0.0f, 0.0f, 0.6f, 1.0f, 1, 1, m, a, b);
  CHECK(near(a[0], 0.6f) && near(a[1], 0.6f) && near(a[2], 0.6f), "rgb white %u %u %u", a[0], a[1], a[2]);
  // RGBW: white on W
  pixel_to_mab(RGBW, 0.2f, 0.0f, 0.0f, 0.8f, 1.0f, 1, 1, m, a, b);
  CHECK(near(a[0], 0.2f) && a[1] == 0 && near(a[3], 0.8f), "rgbw %u %u", a[0], a[3]);
  // RGBCCT: white on CW / WW in the light's own mix (warm 1, cold 0.5)
  pixel_to_mab(RGBWW, 0.0f, 0.0f, 1.0f, 0.8f, 1.0f, 0.5f, 1.0f, m, a, b);
  CHECK(near(a[2], 1.0f) && near(a[3], 0.8f) && near(a[4], 0.4f), "rgbww ww %u cw %u", a[3], a[4]);
  // CCT: colour is lost, the value stays (max of the channels), in the light's mix
  pixel_to_mab(CCT, 0.3f, 0.9f, 0.1f, 0.0f, 1.0f, 1.0f, 0.25f, m, a, b);
  CHECK(near(a[0], 0.225f) && near(a[1], 0.9f), "cct ww %u cw %u", a[0], a[1]);
  // monochromatic: value x level through M
  pixel_to_mab(MONO, 0.0f, 0.5f, 0.0f, 0.0f, 0.5f, 1, 1, m, a, b);
  CHECK(near(m, 0.25f) && a[4] == 65535, "mono m %u", m);
  // a single white channel: the value
  pixel_to_mab(WHITE1, 0.7f, 0.0f, 0.0f, 0.0f, 1.0f, 1, 1, m, a, b);
  CHECK(near(a[3], 0.7f), "white1 %u", a[3]);
  // dark: M = 0, A = 0 (a dark pixel and a dark strip compare equal)
  pixel_to_mab(RGBW, 0, 0, 0, 0, 1.0f, 1, 1, m, a, b);
  uint16_t m2, a2[5];
  pixel_to_mab(RGBW, 1, 1, 1, 1, 0.0f, 1, 1, m2, a2, b);
  CHECK(m == 0 && m2 == 0 && same_output(m, a, m2, a2), "dark");
  // linear: the middle of two pixels maps to the middle of their outputs
  uint16_t ma, aa[5], mb, ab[5], mm, am[5];
  pixel_to_mab(RGB, 1.0f, 0.2f, 0.0f, 0, 1, 1, 1, ma, aa, b);
  pixel_to_mab(RGB, 0.0f, 0.6f, 1.0f, 0, 1, 1, 1, mb, ab, b);
  pixel_to_mab(RGB, 0.5f, 0.4f, 0.5f, 0, 1, 1, 1, mm, am, b);
  bool lin = true;
  for (int i = 0; i < 3; i++)
    lin &= std::abs((aa[i] + ab[i]) / 2 - am[i]) <= 1;
  CHECK(lin, "linear in the pixel");
  float cw, ww;
  white_mix(0.2f, 0.8f, cw, ww);
  CHECK(std::fabs(cw - 0.25f) < 1e-6f && ww == 1.0f, "white mix %f %f", cw, ww);
  white_mix(0, 0, cw, ww);
  CHECK(cw == 1.0f && ww == 1.0f, "white mix of nothing");
  std::printf("  pixel mapping: RGB / RGBW / RGBCCT / CCT / mono / white, dark, linear\n");
}

static void test_glide_rule() {
  const uint16_t full[5] = {65535, 65535, 65535, 65535, 65535}, dark[5] = {0, 0, 0, 0, 0};
  const uint16_t red[5] = {65535, 0, 0, 0, 0}, red2[5] = {60000, 0, 0, 0, 0};
  CHECK(value_of(65535, full) == 65535 && value_of(32768, red) == 32768, "value");
  CHECK(change_of(65535, red, 65535, red2) == 5535 && change_of(0, dark, 65535, full) == 65535, "change");
  CHECK(glide_for(30, 5535, false) == 30, "a small step glides the whole interval");
  CHECK(glide_for(150, 65535, true) == 0, "a jump up is immediate");
  CHECK(glide_for(150, 65535, false) == GLIDE_TAIL_MS, "a jump down fades %u ms", GLIDE_TAIL_MS);
  CHECK(glide_for(20, 65535, false) == 20, "never longer than the interval");
  std::printf("  glide rule: small steps glide the interval, jumps up at once, jumps down %u ms\n", GLIDE_TAIL_MS);
}

static void test_glide_timer() {
  GlideTimer g;
  CHECK(g.step(1000) == 0 && !g.steady(), "first step jumps");
  CHECK(g.step(1050) == 50 && g.steady(), "second step: the interval");
  uint32_t v = 0;
  for (uint32_t t = 1100; t < 3000; t += 20)
    v = g.step(t);
  CHECK(v >= 19 && v <= 23, "follows a faster rate (%u)", v);
  CHECK(g.step(3000 + GLIDE_GAP_MS + 1) == 0 && !g.steady(), "a pause starts over");
  g.step(10000);
  CHECK(g.step(13000) == 0, "3 s apart: a new start");
  g.reset();
  g.step(20000);
  CHECK(g.step(21500) == GLIDE_MAX_MS, "glide at most %u ms", GLIDE_MAX_MS);
  // jittery steps (ESPHome's loop): the average stays near the mean interval
  std::mt19937 rng(3);
  std::uniform_int_distribution<int> jit(-8, 8);
  g.reset();
  uint32_t t = 50000;
  g.step(t);
  for (int i = 0; i < 200; i++) {
    t += 32 + jit(rng);
    v = g.step(t);
  }
  CHECK(v >= 26 && v <= 38, "jittered 32 ms steps -> %u ms", v);
  std::printf("  glide timer: first step jumps, follows the step rate (%u ms for 32 +-8 ms), pauses reset\n", v);
}

#ifdef WITH_ENGINE
/// A host-side effect: a new random brightness every 40 ms (+-10 ms, like ESPHome's loop). Through the scheduler and
/// the board engine, once as hard steps and once gliding over the measured interval. Gamma 1: output = M.
static double test_glide_on_board(bool glide) {
  Scheduler s;
  s.init(1, 77, 3);
  ultra_t u;
  ultra_init(&u, 5);
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  s.light_def(0, 0, MONO, 1000, 0, z, one);
  std::vector<uint8_t> f(s.frame_bytes());
  std::mt19937 rng(11);
  std::uniform_int_distribution<int> val(0, 65535), jit(-10, 10);
  const int64_t T0 = 1000000000LL;
  GlideTimer timer;
  double next_step = 1e6, next_frame = 0, wire_end = -1;
  uint16_t out[5], prev = 0;
  bool started = false;
  double max_jump = 0;
  for (double t = 0; t < 6e6; t += 250) {
    const int64_t now = T0 + static_cast<int64_t>(t);
    if (t >= next_step) {
      const uint16_t target = static_cast<uint16_t>(val(rng));
      const uint32_t g = glide ? timer.step(static_cast<uint32_t>(t / 1000)) : 0;
      s.light_set(0, 0, 0, target, one, one, g, now + 30000);
      next_step = t + (40 + jit(rng)) * 1000.0;
      next_frame = std::min(next_frame, t);
    }
    // protocol v2: the slots, then the header with the time of the frame's last bit; the board decodes the frame then
    if (wire_end >= 0 && t >= wire_end) {
      uint32_t hus;
      ultra_parse_header(f.data() + f.size() - ULTRA_HDR_BYTES, &hus);
      const uint32_t tk = static_cast<uint32_t>((wire_end + 25) / 50.0);
      ultra_on_header(&u, tk, hus);
      ultra_on_slot(&u, f.data(), tk);
      wire_end = -1;
    }
    if (wire_end < 0 && t >= next_frame) {
      const bool urgent = s.pending();
      wire_end = t + static_cast<double>(s.build_frame(now, f.data()) - now);
      next_frame = urgent ? wire_end + 300 : t + 200000.0;
    }
    if (static_cast<int64_t>(t) % 1000 != 0)
      continue;
    ultra_render(&u, static_cast<uint32_t>(t / 50.0), out);
    if (t > 2e6) {
      if (started)
        max_jump = std::max(max_jump, static_cast<double>(std::abs(out[4] - prev)));
      started = true;
    }
    prev = out[4];
  }
  return max_jump;
}

static void test_glide_engine() {
  const double hard_jump = test_glide_on_board(false);
  const double glide_jump = test_glide_on_board(true);
  std::printf("  host effect, 40 +-10 ms steps on the board: largest change per 1 ms %.0f counts hard, %.0f gliding\n",
              hard_jump, glide_jump);
  CHECK(hard_jump > 20000, "hard steps jump (%.0f)", hard_jump);
  CHECK(glide_jump < 3500, "glides move at most ~one step per interval per ms (%.0f)", glide_jump);
}

/// Quindor's downlights: five single-channel lights on board 0 (gamma 2.8) as five pixels of a strip, a chase (one
/// light on, moving every 150 ms) as an addressable scan makes it. On a line of `boards` boards, per glide mode
/// (0 off, 1 always = the whole interval, 2 auto): the least and most total light (sum of the five outputs after gamma,
/// in % of one light at full), and the frames a step needs.
struct ChaseResult {
  double min_total, max_total;
  int frames_per_step;
  double start_ms;
};

static ChaseResult chase_on_board(int boards, int mode, bool stream, bool lead) {
  Scheduler s;
  s.init(static_cast<uint16_t>(boards), 99, 3);
  ultra_t u;
  ultra_init(&u, 9);
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  uint8_t roles[5][5] = {};
  for (int i = 0; i < 5; i++) {
    roles[i][i] = ULTRA_ROLE_BRIGHTNESS;
    s.light_def(0, static_cast<uint8_t>(i), roles[i], 2800, 0, z, one);
  }
  std::vector<uint8_t> f(s.frame_bytes());
  // the strip's start: the hub's start delay, plus one frame per pixel on the busiest board, so all pixels of a step
  // are there before they start (a frame of the light definitions is lost here, the repeats cover it)
  const int64_t frame_us = s.build_frame(0, f.data()) + 500;
  const int64_t start_us = 30000 + (lead ? 5 * frame_us : 0);
  std::mt19937 rng(21);
  std::uniform_int_distribution<int> jit(-5, 5);
  const int64_t T0 = 3000000000LL;
  GlideTimer timer;
  uint16_t sent_m[5] = {0, 0, 0, 0, 0}, sent_a[5][5] = {};
  double next_step = 1e6, next_frame = 0, wire_end = -1;
  int step = 0, frames_this_step = 0, max_frames = 0;
  ChaseResult r{1e9, 0, 0, start_us / 1000.0};
  uint16_t out[5];
  for (double t = 0; t < 8e6; t += 250) {
    const int64_t now = T0 + static_cast<int64_t>(t);
    if (t >= next_step) {
      const uint32_t interval = timer.step(static_cast<uint32_t>(t / 1000));
      for (int i = 0; i < 5; i++) {
        const float v = (step % 5) == i ? 1.0f : 0.0f;
        uint16_t m, a[5], b[5];
        pixel_to_mab(roles[i], v, v, v, 0, 1.0f, 1, 1, m, a, b);
        if (same_output(sent_m[i], sent_a[i], m, a))
          continue;
        uint32_t g = 0;
        if (mode == 1)
          g = interval;
        else if (mode == 2)
          g = glide_for(interval, change_of(sent_m[i], sent_a[i], m, a),
                        value_of(m, a) >= value_of(sent_m[i], sent_a[i]));
        if (stream)
          s.light_stream(0, static_cast<uint8_t>(i), m, a, one, g, now + start_us);
        else
          s.light_set(0, static_cast<uint8_t>(i), 0, m, a, one, g, now + start_us);
        sent_m[i] = m;
        for (int k = 0; k < 5; k++)
          sent_a[i][k] = a[k];
      }
      if (step++ > 3)
        max_frames = std::max(max_frames, frames_this_step);
      frames_this_step = 0;
      next_step = t + (150 + jit(rng)) * 1000.0;
      next_frame = std::min(next_frame, t);
    }
    // the board decodes a frame when its last bit has arrived
    if (wire_end >= 0 && t >= wire_end) {
      uint32_t hus;
      ultra_parse_header(f.data() + f.size() - ULTRA_HDR_BYTES, &hus);
      const uint32_t tk = static_cast<uint32_t>((wire_end + 25) / 50.0);
      ultra_on_header(&u, tk, hus);
      ultra_on_slot(&u, f.data(), tk);
      wire_end = -1;
    }
    if (wire_end < 0 && t >= next_frame) {
      const bool urgent = s.pending();
      if (urgent)
        frames_this_step++;
      wire_end = t + static_cast<double>(s.build_frame(now, f.data()) - now);
      next_frame = urgent ? wire_end + 500 : t + 200000.0;  // reset + ESPHome loop gap
    }
    if (static_cast<int64_t>(t) % 1000 != 0)
      continue;
    ultra_render(&u, static_cast<uint32_t>(t / 50.0), out);
    if (t > 2.5e6) {
      double total = 0;
      for (int i = 0; i < 5; i++)
        total += out[i];
      r.min_total = std::min(r.min_total, total * 100.0 / 65535);
      r.max_total = std::max(r.max_total, total * 100.0 / 65535);
    }
  }
  r.frames_per_step = max_frames;
  return r;
}

static void test_downlight_chase(bool stream) {
  static const char *const MODE[] = {"off", "always", "auto"};
  std::printf("  %s:\n", stream ? "streamed (sent once)" : "commands (sent 3 times)");
  for (int boards : {1, 10, 50}) {
    for (int mode = 0; mode < 3; mode++) {
      for (int lead = 0; lead < 2; lead++) {
        const ChaseResult r = chase_on_board(boards, mode, stream, lead != 0);
        std::printf("  5 downlights on board 0 of %2d, chase 150 ms, effect_glide %-6s, start +%3.0f ms: total light "
                    "%3.0f..%3.0f %% of one light, %d frames per step\n",
                    boards, MODE[mode], r.start_ms, r.min_total, r.max_total, r.frames_per_step);
        if (mode == 1)
          CHECK(r.min_total < 50, "a full crossfade dips (%.0f %%)", r.min_total);
        else if (lead || boards <= 10)
          CHECK(r.min_total > 95, "%s: no dip with %d boards (%.0f %%)", MODE[mode], boards, r.min_total);
      }
    }
  }
}
#endif

// ------------------------------------------------------------------------------------------- scheduler (no engine)
/// Frames of a one-board scheduler until its queue is empty; the slots of board 0 in order.
static std::vector<std::vector<uint8_t>> drain(Scheduler &s, int64_t &t, int max_frames = 200) {
  std::vector<std::vector<uint8_t>> out;
  std::vector<uint8_t> f(s.frame_bytes());
  while (s.pending() && max_frames-- > 0) {
    s.build_frame(t, f.data());
    out.emplace_back(f.begin(), f.begin() + ULTRA_SLOT_BYTES);
    t += 1000;
  }
  return out;
}

static int32_t effect_start_ms(const std::vector<uint8_t> &slot) {
  return static_cast<int32_t>(ultra_get32(slot.data() + ULTRA_PAYLOAD + 1));
}

static void test_scheduler_stream() {
  const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535}, red[5] = {65535, 0, 0, 0, 0};
  int64_t t = 1000000000LL;
  {  // a streamed step goes out once, a command `repeats` times
    Scheduler s;
    s.init(1, 1, 3);
    s.light_stream(0, 0, 30000, red, one, 40, t + 30000);
    const size_t streamed = drain(s, t).size();
    s.light_set(0, 0, 0, 30000, red, one, 0, t + 30000);
    const size_t command = drain(s, t).size();
    CHECK(streamed == 1 && command == 3, "stream sent %zu x, command %zu x", streamed, command);
  }
  {  // it never starts before it arrives
    Scheduler s;
    s.init(1, 1, 3);
    s.light_stream(0, 0, 30000, red, one, 40, t - 50000);
    const auto slots = drain(s, t);
    const int16_t start = static_cast<int16_t>(ultra_get16(slots[0].data() + ULTRA_PAYLOAD + 25));
    CHECK(start >= 0, "a late streamed SET starts on arrival (offset %d ms)", start);
  }
  {  // waiting behind others: the glide stretches to the time since the light's last step
    Scheduler s;
    s.init(1, 1, 3);
    s.light_stream(0, 0, 1000, red, one, 10, t + 30000);
    drain(s, t);
    const int64_t first = t - 1000;  // when light 0's step went out
    for (uint8_t l = 1; l < 5; l++)
      s.light_stream(0, l, 1000, red, one, 10, t + 30000);
    s.light_stream(0, 0, 2000, red, one, 10, t + 30000);
    s.light_stream(0, 0, 3000, red, one, 10, t + 30000);  // replaces the one above before it went out
    const auto slots = drain(s, t);
    uint32_t glide0 = 0, glide1 = 0;
    for (const auto &sl : slots) {
      if ((sl[1] >> 4) == 0)
        glide0 = ultra_get24(sl.data() + ULTRA_PAYLOAD + 22);
      if ((sl[1] >> 4) == 1)
        glide1 = ultra_get24(sl.data() + ULTRA_PAYLOAD + 22);
    }
    const uint32_t since = static_cast<uint32_t>((t - 1000 - first) / 1000);
    CHECK(glide1 == 10 && glide0 >= since && glide0 >= 5, "glides: on time %u ms, behind %u ms (%u since)", glide1, glide0,
          since);
  }
  {  // replay keeps an effect's start; a transition gets the new one
    Scheduler s;
    s.init(1, 1, 3);
    uint8_t p[5];
    ultra_put24(p, 30000);
    ultra_put16(p + 3, 65535);
    const int64_t fx_start = t - 12345000;
    s.light_effect(0, 0, ULTRA_FX_COLOR_LOOP, p, 5, fx_start);
    drain(s, t);
    s.replay(t + 30000);
    std::vector<uint8_t> f(s.frame_bytes());
    int32_t got = 0;
    int64_t when = 0;
    while (s.pending()) {
      when = s.build_frame(t, f.data());
      if (f[0] == ULTRA_EFFECT)
        got = effect_start_ms(std::vector<uint8_t>(f.begin(), f.begin() + ULTRA_SLOT_BYTES));
      t += 1000;
      if (f[0] == ULTRA_EFFECT)
        break;
    }
    const double want = (fx_start - when) / 1000.0;
    CHECK(std::fabs(got - want) <= 1, "replay keeps the effect's phase (offset %d ms, want %.0f)", got, want);
  }
  {  // an effect running for 30 days: offsets stay in int32 and the phase is kept
    Scheduler s;
    s.init(1, 1, 3);
    uint8_t p[5];
    ultra_put24(p, 7000);
    ultra_put16(p + 3, 65535);
    const int64_t fx_start = t - 30LL * 86400 * 1000000 - 123000;
    s.light_effect(0, 0, ULTRA_FX_COLOR_LOOP, p, 5, fx_start);
    std::vector<uint8_t> f(s.frame_bytes());
    const int64_t when = s.build_frame(t, f.data());
    const int64_t off = effect_start_ms(std::vector<uint8_t>(f.begin(), f.begin() + ULTRA_SLOT_BYTES));
    const int64_t phase_want = ((when - fx_start) / 1000) % 7000, phase_got = (-off) % 7000;
    CHECK(off <= 0 && off > -86400LL * 1000 - 7000 && std::llabs(phase_want - phase_got) <= 1,
          "30-day-old effect: offset %lld ms, phase %lld vs %lld", static_cast<long long>(off),
          static_cast<long long>(phase_got), static_cast<long long>(phase_want));
  }
  {  // a strip shows on the light: its effect stops and is kept (also one set meanwhile), then runs again in phase
    Scheduler s;
    s.init(1, 1, 3);
    uint8_t p[5];
    ultra_put24(p, 30000);
    ultra_put16(p + 3, 65535);
    const int64_t fx_start = t - 5000000;
    s.light_effect(0, 2, ULTRA_FX_COLOR_LOOP, p, 5, fx_start);
    drain(s, t);
    s.hold_effect(0, 2, t + 30000);
    auto slots = drain(s, t);
    CHECK(!slots.empty() && slots[0][0] == ULTRA_EFFECT && slots[0][ULTRA_PAYLOAD] == ULTRA_FX_NONE,
          "held: the board's effect stops");
    ultra_put24(p, 20000);
    s.light_effect(0, 2, ULTRA_FX_COLOR_LOOP, p, 5, fx_start);  // changed while held: kept, not sent
    CHECK(!s.pending() && s.effect_held(0, 2), "an effect set while held is kept");
    s.release_effect(0, 2);
    std::vector<uint8_t> f(s.frame_bytes());
    const int64_t when = s.build_frame(t, f.data());
    const int32_t off = effect_start_ms(std::vector<uint8_t>(f.begin(), f.begin() + ULTRA_SLOT_BYTES));
    CHECK(f[0] == ULTRA_EFFECT && f[ULTRA_PAYLOAD] == ULTRA_FX_COLOR_LOOP && ultra_get24(&f[ULTRA_PAYLOAD + 5]) == 20000 &&
              std::fabs(off - (fx_start - when) / 1000.0) <= 1 && slots[0][2] != f[2],
          "released: the kept effect at its own start, new sequence number");
    drain(s, t);
  }
  {  // a strobe of 8 colours on all 5 lights of a board, started at once: nothing is dropped
    Scheduler s;
    s.init(1, 1, 3);
    std::vector<Step> steps(8);
    for (auto &st : steps)
      st = Step{100, 0, 65535, {65535, 0, 0, 0, 0}, {65535, 65535, 65535, 65535, 65535}};
    uint8_t p[1] = {8};
    for (uint8_t l = 0; l < 5; l++)
      s.light_effect(0, l, ULTRA_FX_STROBE, p, 1, t + 30000, &steps);
    CHECK(s.queued(0) == 45, "5 strobes x (EFFECT + 8 STEPs) queued: %zu of 45", s.queued(0));
  }
  {  // periodic effects on the host clock's phase
    const int64_t a = phase_start_us(5000123456LL, 7000), b = phase_start_us(5003456789LL, 7000);
    CHECK(a % 7000000 == 0 && b % 7000000 == 0 && a <= 5000123456LL && 5000123456LL - a < 7000000,
          "phase starts %lld %lld", static_cast<long long>(a), static_cast<long long>(b));
    uint8_t p[9] = {0};
    ultra_put24(p + 6, 500);
    CHECK(effect_period_ms(ULTRA_FX_PULSE, p, 13, nullptr) == 1000 && effect_period_ms(ULTRA_FX_CANDLE, p, 4, nullptr) == 0,
          "effect periods");
  }
  std::printf("  scheduler: streamed steps once, never late, glide stretched when behind; replay and 30-day effects "
              "keep their phase; effects held under a strip; 45 queued\n");
}

int main() {
  std::printf("ultra stream host test\n");
  test_pixel_map();
  test_glide_timer();
  test_glide_rule();
  test_scheduler_stream();
#ifdef WITH_ENGINE
  test_glide_engine();
  test_downlight_chase(false);
  test_downlight_chase(true);
#endif
  if (failures) {
    std::printf("%d FAILED\n", failures);
    return 1;
  }
  std::printf("all passed\n");
  return 0;
}
