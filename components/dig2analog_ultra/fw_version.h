#pragma once

// Board firmware versions: "1.2.3" or "1.2.3-alpha.N" / "-beta.N" / "-rc.N" <-> the descriptor's version word
// (major << 24 | minor << 16 | patch << 8 | pre-release byte; ultra_update.h). As a number it sorts like semver:
// 0.6.0-alpha.3 < 0.6.0-beta.1 < 0.6.0-rc.1 < 0.6.0. Same rules as the firmware's tools/release/qdu.py.

#include <cstdint>
#include <cstdio>
#include <string>

#include "ultra_update.h"

namespace esphome::dig2analog_ultra {

inline bool parse_fw_version(const std::string &text, uint32_t &out) {
  const char *s = text.c_str();
  if (*s == 'v')
    s++;
  uint32_t part[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    if (*s < '0' || *s > '9')
      return false;
    uint32_t v = 0;
    while (*s >= '0' && *s <= '9') {
      v = v * 10 + (uint32_t) (*s++ - '0');
      if (v > 255)
        return false;
    }
    part[i] = v;
    if (i < 2 && *s++ != '.')
      return false;
  }
  uint32_t pre = UPD_PRE_STABLE;
  if (*s == '-') {
    s++;
    uint32_t base;
    std::string stage;
    while (*s >= 'a' && *s <= 'z')
      stage += *s++;
    if (stage == "alpha")
      base = UPD_PRE_ALPHA;
    else if (stage == "beta")
      base = UPD_PRE_BETA;
    else if (stage == "rc")
      base = UPD_PRE_RC;
    else
      return false;
    if (*s++ != '.' || *s < '0' || *s > '9')
      return false;
    uint32_t n = 0;
    while (*s >= '0' && *s <= '9') {
      n = n * 10 + (uint32_t) (*s++ - '0');
      if (n > 63)
        return false;
    }
    pre = base + n;
  }
  if (*s != '\0')
    return false;
  out = (part[0] << 24) | (part[1] << 16) | (part[2] << 8) | pre;
  return true;
}

inline std::string format_fw_version(uint32_t w) {
  char buf[32];
  const uint32_t pre = w & 0xFF;
  int n = snprintf(buf, sizeof(buf), "%u.%u.%u", (unsigned) (w >> 24), (unsigned) ((w >> 16) & 0xFF),
                   (unsigned) ((w >> 8) & 0xFF));
  if (pre != UPD_PRE_STABLE) {
    const char *stage = pre >= UPD_PRE_RC ? "rc" : pre >= UPD_PRE_BETA ? "beta" : "alpha";
    const uint32_t base = pre >= UPD_PRE_RC ? UPD_PRE_RC : pre >= UPD_PRE_BETA ? UPD_PRE_BETA : UPD_PRE_ALPHA;
    snprintf(buf + n, sizeof(buf) - n, "-%s.%u", stage, (unsigned) (pre - base));
  }
  return buf;
}

}  // namespace esphome::dig2analog_ultra
