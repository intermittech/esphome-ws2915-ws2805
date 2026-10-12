#pragma once

// Board firmware updates from GitHub (the firmware repo's research/firmware-update.md §4): the parts of the update
// entity that need no ESPHome - channels, manifest URLs, the choice of image, the downloaded image's checks and when
// an update is offered. Plain C++: the firmware's bootloader host tests (test_sender.cpp) run them too.
//
// A channel manifest (tools/release/make_manifest.py; signed as it is, fw_signature.h):
//   {"name": ..., "channel": "beta", "version": "0.6.0-beta.1", "release_url": ..., "summary": ...,
//    "images": [{"hw_id": 1, "hw_rev_mask": 6, "min_bootloader": 1, "size": 43288, "sha256": "...", "url": ...}]}

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "fw_signature.h"
#include "fw_version.h"
#include "ultra_update.h"

namespace esphome::dig2analog_ultra {

/// stable / beta / alpha (FW_CH_*); nullptr for anything else
inline const char *fw_channel_name(uint8_t ch) {
  return ch == FW_CH_STABLE ? "stable" : ch == FW_CH_BETA ? "beta" : ch == FW_CH_ALPHA ? "alpha" : nullptr;
}
inline uint8_t fw_channel_from_name(const std::string &name) {
  return name == "stable" ? FW_CH_STABLE : name == "beta" ? FW_CH_BETA : name == "alpha" ? FW_CH_ALPHA : 0;
}

/// The manifest URL of a channel. source: "OWNER/REPO" (GitHub: the repo's `channels` branch), or an http(s) URL in
/// which "{channel}" is replaced (a URL without it is one fixed manifest).
inline std::string fw_manifest_url(const std::string &source, uint8_t channel) {
  const char *name = fw_channel_name(channel);
  if (name == nullptr)
    return "";
  if (source.find("://") == std::string::npos)
    return "https://raw.githubusercontent.com/" + source + "/channels/" + name + ".json";
  std::string url = source;
  const size_t at = url.find("{channel}");
  if (at != std::string::npos)
    url.replace(at, 9, name);
  return url;
}

/// An image URL from the manifest: absolute as it is, else relative to the manifest's directory (or its host for
/// "/path").
inline std::string fw_resolve_url(const std::string &manifest_url, const std::string &url) {
  if (url.find("://") != std::string::npos || manifest_url.empty())
    return url;
  if (!url.empty() && url[0] == '/') {
    const size_t scheme = manifest_url.find("://");
    const size_t host_end = manifest_url.find('/', scheme == std::string::npos ? 0 : scheme + 3);
    return manifest_url.substr(0, host_end) + url;
  }
  return manifest_url.substr(0, manifest_url.rfind('/') + 1) + url;
}

struct FwImageInfo {
  uint8_t hw_id{0};
  uint16_t hw_rev_mask{0};
  uint8_t min_bootloader{0};
  uint32_t size{0};
  std::string sha256;  // 64 hex digits
  std::string url;
};

struct FwManifest {
  std::string name, channel, version, release_url, summary;
  uint32_t version_word{0};
  std::vector<FwImageInfo> images;
};

/// The image for this line's boards (hardware id, board revision = bit of hw_rev_mask); -1 if none
inline int fw_pick_image(const FwManifest &m, uint8_t hw_id, uint8_t hw_rev) {
  for (size_t i = 0; i < m.images.size(); i++) {
    if (m.images[i].hw_id == hw_id && hw_rev < 16 && (m.images[i].hw_rev_mask & (1u << hw_rev)) != 0)
      return static_cast<int>(i);
  }
  return -1;
}

/// Checks of a manifest after parsing: the channel it names, its version, the image entry. nullptr = fine.
inline const char *fw_check_manifest(FwManifest &m, uint8_t channel) {
  const char *name = fw_channel_name(channel);
  if (name == nullptr || m.channel != name)
    return "the manifest is for another channel";
  if (!parse_fw_version(m.version, m.version_word))
    return "the manifest's version is not x.y.z[-alpha|beta|rc.n]";
  return nullptr;
}

inline bool fw_hex_to_bytes(const std::string &hex, uint8_t *out, size_t n) {
  if (hex.size() != 2 * n)
    return false;
  for (size_t i = 0; i < 2 * n; i++) {
    const char c = hex[i];
    const int v = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
    if (v < 0)
      return false;
    out[i / 2] = static_cast<uint8_t>((i & 1) ? (out[i / 2] | v) : (v << 4));
  }
  return true;
}

/// The downloaded image (descriptor + body) against the manifest: size, SHA-256 (sha = its hash), and the
/// descriptor the boards will check too (magic, CRC, hardware id, revision, length, version). nullptr = fine.
inline const char *fw_check_image(const uint8_t *img, size_t size, const uint8_t *sha, const FwImageInfo &info,
                                  uint32_t version_word, uint8_t hw_rev) {
  uint8_t want[32];
  if (size != info.size)
    return "size differs from the manifest";
  if (!fw_hex_to_bytes(info.sha256, want, sizeof(want)) || memcmp(want, sha, sizeof(want)) != 0)
    return "SHA-256 differs from the manifest";
  if (size <= UPD_DESC_BYTES || size - UPD_DESC_BYTES > UPD_APP_BODY_MAX)
    return "size out of range";
  if (img[0] != 'Q' || img[1] != 'D' || img[2] != 'U' || img[3] != '5' ||
      upd_get32(img + UPD_D_CRC) != upd_crc32_words(0xFFFFFFFFu, img, 15u))
    return "no valid descriptor";
  if (img[UPD_D_HW_ID] != info.hw_id || hw_rev >= 16 || (upd_get16(img + UPD_D_HW_REV_MASK) & (1u << hw_rev)) == 0)
    return "the descriptor is for other hardware";
  if (upd_get32(img + UPD_D_BODY_LEN) != size - UPD_DESC_BYTES)
    return "the descriptor's body length differs";
  if (upd_get32(img + UPD_D_FW_VERSION) != version_word)
    return "the descriptor's version differs from the manifest";
  return nullptr;
}

/// Offer the channel's version? installed = what this line was last updated to (0 = not known: offer it).
inline bool fw_update_available(uint32_t installed, uint32_t latest) { return latest != 0 && (installed == 0 || latest > installed); }

/// PING frames after a preamble: a freshly installed app confirms itself at a whole PING frame, and a board sees whole
/// UPD frames only once the board before it forwards them (~4.5 frames per board in the firmware's wire-level test).
inline uint32_t fw_ping_burst_frames(uint16_t boards) { return 5u * boards + 4u; }
/// DARK frames after the burst: boards still transparent forward the next frames whole, so a board behind them would
/// take another board's slot; DARK slots are never valid.
static constexpr uint32_t FW_PING_DARK_TAIL = 3;

}  // namespace esphome::dig2analog_ultra
