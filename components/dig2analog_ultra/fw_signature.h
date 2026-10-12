#pragma once

// Signature check of a board firmware channel manifest (ECDSA P-256, SHA-256, DER signature; the firmware repo's
// tools/release/sign_release.py and KEY_MANAGEMENT.md). Only the ESP32 checks signatures; the boards trust their
// ESP32 and check CRC, hardware id and version themselves.

#include <cstddef>
#include <cstdint>

namespace esphome::dig2analog_ultra {

enum FwChannel : uint8_t { FW_CH_STABLE = 0x01, FW_CH_BETA = 0x02, FW_CH_ALPHA = 0x04 };

struct FwPublicKey {
  const char *name;
  const uint8_t *der;     // SubjectPublicKeyInfo (sign_release.py pubkey)
  size_t der_len;
  uint8_t channels;       // FW_CH_* this key may sign: the offline release key all three, the CI key beta + alpha
};

/// true if `sig` over `data` verifies with one of `keys` that may sign `channel` (one FW_CH_* bit)
bool fw_verify_signature(const uint8_t *data, size_t len, const uint8_t *sig, size_t sig_len, uint8_t channel,
                         const FwPublicKey *keys, size_t nkeys, const char **key_name = nullptr);
/// SHA-256 of a downloaded image (the manifest holds it, so the signature covers the image)
bool fw_sha256(const uint8_t *data, size_t len, uint8_t out[32]);

}  // namespace esphome::dig2analog_ultra
