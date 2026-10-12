#include "esphome/core/defines.h"   // USE_ESP32, USE_DIG2ANALOG_ULTRA_UPDATE (cg.add_define)
#include "fw_signature.h"

// Built only with the update platform (it defines USE_DIG2ANALOG_ULTRA_UPDATE): mbedTLS is in every ESP-IDF build
// for TLS already, so the check costs ~1-2 KB of code.
#if defined(USE_ESP32) && defined(USE_DIG2ANALOG_ULTRA_UPDATE)

#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>
#if defined(MBEDTLS_USE_PSA_CRYPTO)
#include <psa/crypto.h>
#endif

namespace esphome::dig2analog_ultra {

bool fw_verify_signature(const uint8_t *data, size_t len, const uint8_t *sig, size_t sig_len, uint8_t channel,
                         const FwPublicKey *keys, size_t nkeys, const char **key_name) {
  unsigned char hash[32];
#if defined(MBEDTLS_USE_PSA_CRYPTO)
  psa_crypto_init();
#endif
  if (mbedtls_sha256(data, len, hash, 0) != 0)
    return false;
  for (size_t i = 0; i < nkeys; i++) {
    if ((keys[i].channels & channel) == 0)
      continue;                       // e.g. the CI key never counts for the stable channel
    mbedtls_pk_context pk;
    mbedtls_pk_init(&pk);
    bool ok = mbedtls_pk_parse_public_key(&pk, keys[i].der, keys[i].der_len) == 0 &&
              mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, sizeof(hash), sig, sig_len) == 0;
    mbedtls_pk_free(&pk);
    if (ok) {
      if (key_name != nullptr)
        *key_name = keys[i].name;
      return true;
    }
  }
  return false;
}

bool fw_sha256(const uint8_t *data, size_t len, uint8_t out[32]) { return mbedtls_sha256(data, len, out, 0) == 0; }

}  // namespace esphome::dig2analog_ultra

#endif
