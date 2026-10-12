#pragma once

// One downloaded board firmware image, shared by the update entities of several data lines (fw_update.cpp): an
// "update all" downloads and checks the image once, then each line sends it in turn. Looked up by the manifest's
// SHA-256, so only an identical image is reused. Main loop only (no locking). Plain C++: the firmware's bootloader
// host tests (test_sender.cpp) run it too.

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome::dig2analog_ultra {

class FwImageCache {
 public:
  using FreeFn = void (*)(uint8_t *img, size_t size);
  explicit FwImageCache(FreeFn free_fn) : free_fn_(free_fn) {}

  /// The cached image with this SHA-256 (hex, as in the manifest) and size: one more user. nullptr: not cached.
  uint8_t *acquire(const std::string &sha256, size_t size) {
    if (this->img_ == nullptr || size != this->size_ || !same_hex_(sha256, this->sha256_))
      return nullptr;
    this->users_++;
    return this->img_;
  }
  /// A freshly downloaded and checked image. Cached with one user (true), unless another image is still in use:
  /// then the caller keeps it to itself and frees it (false). An unused older image is freed first.
  bool put(const std::string &sha256, uint8_t *img, size_t size) {
    if (this->img_ != nullptr) {
      if (this->users_ != 0)
        return false;
      this->drop_();
    }
    this->sha256_ = sha256;
    this->img_ = img;
    this->size_ = size;
    this->users_ = 1;
    return true;
  }
  /// true if img is the cached image (the user releases it here instead of freeing it)
  bool holds(const uint8_t *img) const { return img != nullptr && img == this->img_; }
  /// A user is done with the cached image (its session ended or was aborted)
  void release(uint32_t now_ms) {
    if (this->users_ != 0 && --this->users_ == 0)
      this->idle_ms_ = now_ms;
  }
  /// Every loop: an unused image is freed hold_ms after its last user (0: at once). Lines whose install is queued
  /// right behind (an "update all") pick it up within that time.
  void service(uint32_t now_ms, uint32_t hold_ms) {
    if (this->img_ != nullptr && this->users_ == 0 && now_ms - this->idle_ms_ >= hold_ms)
      this->drop_();
  }
  /// Before another download: an unused image goes now, so the two never take heap together
  void trim() {
    if (this->img_ != nullptr && this->users_ == 0)
      this->drop_();
  }
  bool empty() const { return this->img_ == nullptr; }
  uint8_t users() const { return this->users_; }

 protected:
  static bool same_hex_(const std::string &a, const std::string &b) {
    if (a.size() != b.size() || a.empty())
      return false;
    for (size_t i = 0; i < a.size(); i++) {
      const char x = a[i] >= 'A' && a[i] <= 'F' ? static_cast<char>(a[i] - 'A' + 'a') : a[i];
      const char y = b[i] >= 'A' && b[i] <= 'F' ? static_cast<char>(b[i] - 'A' + 'a') : b[i];
      if (x != y)
        return false;
    }
    return true;
  }
  void drop_() {
    this->free_fn_(this->img_, this->size_);
    this->img_ = nullptr;
    this->size_ = 0;
    this->users_ = 0;
    this->sha256_.clear();
  }

  FreeFn free_fn_;
  std::string sha256_;
  uint8_t *img_{nullptr};
  size_t size_{0};
  uint8_t users_{0};
  uint32_t idle_ms_{0};
};

}  // namespace esphome::dig2analog_ultra
