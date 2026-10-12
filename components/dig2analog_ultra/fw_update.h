#pragma once

// Board firmware update entity (`update: - platform: dig2analog_ultra`, update.py): one per hub, i.e. per data line.
// It reads the channel's signed manifest from GitHub (fw_manifest.h, fw_signature.h), and on Install downloads the
// image, checks it, and sends it to every board of the line in one session (the hub's update task,
// update_session.h). The boards decide themselves: newer -> install, same -> skip, older -> reject (unless forced),
// and blink the result; the ESP32 can't read their versions, so "installed" is what this line was last updated to.
// Design: the firmware repo's research/firmware-update.md §4.

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_DIG2ANALOG_ULTRA_UPDATE)

#include "esphome/components/http_request/http_request.h"
#include "esphome/components/update/update_entity.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "dig2analog_ultra.h"
#include "fw_image_cache.h"
#include "fw_manifest.h"

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::dig2analog_ultra {

class UltraFirmwareUpdate final : public update::UpdateEntity, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;  // look for a new release (never installs by itself)
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  /// Install the channel's release on every board of the line. force: also the same or an older version (the
  /// boards would skip / reject it otherwise), and run even when "installed" is already that version.
  void perform(bool force) override;
  void check() override { this->update(); }

  // Configuration (codegen)
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_http(http_request::HttpRequestComponent *http) { this->http_ = http; }
  void set_source(const std::string &source) { this->source_ = source; }
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_keys(const FwPublicKey *keys, size_t n) {
    this->keys_ = keys;
    this->nkeys_ = n;
  }
  void set_verify_signature(bool verify) { this->verify_ = verify; }
  void set_hardware(uint8_t hw_id, uint8_t hw_rev) {
    this->hw_id_ = hw_id;
    this->hw_rev_ = hw_rev;
  }
  void set_passes(uint8_t passes) { this->passes_ = passes; }
#ifdef USE_TEXT_SENSOR
  void set_status_sensor(text_sensor::TextSensor *s) { this->status_sensor_ = s; }
#endif

  /// The channel select (stored there): switch, then look again (check = false at boot)
  void change_channel(uint8_t channel, bool check);
  uint8_t get_channel() const { return this->channel_; }
  /// Stop a download or a session (ABORT frames: boards that started erasing wait in their bootloader)
  void abort();
  /// Like a forced install, with a long lead-in of ANNOUNCE frames: boards whose app no longer listens (e.g. a broken
  /// release) are reached while they boot. With a power supply on the hub and power_cycle, the hub switches it off and
  /// on during the lead-in; otherwise power-cycle the boards by hand then (the status says so).
  void recover(uint32_t lead_in_ms, bool power_cycle);

 protected:
  enum Phase : uint8_t { IDLE, QUEUED, DOWNLOADING, WAIT_POWER, SENDING };
  struct CheckResult;
  struct DownloadResult;
  static void check_task_(void *arg);
  static void download_task_(void *arg);
  bool fetch_(const std::string &url, std::vector<uint8_t> &out, size_t max_len);
  void on_checked_(CheckResult *r);
  void on_downloaded_(DownloadResult *r);
  bool take_shared_image_();
  void image_ready_();
  void start_install_();
  void start_session_();
  void finish_(const std::string &status, bool installed);
  void free_image_();
  void set_status_(const std::string &text);
  void publish_progress_(float percent, bool force_publish);
  std::string installed_text_() const;

  UltraHub *hub_{nullptr};
  http_request::HttpRequestComponent *http_{nullptr};
  std::string source_;
  uint8_t channel_{FW_CH_STABLE};
  const FwPublicKey *keys_{nullptr};
  size_t nkeys_{0};
  bool verify_{true};
  uint8_t hw_id_{UPD_HW_ID_D2A_ULTRA_5CH};
  uint8_t hw_rev_{UPD_REV_PB11};
  uint8_t passes_{3};
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *status_sensor_{nullptr};
#endif

  ESPPreferenceObject pref_;
  uint32_t installed_{0};  // version word this line was last updated to, 0 = not known

  // the last good check (main loop only)
  bool have_release_{false};
  uint32_t latest_{0};
  FwImageInfo image_;
  std::string image_url_;

  TaskHandle_t check_task_handle_{nullptr};
  uint8_t check_channel_{FW_CH_STABLE};
  bool initial_check_done_{false};

  Phase phase_{IDLE};
  bool force_{false};
  bool recover_{false};
  bool power_cycle_{false};
  uint32_t lead_in_ms_{500};
  uint32_t phase_ms_{0};
  uint32_t last_publish_ms_{0};
  std::atomic<uint32_t> dl_bytes_{0};
  std::atomic<bool> dl_abort_{false};
  uint8_t *img_{nullptr};
  size_t img_size_{0};
  uint32_t job_version_{0};  // the version of the image being installed
  std::string status_text_;

  /// one download + session at a time on this ESP32 (each image takes up to 54 KB of heap); lines that install the
  /// same image share one download (fw_image_cache.h)
  static UltraFirmwareUpdate *active_;
  static uint8_t entities_;
};

#ifdef USE_SELECT
/// The firmware channel of an update entity (stable / beta / alpha), stored on the ESP32
class FwChannelSelect final : public select::Select, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::DATA; }  // before the entity's first check
  void set_update(UltraFirmwareUpdate *u) { this->update_ = u; }

 protected:
  void control(size_t index) override;
  UltraFirmwareUpdate *update_{nullptr};
  ESPPreferenceObject pref_;
};
#endif

template<typename... Ts> class AbortUpdateAction final : public Action<Ts...>, public Parented<UltraFirmwareUpdate> {
 public:
  void play(const Ts &...x) override { this->parent_->abort(); }
};

template<typename... Ts> class RecoverAction final : public Action<Ts...>, public Parented<UltraFirmwareUpdate> {
 public:
  TEMPLATABLE_VALUE(uint32_t, lead_in)
  TEMPLATABLE_VALUE(bool, power_cycle)
  void play(const Ts &...x) override {
    this->parent_->recover(this->lead_in_.value(x...), this->power_cycle_.value(x...));
  }
};

}  // namespace esphome::dig2analog_ultra

#endif
