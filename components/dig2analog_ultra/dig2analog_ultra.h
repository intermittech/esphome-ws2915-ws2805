#pragma once

#ifdef USE_ESP32

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "ultra_core.h"

#ifdef USE_POWER_SUPPLY
#include "esphome/components/power_supply/power_supply.h"
#endif

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

#include <vector>

#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
#include "update_session.h"

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace esphome::dig2analog_ultra {

class UltraLight;
using EncoderParams = FrameSymbols<rmt_symbol_word_t>;

/// One data line with num_boards QuinLED dig2analog boards in Ultra mode. The boards run transitions, gamma and
/// effects themselves; this hub sends them commands (one 32-byte slot per board per frame, then a header with
/// the host time at the frame end that keeps all boards on one clock). Board settings are host-managed: the YAML
/// values are sent at boot and repeated, and the runtime setters below change them (in the boards' RAM, never their
/// flash; the boot protocol is the one exception, stored by the boards when it differs).
class UltraHub final : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // Configuration (codegen)
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_num_boards(uint16_t n) { this->num_boards_ = n; }
  void set_rmt_symbols(uint32_t n) { this->rmt_symbols_ = n; }
  void set_use_dma(bool use_dma) { this->use_dma_ = use_dma; }
  void set_timing(uint32_t bit0_high_ns, uint32_t bit0_low_ns, uint32_t bit1_high_ns, uint32_t bit1_low_ns,
                  uint32_t reset_ns) {
    this->bit0_high_ns_ = bit0_high_ns;
    this->bit0_low_ns_ = bit0_low_ns;
    this->bit1_high_ns_ = bit1_high_ns;
    this->bit1_low_ns_ = bit1_low_ns;
    this->reset_ns_ = reset_ns;
  }
  /// A frame at least this often (clock sync, state refresh), also when nothing changes.
  void set_refresh_interval(uint32_t ms) { this->refresh_interval_ms_ = ms; }
  /// Commands take effect this long after they are queued, so all boards and all copies start together.
  void set_start_delay(uint32_t ms) { this->start_delay_us_ = static_cast<int64_t>(ms) * 1000; }
  void set_repeats(uint8_t n) { this->repeats_ = n; }
  void set_power_on_delay(uint32_t ms) { this->power_on_delay_ms_ = ms; }
#ifdef USE_POWER_SUPPLY
  /// Supply of the boards (and LEDs): requested while any light is on; after it really switched on, everything
  /// is sent again after power_on_delay, so fades that ran while the boards had no power are replayed.
  void set_power_supply(power_supply::PowerSupply *psu, uint32_t keep_on_time_ms) {
    this->psu_ = psu;
    this->psu_req_.set_parent(psu);
    this->psu_keep_on_ms_ = keep_on_time_ms;
  }
#endif

  // Board settings (ULTRA_CFG_* ids), board -1 = all boards. Before setup they are the YAML defaults.
  void set_board_config(int board, uint8_t id, uint16_t value);
  uint16_t get_board_config(uint16_t board, uint8_t id) const { return this->sched_.get_config(board, id); }
  /// PWM frequency in Hz (2197..100000): a board preset when one is within 0.5 %, else the exact period.
  void set_pwm_frequency(int board, float hz);
  void set_dithering(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_DITHER, mode); }
  void set_dither_threshold(int board, float fraction) {
    this->set_board_config(board, ULTRA_CFG_DITHER_THRESH, clamp_u16_(fraction * 1000.0f + 0.5f, 1, 255));
  }
  void set_dim_floor(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_DIM_FLOOR, mode); }
  void set_min_pulse(int board, uint32_t ns) { this->set_board_config(board, ULTRA_CFG_MIN_PULSE, clamp_u16_(ns / 10.0f + 0.5f, 5, 100)); }
  /// 0 off, 1 fixed (thirds of a period), 2 fixed + spread-spectrum PWM, 3 adaptive (least LED_+ ripple for the
  /// current colour mix), 4 adaptive + spread spectrum
  void set_stagger(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_STAGGER, mode > 4 ? 4 : mode); }
  void set_signal_loss(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_LOSS, mode); }
  void set_signal_loss_timeout(int board, uint32_t ms) {
    this->set_board_config(board, ULTRA_CFG_LOSS_TIMEOUT, clamp_u16_(ms / 100.0f + 0.5f, 5, 600));
  }
  void set_feedback_channel(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_FEEDBACK, mode); }
  /// Protocol mode after power-up, stored on the board (written only when it differs): 0 auto, 1 WS2915, 2 WS2805,
  /// 3 Ultra. Auto / Ultra apply at once; WS2915 / WS2805 from the board's next power-up on (it then ignores Ultra).
  void set_boot_protocol(int board, uint8_t mode) {
    this->set_board_config(board, ULTRA_CFG_BOOT_PROTOCOL, mode > 3 ? 3 : mode);
  }
  /// Power limit: LED current of a channel at 100 % (A), limit per channel and per board (A, 0 = none).
  void set_channel_current(int board, uint8_t channel, float amps) {
    if (channel < CHANNELS)
      this->set_board_config(board, ULTRA_CFG_CH_CURRENT + channel, clamp_u16_(amps * 1000.0f + 0.5f, 0, 65535));
  }
  void set_max_channel_current(int board, uint8_t channel, float amps) {
    if (channel < CHANNELS)
      this->set_board_config(board, ULTRA_CFG_CH_MAX + channel, clamp_u16_(amps * 1000.0f + 0.5f, 0, 65535));
  }
  void set_max_board_current(int board, float amps) {
    this->set_board_config(board, ULTRA_CFG_BOARD_MAX, clamp_u16_(amps * 1000.0f + 0.5f, 0, 65535));
  }

  /// Blink the identify pattern on a board (on), or stop it.
  void identify(uint16_t board, bool on = true) {
    this->sched_.identify(board, on);
    this->wake_();
  }
  /// Boards with the input current sense, the current calibration of this pin (research/current-calibration.md):
  /// every board on the pin fades dark (0.3 s), then one board after the other in chain order measures each channel
  /// alone (duty_pct 0: at 25 / 50 / 75 / 100 %, ~1.1 s per board; 5..100: at that duty only, ~0.33 s), then the pin
  /// fades back (states, transitions and effects continue: the boards' engines kept running). boards: which ones to
  /// test (empty = all); the whole pin goes dark either way. Other pins keep running. Ignored (logged) while this
  /// pin's run is in progress; with a power_supply it is switched on for the run. Boards without current sense
  /// ignore the test (they just go dark and back).
  void calibrate_boards(const std::vector<uint16_t> &boards, uint8_t duty_pct = 0);
  /// Stop the run: the pin fades back at once (a test already running finishes on its board, <= 1 s).
  void cancel_calibration();
  bool calibration_running() const { return this->cal_pending_ || this->cal_seq_.running(); }
  /// progress: the board being tested, its position (1-based; 0 while the pin goes dark or nothing runs), the count
  uint16_t calibration_board() const { return this->cal_seq_.board(); }
  size_t calibration_step() const { return this->cal_seq_.running() && this->cal_started_ ? this->cal_seq_.index() + 1 : 0; }
  size_t calibration_count() const { return this->cal_seq_.count(); }
  /// One board alone, without the dark pin or any sequencing (lambdas; the action uses calibrate_boards).
  void calibrate_current(uint16_t board, uint8_t duty_pct = 0) {
    this->sched_.calibrate_current(board, duty_pct);
    this->wake_();
  }
  /// Automatic runs (YAML current_calibration): once after boot and / or every interval_ms (0 = never), optionally
  /// only while every light on this line is off. Off unless configured: calibration is a manual step.
  void set_calibration_schedule(bool on_boot, uint32_t interval_ms, bool only_when_off,
                                const std::vector<uint16_t> &boards) {
    this->cal_on_boot_ = on_boot;
    this->cal_interval_ms_ = interval_ms;
    this->cal_only_off_ = only_when_off;
    this->cal_boards_ = boards;
  }
  /// Send everything again now (e.g. after replacing a board); running transitions and effects restart.
  /// power_on: the boards' supply just switched on: once a full preamble reached them with power, a short one does
  /// (power_on_preamble_frames()).
  void resync(bool power_on = false);
  /// Boards forced to WS2805 / WS2915 (boot_protocol ws2805 / ws2915) ignore Ultra frames. This sends their own WS
  /// config frames "protocol = Ultra" in both formats (ultra_protocol.h ultra_switch_frame), then the start-up
  /// preamble and everything again, and stores boot_protocol (0 auto, 3 Ultra) on the boards. Only on request: every
  /// board that takes it blinks its acknowledge pattern, and boards of the second format flash for a few ms.
  void switch_boards_to_ultra(uint8_t boot_protocol);
  uint16_t get_num_boards() const { return this->num_boards_; }
  bool is_power_ready() const { return !this->power_waiting_; }

#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  // Board firmware update (fw_update.h). One session on this line, sent by its own task: the normal frames pause
  // meanwhile (light commands keep queueing) and come back with resync() a second after it.
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  /// image (descriptor + body) must stay valid until the session ends; false: busy, or the image is too small
  bool start_update(const uint8_t *image, size_t size, const UpdateSession::Options &opt);
  bool update_running() const { return this->upd_state_.load() != UPD_IDLE || this->upd_quiet_; }
  /// 0..1 of the running session, and its pass (0-based)
  float update_progress() const;
  uint8_t update_pass() const { return this->upd_pass_.load(); }
  /// ABORT frames, then the session ends (boards that erased stay in their bootloader, waiting for a session)
  void abort_update() { this->upd_abort_.store(true); }
  /// the result of the last session once its follow-up resync started (UPD_NONE until then); read once
  UpdateResult take_update_result();
  /// One UPD frame on the wire incl. its reset (us)
  uint32_t update_frame_us() const;
  /// PING bursts after the start-up preamble and every resync: freshly installed boards confirm themselves
  void set_ping_burst(bool on) { this->ping_burst_ = on; }
  /// Keep the power supply requested (update sessions)
  void hold_power(bool on) { this->upd_power_hold_ = on; }
  bool has_power_supply() const;
  /// recover: the supply off until it reports off (other requesters may keep it on), 1.5 s, then on again
  enum PowerCycle : uint8_t { CYCLE_NONE = 0, CYCLE_WAIT_OFF, CYCLE_OFF, CYCLE_DONE, CYCLE_FAILED };
  void power_cycle();
  PowerCycle power_cycle_state() const { return this->cycle_; }
#endif

  // For UltraLight / effects
  int64_t start_time_us() const { return this->now_us_() + this->start_delay_us_; }
  uint32_t start_delay_ms() const { return static_cast<uint32_t>(this->start_delay_us_ / 1000); }
  /// One frame on the wire, worst case (us; 0 before setup)
  uint32_t get_frame_us() const { return this->frame_us_; }
  Scheduler &scheduler() { return this->sched_; }
  void wake_();
  void add_light(UltraLight *light) { this->lights_.push_back(light); }

 protected:
  static uint16_t clamp_u16_(float v, uint16_t lo, uint16_t hi) {
    return v < lo ? lo : v > hi ? hi : static_cast<uint16_t>(v);
  }
  static int64_t now_us_();
  void transmit_();
  void service_power_();
  void service_calibration_();
  bool any_lit_(uint32_t now) const;

  uint8_t pin_{0};
  bool inverted_{false};
  bool use_dma_{false};
  uint16_t num_boards_{1};
  uint32_t rmt_symbols_{64};
  uint32_t bit0_high_ns_{250};
  uint32_t bit0_low_ns_{1050};
  uint32_t bit1_high_ns_{700};
  uint32_t bit1_low_ns_{650};
  uint32_t reset_ns_{300000};
  uint32_t refresh_interval_ms_{200};
  int64_t start_delay_us_{30000};
  uint8_t repeats_{3};

  Scheduler sched_;
  struct PendingConfig {
    int board;
    uint8_t id;
    uint16_t value;
  };
  std::vector<PendingConfig> early_config_;  // set before setup (codegen), applied in setup()
  bool started_{false};

  uint32_t last_frame_ms_{0};
  uint32_t last_busy_ms_{0};
  uint32_t frame_us_{0};    // one frame on the wire, worst case
  uint16_t preamble_{0};    // DARK frames still to send before the commands (the boards lock to Ultra)
  bool preamble_full_{true};   // the preamble running is the full 4N + 2
  bool synced_once_{false};    // a full preamble reached the boards with their supply on (they all stored Ultra)
  bool boards_powered_() const;
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};  // next WS config frame of switch_boards_to_ultra (none pending)
  uint32_t switch_frame_us_{0};                   // its longest frame on the wire
  bool has_sent_{false};
  HighFrequencyLoopRequester high_freq_;
  bool high_freq_active_{false};
  uint32_t frames_{0};

  // current calibration: the run on this pin, its request (waiting for the boards' power), automatic runs
  CalSequencer cal_seq_;
  bool cal_pending_{false};
  bool cal_started_{false};             // the first board's test was sent (progress)
  std::vector<uint16_t> cal_req_boards_;
  uint8_t cal_req_duty_{0};
  uint32_t cal_wait_ms_{0};             // the request waits for the supply since
  bool cal_power_hold_{false};          // the supply stays requested for the run
  bool cal_on_boot_{false};
  bool cal_boot_done_{false};
  uint32_t cal_interval_ms_{0};
  uint32_t cal_last_ms_{0};
  bool cal_only_off_{true};
  std::vector<uint16_t> cal_boards_;

  uint32_t power_on_delay_ms_{0};
  std::vector<UltraLight *> lights_;
  bool power_waiting_{false};
  uint32_t power_on_ms_{0};
#ifdef USE_POWER_SUPPLY
  power_supply::PowerSupply *psu_{nullptr};
  power_supply::PowerSupplyRequester psu_req_;
  uint32_t psu_keep_on_ms_{0};
  bool psu_requested_{false};
  bool psu_released_{false};
  uint32_t psu_released_ms_{0};
#endif

  uint8_t *tx_buf_{nullptr};
  EncoderParams params_{};
  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};

#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  enum UpdateState : uint8_t { UPD_IDLE = 0, UPD_RUNNING, UPD_FINISHED };
  static void update_task_(void *arg);
  void arm_ping_burst_();
  UpdateSession upd_session_;
  TaskHandle_t upd_task_{nullptr};
  std::atomic<uint8_t> upd_state_{UPD_IDLE};
  std::atomic<uint8_t> upd_result_{UPD_NONE};
  std::atomic<bool> upd_abort_{false};
  std::atomic<uint32_t> upd_sent_{0};
  std::atomic<uint8_t> upd_pass_{0};
  uint32_t upd_total_{0};
  bool upd_quiet_{false};         // after a session: the line stays low while the boards commit and restart
  uint32_t upd_quiet_ms_{0};
  UpdateResult upd_done_{UPD_NONE};
  bool ping_burst_{false};
  uint16_t ping_left_{0};         // PING frames still to send after the preamble
  uint8_t dark_tail_{0};          // DARK frames after them
  bool upd_power_hold_{false};
  PowerCycle cycle_{CYCLE_NONE};
  uint32_t cycle_ms_{0};
#endif
};

template<typename... Ts> class CalibrateCurrentAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  void set_boards(const std::vector<uint16_t> &boards) { this->boards_ = boards; }
  void set_level(uint8_t duty_pct) { this->duty_pct_ = duty_pct; }

  void play(const Ts &...x) override { this->parent_->calibrate_boards(this->boards_, this->duty_pct_); }

 protected:
  std::vector<uint16_t> boards_;
  uint8_t duty_pct_{0};
};

template<typename... Ts> class CancelCalibrationAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  void play(const Ts &...x) override { this->parent_->cancel_calibration(); }
};

template<typename... Ts> class SwitchBoardsToUltraAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  TEMPLATABLE_VALUE(uint8_t, boot_protocol)

  void play(const Ts &...x) override { this->parent_->switch_boards_to_ultra(this->boot_protocol_.value(x...)); }
};

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
