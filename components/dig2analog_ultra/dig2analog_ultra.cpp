#include "dig2analog_ultra.h"
#include "ultra_light.h"
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
#include "fw_manifest.h"
#endif

#ifdef USE_ESP32

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <esp_attr.h>
#include <esp_clk_tree.h>
#include <esp_random.h>
#include <esp_timer.h>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra";

static constexpr uint32_t RMT_DURATION_MAX = 0x7FFF;  // 15-bit duration field
// Keep the main loop in high-frequency mode this long after the last command, so the copies and
// commands that follow (effect steps, a scene of several lights) go out back to back.
static constexpr uint32_t BUSY_HOLD_MS = 250;

static uint32_t rmt_resolution_hz() {
  uint32_t freq = 0;
  esp_clk_tree_src_get_freq_hz((soc_module_clk_t) RMT_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &freq);
  return freq;
}

static uint32_t ns_to_ticks(uint32_t ns, uint32_t res_hz) {
  return static_cast<uint32_t>((static_cast<uint64_t>(ns) * res_hz + 500000000ULL) / 1000000000ULL);
}

static rmt_symbol_word_t make_symbol(uint32_t ticks0, uint32_t level0, uint32_t ticks1, uint32_t level1) {
  rmt_symbol_word_t s{};
  s.duration0 = ticks0;
  s.level0 = level0;
  s.duration1 = ticks1;
  s.level1 = level1;
  return s;
}

static size_t IRAM_ATTR HOT encoder_callback(const void *data, size_t size, size_t symbols_written,
                                             size_t symbols_free, rmt_symbol_word_t *symbols, bool *done,
                                             void *arg) {
  return encode_symbols(*static_cast<const EncoderParams *>(arg), static_cast<const uint8_t *>(data), size,
                        symbols_written, symbols_free, symbols, done);
}

int64_t UltraHub::now_us_() { return esp_timer_get_time(); }

void UltraHub::set_board_config(int board, uint8_t id, uint16_t value) {
  if (!this->started_) {
    this->early_config_.push_back({board, id, value});
    return;
  }
  for (uint16_t b = 0; b < this->num_boards_; b++) {
    if (board < 0 || board == b)
      this->sched_.set_config(b, id, value);
  }
  this->sched_.flush_config();
  this->wake_();
}

void UltraHub::set_pwm_frequency(int board, float hz) {
  uint16_t preset, period;
  pwm_for_frequency(hz, preset, period);
  if (!this->started_) {
    this->early_config_.push_back({board, ULTRA_CFG_PWM_PRESET, preset});
    this->early_config_.push_back({board, ULTRA_CFG_PWM_PERIOD, period});
    return;
  }
  for (uint16_t b = 0; b < this->num_boards_; b++) {
    if (board < 0 || board == b) {
      this->sched_.set_config(b, ULTRA_CFG_PWM_PRESET, preset);
      this->sched_.set_config(b, ULTRA_CFG_PWM_PERIOD, period);
    }
  }
  this->sched_.flush_config();
  this->wake_();
}

// A board forwards Ultra frames only once it is locked: a preamble of DARK frames (preamble_frames()) lets the
// whole chain lock (boards that were locked to a WS protocol show dark meanwhile) before the commands come.
void UltraHub::resync() {
  this->preamble_ = preamble_frames(this->num_boards_);
  int64_t lead_us = static_cast<int64_t>(this->preamble_) * (this->frame_us_ + 200);
  if (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES)
    lead_us += static_cast<int64_t>(ULTRA_SWITCH_WS_FRAMES - this->switch_frame_) * (this->switch_frame_us_ + 200);
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  this->arm_ping_burst_();
  lead_us += static_cast<int64_t>(this->ping_left_) * (this->update_frame_us() + 200) +
             static_cast<int64_t>(this->dark_tail_) * (this->frame_us_ + 200);
#endif
  this->sched_.replay(this->now_us_() + lead_us + this->start_delay_us_);
  this->wake_();
}

#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
// After the DARK preamble (the boards have seen normal Ultra frames): PING frames, then a few DARK ones (fw_manifest.h).
void UltraHub::arm_ping_burst_() {
  if (!this->ping_burst_)
    return;
  this->ping_left_ = static_cast<uint16_t>(fw_ping_burst_frames(this->num_boards_));
  this->dark_tail_ = static_cast<uint8_t>(FW_PING_DARK_TAIL);
}

uint32_t UltraHub::update_frame_us() const {
  const uint32_t worst = std::max(this->bit0_high_ns_ + this->bit0_low_ns_, this->bit1_high_ns_ + this->bit1_low_ns_);
  return (UPD_FRAME_BITS * worst) / 1000 + this->reset_ns_ / 1000;
}

bool UltraHub::start_update(const uint8_t *image, size_t size, const UpdateSession::Options &opt) {
  if (!this->started_ || this->is_failed() || this->update_running())
    return false;
  UpdateSession::Options o = opt;
  o.boards = this->num_boards_;
  // the task's own pacing adds ~50 us per frame to the wire time
  if (!this->upd_session_.begin(image, static_cast<uint32_t>(size), static_cast<uint8_t>(esp_random() | 1u), o,
                                this->update_frame_us() + 50))
    return false;
  this->upd_total_ = this->upd_session_.total_frames();
  this->upd_sent_.store(0);
  this->upd_pass_.store(0);
  this->upd_abort_.store(false);
  this->upd_result_.store(UPD_NONE);
  this->upd_state_.store(UPD_RUNNING);
  if (xTaskCreate(UltraHub::update_task_, "d2a_update", 4096, this, 5, &this->upd_task_) != pdPASS) {
    this->upd_state_.store(UPD_IDLE);
    ESP_LOGE(TAG, "Cannot start the update task");
    return false;
  }
  ESP_LOGI(TAG, "Firmware update session: %" PRIu32 " frames to %u boards", this->upd_total_, this->num_boards_);
  return true;
}

float UltraHub::update_progress() const {
  return this->upd_total_ ? static_cast<float>(this->upd_sent_.load()) / static_cast<float>(this->upd_total_) : 0.0f;
}

UltraHub::UpdateResult UltraHub::take_update_result() {
  const UpdateResult r = this->upd_done_;
  this->upd_done_ = UPD_NONE;
  return r;
}

bool UltraHub::has_power_supply() const {
#ifdef USE_POWER_SUPPLY
  return this->psu_ != nullptr;
#else
  return false;
#endif
}

void UltraHub::power_cycle() {
  if (!this->has_power_supply())
    return;
  this->cycle_ = CYCLE_WAIT_OFF;
  this->cycle_ms_ = millis();
}

// The session owns the RMT channel (the main loop sends nothing meanwhile). Frame by frame: wait until the last one
// (and its reset) is on the wire, the page pause if it asked for one, then the next.
void UltraHub::update_task_(void *arg) {
  auto *hub = static_cast<UltraHub *>(arg);
  rmt_transmit_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  uint8_t result = UPD_DONE;
  uint32_t pause_us = 0;
  const uint32_t tick_us = portTICK_PERIOD_MS * 1000;
  for (;;) {
    if (rmt_tx_wait_all_done(hub->channel_, 200) != ESP_OK) {
      result = UPD_FAILED;
      break;
    }
    if (pause_us != 0)
      vTaskDelay((pause_us + tick_us - 1) / tick_us + 1);   // >= pause_us (the boards program a page)
    if (hub->upd_abort_.load()) {
      result = UPD_ABORTED;
      break;
    }
    if (!hub->upd_session_.next(hub->tx_buf_, static_cast<uint32_t>(now_us_()), &pause_us))
      break;
    if (rmt_transmit(hub->channel_, hub->encoder_, hub->tx_buf_, UPD_FRAME_BYTES, &cfg) != ESP_OK) {
      result = UPD_FAILED;
      break;
    }
    hub->upd_sent_.store(hub->upd_session_.frames_sent());
    hub->upd_pass_.store(hub->upd_session_.pass());
  }
  if (result == UPD_ABORTED) {
    for (int i = 0; i < 12 && rmt_tx_wait_all_done(hub->channel_, 200) == ESP_OK; i++) {
      hub->upd_session_.abort_frame(hub->tx_buf_, static_cast<uint32_t>(now_us_()));
      if (rmt_transmit(hub->channel_, hub->encoder_, hub->tx_buf_, UPD_FRAME_BYTES, &cfg) != ESP_OK)
        break;
    }
  }
  rmt_tx_wait_all_done(hub->channel_, 200);
  hub->upd_result_.store(result);
  hub->upd_state_.store(UPD_FINISHED);
  vTaskDelete(nullptr);
}
#endif

// WS config frames first (boards forced to WS take only those), then the preamble and the replay (resync), with the
// boot protocol queued as CONFIG: the boards run Ultra by then. Boards already in Ultra mode ignore the WS frames
// (their lengths never make a board count 3 frames of one WS protocol, see ultra_switch_frame).
void UltraHub::switch_boards_to_ultra(uint8_t boot_protocol) {
  if (!this->started_ || this->is_failed())
    return;
  const uint8_t p = boot_protocol == 3 ? 3 : 0;
  ESP_LOGI(TAG, "Switching boards to Ultra: WS config frames, then the start-up preamble (boot protocol %s)",
           p == 3 ? "ultra" : "auto");
  this->switch_frame_ = 0;
  this->set_board_config(-1, ULTRA_CFG_BOOT_PROTOCOL, p);
  this->resync();
}

// Something to send: frames back to back until the queues are empty.
void UltraHub::wake_() {
  this->last_busy_ms_ = millis();
  if (!this->high_freq_active_) {
    this->high_freq_.start();
    this->high_freq_active_ = true;
  }
  this->enable_loop();
}

// Request the supply while any light is on (or still fading out). After it really switched on, the boards just
// booted: wait power_on_delay, then send everything again with transitions and effects restarted.
static constexpr uint32_t CAL_BOOT_DELAY_MS = 10000;
static constexpr uint32_t CAL_POWER_WAIT_MS = 30000;

void UltraHub::calibrate_boards(const std::vector<uint16_t> &boards, uint8_t duty_pct) {
  if (this->calibration_running()) {
    ESP_LOGW(TAG, "Current calibration on GPIO%u: already running, request ignored", this->pin_);
    return;
  }
  this->cal_req_boards_ = boards;
  this->cal_req_duty_ = duty_pct;
  this->cal_pending_ = true;  // starts in service_calibration_() once the boards have power and Ultra
  this->cal_started_ = false;
  this->cal_wait_ms_ = millis();
}

void UltraHub::cancel_calibration() {
  if (this->cal_pending_) {
    this->cal_pending_ = false;
    ESP_LOGI(TAG, "Current calibration on GPIO%u: cancelled before it started", this->pin_);
  }
  if (this->cal_seq_.running()) {
    this->cal_seq_.cancel(this->sched_);
    this->wake_();
    ESP_LOGI(TAG, "Current calibration on GPIO%u: cancelled after %u of %u board(s), the pin fades back", this->pin_,
             static_cast<unsigned>(this->cal_started_ ? this->cal_seq_.index() : 0),
             static_cast<unsigned>(this->cal_seq_.count()));
  }
}

bool UltraHub::any_lit_(uint32_t now) const {
  for (auto *l : this->lights_) {
    if (l->is_lit(now))
      return true;
  }
  return false;
}

void UltraHub::service_calibration_() {
  const uint32_t now = millis();
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  if (this->update_running()) {
    if (this->calibration_running())
      this->cancel_calibration();  // the session owns the line; the release goes out after it
    this->cal_power_hold_ = false;
    return;
  }
#endif
  // automatic runs (off unless configured)
  const bool boot_due = this->cal_on_boot_ && !this->cal_boot_done_ && now >= CAL_BOOT_DELAY_MS;
  const bool interval_due = this->cal_interval_ms_ != 0 && now - this->cal_last_ms_ >= this->cal_interval_ms_;
  if ((boot_due || interval_due) && !this->calibration_running() && !(this->cal_only_off_ && this->any_lit_(now))) {
    this->cal_last_ms_ = now;
    this->cal_boot_done_ = true;
    this->calibrate_boards(this->cal_boards_, 0);
  }
  // a request waits until the boards have power and took the preamble
  if (this->cal_pending_) {
    bool ready = this->started_ && this->preamble_ == 0;
#ifdef USE_POWER_SUPPLY
    if (this->psu_ != nullptr)
      ready &= this->psu_->is_enabled() && !this->power_waiting_;
#endif
    if (ready) {
      this->cal_pending_ = false;
      if (this->cal_seq_.start(this->sched_, this->num_boards_, this->cal_req_boards_, this->cal_req_duty_, now)) {
        const size_t n = this->cal_seq_.count();
        ESP_LOGI(TAG,
                 "Current calibration on GPIO%u: %u of %u board(s), %s, ~%u s; the pin goes dark, other pins run on",
                 this->pin_, static_cast<unsigned>(n), static_cast<unsigned>(this->num_boards_),
                 this->cal_req_duty_ == 0 ? "25/50/75/100 %" : "one level",
                 static_cast<unsigned>((CalSequencer::FADE_MS * 2 + n * (CalSequencer::board_ms(this->cal_req_duty_) + 60) +
                                        999) / 1000));
        this->wake_();
      }
    } else if (now - this->cal_wait_ms_ > CAL_POWER_WAIT_MS) {
      this->cal_pending_ = false;
      ESP_LOGW(TAG, "Current calibration on GPIO%u skipped: the boards did not get power / Ultra in 30 s", this->pin_);
    }
  }
  if (this->cal_seq_.running()) {
    switch (this->cal_seq_.poll(this->sched_, now)) {
      case CalSequencer::EV_BOARD:
        this->cal_started_ = true;
        ESP_LOGI(TAG, "Current calibration on GPIO%u: testing board %u (%u of %u)", this->pin_,
                 static_cast<unsigned>(this->cal_seq_.board()), static_cast<unsigned>(this->cal_seq_.index() + 1),
                 static_cast<unsigned>(this->cal_seq_.count()));
        this->wake_();
        break;
      case CalSequencer::EV_DONE:
        ESP_LOGI(TAG, "Current calibration on GPIO%u: done, %u board(s) in %.1f s; the pin fades back", this->pin_,
                 static_cast<unsigned>(this->cal_seq_.count()), this->cal_seq_.elapsed_ms(now) / 1000.0f);
        this->wake_();
        break;
      default:
        break;
    }
  }
  this->cal_power_hold_ = this->calibration_running();
}

void UltraHub::service_power_() {
#ifdef USE_POWER_SUPPLY
  if (this->psu_ == nullptr)
    return;
  const uint32_t now = millis();
  bool lit = this->cal_power_hold_;
  for (auto *l : this->lights_)
    lit |= l->is_lit(now);
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  lit |= this->upd_power_hold_;
  // power_cycle(): released until the supply reports off (other requesters may keep it on: give up after its
  // keep-on time + 5 s), off for 1.5 s, then the normal rule again
  if (this->cycle_ == CYCLE_WAIT_OFF) {
    if (!this->psu_->is_enabled()) {
      this->cycle_ = CYCLE_OFF;
      this->cycle_ms_ = now;
    } else if (now - this->cycle_ms_ > this->psu_keep_on_ms_ + 5000) {
      this->cycle_ = CYCLE_FAILED;
      ESP_LOGW(TAG, "Power cycle: the supply stayed on (another component keeps it on?)");
    }
  } else if (this->cycle_ == CYCLE_OFF && now - this->cycle_ms_ >= 1500) {
    this->cycle_ = CYCLE_DONE;
  }
  if (this->cycle_ == CYCLE_WAIT_OFF || this->cycle_ == CYCLE_OFF)
    lit = false;
#endif
  if (lit && !this->psu_requested_) {
    const bool was_on =
        this->psu_->is_enabled() || (this->psu_released_ && millis() - this->psu_released_ms_ < this->psu_keep_on_ms_);
    this->psu_req_.request();
    this->psu_requested_ = true;
    if (!was_on) {
      this->power_waiting_ = true;
      this->power_on_ms_ = millis();
      ESP_LOGD(TAG, "Power supply on: replaying the boards' state in %" PRIu32 " ms", this->power_on_delay_ms_);
    }
  } else if (!lit && this->psu_requested_) {
    this->psu_req_.unrequest();
    this->psu_requested_ = false;
    this->psu_released_ = true;
    this->psu_released_ms_ = millis();
  }
#endif
}

void UltraHub::setup() {
  this->sched_.init(this->num_boards_, esp_random(), this->repeats_);
  this->started_ = true;
  for (const auto &c : this->early_config_) {
    for (uint16_t b = 0; b < this->num_boards_; b++) {
      if (c.board < 0 || c.board == b)
        this->sched_.set_config(b, c.id, c.value);
    }
  }
  this->early_config_.clear();
  this->early_config_.shrink_to_fit();
  this->sched_.flush_config();

  RAMAllocator<uint8_t> alloc(RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  size_t buf_bytes = std::max<size_t>(this->sched_.frame_bytes(), ultra_switch_frame_bytes(this->num_boards_));
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  buf_bytes = std::max<size_t>(buf_bytes, UPD_FRAME_BYTES);   // update sessions and PING frames
#endif
  this->tx_buf_ = alloc.allocate(buf_bytes);
  if (this->tx_buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate the frame buffer (%u B)", static_cast<unsigned>(buf_bytes));
    this->mark_failed();
    return;
  }

  const uint32_t res = rmt_resolution_hz();
  const uint32_t t0h = ns_to_ticks(this->bit0_high_ns_, res), t0l = ns_to_ticks(this->bit0_low_ns_, res);
  const uint32_t t1h = ns_to_ticks(this->bit1_high_ns_, res), t1l = ns_to_ticks(this->bit1_low_ns_, res);
  const uint32_t rst = ns_to_ticks(this->reset_ns_, res);
  const uint32_t rst0 = rst / 2, rst1 = rst - rst0;
  if (res == 0 || t0h == 0 || t0l == 0 || t1h == 0 || t1l == 0 || t0h > RMT_DURATION_MAX ||
      t0l > RMT_DURATION_MAX || t1h > RMT_DURATION_MAX || t1l > RMT_DURATION_MAX || rst1 > RMT_DURATION_MAX) {
    ESP_LOGE(TAG, "Timing does not fit an RMT symbol at %" PRIu32 " Hz", res);
    this->mark_failed();
    return;
  }
  this->sched_.set_bit_timing(this->bit0_high_ns_, this->bit0_low_ns_, this->bit1_high_ns_, this->bit1_low_ns_);
  this->params_.bit0 = make_symbol(t0h, 1, t0l, 0);
  this->params_.bit1 = make_symbol(t1h, 1, t1l, 0);
  this->params_.reset = make_symbol(rst0, 0, rst1, 0);

  rmt_tx_channel_config_t ch;
  memset(&ch, 0, sizeof(ch));
  ch.clk_src = RMT_CLK_SRC_DEFAULT;
  ch.resolution_hz = res;
  ch.gpio_num = static_cast<gpio_num_t>(this->pin_);
  ch.mem_block_symbols = this->rmt_symbols_;
  ch.trans_queue_depth = 1;
  ch.flags.invert_out = this->inverted_;
  ch.flags.with_dma = this->use_dma_;
  ch.intr_priority = 0;
  if (rmt_new_tx_channel(&ch, &this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX channel creation failed");
    this->mark_failed();
    return;
  }
  rmt_simple_encoder_config_t enc;
  memset(&enc, 0, sizeof(enc));
  enc.callback = encoder_callback;
  enc.arg = &this->params_;
  enc.min_chunk_size = 1;
  if (rmt_new_simple_encoder(&enc, &this->encoder_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT encoder creation failed");
    this->mark_failed();
    return;
  }
  if (rmt_enable(this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT enable failed");
    this->mark_failed();
    return;
  }
  const uint32_t worst = std::max(this->bit0_high_ns_ + this->bit0_low_ns_, this->bit1_high_ns_ + this->bit1_low_ns_);
  this->frame_us_ = static_cast<uint32_t>((static_cast<uint64_t>(this->sched_.frame_bytes()) * 8 * worst) / 1000) +
                    this->reset_ns_ / 1000;
  this->switch_frame_us_ =
      static_cast<uint32_t>((static_cast<uint64_t>(ultra_switch_frame_bytes(this->num_boards_)) * 8 * worst) / 1000) +
      this->reset_ns_ / 1000;
  this->preamble_ = preamble_frames(this->num_boards_);
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  this->arm_ping_burst_();
#endif
  // Line is idle low now; guarantee one full reset period before the first frame.
  delayMicroseconds(this->reset_ns_ / 1000 + 1);
  this->wake_();
}

void UltraHub::loop() {
  this->service_calibration_();
#ifdef USE_POWER_SUPPLY
  this->service_power_();
  if (this->power_waiting_ && millis() - this->power_on_ms_ >= this->power_on_delay_ms_) {
    this->power_waiting_ = false;
    ESP_LOGD(TAG, "Power ready: replaying the boards' state");
    this->resync();
  }
#endif
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  // An update session owns the line; afterwards it stays low for a second (the boards commit and restart), then
  // the normal frames come back with a resync (preamble, PING burst, everything again).
  if (this->upd_state_.load() != UPD_IDLE) {
    if (this->upd_state_.load() == UPD_FINISHED) {
      this->upd_task_ = nullptr;
      this->upd_state_.store(UPD_IDLE);
      this->upd_quiet_ = true;
      this->upd_quiet_ms_ = millis();
      static const char *const RESULTS[] = {"", "done", "aborted", "failed (RMT)"};
      ESP_LOGI(TAG, "Firmware update session %s", RESULTS[this->upd_result_.load() & 3]);
    }
    return;
  }
  if (this->upd_quiet_) {
    if (millis() - this->upd_quiet_ms_ < 1000)
      return;
    this->upd_quiet_ = false;
    this->upd_done_ = static_cast<UpdateResult>(this->upd_result_.load());
    this->resync();
  }
  const bool ping_pending = this->ping_left_ > 0 || this->dark_tail_ > 0;
#else
  const bool ping_pending = false;
#endif
  const bool urgent = this->preamble_ > 0 || ping_pending || this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES ||
                      this->sched_.pending();
  if (urgent)
    this->last_busy_ms_ = millis();
  if (this->high_freq_active_ && !urgent && millis() - this->last_busy_ms_ > BUSY_HOLD_MS) {
    this->high_freq_.stop();
    this->high_freq_active_ = false;
  }
  if (!urgent && this->has_sent_ && millis() - this->last_frame_ms_ < this->refresh_interval_ms_)
    return;
  // Non-blocking: if the previous frame (incl. its reset) is still on the wire, retry next loop.
  if (rmt_tx_wait_all_done(this->channel_, 0) != ESP_OK)
    return;
  this->transmit_();
}

void UltraHub::transmit_() {
  // The header (the frame's trailer) carries the host clock at the frame's last bit: the start, taken right before
  // the frame goes out, plus its bits on the wire.
  size_t bytes = this->sched_.frame_bytes();
  const int64_t start = now_us_();
  if (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES) {
    // switch_boards_to_ultra: a WS config frame (no Ultra header), with the same bit timing
    bytes = ultra_switch_frame(this->tx_buf_, this->switch_frame_++, this->num_boards_);
  } else if (this->preamble_ > 0) {
    this->preamble_--;
    this->sched_.build_dark_frame(start, this->tx_buf_);
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  } else if (this->ping_left_ > 0) {
    this->ping_left_--;
    UpdateSession::ping_frame(this->tx_buf_, static_cast<uint32_t>(start));
    bytes = UPD_FRAME_BYTES;
  } else if (this->dark_tail_ > 0) {
    this->dark_tail_--;
    this->sched_.build_dark_frame(start, this->tx_buf_);
#endif
  } else {
    this->sched_.build_frame(start, this->tx_buf_);
  }
  rmt_transmit_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  const esp_err_t err = rmt_transmit(this->channel_, this->encoder_, this->tx_buf_, bytes, &cfg);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
  this->last_frame_ms_ = millis();
  this->has_sent_ = true;
  this->frames_++;
}

void UltraHub::dump_config() {
  const uint32_t period0 = this->bit0_high_ns_ + this->bit0_low_ns_;
  const uint32_t period1 = this->bit1_high_ns_ + this->bit1_low_ns_;
  const uint32_t worst = period0 > period1 ? period0 : period1;
  const uint32_t bits = static_cast<uint32_t>(ULTRA_HDR_BITS + ULTRA_SLOT_BITS * this->num_boards_);
  const uint32_t frame_us = static_cast<uint32_t>((static_cast<uint64_t>(bits) * worst) / 1000) + this->reset_ns_ / 1000;
  ESP_LOGCONFIG(TAG,
                "QuinLED dig2analog Ultra line:\n"
                "  Boards: %u\n"
                "  Pin: GPIO%u%s\n"
                "  RMT symbols: %" PRIu32 "%s\n"
                "  Bit0: %" PRIu32 "/%" PRIu32 " ns, Bit1: %" PRIu32 "/%" PRIu32 " ns, Reset: %" PRIu32 " us\n"
                "  Frame: %" PRIu32 " us (%" PRIu32 " frames/s max)\n"
                "  Refresh: every %" PRIu32 " ms, start delay %" PRIu32 " ms, %u copies per command\n"
                "  Preamble: %u frames of DARK slots",
                this->num_boards_, this->pin_, this->inverted_ ? " (inverted)" : "", this->rmt_symbols_,
                this->use_dma_ ? " (DMA)" : "", this->bit0_high_ns_, this->bit0_low_ns_, this->bit1_high_ns_,
                this->bit1_low_ns_, this->reset_ns_ / 1000, frame_us, frame_us ? 1000000 / frame_us : 0,
                this->refresh_interval_ms_, static_cast<uint32_t>(this->start_delay_us_ / 1000), this->repeats_,
                preamble_frames(this->num_boards_));
  if (this->num_boards_ > 0 && this->sched_.has_config(0, ULTRA_CFG_BOOT_PROTOCOL)) {
    static const char *const MODES[] = {"auto", "WS2915", "WS2805", "Ultra"};
    const uint16_t m = this->sched_.get_config(0, ULTRA_CFG_BOOT_PROTOCOL);
    ESP_LOGCONFIG(TAG, "  Boot protocol (stored on the boards): %s", MODES[m < 4 ? m : 0]);
  }
  for (uint16_t b = 0; b < this->num_boards_ && b < 4; b++) {
    const uint16_t preset = this->sched_.get_config(b, ULTRA_CFG_PWM_PRESET);
    const uint16_t period = this->sched_.get_config(b, ULTRA_CFG_PWM_PERIOD);
    const uint32_t steps = period != 0 ? period : (preset < 15 ? PWM_PRESET_STEPS[preset] : 0);
    ESP_LOGCONFIG(TAG,
                  "  Board %u: PWM %.1f Hz (%" PRIu32 " steps), dithering %u, threshold %.1f %%, dim floor %u, "
                  "min pulse %u ns, stagger %u, signal loss %u after %.1f s, board limit %.2f A",
                  b, steps ? PWM_CLOCK_HZ / static_cast<float>(steps) : 0.0f, steps,
                  this->sched_.get_config(b, ULTRA_CFG_DITHER), this->sched_.get_config(b, ULTRA_CFG_DITHER_THRESH) / 10.0f,
                  this->sched_.get_config(b, ULTRA_CFG_DIM_FLOOR), this->sched_.get_config(b, ULTRA_CFG_MIN_PULSE) * 10u,
                  this->sched_.get_config(b, ULTRA_CFG_STAGGER), this->sched_.get_config(b, ULTRA_CFG_LOSS),
                  this->sched_.get_config(b, ULTRA_CFG_LOSS_TIMEOUT) / 10.0f,
                  this->sched_.get_config(b, ULTRA_CFG_BOARD_MAX) / 1000.0f);
  }
#ifdef USE_POWER_SUPPLY
  if (this->psu_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Power supply: replay after %" PRIu32 " ms", this->power_on_delay_ms_);
#endif
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  if (this->ping_burst_)
    ESP_LOGCONFIG(TAG, "  Firmware update: %" PRIu32 " PING frames after each preamble (UPD frame %" PRIu32 " us)",
                  fw_ping_burst_frames(this->num_boards_), this->update_frame_us());
#endif
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup FAILED");
}

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
