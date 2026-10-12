#pragma once

// Firmware update over the data line: one session on one hub's line (ultra_update.h; the QuinLED dig2analog-ultra
// firmware repo's research/firmware-update.md). Plain C++, no ESPHome dependency: the bootloader host tests send
// these exact frames through the real bootloader logic.

#include <cstdint>
#include <cstring>

#include "ultra_update.h"

namespace esphome::dig2analog_ultra {

/// Builds the frames of one update session:
///   lead-in   ANNOUNCE halves (boards enter update mode, accept and erase; boards forced to a WS protocol switch
///             one after the other, hence the per-board time),
///   passes    every flash page as 8 DATA chunks, then a pause for the boards' page program; odd passes run
///             backwards; every few pages both ANNOUNCE halves again (late joiners),
///   END       the image's CRC and length, repeated.
/// next() fills one 48-byte frame and says how long the line must stay low after the frame's normal reset.
class UpdateSession {
 public:
  struct Options {
    uint32_t lead_in_us{500000};
    uint32_t lead_in_per_board_us{15000};
    uint16_t boards{1};
    uint32_t end_us{250000};
    uint32_t page_pause_us{2700};    // >= one 256-byte page program (2 ms typ., datasheet)
    uint8_t passes{3};
    uint8_t announce_every{4};       // pages between ANNOUNCE repeats
    bool force{false};               // also the same or an older version (UPD_F_FORCE)
  };

  /// image = descriptor (64 bytes) + body, valid until the session ends. frame_us = one UPD frame incl. the reset
  /// on this line's timing (lead-in and END are counted in frames). false: the image is too small.
  bool begin(const uint8_t *image, uint32_t size, uint8_t session, const Options &opt, uint32_t frame_us) {
    if (size <= UPD_DESC_BYTES || frame_us == 0)
      return false;
    this->img_ = image;
    this->body_len_ = size - UPD_DESC_BYTES;
    this->session_ = session;
    this->opt_ = opt;
    if (this->opt_.passes == 0)
      this->opt_.passes = 1;
    this->npages_ = (this->body_len_ + UPD_PAGE_BYTES - 1) / UPD_PAGE_BYTES;
    this->lead_frames_ = (opt.lead_in_us + opt.lead_in_per_board_us * opt.boards + frame_us - 1) / frame_us;
    this->end_frames_ = (opt.end_us + frame_us - 1) / frame_us;
    const uint32_t chunks = (this->body_len_ + UPD_CHUNK_BYTES - 1) / UPD_CHUNK_BYTES;
    const uint32_t announces = opt.announce_every ? 2 * (this->npages_ / opt.announce_every) : 0;
    this->total_ = this->lead_frames_ + this->opt_.passes * (chunks + announces) + this->end_frames_;
    this->sent_ = 0;
    this->phase_ = LEAD;
    this->n_ = 0;
    this->pass_ = 0;
    this->k_ = 0;
    this->chunk_ = 0;
    this->ann_ = 0;
    return true;
  }

  /// One frame; false when the session is over. *pause_us: extra low time after the frame's reset.
  bool next(uint8_t *frame, uint32_t time_us, uint32_t *pause_us) {
    *pause_us = 0;
    const uint8_t flags = this->opt_.force ? UPD_F_FORCE : 0;
    switch (this->phase_) {
      case LEAD:
        upd_build_frame(frame, UPD_ANNOUNCE, this->session_, this->n_ & 1, 0, flags,
                        this->img_ + UPD_CHUNK_BYTES * (this->n_ & 1), time_us);
        if (++this->n_ >= this->lead_frames_) {
          this->phase_ = DATA;
          this->n_ = 0;
        }
        break;
      case DATA: {
        if (this->ann_ != 0) {        // the ANNOUNCE repeat after a group of pages
          const uint32_t half = 2 - this->ann_;
          upd_build_frame(frame, UPD_ANNOUNCE, this->session_, half, 0, flags, this->img_ + UPD_CHUNK_BYTES * half,
                          time_us);
          this->ann_--;
          if (this->ann_ == 0)
            this->advance_page_();
          break;
        }
        const uint32_t page = (this->pass_ & 1) ? this->npages_ - 1 - this->k_ : this->k_;
        const uint32_t idx = page * UPD_CHUNKS_PER_PAGE + this->chunk_;
        const uint32_t off = idx * UPD_CHUNK_BYTES;
        uint8_t data[UPD_CHUNK_BYTES];
        const uint32_t n = this->body_len_ - off < UPD_CHUNK_BYTES ? this->body_len_ - off : UPD_CHUNK_BYTES;
        memset(data, 0xFF, sizeof(data));
        memcpy(data, this->img_ + UPD_DESC_BYTES + off, n);
        upd_build_frame(frame, UPD_DATA, this->session_, idx, (uint8_t) ((this->pass_ << 4) | this->opt_.passes), 0,
                        data, time_us);
        const bool last = this->chunk_ + 1 == UPD_CHUNKS_PER_PAGE || off + UPD_CHUNK_BYTES >= this->body_len_;
        if (!last) {
          this->chunk_++;
          break;
        }
        *pause_us = this->opt_.page_pause_us;   // the boards program the page now
        this->chunk_ = 0;
        if (this->opt_.announce_every != 0 && this->k_ % this->opt_.announce_every == this->opt_.announce_every - 1u) {
          this->ann_ = 2;
        } else {
          this->advance_page_();
        }
        break;
      }
      case END: {
        uint8_t data[UPD_CHUNK_BYTES];
        memset(data, 0, sizeof(data));
        memcpy(data, this->img_ + UPD_D_BODY_CRC, 4);
        memcpy(data + 4, this->img_ + UPD_D_BODY_LEN, 4);
        upd_build_frame(frame, UPD_END, this->session_, 0, 0, 0, data, time_us);
        if (++this->n_ >= this->end_frames_)
          this->phase_ = DONE;
        break;
      }
      case DONE:
      default:
        return false;
    }
    this->sent_++;
    return true;
  }

  /// An ABORT frame (the session is cancelled: boards that erased stay in their bootloader)
  void abort_frame(uint8_t *frame, uint32_t time_us) const {
    upd_build_frame(frame, UPD_ABORT, this->session_, 0, 0, 0, nullptr, time_us);
  }
  /// A PING frame (not a session): a freshly installed app that sees it confirms itself
  static void ping_frame(uint8_t *frame, uint32_t time_us) {
    upd_build_frame(frame, UPD_PING, 0, 0, 0, 0, nullptr, time_us);
  }

  bool done() const { return this->phase_ == DONE; }
  uint32_t frames_sent() const { return this->sent_; }
  uint32_t total_frames() const { return this->total_; }
  float progress() const { return this->total_ ? (float) this->sent_ / (float) this->total_ : 0.0f; }
  uint8_t pass() const { return this->pass_; }

 protected:
  void advance_page_() {
    if (++this->k_ < this->npages_)
      return;
    this->k_ = 0;
    if (++this->pass_ >= this->opt_.passes) {
      this->phase_ = END;
      this->n_ = 0;
    }
  }

  enum Phase : uint8_t { LEAD, DATA, END, DONE };
  const uint8_t *img_{nullptr};
  uint32_t body_len_{0}, npages_{0}, lead_frames_{0}, end_frames_{0}, total_{0}, sent_{0}, n_{0}, k_{0};
  Options opt_{};
  Phase phase_{DONE};
  uint8_t session_{0}, pass_{0}, chunk_{0}, ann_{0};
};

}  // namespace esphome::dig2analog_ultra
