/* SPDX-License-Identifier: MIT
 * Copyright (C) 2026 QuinLED
 *
 * QuinLED dig2analog Ultra protocol v2 - shared by the board firmware (C) and the controllers (C / C++).
 * Master: include/ultra_protocol.h in https://github.com/intermittech/quinled-ultra-protocol (spec in its wiki).
 * Every copy (board firmware, ESPHome component, other controllers) must be byte-identical to one tagged release.
 *
 * Link: WS28xx-style one-wire pulses (same timing as WS2915), MSB first, one way host -> boards.
 * Frame:  N x SLOT (32 bytes, one per board)  +  HEADER (8 bytes, a trailer)  +  reset >= 280 us
 *         Each board takes the first slot of what it sees (its hardware gate holds the output LOW for those 256
 *         bits, as in WS2805 mode) and forwards the rest, so the position in the chain is the address and every
 *         board sees one contiguous frame ending with the header. Frame bits = 256 * N + 64.
 *         (v1 had the header first: the board's closed gate then left a 320 us LOW gap after the header, which
 *         ended the frame at the next board, so boards after the first never got their slot.)
 * HEADER: MAGIC (2) | VERSION<<4 | FLAGS (1) | TIME (4) | CRC-8 (1, SMBus poly 0x07)
 *         TIME = host clock (us, low 32 bits) at the end of the frame (the header's last bit); every board sees
 *         that moment within gate delays. FLAGS: ULTRA_HDR_* below.
 * SLOT:   TYPE (1) | LIGHT<<4 | FLAGS (1) | SEQ (1) | PAYLOAD (27) | CRC-16 (2, CCITT-FALSE)
 *         All multi-byte fields big-endian. Values: 16-bit fractions 0..65535 = 0..1.
 * Each command carries a sequence number per (light, type): the board applies a command once, so the host can
 * repeat it (redundancy) and re-send the current state periodically (keepalive / catch-up after a reset).
 * Preamble: before its first commands (start-up, resync after power-on) the host sends ultra_preamble_frames(N)
 *         frames of DARK slots (all zero, never valid) + header. Every board locks to Ultra on the headers, one
 *         after the other, also a board still locked to WS2805 / WS2915 (it shows its WS window of those zeros:
 *         dark, never a colour). Idle frames afterwards use NOP slots.
 *
 * Compatibility rules (ULTRA_PROTOCOL.md): a board ignores what it doesn't know, so new features never disturb
 * boards with older firmware; the host sends only values defined here (the builders below never set anything else).
 *  - Unknown slot TYPE (0x0A-0xFF), LIGHT 5-14, IDENTIFY value or argument, SCENE mode: the slot is ignored.
 *  - Unknown CONFIG id: that pair is skipped, the others apply. Unknown effect id: ULTRA_FX_NONE (the light's plain
 *    state). Unknown role: the channel is not part of the light.
 *  - Unknown flag bits (header HINT, slot FLAGS, SET / LIGHT_DEF / LIGHT_OPT flags), reserved bits and spare payload
 *    bytes: ignored. The host sends them as 0, so a future field's value 0 must always mean the old behaviour.
 *  - Header EXT ("must understand"): a board without the frame's extension keeps its lock and clock sync but ignores
 *    every slot of that frame, as for RW. New frame-level features go behind EXT (or RW / UPD) unless boards that
 *    don't know them can safely process the slots.
 *  - ULTRA_VERSION changes only for incompatible changes: a board rejects every frame of another version.
 *  - Broadcast / extension slots (reserved): k slots after the last board's slot, types >= ULTRA_TYPE_BCAST only,
 *    frame bits 256 x (N + k) + 64. Every board forwards everything after its own slot, so they reach every board;
 *    boards of this firmware never read them, and a board never applies one as its own slot (e.g. when the hub's
 *    board count is wrong). During the preamble they are DARK like the board slots.
 */
#ifndef ULTRA_PROTOCOL_H
#define ULTRA_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ULTRA_MAGIC        0x5AD2u
#define ULTRA_VERSION      2u          /* 1: header first (withdrawn, see above). Only an incompatible change bumps it:
                                          a board rejects other versions entirely (no lock, no clock, no slot) */
#define ULTRA_HDR_BYTES    8u
#define ULTRA_SLOT_BYTES   32u
#define ULTRA_HDR_BITS     (ULTRA_HDR_BYTES * 8u)
#define ULTRA_SLOT_BITS    (ULTRA_SLOT_BYTES * 8u)
#define ULTRA_PAYLOAD      3u          /* payload offset in a slot */
#define ULTRA_PAYLOAD_LEN  27u
#define ULTRA_LIGHTS       5u          /* lights per board (5 channels); LIGHT 5-14 reserved (groups / virtual lights) */
#define ULTRA_LIGHT_BOARD  0x0Fu       /* LIGHT field for board-level commands */
#define ULTRA_MAX_STEPS    8u          /* strobe / sequence steps per light */

/* header FLAGS (low nibble of byte 2), for later protocol parts. Firmware 0.4 implements none of them: on an RW or EXT
 * frame a board keeps its lock and clock sync but ignores every slot, an UPD frame it leaves to the update side (no
 * clock sync, no slot), HINT it ignores. RW and UPD are never set together; a board ignores such a frame. The hub sets
 * none of them in normal frames (0x00). */
#define ULTRA_HDR_RW       0x01u       /* a response window follows this frame; every slot is a POLL (wiki: Reserved extensions) */
#define ULTRA_HDR_HINT     0x02u       /* optional: a frame-level hint (none defined yet); a board that doesn't know it
                                          processes the frame as usual */
#define ULTRA_HDR_EXT      0x04u       /* must understand: the frame uses an extension (none defined yet; it is named
                                          with it, e.g. in a broadcast slot); a board without it ignores every slot */
#define ULTRA_HDR_UPD      0x08u       /* firmware update session (wiki: Reserved extensions) */

/* slot types */
enum {
    ULTRA_NOP = 0x00,          /* keepalive, nothing to do */
    ULTRA_LIGHT_DEF = 0x01,    /* light: channels, roles, gamma, min / max power */
    ULTRA_SET = 0x02,          /* light: target state + transition */
    ULTRA_EFFECT = 0x03,       /* light: start / stop a board effect */
    ULTRA_STEP = 0x04,         /* light: one strobe step (SEQ = the EFFECT's SEQ) */
    ULTRA_CONFIG = 0x05,       /* board: up to 9 settings (id, value) */
    ULTRA_IDENTIFY = 0x06,     /* board: blink pattern */
    ULTRA_POLL = 0x07,         /* reserved: answer / relay in a response window (RW frames only; ignored) */
    ULTRA_SCENE = 0x08,        /* light: power-on scene (persisted on the board) */
    ULTRA_LIGHT_OPT = 0x09,    /* light: options (dim to warm) */
    ULTRA_TYPES = 0x0A,        /* 0x0A-0x7F: future per-board commands (ignored) */
    ULTRA_TYPE_BCAST = 0x80,   /* 0x80-0xFF: broadcast / extension slots only (after the last board's slot), never
                                  applied as a board's own slot. (The UPD frame's block types 0x81-0x85 are another
                                  namespace: ultra_update.h) */
};

/* Return channel (reserved, not implemented; wiki: Reserved extensions). All times in us after t0 = the last
 * falling edge of the RW frame as each board measures it locally (the last edge the tail copy finds in the DIN ring,
 * not the shared clock's phase). During the window the forwarding gate is held CLOSED (DOUT LOW, not high-Z): DIN
 * sees the relayed answers, and a re-armed plan would forward them downstream. No POLL before the chain is locked
 * (after the DARK preamble) or during an update session; UID16 in the answers shows index shifts. */
#define ULTRA_POLL_RELAY_ONLY 0xFFu    /* POLL index: relay, don't answer */
#define ULTRA_RW_MAX_SLOTS    16u      /* INDEX is a nibble in the answer */
#define ULTRA_RW_RELEASE_US   150u     /* host / sender has stopped driving */
#define ULTRA_RW_DIR_MIN_US   250u     /* boards turn around between MIN and MAX */
#define ULTRA_RW_DIR_MAX_US   900u
#define ULTRA_RW_FIRST_US     1000u    /* response slot 0 */
#define ULTRA_RW_SLOT_US      450u     /* response slot pitch */
#define ULTRA_RW_HOLDOFF_US   200u     /* host: next frame >= window end + this */
#define ULTRA_RW_BAUD         1000000u /* UART 8N1, LSB first, line idle LOW between bursts */
/* response slot: SYNC | PAGE | INDEX<<4 | RFLAGS | SEQ | UID16 (2) | DATA (24) | CRC-16 (2) */
#define ULTRA_RSP_SYNC        0xA5u
#define ULTRA_RSP_DATA        6u
#define ULTRA_RSP_DATA_LEN    24u
#define ULTRA_RSP_EVENT       0x01u
#define ULTRA_RSP_STALE       0x02u
#define ULTRA_RSP_CONTENTION  0x04u
enum {
    ULTRA_PAGE_STATUS = 0x00, ULTRA_PAGE_OUTPUTS = 0x01, ULTRA_PAGE_ACKS = 0x02, ULTRA_PAGE_ERRORS = 0x03,
    ULTRA_PAGE_IDENTITY = 0x04, ULTRA_PAGE_FW_UPDATE = 0x05,
    ULTRA_PAGE_CALIBRATION = 0x06,     /* current sense: the model, connected load and limit in effect */
    ULTRA_PAGE_CAL_TEST = 0x07,        /* the stored test, in two parts (POLL argument 0: 0 = CH1-3, 1 = CH4-5 + rest) */
    ULTRA_PAGE_HW = 0x80,              /* 0x08-0x7F: future official pages; 0x80-0xFD: hardware-specific */
    ULTRA_PAGE_BUSY = 0xFE, ULTRA_PAGE_UNSUPPORTED = 0xFF,
};
/* IDENTITY page DATA: MCU UID (12) | firmware version word (4, ultra_update.h; 0 = unknown) | bootloader version (1,
 * 0 = none) | hardware id (1) | hardware capabilities (2, wiki: Reserved extensions) | firmware features (4,
 * ULTRA_FEAT_*). Capability reporting is reserved: without a return channel the host can't know a board's features and
 * relies on the compatibility rules above (an unknown command does nothing, an unknown effect shows the plain state). */
#define ULTRA_ID_FW_VERSION       12u
#define ULTRA_ID_FEATURES         20u
/* STATUS page DATA (wiki: Reserved extensions), the fields the firmware measures already (console `status`):
 * the MCU (die) temperature from the CH32V203's internal sensor, int16 0.1 C, ULTRA_ST_NA = not valid (no good
 * conversion yet, or 5 bad ones in a row; core/mcutemp.h), and the 3.3 V rail from VREFINT, mV (0 = not known).
 * The temperature's absolute offset varies per chip (up to ~+-20 C): a trend / over-temperature value. */
#define ULTRA_ST_MCU_TEMP         9u
#define ULTRA_ST_VDDA             15u
#define ULTRA_ST_NA               0x8000u
#define ULTRA_FEAT_SCENE          0x00000001u  /* SCENE: power-on scene */
#define ULTRA_FEAT_DIM_TO_WARM    0x00000002u  /* LIGHT_OPT dim to warm with red assist */
#define ULTRA_FEAT_DTW_POINTS     0x00000004u  /* LIGHT_OPT set points */
#define ULTRA_FEAT_CURRENT_SENSE  0x00000008u  /* the board current sense in use (CONFIG 21 = the measured current) */
#define ULTRA_FEAT_CALIBRATION    0x00000010u  /* IDENTIFY 2 (current calibration), pages CALIBRATION / CAL_TEST */
#define ULTRA_FEAT_DARK_HOLD      0x00000020u  /* IDENTIFY 3 / 4 */
#define ULTRA_FEAT_FW_UPDATE      0x00000040u  /* UPD sessions (the app with a bootloader) */
#define ULTRA_FEAT_BOOT_PROTOCOL  0x00000080u  /* CONFIG 22 */
#define ULTRA_FEAT_STAGGER_ADAPTIVE 0x00000100u /* CONFIG 7 values 3 / 4 */
#define ULTRA_FEAT_MCU_TEMP       0x00000200u  /* STATUS: the MCU temperature and the 3.3 V rail (ULTRA_ST_*) */
/*      0x00000400-0x00800000     future official features (e.g. broadcast slots, an EXT extension);
 *      0x01000000-0x80000000     hardware-specific */

/* SET flags (low nibble of byte 1; the other slot types send FLAGS 0) */
#define ULTRA_SET_SMOOTH   0x01u       /* ESPHome's smoothstep curve (6x^5 - 15x^4 + 10x^3), else linear */
#define ULTRA_SET_VIA_OFF  0x02u       /* color mode change: fade to 0, switch, fade up (ESPHome behaviour) */
#define ULTRA_SET_FLAGS    (ULTRA_SET_SMOOTH | ULTRA_SET_VIA_OFF)

/* channel roles (LIGHT_DEF); 8-15 reserved (e.g. amber, UV, lime): a board takes them as ULTRA_ROLE_NONE */
enum {
    ULTRA_ROLE_NONE = 0, ULTRA_ROLE_BRIGHTNESS = 1, ULTRA_ROLE_RED = 2, ULTRA_ROLE_GREEN = 3,
    ULTRA_ROLE_BLUE = 4, ULTRA_ROLE_WHITE = 5, ULTRA_ROLE_COLD_WHITE = 6, ULTRA_ROLE_WARM_WHITE = 7,
};

/* effects (EFFECT byte 0); parameters at payload offset 5.. (big-endian, <= 22 bytes) */
enum {
    ULTRA_FX_NONE = 0,         /* stop: back to the light's state */
    ULTRA_FX_PULSE = 1,        /* on_ms (3) off_ms (3) interval_ms (3) min (2) max (2)   - like ESPHome pulse */
    ULTRA_FX_STROBE = 2,       /* steps (1); steps sent as ULTRA_STEP                    - like ESPHome strobe */
    ULTRA_FX_FLICKER = 3,      /* alpha (2) intensity (2) interval_ms (2)                - like ESPHome flicker */
    ULTRA_FX_RANDOM = 4,       /* transition_ms (3) interval_ms (3)                      - like ESPHome random */
    ULTRA_FX_CANDLE = 5,       /* intensity (2) speed (2, % of nominal)                  - flame flicker */
    ULTRA_FX_BREATHE = 6,      /* period_ms (3) min (2) max (2)                          - sine breathing */
    ULTRA_FX_COLOR_LOOP = 7,   /* period_ms (3) saturation (2)                           - hue rotation (RGB) */
    ULTRA_FX_SUNRISE = 8,      /* duration_ms (3)                                         - warm dawn to the state */
    ULTRA_FX_SUNSET = 9,       /* duration_ms (3)                                         - state to warm dusk, off */
    ULTRA_FX_COUNT = 10,       /* 10-127: future official effects; a board runs an unknown id as ULTRA_FX_NONE */
    ULTRA_FX_CUSTOM = 0x80,    /* 128-255: custom effects */
};

/* CONFIG ids (value 16-bit). Host-managed: RAM only, they end with Ultra mode - except the ids marked
 * "persisted", which the board stores in flash (written only when the value differs from the stored one). */
enum {
    ULTRA_CFG_NONE = 0,
    ULTRA_CFG_PWM_PRESET = 1,      /* 0..14 */
    ULTRA_CFG_PWM_PERIOD = 2,      /* raw period in 144 MHz counts, 1440..65535; 0 = use the preset */
    ULTRA_CFG_DITHER = 3,          /* 0 on, 1 off at low levels, 2 off */
    ULTRA_CFG_DITHER_THRESH = 4,   /* 1..255 = 0.1..25.5 % */
    ULTRA_CFG_DIM_FLOOR = 5,       /* 0 skip, 1 clamp, 2 off */
    ULTRA_CFG_MIN_PULSE = 6,       /* 5..100 x 10 ns */
    ULTRA_CFG_STAGGER = 7,         /* 0 off, 1 fixed (thirds), 2 fixed + spread spectrum,
                                      3 adaptive (least LED_+ ripple), 4 adaptive + spread spectrum */
    ULTRA_CFG_LOSS = 8,            /* 0 hold, 1 off, 2 fade off */
    ULTRA_CFG_LOSS_TIMEOUT = 9,    /* x 100 ms, 5..600 */
    ULTRA_CFG_FEEDBACK = 10,       /* identify channel: 0 red, 1 all, 2 off */
    ULTRA_CFG_CH_CURRENT = 11,     /* 11..15: LED current of CH1..CH5 at 100 %, mA (power limit) */
    ULTRA_CFG_CH_MAX = 16,         /* 16..20: limit per channel, mA (0 = none) */
    ULTRA_CFG_BOARD_MAX = 21,      /* limit for the board, mA (0 = none); boards with the input current sense:
                                      the measured board current (0 = the board rating) */
    ULTRA_CFG_BOOT_PROTOCOL = 22,  /* persisted: protocol mode after power-up, 0 auto, 1 WS2915, 2 WS2805, 3 Ultra
                                      (auto / Ultra apply at once; WS2915 / WS2805 from the next power-up on) */
    ULTRA_CFG_COUNT = 23,          /* 23-127: future official ids; a board skips an unknown id (the others apply) */
    ULTRA_CFG_HW = 128,            /* 128-255: hardware-specific (e.g. a Diff-Power receiver, a current-sense revision) */
    ULTRA_CFG_MCU_TEMP_OFFSET = 128,   /* reserved, not settable yet: MCU temperature offset, int16 0.1 C (a one-point
                                          calibration, core/mcutemp.h); the board skips it like any unknown id */
};

/* ----------------------------------------------------------------------------------------------- helpers */
/* CRC-8/SMBus (poly 0x07, init 0) and CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), a nibble at a time: 16-entry
 * tables (16 / 32 bytes) instead of 8 shifts per byte, and no 256-entry table (GCC 15 turns the bitwise loop into
 * one) */
static inline uint8_t ultra_crc8(const uint8_t *d, uint32_t n)
{
    static const uint8_t t[16] = { 0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
                                   0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D };
    uint32_t c = 0;
    while (n--) {
        c ^= *d++;
        c = ((c << 4) & 0xFFu) ^ t[c >> 4];
        c = ((c << 4) & 0xFFu) ^ t[c >> 4];
    }
    return (uint8_t)c;
}

static inline uint16_t ultra_crc16(const uint8_t *d, uint32_t n)
{
    static const uint16_t t[16] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
                                    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF };
    uint32_t c = 0xFFFFu;
    while (n--) {
        c ^= (uint32_t)*d++ << 8;
        c = ((c << 4) & 0xFFFFu) ^ t[c >> 12];
        c = ((c << 4) & 0xFFFFu) ^ t[c >> 12];
    }
    return (uint16_t)c;
}

static inline void ultra_put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline void ultra_put24(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 16); p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)v; }
static inline void ultra_put32(uint8_t *p, uint32_t v) { ultra_put16(p, v >> 16); ultra_put16(p + 2, v); }
static inline uint32_t ultra_get16(const uint8_t *p) { return ((uint32_t)p[0] << 8) | p[1]; }
static inline uint32_t ultra_get24(const uint8_t *p) { return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]; }
static inline uint32_t ultra_get32(const uint8_t *p) { return (ultra_get16(p) << 16) | ultra_get16(p + 2); }

/* ----------------------------------------------------------------------------------------------- frame */
/* layout (v2): slot i at byte ULTRA_SLOT_BYTES x i, the header after the n slots */
static inline uint32_t ultra_frame_bytes(uint32_t n) { return ULTRA_SLOT_BYTES * n + ULTRA_HDR_BYTES; }
static inline uint32_t ultra_header_offset(uint32_t n) { return ULTRA_SLOT_BYTES * n; }

/* Preamble frames for a chain of n boards. A board locks to Ultra on 3 headers (a DARK slot is never valid; a board
 * locked to a WS protocol decodes only its WS window anyway) and forwards with the Ultra plan one frame later, so
 * the next board sees Ultra lengths ~4 frames after its predecessor; + 2 margin. Bit-level chain test
 * (firmware test_wire.c): 10 boards locked to WS2805 / WS2915 need 40, all boards get the first command after 42. */
static inline uint32_t ultra_preamble_frames(uint32_t n) { return 4u * n + 2u; }

/* ----------------------------------------------------------------------------------------------- header */
/* the trailer: time_us = host clock at the end of the frame, flags = ULTRA_HDR_* (0 for normal frames) */
static inline void ultra_build_header_flags(uint8_t *h, uint32_t time_us, uint8_t flags)
{
    ultra_put16(h, ULTRA_MAGIC);
    h[2] = (uint8_t)((ULTRA_VERSION << 4) | (flags & 0x0Fu));
    ultra_put32(h + 3, time_us);
    h[7] = ultra_crc8(h, 7);
}

static inline void ultra_build_header(uint8_t *h, uint32_t time_us)
{
    ultra_build_header_flags(h, time_us, 0);
}

/* 0 = valid Ultra header (magic, version, CRC) */
static inline int ultra_parse_header(const uint8_t *h, uint32_t *time_us)
{
    if (ultra_get16(h) != ULTRA_MAGIC || (h[2] >> 4) != ULTRA_VERSION || ultra_crc8(h, 7) != h[7]) {
        return -1;
    }
    *time_us = ultra_get32(h + 3);
    return 0;
}

static inline uint8_t ultra_header_flags(const uint8_t *h)
{
    return (uint8_t)(h[2] & 0x0Fu);
}

/* ----------------------------------------------------------------------------------------------- slots */
static inline void ultra_slot_begin(uint8_t *s, uint8_t type, uint8_t light, uint8_t flags, uint8_t seq)
{
    uint32_t i;
    for (i = 0; i < ULTRA_SLOT_BYTES; i++) {
        s[i] = 0;
    }
    s[0] = type;
    s[1] = (uint8_t)((light << 4) | (flags & 0x0Fu));
    s[2] = seq;
}

static inline void ultra_slot_seal(uint8_t *s)
{
    ultra_put16(s + 30, ultra_crc16(s, 30));
}

/* 0 = CRC ok */
static inline int ultra_slot_check(const uint8_t *s)
{
    return ultra_crc16(s, 30) == ultra_get16(s + 30) ? 0 : -1;
}

/* SET: target M (state x brightness) and per channel A (color component: red / green / blue, white, cold_white,
 * warm_white; 65535 for a brightness channel) and B (second factor: color_brightness on RGB channels, else 65535).
 * The board interpolates M, A and B separately (like ESPHome interpolates its LightColorValues) from its current
 * values over dur_ms (curve per flags), starting at header TIME + start_ms (signed: a late copy catches up), and
 * computes every 1 ms with an exact power-law gamma on the full product, no 8-bit / table steps:
 *   out = gamma(M x A x B),  or for the CW / WW channels of a constant_brightness light
 *   out = g(M) x max(g(cw), g(ww)) x g(this) / (g(cw) + g(ww))   (ESPHome's formula, g = gamma)
 * then min / max power: out = min + out x (max - min) (0 stays 0 with zero_means_zero). */
static inline void ultra_slot_set(uint8_t *s, uint8_t light, uint8_t seq, uint8_t flags, uint16_t m,
                                  const uint16_t *a, const uint16_t *b, uint32_t dur_ms, int16_t start_ms)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_SET, light, (uint8_t)(flags & ULTRA_SET_FLAGS), seq);
    ultra_put16(p, m);
    for (i = 0; i < 5u; i++) {
        ultra_put16(p + 2 + 2 * i, a[i]);
        ultra_put16(p + 12 + 2 * i, b[i]);
    }
    ultra_put24(p + 22, dur_ms > 0xFFFFFFu ? 0xFFFFFFu : dur_ms);
    ultra_put16(p + 25, (uint16_t)start_ms);
    ultra_slot_seal(s);
}

/* LIGHT_DEF: role per channel CH1..CH5 (ULTRA_ROLE_NONE = not part of this light), gamma x1000, flags,
 * min / max power per channel (0..65535 = 0..1, ESPHome output min_power / max_power).
 * Payload: roles as nibbles (p0 = CH1<<4 | CH2, p1 = CH3<<4 | CH4, p2 = CH5<<4), gamma (2), flags (1),
 * min[5] (2 each), max[5] (2 each), 1 spare byte. */
#define ULTRA_DEF_ZERO_MEANS_ZERO      0x01u
#define ULTRA_DEF_CONSTANT_BRIGHTNESS  0x02u
#define ULTRA_DEF_FLAGS                (ULTRA_DEF_ZERO_MEANS_ZERO | ULTRA_DEF_CONSTANT_BRIGHTNESS)
static inline void ultra_slot_light_def(uint8_t *s, uint8_t light, uint8_t seq, const uint8_t *roles,
                                        uint16_t gamma_x1000, uint8_t def_flags, const uint16_t *min_p,
                                        const uint16_t *max_p)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_LIGHT_DEF, light, 0, seq);
    for (i = 0; i < 5u; i++) {
        const uint8_t r = roles[i] > ULTRA_ROLE_WARM_WHITE ? (uint8_t)ULTRA_ROLE_NONE : roles[i];
        p[i >> 1] |= (uint8_t)(r << ((i & 1u) ? 0 : 4));
        ultra_put16(p + 6 + 2 * i, min_p[i]);
        ultra_put16(p + 16 + 2 * i, max_p[i]);
    }
    ultra_put16(p + 3, gamma_x1000);
    p[5] = (uint8_t)(def_flags & ULTRA_DEF_FLAGS);
    ultra_slot_seal(s);
}

/* LIGHT_OPT: light options beyond LIGHT_DEF.
 * Dim to warm (flag ULTRA_OPT_DIM_TO_WARM, needs cold_white + warm_white channels): every 1 ms the board moves the
 * CW / WW mix with M (state x brightness), linearly in mireds, from the light's own colour temperature at M = 1 to
 * warm_mired at M = 0, keeping ESPHome's normalisation (constant_brightness: CW + WW = g(M) x max(g(cw), g(ww));
 * else the brighter channel stays at g(M) x max), so at full brightness nothing changes. Below the WW LED's
 * temperature (warm_mired > ww_mired) red assist adds red (and red / 4 on green) up to red_assist x the WW level at
 * M = 0, on the light's own red / green channels while its RGB part is off.
 * Set points (flag ULTRA_OPT_POINTS): instead of the straight line, up to ULTRA_OPT_POINTS_MAX (M, mireds x 10)
 * pairs at payload 9.. (u16 each, unused = 0, any order). The board interpolates linearly in mireds between them;
 * above the highest point up to the light's own temperature at M = 1 (unless a point is at M = 65535), below the
 * lowest point it holds that point's temperature. Never colder than the light's own temperature. warm_mired10 = the
 * warmest point (the red assist scale).
 * Mireds x 10 (u16); red_assist 0..65535 = 0..100 %. */
#define ULTRA_OPT_DIM_TO_WARM 0x01u
#define ULTRA_OPT_POINTS      0x02u
#define ULTRA_OPT_FLAGS       (ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS)
#define ULTRA_OPT_POINTS_MAX  4u
static inline void ultra_slot_light_opt_points(uint8_t *s, uint8_t light, uint8_t seq, uint8_t flags,
                                               uint16_t cw_mired10, uint16_t ww_mired10, uint16_t warm_mired10,
                                               uint16_t red_assist, uint32_t n, const uint16_t *pt_m,
                                               const uint16_t *pt_mired10)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_LIGHT_OPT, light, 0, seq);
    p[0] = (uint8_t)(flags & ULTRA_OPT_FLAGS);
    ultra_put16(p + 1, cw_mired10);
    ultra_put16(p + 3, ww_mired10);
    ultra_put16(p + 5, warm_mired10);
    ultra_put16(p + 7, red_assist);
    for (i = 0; i < n && i < ULTRA_OPT_POINTS_MAX; i++) {
        ultra_put16(p + 9 + 4 * i, pt_m[i]);
        ultra_put16(p + 11 + 4 * i, pt_mired10[i]);
    }
    ultra_slot_seal(s);
}

static inline void ultra_slot_light_opt(uint8_t *s, uint8_t light, uint8_t seq, uint8_t flags, uint16_t cw_mired10,
                                        uint16_t ww_mired10, uint16_t warm_mired10, uint16_t red_assist)
{
    ultra_slot_light_opt_points(s, light, seq, flags, cw_mired10, ww_mired10, warm_mired10, red_assist, 0, 0, 0);
}

/* EFFECT: id, start time (ms, signed 32-bit, relative to header TIME: the effect's phase, so a board that
 * joins later runs in step), raw parameter bytes (<= 22). The board switches (also to FX_NONE) at that time. */
#define ULTRA_FX_PARAMS 22u
/* parameter bytes of each effect (the table above); the rest of the payload is spare (0) */
static inline uint32_t ultra_fx_param_bytes(uint8_t fx)
{
    static const uint8_t n[ULTRA_FX_COUNT] = { 0, 13, 1, 6, 6, 4, 7, 5, 3, 3 };
    return fx < ULTRA_FX_COUNT ? n[fx] : 0u;
}

static inline void ultra_slot_effect(uint8_t *s, uint8_t light, uint8_t seq, uint8_t fx, int32_t start_ms,
                                     const uint8_t *params, uint32_t nparams)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_EFFECT, light, 0, seq);
    p[0] = fx < ULTRA_FX_COUNT ? fx : (uint8_t)ULTRA_FX_NONE;
    ultra_put32(p + 1, (uint32_t)start_ms);
    for (i = 0; i < nparams && i < ultra_fx_param_bytes(p[0]); i++) {
        p[5 + i] = params[i];
    }
    ultra_slot_seal(s);
}

/* STEP: index / count, duration and transition (ms), values like SET. seq = the strobe EFFECT's seq. */
static inline void ultra_slot_step(uint8_t *s, uint8_t light, uint8_t seq, uint8_t index, uint8_t count,
                                   uint16_t dur_ms, uint16_t trans_ms, uint16_t m, const uint16_t *a,
                                   const uint16_t *b)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_STEP, light, 0, seq);
    p[0] = (uint8_t)((index << 4) | (count & 0x0Fu));
    ultra_put16(p + 1, dur_ms);
    ultra_put16(p + 3, trans_ms);
    ultra_put16(p + 5, m);
    for (i = 0; i < 5u; i++) {
        ultra_put16(p + 7 + 2 * i, a[i]);
        ultra_put16(p + 17 + 2 * i, b[i]);
    }
    ultra_slot_seal(s);
}

/* CONFIG: up to 9 (id, value) pairs; an unused pair is (0, 0). Ids that aren't defined are left out. */
static inline void ultra_slot_config(uint8_t *s, uint8_t seq, const uint8_t *ids, const uint16_t *values, uint32_t n)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i, j = 0;
    ultra_slot_begin(s, ULTRA_CONFIG, ULTRA_LIGHT_BOARD, 0, seq);
    for (i = 0; i < n && j < 9u; i++) {
        if (ids[i] != ULTRA_CFG_NONE && ids[i] < ULTRA_CFG_COUNT) {
            p[3 * j] = ids[i];
            ultra_put16(p + 3 * j + 1, values[i]);
            j++;
        }
    }
    ultra_slot_seal(s);
}

/* IDENTIFY: on = 1 blinks the board's identify pattern (30 s), 0 stops it.
 * ULTRA_IDENT_CAL: boards with the input current sense measure each channel alone (zero, then R, G, B, W1, W2) and
 * store the result: `arg` 0 = the standard test at 25 / 50 / 75 / 100 % (0.95 s), 5..100 = at that duty only
 * (0.18 s; older firmware takes 0 as 25 %). Boards without it ignore it.
 * ULTRA_IDENT_DARK: every output fades to dark (0.3 s) and stays dark for `arg` s (0 = 60 s) or until
 * ULTRA_IDENT_RELEASE, which fades back; the light engine keeps running underneath, so states and effects continue.
 * The hub darkens a whole pin this way and then calibrates its boards one after the other.
 * Values 5-255 and CAL arguments 1-4 / 101-255 are reserved: a board ignores the slot. STOP / ON / RELEASE send
 * argument 0. */
#define ULTRA_IDENT_STOP 0u
#define ULTRA_IDENT_ON   1u
#define ULTRA_IDENT_CAL  2u
#define ULTRA_IDENT_DARK 3u
#define ULTRA_IDENT_RELEASE 4u
#define ULTRA_IDENT_CAL_MIN 5u         /* CAL argument: 0 = the standard test, else MIN..MAX % */
#define ULTRA_IDENT_CAL_MAX 100u
static inline void ultra_slot_identify(uint8_t *s, uint8_t seq, uint8_t on)
{
    ultra_slot_begin(s, ULTRA_IDENTIFY, ULTRA_LIGHT_BOARD, 0, seq);
    s[ULTRA_PAYLOAD] = on <= ULTRA_IDENT_RELEASE ? on : (uint8_t)ULTRA_IDENT_STOP;
    ultra_slot_seal(s);
}

static inline void ultra_slot_calibrate(uint8_t *s, uint8_t seq, uint8_t duty_pct)
{
    ultra_slot_begin(s, ULTRA_IDENTIFY, ULTRA_LIGHT_BOARD, 0, seq);
    s[ULTRA_PAYLOAD] = ULTRA_IDENT_CAL;
    s[ULTRA_PAYLOAD + 1] = duty_pct == 0u ? 0u : duty_pct < ULTRA_IDENT_CAL_MIN ? (uint8_t)ULTRA_IDENT_CAL_MIN
                         : duty_pct > ULTRA_IDENT_CAL_MAX ? (uint8_t)ULTRA_IDENT_CAL_MAX : duty_pct;
    ultra_slot_seal(s);
}

static inline void ultra_slot_hold_dark(uint8_t *s, uint8_t seq, uint8_t hold_s)
{
    ultra_slot_begin(s, ULTRA_IDENTIFY, ULTRA_LIGHT_BOARD, 0, seq);
    s[ULTRA_PAYLOAD] = ULTRA_IDENT_DARK;
    s[ULTRA_PAYLOAD + 1] = hold_s;
    ultra_slot_seal(s);
}

/* SCENE: what the light's channels show from the board's power-up until the controller takes over (stored in the
 * board's flash, written only when the resulting outputs change). mode 0 = not part of the scene (dark), 1 = these
 * values (like SET: M, A x5, B x5; the board computes the outputs with the light's definition and the power limit);
 * modes 2-255 reserved (a board ignores the slot, its stored scene stays).
 * fade_100ms: the take-over cross-fade from the scene to the controller's first state (x 100 ms). */
static inline void ultra_slot_scene(uint8_t *s, uint8_t light, uint8_t seq, uint8_t mode, uint8_t fade_100ms, uint16_t m,
                                    const uint16_t *a, const uint16_t *b)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_SCENE, light, 0, seq);
    p[0] = mode != 0u ? 1u : 0u;
    p[1] = fade_100ms;
    ultra_put16(p + 2, m);
    for (i = 0; i < 5u; i++) {
        ultra_put16(p + 4 + 2 * i, a[i]);
        ultra_put16(p + 14 + 2 * i, b[i]);
    }
    ultra_slot_seal(s);
}

/* NOP: a valid slot that does nothing (keep-alive frames to boards that are locked to Ultra) */
static inline void ultra_slot_nop(uint8_t *s)
{
    ultra_slot_begin(s, ULTRA_NOP, 0, 0, 0);
    ultra_slot_seal(s);
}

/* DARK: 256 zero bits, deliberately not a valid slot (the CRC-16 of zeros is not zero), so every board ignores it.
 * The preamble uses it: a board still locked to WS2805 / WS2915 cuts its WS window anywhere in the slots (along a
 * chain the windows of WS2805 boards cover every bit of a slot), so any 1 bit there - like a NOP's CRC - would show
 * as a colour; zeros show dark. Boards lock to Ultra on the trailer headers of these frames (3 frames each). */
static inline void ultra_slot_dark(uint8_t *s)
{
    uint32_t i;
    for (i = 0; i < ULTRA_SLOT_BYTES; i++) {
        s[i] = 0;
    }
}

/* ------------------------------------------------- switching boards that are forced to WS2805 / WS2915 to Ultra */
/* A board forced to a WS protocol (boot_protocol ws2915 / ws2805, config frame or console `protocol 1` / `2`) ignores
 * Ultra frames. The hub reaches it with the board's own WS config frames (firmware config.h): the message
 * MAGIC 0xD2A5 | CMD | ARG | CRC-8 (SMBus), in WS2805 format as the 5 channel bytes of each slot, in WS2915 format
 * after a 32-bit header as the 10-byte slot (the message, then its complement). A board takes a command after 3
 * identical frames, the same command again only after a normal frame. The hub can't know the boards' format, so
 * the sequence has both:
 *   WS2805: ULTRA_WSCFG_COPIES x "protocol = Ultra", then one normal frame (all zero: dark);  then the same in WS2915.
 * Padding: consecutive frames differ in length by 8 bits (1 or 2 zero bytes at the end, alternating). Every board
 * sees a frame minus whole bytes (40, 80 or 256 bits per board before it), so of two consecutive frames at most one
 * has a WS2805 or WS2915 length at any board: no board counts 3 frames of one WS protocol in a row, and every board
 * keeps its lock (Ultra boards and boards in auto locked to the other WS format). A board of the frame's format still
 * decodes its slot (its config frames need identical slots, not lengths).
 * Each board that takes it blinks the acknowledge pattern and runs forced Ultra at once (so it ignores the second
 * format). A board of the second format shows the first format's config frames as its slot: a colour flash of up
 * to ULTRA_WSCFG_COPIES frames, then dark. Then the hub sends the start-up preamble (ultra_preamble_frames) and sets
 * the boot protocol it wants (auto or Ultra) with CONFIG ULTRA_CFG_BOOT_PROTOCOL.
 * Limit: a board in Ultra mode blocks the first 256 bits of every frame, so WS boards behind it get the frames cut at
 * the wrong place and are not switched; neither are boards behind a board of the other WS format. */
#define ULTRA_WSCFG_MAGIC      0xD2A5u
#define ULTRA_WSCFG_PROTOCOL   0x01u   /* firmware CMD_PROTOCOL; ARG 3 = Ultra */
#define ULTRA_WSCFG_COPIES     4u      /* identical frames per format (a board needs 3) */
#define ULTRA_SWITCH_WS_FRAMES (2u * (ULTRA_WSCFG_COPIES + 1u))
/* bytes of the longest frame of the sequence for n boards */
static inline uint32_t ultra_switch_frame_bytes(uint32_t n) { return 4u + 10u * n + 2u; }

/* frame i (0 .. ULTRA_SWITCH_WS_FRAMES - 1) of the sequence for n boards into f; returns its length in bytes */
static inline uint32_t ultra_switch_frame(uint8_t *f, uint32_t i, uint32_t n)
{
    const uint32_t ws2915 = i > ULTRA_WSCFG_COPIES, k = ws2915 ? i - (ULTRA_WSCFG_COPIES + 1u) : i;
    const uint32_t len = (ws2915 ? 4u + 10u * n : 5u * n) + 1u + (i & 1u);   /* + 1 / 2 bytes of padding */
    uint8_t m[10];
    uint32_t b, j;
    for (j = 0; j < len; j++) {
        f[j] = 0;
    }
    for (j = 0; j < 10u; j++) {
        m[j] = 0;                                        /* the normal frame: all zero */
    }
    if (k < ULTRA_WSCFG_COPIES) {
        m[0] = (uint8_t)(ULTRA_WSCFG_MAGIC >> 8);
        m[1] = (uint8_t)(ULTRA_WSCFG_MAGIC & 0xFFu);
        m[2] = ULTRA_WSCFG_PROTOCOL;
        m[3] = 3u;
        m[4] = ultra_crc8(m, 4);
        for (j = 0; j < 5u; j++) {
            m[5u + j] = (uint8_t)~m[j];
        }
    }
    if (ws2915) {
        f[0] = 0xFFu;                                    /* header: gains 31 (the boards ignore them), check bits 0 */
        f[1] = 0xFFu;
        f[2] = 0xFFu;
        f[3] = 0x80u;
        for (b = 0; b < n; b++) {
            for (j = 0; j < 10u; j++) {
                f[4u + 10u * b + j] = m[j];
            }
        }
    } else {
        for (b = 0; b < n; b++) {
            for (j = 0; j < 5u; j++) {
                f[5u * b + j] = m[j];
            }
        }
    }
    return len;
}

#ifdef __cplusplus
}
#endif

#endif
