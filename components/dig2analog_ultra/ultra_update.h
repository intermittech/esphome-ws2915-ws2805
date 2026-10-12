/* QuinLED dig2analog Ultra firmware update over the data line (research/firmware-update.md).
 * Shared by the bootloader (firmware/bootloader), the app and the ESPHome component (a copy:
 * esphome-ws2915/components/dig2analog_ultra/ultra_update.h). This file is the master; keep the copy identical.
 *
 * FROZEN: every board's bootloader parses these formats for its whole life (it is never updated in the field).
 * Only add new block types or flags that old bootloaders may ignore; never change a layout or a meaning.
 *
 * UPD frame (48 bytes = 384 bits, sent like any Ultra frame: WS28xx pulses, MSB first):
 *   BLOCK (40) | HEADER (8, trailer: MAGIC 0x5AD2 | 2 << 4 | ULTRA_HDR_UPD (0x08) | TIME | CRC-8)
 *   BLOCK: TYPE (1) | SESSION (1) | INDEX (2) | PASS (1) | FLAGS (1) | DATA (32) | CRC-16 (2, CCITT-FALSE over 0..37)
 *   The header version nibble of UPD frames is always 2, whatever the version of normal Ultra frames becomes.
 *
 * Image = DESCRIPTOR page (256 bytes) + BODY (the app as linked at UPD_APP_BODY). The body travels in 32-byte
 * chunks (DATA INDEX = chunk number), 8 chunks per 256-byte flash page; the descriptor travels in two ANNOUNCE
 * halves (INDEX 0 / 1). All multi-byte fields are big-endian, except that the CRC-32 runs over little-endian
 * 32-bit words (see upd_crc32_words).
 */
#ifndef ULTRA_UPDATE_H
#define ULTRA_UPDATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------------ frame */
#define UPD_HDR_MAGIC        0x5AD2u      /* = ULTRA_MAGIC */
#define UPD_HDR_VERSION      2u           /* header version nibble of UPD frames, frozen */
#define UPD_HDR_FLAG         0x08u        /* = ULTRA_HDR_UPD */
#define UPD_HDR_BYTES        8u
#define UPD_BLOCK_BYTES      40u
#define UPD_FRAME_BYTES      48u          /* block + header */
#define UPD_FRAME_BITS       384u

/* block fields (byte offsets) */
#define UPD_B_TYPE           0u
#define UPD_B_SESSION        1u
#define UPD_B_INDEX          2u           /* 2 bytes */
#define UPD_B_PASS           4u           /* pass << 4 | passes (information only) */
#define UPD_B_FLAGS          5u
#define UPD_B_DATA           6u           /* 32 bytes */
#define UPD_B_CRC            38u          /* CRC-16 CCITT-FALSE over bytes 0..37 */
#define UPD_CHUNK_BYTES      32u

/* block types (>= 0x80: unknown to every normal Ultra slot parser) */
enum {
    UPD_ANNOUNCE = 0x81,     /* INDEX 0 / 1: descriptor bytes 0..31 / 32..63. FLAGS: UPD_F_FORCE */
    UPD_DATA = 0x82,         /* INDEX: 32-byte chunk of the body */
    UPD_END = 0x83,          /* DATA: body CRC-32 (4), body length (4); commit now */
    UPD_ABORT = 0x84,        /* the session is cancelled */
    UPD_PING = 0x85,         /* not a session: an app that sees it may confirm itself (UPD_ST_CONFIRMED) */
};
#define UPD_F_FORCE          0x01u        /* ANNOUNCE: also install the same or an older version */

/* ------------------------------------------------------------------------------------------------ flash map */
#define UPD_FLASH_BASE       0x08000000u
#define UPD_BL_BASE          0x08000000u  /* bootloader, 8 KB, write-protected (WRPR0 bits 0-1) */
#define UPD_BL_SIZE          0x2000u
#define UPD_BL_INFO          0x08000010u  /* upd_bl_info_t, inside the bootloader */
#define UPD_APP_DESC         0x08002000u  /* descriptor page: descriptor (64) + status records (192) */
#define UPD_APP_BODY         0x08002100u  /* app body: the app is linked here (alias 0x00002100) */
#define UPD_APP_END          0x0800F700u  /* scene log 0x0800F700-0x0800FEFF, settings 0x0800FF00 */
#define UPD_APP_BODY_MAX     (UPD_APP_END - UPD_APP_BODY)          /* 54,784 bytes */
#define UPD_PAGE_BYTES       256u
#define UPD_SECTOR_BYTES     4096u
#define UPD_CHUNKS_PER_PAGE  (UPD_PAGE_BYTES / UPD_CHUNK_BYTES)    /* 8 */
#define UPD_APP_ENTRY        0x00002100u  /* where the bootloader jumps (the reset alias of UPD_APP_BODY) */

/* ------------------------------------------------------------------------------------------------ descriptor */
#define UPD_DESC_BYTES       64u
#define UPD_DESC_VERSION     1u
/* offsets */
#define UPD_D_MAGIC          0u           /* 'Q' 'D' 'U' '5' */
#define UPD_D_DESC_VERSION   4u
#define UPD_D_HW_ID          5u
#define UPD_D_HW_REV_MASK    6u           /* 2 bytes: bit n = board revision n is supported */
#define UPD_D_FW_VERSION     8u           /* 4 bytes: major, minor, patch, pre-release (UPD_PRE_*) */
#define UPD_D_BODY_LEN       12u          /* 4 bytes, multiple of 4, <= UPD_APP_BODY_MAX */
#define UPD_D_BODY_CRC       16u          /* 4 bytes, upd_crc32_words over the body */
#define UPD_D_LOAD_ADDR      20u          /* 4 bytes = UPD_APP_BODY */
#define UPD_D_ENTRY          24u          /* 4 bytes = UPD_APP_ENTRY */
#define UPD_D_MIN_BL         28u          /* oldest bootloader version that can install it */
#define UPD_D_FLAGS          29u          /* reserved, 0 */
#define UPD_D_BUILD_ID       32u          /* 16 bytes: git hash (ASCII), informational */
#define UPD_D_CRC            60u          /* 4 bytes, upd_crc32_words over bytes 0..59 */

#define UPD_HW_ID_D2A_ULTRA_5CH  1u       /* QuinLED dig2analog-ultra-5ch */
/* board revisions (bit numbers of the revision mask) */
#define UPD_REV_V01_PB15     0u           /* v0.1 as first drawn: PWM_B on PB15 only (HW_PWM_3TIMERS=0 build) */
#define UPD_REV_PB11         1u           /* PB11 wired to PWM_B (3-timer build) */
#define UPD_REV_ISENSE       2u           /* + input current sense (PA7) */

/* version word: major << 24 | minor << 16 | patch << 8 | pre; compared as an unsigned number this sorts like
 * semver: 0.6.0-alpha.3 < 0.6.0-beta.1 < 0.6.0-rc.1 < 0.6.0 */
#define UPD_PRE_ALPHA        0x00u        /* + n, n <= 63 */
#define UPD_PRE_BETA         0x40u
#define UPD_PRE_RC           0x80u
#define UPD_PRE_STABLE       0xFFu

/* ------------------------------------------------------------------------------------------------ status area */
/* Half-word records appended after the descriptor in its page (standard programming, one half-word each; the
 * page is only ever erased together with the descriptor). Erased reads 0xE339 (RM 32.5.4) or 0xFFFF. */
#define UPD_ST_FIRST         (UPD_DESC_BYTES / 2u)        /* half-word index 32 */
#define UPD_ST_COUNT         ((UPD_PAGE_BYTES - UPD_DESC_BYTES) / 2u)   /* 96 records */
#define UPD_ST_VERIFIED      0xA5B7u      /* bootloader: body CRC checked after a reset */
#define UPD_ST_CONFIRMED     0xA5C3u      /* app: received normal frames and a PING; boot without a window */
#define UPD_ST_TRIAL_QUIET   0xA5A0u      /* bootloader: unconfirmed boot, no Ultra frame in the window */
#define UPD_ST_TRIAL_SEEN    0xA5A1u      /* bootloader: unconfirmed boot, Ultra frames were there (a strike) */

/* ------------------------------------------------------------------------------------------------ bootloader info */
/* At UPD_BL_INFO (fixed): what the bootloader is and which board it was built for (production). */
typedef struct {
    uint8_t  magic[4];       /* 'Q' 'D' 'B' 'L' */
    uint8_t  bl_version;     /* 1 */
    uint8_t  hw_id;
    uint8_t  hw_rev;         /* bit number in the revision mask */
    uint8_t  reserved;
    uint8_t  build_id[8];
} upd_bl_info_t;

/* ------------------------------------------------------------------------------------------------ hand-over */
/* 32 bytes at the start of RAM, outside every linker section: survives software and watchdog resets (not power
 * loss; valid only when magic and check match). */
#define UPD_HANDOVER_ADDR    0x20000000u
#define UPD_HO_MAGIC         0x51444F48u  /* "QDOH" */
typedef struct {
    uint32_t magic;
    uint32_t request;        /* app / bootloader -> bootloader at the next boot */
    uint32_t result;         /* bootloader -> app: what the last session did (UPD_RES_*) */
    uint32_t result_version; /* the version the result is about */
    uint32_t wd_resets;      /* watchdog resets in a row; the app clears it after 10 s of normal running */
    uint32_t reset_flags;    /* RCC->RSTSCKR of the resets since the app last looked: the bootloader clears the
                                RCC flags at every boot (its own software resets would pile up otherwise) and
                                ORs them in here; the app reads and clears this instead of RCC->RSTSCKR */
    uint32_t reserved;
    uint32_t check;          /* ~(magic ^ request ^ result ^ result_version ^ wd_resets) */
} upd_handover_t;
#define UPD_REQ_NONE         0u
#define UPD_REQ_ENTER        0x55504421u  /* "UPD!": the app saw an ANNOUNCE it wants: stay and listen */
#define UPD_REQ_BOOT_ONCE    0x424F4F54u  /* "BOOT": jump without a listen window (set by the bootloader) */
#define UPD_RES_NONE         0u
#define UPD_RES_UPDATED      1u           /* app: play ACK + version */
#define UPD_RES_SAME         2u           /* app: play "already active" (1 short) */
#define UPD_RES_REJECTED     3u           /* app: play NACK (1 long) */
#define UPD_RES_INCOMPLETE   4u           /* app: play "2 long" (staging only: old firmware kept) */

static inline uint32_t upd_ho_sum(const upd_handover_t *ho)
{
    return ~(ho->magic ^ ho->request ^ ho->result ^ ho->result_version ^ ho->wd_resets);
}
static inline int upd_ho_valid(const upd_handover_t *ho)
{
    return ho->magic == UPD_HO_MAGIC && ho->check == upd_ho_sum(ho);
}
static inline void upd_ho_seal(upd_handover_t *ho)
{
    ho->magic = UPD_HO_MAGIC;
    ho->check = upd_ho_sum(ho);
}

/* ------------------------------------------------------------------------------------------------ helpers */
static inline uint32_t upd_get16(const uint8_t *p) { return ((uint32_t)p[0] << 8) | p[1]; }
static inline uint32_t upd_get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline void upd_put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline void upd_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

/* CRC-16 CCITT-FALSE (poly 0x1021, init 0xFFFF), as ultra_crc16 */
static inline uint16_t upd_crc16(const uint8_t *d, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        uint32_t b;
        c ^= (uint16_t)((uint16_t)*d++ << 8);
        for (b = 0; b < 8u; b++) {
            c = (uint16_t)((c & 0x8000u) ? (((uint32_t)c << 1) ^ 0x1021u) : ((uint32_t)c << 1));
        }
    }
    return c;
}

/* CRC-8 SMBus (poly 0x07, init 0), as ultra_crc8 */
static inline uint8_t upd_crc8(const uint8_t *d, uint32_t n)
{
    uint8_t c = 0;
    while (n--) {
        uint32_t b;
        c ^= *d++;
        for (b = 0; b < 8u; b++) {
            c = (uint8_t)((c & 0x80u) ? (((uint32_t)c << 1) ^ 0x07u) : ((uint32_t)c << 1));
        }
    }
    return c;
}

/* CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no reflection, no final XOR) over n little-endian 32-bit words,
 * each word MSB first: exactly what the CH32 CRC unit computes when the CPU writes the words as it reads them from
 * flash. `crc` = 0xFFFFFFFF to start; can be chained. */
static inline uint32_t upd_crc32_words(uint32_t crc, const uint8_t *p, uint32_t nwords)
{
    while (nwords--) {
        uint32_t w = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        uint32_t b;
        p += 4;
        crc ^= w;
        for (b = 0; b < 32u; b++) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
    }
    return crc;
}

/* erased flash, as read back */
static inline int upd_erased16(uint32_t v) { return v == 0xFFFFu || v == 0xE339u; }

/* A descriptor for this board? crc = upd_crc32_words over its first 60 bytes (the caller computes it: the CRC unit
 * on flash, software on RAM). 0 = usable; -1 = not a descriptor (magic / CRC); -2 = a descriptor, but not for this
 * board (hardware id, revision, size, load address, bootloader version). The bootloader and the app both use it. */
static inline int upd_desc_check(const uint8_t *d, uint32_t crc, uint32_t hw_id, uint32_t hw_rev, uint32_t bl_version)
{
    uint32_t len = upd_get32(d + UPD_D_BODY_LEN);
    if (d[0] != 'Q' || d[1] != 'D' || d[2] != 'U' || d[3] != '5' || crc != upd_get32(d + UPD_D_CRC)) {
        return -1;
    }
    if (d[UPD_D_DESC_VERSION] != UPD_DESC_VERSION || d[UPD_D_HW_ID] != hw_id ||
        ((upd_get16(d + UPD_D_HW_REV_MASK) >> hw_rev) & 1u) == 0u || len == 0u || len > UPD_APP_BODY_MAX ||
        (len & 3u) != 0u || upd_get32(d + UPD_D_LOAD_ADDR) != UPD_APP_BODY ||
        upd_get32(d + UPD_D_ENTRY) != UPD_APP_ENTRY || d[UPD_D_MIN_BL] > bl_version) {
        return -2;
    }
    return 0;
}

/* UPD frame trailer: 0 = a valid UPD header (magic, version 2, UPD flag, CRC-8) */
static inline int upd_check_header(const uint8_t *h)
{
    return (upd_get16(h) == UPD_HDR_MAGIC && (h[2] >> 4) == UPD_HDR_VERSION && (h[2] & UPD_HDR_FLAG) != 0u &&
            upd_crc8(h, 7) == h[7]) ? 0 : -1;
}

/* builds a whole UPD frame (48 bytes): block fields + header trailer with the host time */
static inline void upd_build_frame(uint8_t *f, uint8_t type, uint8_t session, uint32_t index, uint8_t pass,
                                   uint8_t flags, const uint8_t *data32, uint32_t time_us)
{
    uint32_t i;
    f[UPD_B_TYPE] = type;
    f[UPD_B_SESSION] = session;
    upd_put16(f + UPD_B_INDEX, index);
    f[UPD_B_PASS] = pass;
    f[UPD_B_FLAGS] = flags;
    for (i = 0; i < UPD_CHUNK_BYTES; i++) {
        f[UPD_B_DATA + i] = data32 ? data32[i] : 0u;
    }
    upd_put16(f + UPD_B_CRC, upd_crc16(f, UPD_B_CRC));
    upd_put16(f + UPD_BLOCK_BYTES, UPD_HDR_MAGIC);
    f[UPD_BLOCK_BYTES + 2u] = (uint8_t)((UPD_HDR_VERSION << 4) | UPD_HDR_FLAG);
    upd_put32(f + UPD_BLOCK_BYTES + 3u, time_us);
    f[UPD_BLOCK_BYTES + 7u] = upd_crc8(f + UPD_BLOCK_BYTES, 7);
}

/* 0 = a valid UPD frame (header + block CRC) */
static inline int upd_check_frame(const uint8_t *f)
{
    return (upd_check_header(f + UPD_BLOCK_BYTES) == 0 && upd_get16(f + UPD_B_CRC) == upd_crc16(f, UPD_B_CRC))
               ? 0 : -1;
}

#ifdef __cplusplus
}
#endif

#endif
