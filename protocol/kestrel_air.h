// kestrel_air.h - what kestrel-air puts on the radio for the ground to read:
// the video packet around each slice, and what is ours in it and in the
// messages. The stock air app's layout (reverse-engineered) with kestrel-air's
// additions in bytes the stock app sends as zero. Plain C, no dependencies, so
// a ground (kestrel-gnd) can take this file as it is.
#ifndef KESTREL_AIR_PROTOCOL_H
#define KESTREL_AIR_PROTOCOL_H

#include <stdint.h>

// ---- version and features ---------------------------------------------------
// The version message (ground message 0x04) and every slice header carry the
// tag, the protocol number and the feature bits. A stock air app sends zeros
// there, which is how a ground tells the two apart.
#define KA_PROTOCOL 1
#define KA_TAG      0x4B   // 'K'

#define KA_FEAT_LATINFO       0x01   // header bytes 35..41: the air-side times
#define KA_FEAT_INTRA_REFRESH 0x02   // intra refresh: keyframes only when asked for
#define KA_FEAT_CAM_IMU       0x04   // the camera's own IMU: its samples ride in the video as SEI
#define KA_FEAT_APCLOCK       0x08   // header 26..29 + 2: the radio clock at encoder out, 1/256 ms
#define KA_FEAT_MAXBW         0x10   // takes the ground's bandwidth cap (KA_CMD_MAX_BW)
#define KA_FEAT_FC_IMU        0x20   // the flight controller's IMU instead: MSP_RAW_IMU polled 20 a
                                     // second, its responses relayed to the ground as all others

// Version message: payload byte offsets of what is ours.
#define KA_VER_TAG0   1   // 'K'
#define KA_VER_TAG1   2   // 'A'
#define KA_VER_PROTO  3
#define KA_VER_FEAT   4

// ---- air -> ground messages that are ours -----------------------------------
// What the air unit is and runs, sent after each version message: text,
// "key=value" lines, '\n' after each. Keys: app (kestrel-air), ver (its
// version), os (the fpvOS image), radio (ar_libre), kernel, stock (the stock
// firmware under it, APP_VERSION), board (stock's board type), model, sensor,
// hw (the board-ID version, as in the version message). A key may be missing;
// a ground that does not know one skips it.
#define KA_MSG_INFO 0x50

// ---- ground -> air commands that are ours ------------------------------------
#define KA_CMD_MAX_KBPS 0x40   // u32 LE: video bitrate cap in kbps, 0 = none
#define KA_CMD_MAX_BW   0x41   // u8: video link bandwidth cap in MHz (20, or 40/0 = none)

// ---- the video packet ----------------------------------------------------------
// 00 00 00 01 | header (42) | extras (12) | slice | ee 29 55 9f
#define KA_PKT_START    4
#define KA_HDR_LEN      42
#define KA_EXT_LEN      12
#define KA_TRAILER_LEN  4
#define KA_PKT_OVERHEAD (KA_PKT_START + KA_HDR_LEN + KA_EXT_LEN + KA_TRAILER_LEN)

// Header byte offsets (from the header's first byte, after the start code).
#define KA_H_MAGIC    0    // 0x80
#define KA_H_SUM      1    // sum of bytes 2..41
#define KA_H_APFRAC   2    // ours: the radio clock's sub-ms part, 1/256 ms (KA_FEAT_APCLOCK)
#define KA_H_TYPE     3    // picture type (keyframe)
#define KA_H_LEN      4    // u32 LE: the slice's length (with any SEI after it)
#define KA_H_FPS      9    // encoded frame rate
#define KA_H_PROTO    10   // ours: KA_PROTOCOL
#define KA_H_FEAT     11   // ours: KA_FEAT_* bits
#define KA_H_PIC      12   // u16 LE: picture number
#define KA_H_WIDTH    14   // u16 LE
#define KA_H_HEIGHT   16   // u16 LE
#define KA_H_PTS      18   // u64 LE: capture time, us (MPP clock, CLOCK_MONOTONIC)
#define KA_H_RADIO_MS 26   // u32 LE: the radio clock, ms (with KA_FEAT_APCLOCK: at encoder out)
#define KA_H_CAP_MS   30   // ms from capture to the packet being built
#define KA_H_EXT_LEN  31   // KA_EXT_LEN
#define KA_H_SLICE    32   // slice index | KA_SLICE_MORE | KA_SLICE_LAST
#define KA_H_TAG      33   // ours: KA_TAG
#define KA_H_SET_FPS  34   // the configured frame rate
// Ours (KA_FEAT_LATINFO), each a u16 LE in 10 us units, clipped to 65535:
#define KA_H_T_ENC    35   // capture -> out of the encoder
#define KA_H_T_QUEUE  37   // waiting in the air's packet ring
#define KA_H_T_WRITE  39   // the previous slice's write to the radio
#define KA_H_DEPTH    41   // u8: slices in the ring behind this one (clipped to 255)

#define KA_SLICE_INDEX 0x1f
#define KA_SLICE_MORE  0x20
#define KA_SLICE_LAST  0x40

// The extras: 06 02 <XOR32 of the slice>, 06 01 <bytes sent since link-up>.
#define KA_EXT_XOR   0x02
#define KA_EXT_TOTAL 0x01

// The extras' checksum of the slice: XOR of its little-endian u32 words, then
// of the bytes left over.
static inline uint32_t ka_xor32(const uint8_t *b, uint32_t n) {
    uint32_t x = 0;
    for (uint32_t i = 0; i + 4 <= n; i += 4)
        x ^= (uint32_t)b[i] | (uint32_t)b[i + 1] << 8 | (uint32_t)b[i + 2] << 16 | (uint32_t)b[i + 3] << 24;
    for (uint32_t i = n & ~3u; i < n; i++) x ^= b[i];
    return x;
}

// The header's checksum byte: the sum of bytes 2..41.
static inline uint8_t ka_hdr_sum(const uint8_t *h) {
    uint8_t s = 0;
    for (int i = KA_H_APFRAC; i < KA_HDR_LEN; i++) s += h[i];
    return s;
}

// A time in 10 us units into a LE u16, clipped.
static inline void ka_put_t10(uint8_t *p, uint64_t t10) {
    uint64_t v = t10 > 65535 ? 65535 : t10;
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

#endif
