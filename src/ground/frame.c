// The frame around every message with the ground (reverse-engineered):
//   fe a5 | hsum | seq | flags+len lo | len hi | payload (id first) |
//   00 | CRC-32C (init 0, no final xor) of all before it, little-endian.
// flags: 0x08 an acknowledgement, the low 3 bits the sub-channel (0).
// Pure: no I/O, so it is tested on the host (tests/).
#include "ground/frame.h"

#include <string.h>

#include "common/crc.h"

int frame_build(uint8_t *f, uint8_t seq, int ack, const uint8_t *payload, int len) {
    f[0] = 0xfe;
    f[1] = 0xa5;
    f[3] = seq;
    f[4] = (uint8_t)((ack ? 0x08 : 0) | ((len << 4) & 0xf0));   // sub-channel 0
    f[5] = (uint8_t)(len >> 4);
    f[2] = (uint8_t)(f[3] + f[4] + f[5]);
    memcpy(f + 6, payload, len);
    f[6 + len] = 0;
    uint32_t c = crc32c(0, f, len + 7);
    memcpy(f + 7 + len, &c, 4);
    return len + FRAME_OVERHEAD;
}

int frame_parse(const uint8_t *b, int have, int *len) {
    if (have < 6) return FRAME_MORE;
    if (b[0] != 0xfe || b[1] != 0xa5) return FRAME_NOT;
    if (b[2] != (uint8_t)(b[3] + b[4] + b[5])) return FRAME_BAD_SUM;
    *len = b[5] << 4 | b[4] >> 4;
    if (have < *len + FRAME_OVERHEAD) return FRAME_MORE;
    uint32_t c;
    memcpy(&c, b + *len + 7, 4);
    if (crc32c(0, b, *len + 7) != c) return FRAME_BAD_CRC;
    return FRAME_OK;
}
