#include <string.h>

#include "check.h"
#include "common/crc.h"
#include "ground/frame.h"

void test_frame(void) {
    crc32c_init();
    uint8_t f[4096 + FRAME_OVERHEAD], p[4090];
    int len = -1;

    // A frame worked out independently (Python): seq 1, payload 04 00.
    static const uint8_t ref[] = { 0xfe, 0xa5, 0x21, 0x01, 0x20, 0x00, 0x04, 0x00,
                                   0x00, 0x54, 0xbb, 0xa5, 0x0d };
    const uint8_t pl[2] = { 0x04, 0x00 };
    CHECK(frame_build(f, 1, 0, pl, 2) == (int)sizeof(ref));
    CHECK(memcmp(f, ref, sizeof(ref)) == 0);
    CHECK(frame_parse(ref, sizeof(ref), &len) == FRAME_OK && len == 2);

    // Round trips at the lengths where the length's nibbles change.
    for (int i = 0; i < (int)sizeof(p); i++) p[i] = (uint8_t)(i * 7 + 3);
    const int lens[] = { 0, 1, 15, 16, 255, 256, 4090 };
    for (unsigned k = 0; k < sizeof(lens) / sizeof(lens[0]); k++) {
        int n = frame_build(f, (uint8_t)k, k & 1, p, lens[k]);
        CHECK(n == lens[k] + FRAME_OVERHEAD);
        CHECK(frame_parse(f, n, &len) == FRAME_OK && len == lens[k]);
        CHECK(memcmp(f + 6, p, lens[k]) == 0);
        CHECK(f[3] == (uint8_t)k && ((f[4] & 0x08) != 0) == (int)(k & 1));
        CHECK(frame_parse(f, n - 1, &len) == FRAME_MORE);   // not all there yet
    }

    int n = frame_build(f, 9, 0, pl, 2);
    CHECK(frame_parse(f, 5, &len) == FRAME_MORE);
    f[2] ^= 1;
    CHECK(frame_parse(f, n, &len) == FRAME_BAD_SUM);
    f[2] ^= 1;
    f[6] ^= 0x40;
    CHECK(frame_parse(f, n, &len) == FRAME_BAD_CRC);
    f[6] ^= 0x40;
    f[0] = 0x00;
    CHECK(frame_parse(f, n, &len) == FRAME_NOT);
}
