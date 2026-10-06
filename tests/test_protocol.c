#include <string.h>

#include "check.h"
#include "kestrel_air.h"

void test_protocol(void) {
    uint8_t h[KA_HDR_LEN];
    memset(h, 0, sizeof(h));
    CHECK(ka_hdr_sum(h) == 0);
    h[KA_H_MAGIC] = 0x80;
    h[KA_H_SUM] = 0x55;                       // neither counts
    h[KA_H_APFRAC] = 0x10;
    h[KA_H_DEPTH] = 0xf5;
    CHECK(ka_hdr_sum(h) == (uint8_t)(0x10 + 0xf5));

    uint8_t t[2];
    ka_put_t10(t, 0x1234);
    CHECK(t[0] == 0x34 && t[1] == 0x12);
    ka_put_t10(t, 70000);                     // clipped
    CHECK(t[0] == 0xff && t[1] == 0xff);

    const uint8_t b[7] = { 1, 2, 3, 4, 0x10, 0x20, 0x30 };
    CHECK(ka_xor32(b, 4) == 0x04030201u);
    CHECK(ka_xor32(b, 7) == (0x04030201u ^ 0x10 ^ 0x20 ^ 0x30));
    CHECK(ka_xor32(b, 0) == 0);

    // The layout adds up: the last header byte is the last offset.
    CHECK(KA_H_DEPTH == KA_HDR_LEN - 1);
    CHECK(KA_PKT_OVERHEAD == 62);
}
