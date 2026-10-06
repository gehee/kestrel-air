#include "check.h"
#include "common/crc.h"

void test_crc(void) {
    crc32c_init();
    // CRC-32C with init 0 and no final xor (the standard check, init and
    // xorout ~0, is 0xe3069283).
    CHECK(crc32c(0, "123456789", 9) == 0x58e3fa20u);
    CHECK(crc32c(0xffffffffu, "123456789", 9) == ~0xe3069283u);
    CHECK(crc32c(0, "", 0) == 0);

    uint8_t c = 0;
    for (const char *p = "123456789"; *p; p++) c = crc8_dvb_s2(c, (uint8_t)*p);
    CHECK(c == 0xbc);                         // CRC-8/DVB-S2 check value
}
