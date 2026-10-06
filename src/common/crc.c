#include "crc.h"

static uint32_t tab[256];

void crc32c_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = c & 1 ? (c >> 1) ^ 0x82f63b78 : c >> 1;
        tab[i] = c;
    }
}

uint32_t crc32c(uint32_t crc, const void *data, size_t n) {
    const uint8_t *b = data;
    while (n--) crc = tab[(crc ^ *b++) & 0xff] ^ (crc >> 8);
    return crc;
}
