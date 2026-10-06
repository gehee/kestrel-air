// Checksums used on the links.
#pragma once
#include <stddef.h>
#include <stdint.h>

// CRC-32C (Castagnoli, reflected), as the ground messages use it: start at
// crc (0 there), no final xor. crc32c_init() once before the first call.
void     crc32c_init(void);
uint32_t crc32c(uint32_t crc, const void *data, size_t n);

// CRC-8/DVB-S2 (poly 0xd5), one byte at a time: MSP v2.
static inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0xd5) : (uint8_t)(crc << 1);
    return crc;
}
