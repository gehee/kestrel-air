// The block holds N reference white points: their count at +0x0c, {R/G, B/G}
// float pairs at +0x30 and their colour temperatures (u32 kelvin) at +0xd0
// (8 on the P1: H 2336 K to D75 7442 K). A temperature between two points
// takes the line between their R/G and B/G, outside them the nearest point's;
// the gains are the inverses, truncated to Q8 as the vendor's WB setter does.
#include "backends/ar9311/wb.h"
#include <stdio.h>
#include <string.h>

#define MAX_POINTS 16

int ar9311_wb_gains(const uint8_t *block, size_t size, int kelvin, unsigned *red, unsigned *blue) {
    uint32_t n, cct[MAX_POINTS];
    float rg[MAX_POINTS], bg[MAX_POINTS];
    if (size < 0x10) return -1;
    memcpy(&n, block + 0x0c, 4);
    if (n < 1 || n > MAX_POINTS || size < 0xd0 + 4 * n) return -1;
    int lo = -1, hi = -1;
    for (uint32_t i = 0; i < n; i++) {
        memcpy(&rg[i], block + 0x30 + 8 * i, 4);
        memcpy(&bg[i], block + 0x34 + 8 * i, 4);
        memcpy(&cct[i], block + 0xd0 + 4 * i, 4);
        if (!(rg[i] > 0.05f && rg[i] < 8.0f && bg[i] > 0.05f && bg[i] < 8.0f) ||
            cct[i] < 1000 || cct[i] > 20000) return -1;
        if (cct[i] <= (uint32_t)kelvin && (lo < 0 || cct[i] > cct[lo])) lo = (int)i;
        if (cct[i] >= (uint32_t)kelvin && (hi < 0 || cct[i] < cct[hi])) hi = (int)i;
    }
    if (lo < 0) lo = hi;
    if (hi < 0) hi = lo;
    float t = cct[hi] == cct[lo] ? 0.0f : (float)(kelvin - (int)cct[lo]) / (float)(cct[hi] - cct[lo]);
    float r = (1.0f - t) * rg[lo] + t * rg[hi], b = (1.0f - t) * bg[lo] + t * bg[hi];
    unsigned qr = (unsigned)(256.0f / r), qb = (unsigned)(256.0f / b);
    *red = qr < 64 ? 64 : qr > 2048 ? 2048 : qr;      // the ISP's white_balance range
    *blue = qb < 64 ? 64 : qb > 2048 ? 2048 : qb;
    return 0;
}

int ar9311_wb_gains_file(const char *path, int kelvin, unsigned *red, unsigned *blue) {
    uint8_t block[AR9311_WB_BLOCK_SIZE];
    FILE *f = path ? fopen(path, "rb") : NULL;
    int ok = f && fseek(f, AR9311_WB_BLOCK, SEEK_SET) == 0 && fread(block, sizeof(block), 1, f) == 1;
    if (f) fclose(f);
    return ok ? ar9311_wb_gains(block, sizeof(block), kelvin, red, blue) : -1;
}
