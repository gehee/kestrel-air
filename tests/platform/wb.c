// ar9311_wb_gains against the P1 tuning file's reference points (the vendor's
// lookup, by hand): 4000 K between TL84 3960 and CW 4050, 5045 K twice.
#include <assert.h>
#include <string.h>
#include "backends/ar9311/wb.h"

static void put_u32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static void put_f32(uint8_t *p, float v) { memcpy(p, &v, 4); }

int main(void) {
    static const struct { uint32_t k; float rg, bg; } pts[8] = {
        { 7442, 0.4546f, 0.6184f }, { 6575, 0.4899f, 0.5468f }, { 5045, 0.5357f, 0.4948f },
        { 5045, 0.5357f, 0.4948f }, { 4050, 0.5591f, 0.3357f }, { 3960, 0.6680f, 0.3275f },
        { 2892, 0.9208f, 0.2742f }, { 2336, 1.1356f, 0.2017f },
    };
    uint8_t b[AR9311_WB_BLOCK_SIZE] = { 0 };
    put_u32(b + 0x0c, 8);
    for (int i = 0; i < 8; i++) {
        put_f32(b + 0x30 + 8 * i, pts[i].rg);
        put_f32(b + 0x34 + 8 * i, pts[i].bg);
        put_u32(b + 0xd0 + 4 * i, pts[i].k);
    }
    static const unsigned want[7][3] = {
        { 4000, 413, 773 }, { 4500, 466, 627 }, { 5000, 476, 525 }, { 5500, 490, 501 },
        { 6000, 504, 485 }, { 6500, 520, 470 }, { 7000, 541, 439 },
    };
    unsigned r, bl;
    for (int i = 0; i < 7; i++) {
        assert(ar9311_wb_gains(b, sizeof(b), (int)want[i][0], &r, &bl) == 0);
        assert(r == want[i][1] && bl == want[i][2]);
    }
    assert(ar9311_wb_gains(b, sizeof(b), 9000, &r, &bl) == 0 && r == 563 && bl == 413);   // past D75: D75
    assert(ar9311_wb_gains(b, sizeof(b), 2000, &r, &bl) == 0 && r == 225 && bl == 1269);  // below H: H
    put_u32(b + 0x0c, 0);
    assert(ar9311_wb_gains(b, sizeof(b), 5000, &r, &bl) < 0);
    put_u32(b + 0x0c, 8);
    put_f32(b + 0x30, -1.0f);
    assert(ar9311_wb_gains(b, sizeof(b), 5000, &r, &bl) < 0);
    assert(ar9311_wb_gains_file("/nonexistent", 5000, &r, &bl) < 0);
    return 0;
}
