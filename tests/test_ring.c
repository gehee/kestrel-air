#include <string.h>

#include "check.h"
#include "video/ring.h"

void test_ring(void) {
    CHECK(ring_init() == 0);
    CHECK(ring_count() == 0);
    uint32_t len;
    uint64_t t;
    CHECK(ring_get(&len, &t, 10) == NULL);    // empty: times out

    // In order, with what each packet carried.
    for (int i = 0; i < 3; i++) {
        uint8_t *p = ring_reserve(1000);
        CHECK(p != NULL);
        memset(p, 'a' + i, 1000);
        ring_put(p, 1000 - i, 100 + i);
    }
    CHECK(ring_count() == 3);
    for (int i = 0; i < 3; i++) {
        uint8_t *p = ring_get(&len, &t, 10);
        CHECK(p && len == (uint32_t)(1000 - i) && t == (uint64_t)(100 + i) && p[0] == 'a' + i);
        ring_release();
    }
    CHECK(ring_count() == 0);

    // Fill it (2 MiB of 300 KiB packets), drain, and go round again.
    int put = 0;
    for (;;) {
        uint8_t *p = ring_reserve(300 << 10);
        if (!p) break;
        ring_put(p, 300 << 10, 0);
        put++;
    }
    CHECK(put == 6);
    // Steady state, wrapping round: one out, one in. Wrapping to the start
    // needs strictly more room before the oldest packet than the packet (so a
    // full ring never looks empty): with exactly as much, it waits for one more
    // to go - never longer.
    for (int k = 0; k < 20; k++) {
        CHECK(ring_get(&len, &t, 10) != NULL);
        ring_release();
        uint8_t *p = ring_reserve(300 << 10);
        if (!p) {
            CHECK(ring_get(&len, &t, 10) != NULL);
            ring_release();
            p = ring_reserve(300 << 10);
        }
        CHECK(p != NULL);
        if (p) ring_put(p, 300 << 10, 0);
        CHECK(ring_count() >= 4);
    }
    ring_flush(50);
    CHECK(ring_count() == 0);
}
