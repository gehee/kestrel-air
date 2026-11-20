#pragma once
#include <stdint.h>

/* No catch-up bursts after a stall. Reset when disabled or disconnected;
 * enabling starts a fresh interval, without interrupting the current picture. */
#define SPECTATOR_RECOVERY_US UINT64_C(2000000)
typedef struct { uint64_t last; int active; } spectator_recovery;
static inline int spectator_recovery_due(spectator_recovery *s, int enabled,
                                         int picture_boundary, uint64_t now) {
    if (!enabled) { s->active = 0; return 0; }
    if (!s->active) { s->last = now; s->active = 1; return 0; }
    if (!picture_boundary || now - s->last < SPECTATOR_RECOVERY_US) return 0;
    s->last = now;
    return 1;
}

/* Versioned HEVC suffix SEI, emitted only with opt-in. The 16-byte UUID
 * is the ASCII name padded with zeros. Payload: version, flags, radio MAC[4], width
 * LE16, height LE16, fps LE16, recovery period LE16 (ms). This announces a
 * stream after PHY acquisition; it is not an RF acquisition beacon. */
#include <stddef.h>
#include <string.h>
static inline size_t spectator_sei(uint8_t *out, size_t capacity,
                                   const uint8_t mac[4], unsigned width,
                                   unsigned height, unsigned fps) {
    uint8_t rb[33] = {5, 30};
    memcpy(rb + 2, "kestrel-spec-v1", 15);
    rb[18] = 1; rb[19] = 1;
    memcpy(rb + 20, mac, 4);
    rb[24] = width; rb[25] = width >> 8;
    rb[26] = height; rb[27] = height >> 8;
    rb[28] = fps; rb[29] = fps >> 8;
    rb[30] = 2000 & 255; rb[31] = 2000 >> 8;
    rb[32] = 0x80;
    if (capacity < 56) return 0;
    const uint8_t prefix[6] = {0, 0, 0, 1, 80, 1};
    memcpy(out, prefix, sizeof(prefix));
    size_t n = 6;
    int zeros = 0;
    for (size_t i = 0; i < sizeof(rb); ++i) {
        if (zeros == 2 && rb[i] <= 3) { out[n++] = 3; zeros = 0; }
        out[n++] = rb[i];
        zeros = rb[i] == 0 ? zeros + 1 : 0;
    }
    return n;
}
