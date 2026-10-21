// When the coarse AP-time tracker in video.c reads the radio again. No SDK:
// the host tests use it.
#pragma once
#include <stdint.h>

// Once a second once it has a sample. Without one it asks on every stamp, so
// the fallback starts as soon as it can - but only while it is the fallback:
// with the frame clock good (clock_ok) it keeps to once a second either way,
// as it only keeps itself fresh then and the frame clock thread is reading
// the radio too.
static inline int ap_track_due(int have_sample, int clock_ok, uint64_t now, uint64_t last) {
    if (!have_sample && !clock_ok) return 1;
    return now - last >= 1000000;
}
