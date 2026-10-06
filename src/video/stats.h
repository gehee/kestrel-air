// --bb-verbose timing of the video path (see stats.c).
#pragma once
#include <stdint.h>

// One slice: captured at pts, out of the encoder at got, sent from start to
// end (all us, CLOCK_MONOTONIC).
void tstat_add(uint64_t pts, uint64_t got, uint64_t start, uint64_t end, int last_slice);
