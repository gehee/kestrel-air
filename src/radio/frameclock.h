#pragma once
#include <stdint.h>

// Ours: the radio's TDD frame on our clock, and the radio clock (AP time) to
// some 10-20 us from it (see frameclock.c).

void frameclock_start(void);
// The radio's frame may have changed (its width, its link formed again):
// found again at once, from the frames it can be, and the clock carried onto it.
void frameclock_reframe(void);
// The radio clock, us, at our time t (CLOCK_MONOTONIC us). 0: known.
int frameclock_ap_us(uint64_t t, uint64_t *ap);
// The frame on the radio clock: its period (us, whole 0.64 us ticks), where
// its video part ends (radio clock us, modulo the period), and a generation
// that changes whenever the frame's edges are counted again. 0: known.
int frameclock_frame(double *period, double *phase, unsigned *gen);
