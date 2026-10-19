// The frame's edge from simulated reads of the radio's TDD state: a read goes
// out at t0, takes rtt and sees the state somewhere in between.
#include <math.h>

#include "check.h"
#include "radio/frame_edge.h"

static uint32_t lcg = 12345;
static double rnd(void) {
    lcg = lcg * 1664525u + 1013904223u;
    return (double)(lcg >> 8) / (double)(1u << 24);
}

// Reads back to back from t, us, of a frame of period T whose video part
// (vlen long) ends at E0 + k T.
static int simulate(fe_read *r, int max, uint64_t t, double dur, double T, double vlen, double E0) {
    int n = 0;
    const double end = (double)t + dur;
    while ((double)t < end && n < max) {
        const uint32_t rtt = 200 + (uint32_t)(rnd() * 300);
        const double at = (double)t + rnd() * rtt;
        double x = fmod(at - E0, T);
        if (x < 0) x += T;
        r[n].t0 = t;
        r[n].rtt = rtt;
        r[n].video = x >= T - vlen;          // the vlen before an edge
        n++;
        t += rtt + 20;
    }
    return n;
}

// The edge at or before t.
static double edge_before(double t, double T, double E0) {
    return E0 + floor((t - E0) / T) * T;
}

void test_frame_edge(void) {
    static fe_read r[8192];
    const double rate = 1.0000083;           // our us a radio us
    const double vlen[6] = { 3000, 3000, 3000, 1600, 1600, 1600 };
    for (int f = 0; f < 6; f++) {
        const double T = fe_frames[f] * rate, E0 = 1000000.0 + 777.7 * f;
        const int n = simulate(r, 8192, 2000000, FE_PHASE_US, T, vlen[f], E0);
        double E = 0, p = 0;
        CHECK(fe_phase(r, n, T, &E, &p) == 0);
        CHECK(fabs(E - edge_before((double)r[n - 1].t0, T, E0)) < 60);
        // No other frame of the table sorts these reads.
        for (int g = 0; g < 6; g++) {
            if (g == f) continue;
            double e2, p2;
            CHECK(fe_phase(r, n, fe_frames[g] * rate, &e2, &p2) != 0);
        }
    }
    // A search over a second and a half finds the period as well.
    {
        const double T = fe_frames[5] * rate, E0 = 1234567.0;
        const int n = simulate(r, 8192, 3000000, 1500000, T, 1600, E0);
        double Tm = 0, E = 0, p = 0;
        CHECK(fe_acquire(r, n, &Tm, &E, &p) == 0);
        CHECK(fabs(Tm - T) < 0.1);
        CHECK(fabs(fe_wrap(E - E0, T)) < 40);
    }
}
