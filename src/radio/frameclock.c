// Ours: the radio's TDD frame on our clock, and a radio clock read off it to
// some 10-20 us.
//
// The radio clock (BB_GET_AP_TIME) reads in whole ms; averaged once a second
// it wanders by +-75 us against our clock. The radio's frame runs in whole
// 0.64 us ticks; its edges (where its video part ends) come from the radio's
// TDD state word (radio/frame_edge.h): a second and a half of reads as fast
// as it answers to find them, then one read a frame aimed around the next
// (one a timer tick where the kernel's timers are coarser than that: 3.3 ms
// here, which spreads them over the frame instead), the edge and the period
// followed by a loop. The ms readings (every 100 ms) only say which edge is
// which: once, by their median over a few seconds; then edges are counted,
// through losing and finding the frame again too while the new ones are on
// the old ones' grid. They are not counted again from the ms readings: the
// frame runs ~7-10 ppm off the radio clock's ms (measured 2026-10-11), and
// counting again would move every time on this clock - the frame lock's with
// it. So this is the frame's clock, drifting from the radio clock's ms by
// some 10 us a second (logged). The ground gets the frame (KA_MSG_FRAME, once
// a second) and counts the same edges on its own radio, so both read one
// clock off them.
//
// A new frame - another width or frame mode, the link formed again - is
// found again from a quarter of a second of reads, the frames it can be
// (frame_edge.h's table) put on our clock by the count's rate, and the clock
// is carried onto its edges: no new count from the ms readings (seconds) as
// long as it stays within some ms of them. Meanwhile the last count goes on
// (the radio clock runs on through the change), so the stamps stay on it.
//
// An Open8030 radio says its own clock at its latest frame interrupt (IRQ 5,
// request 0x0c5a0035) - its 40 MHz timer, which its frame stamps give the
// ground's radio too. Each step reads it: when that IRQ 5 was on our clock (to
// the request's round trip) only says which of our edges it is; the edge,
// followed to some 10 us, gets the radio's clock. Nothing counted, from the ms
// readings or carried: the ground reads the same clock off the same frames.
#include "radio/frameclock.h"
#include "radio/frame_edge.h"
#include "radio/radio.h"
#include "radio/internal.h"
#include "ground/ground.h"
#include "common/clock.h"
#include "kestrel_air.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define ACQ_US    1500000    // reads as fast as the radio answers, to find the frame
#define UPDATE_US  500000    // the loop's step
#define KP 0.3               // share of the edge's error taken a step
#define KI 0.05              // ... and into the period, per frame
#define NAP 48               // ms readings kept
#define HOLD_US   7000000    // the last count used on while the frame is found again
#define CARRY_US 60000000    // ... and carried onto a new frame within this
#define DRIFT_MAX 5000.0     // while within this of the ms readings, us

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static int st_valid;
static double st_E, st_Tm, st_Tt, st_phi, st_until;
static int64_t st_m;
static unsigned st_gen;
static int reframe_req;      // the radio's width changed or its link formed (radio.c)

static int read_state(fe_read *r) {
    uint8_t in[4] = { 0x6e, 0x0e, 1, 0 }, out[256];
    const uint64_t t0 = mono_us();
    if (r_get(0x0064, in, 4, out, sizeof(out)) != 0) return -1;
    const uint64_t t1 = mono_us();
    r->t0 = t0;
    r->rtt = (uint32_t)(t1 - t0);
    r->video = (uint8_t)fe_is_video(out[0]);
    return 0;
}

static void sleep_until(uint64_t t) {
    struct timespec ts = { (time_t)(t / 1000000), (long)(t % 1000000) * 1000 };
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) != 0) {}
}

static int cmp_d(const void *a, const void *b) {
    const double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

// The clock: the radio clock at our time t is phi + m Tt + (t - E) Tt/Tm,
// until `until`.
static void publish(int valid, double E, double Tm, double Tt, double phi, int64_t m, unsigned gen, double until) {
    pthread_mutex_lock(&mtx);
    st_valid = valid; st_E = E; st_Tm = Tm; st_Tt = Tt; st_phi = phi; st_m = m; st_gen = gen; st_until = until;
    pthread_mutex_unlock(&mtx);
}

static void hold(double until) {
    pthread_mutex_lock(&mtx);
    if (st_valid && until > st_until) st_until = until;
    pthread_mutex_unlock(&mtx);
}

static void send_frame(double Tt, double phi, unsigned gen) {
    uint8_t m[11];
    const uint32_t t = (uint32_t)llround(Tt * 256), p = (uint32_t)llround(phi * 256);
    m[0] = KA_MSG_FRAME;
    m[1] = (uint8_t)gen;
    memcpy(m + 2, &t, 4);
    memcpy(m + 6, &p, 4);
    m[10] = 0;                       // radio frames a picture: no frame lock here
    ground_send(m, sizeof(m));
}

static void *fc_thread(void *arg) {
    (void)arg;
    enum { MAXR = 8192 };
    static fe_read buf[MAXR];
    double ap_mid[NAP], ap_us[NAP];
    unsigned gen = 0;
    // The count carried from one finding of the frame to the next: the clock
    // at its last good step.
    int carried = 0;
    double E_last = 0, Tm_last = 0, Tt_last = 0, phi_last = 0;
    int64_t m_last = 0;
    // How far the ms readings are from the count, us (checked every 30 s).
    double drift = 0;
    // Finding the frame again after a change: the table first.
    int quick = 0;
    // Open8030: the radio's clock read off its frame interrupts (above), the
    // AP clock unwrapped (us, 32 bits a reading), where its IRQ 5 falls from
    // our edge on average, and a pause after a radio that did not answer.
    const int o8030 = r_is_open8030();
    int stamped_before = 0, info_fails = 0;
    uint64_t s64 = 0, info_quiet_until = 0;
    int s_known = 0;
    double g_avg = 0;
    srand((unsigned)mono_us());
    struct timespec res;
    const int coarse_timer = clock_getres(CLOCK_MONOTONIC, &res) != 0 || res.tv_sec || res.tv_nsec > 100000;
    for (;;) {
        int n = 0, rc = -1;
        double Tm = 0, E = 0, pur = 0;
        __atomic_store_n(&reframe_req, 0, __ATOMIC_RELEASE);   // what is found now is after it
        // After a change: which of the frames it can be, on our clock by the
        // last count's rate (ours against the radio's holds through it).
        // Twice: reads across the change itself fit neither frame.
        for (int pass = 0; pass < 2 && rc && quick && carried; pass++) {
            n = 0;
            const uint64_t end = mono_us() + FE_PHASE_US;
            while (mono_us() < end && n < MAXR)
                if (read_state(&buf[n]) == 0) n++;
                else usleep(10000);
            const double rate = Tm_last / Tt_last;
            for (int i = -1; i < 6; i++) {
                const double T = (i < 0 ? Tt_last : fe_frames[i]) * rate;
                double e, p;
                if (fe_phase(buf, n, T, &e, &p) == 0 && p > pur) { pur = p; E = e; Tm = T; rc = 0; }
            }
            if (!rc) printf("frameclock: frame %.3f us of ours (%.2f radio), %d reads %.3f sorted\n", Tm,
                            round(Tm / FE_TICK_US) * FE_TICK_US, n, pur);
            else if (pass) printf("frameclock: not one of the known frames (%d reads), searching\n", n);
        }
        quick = 0;
        if (rc) {
            n = 0;
            const uint64_t acq_end = mono_us() + ACQ_US;
            while (mono_us() < acq_end && n < MAXR)
                if (read_state(&buf[n]) == 0) n++;
                else usleep(10000);
            rc = fe_acquire(buf, n, &Tm, &E, &pur);
            if (rc) {
                static uint64_t logged;
                if (mono_us() - logged > 30000000) {
                    printf("frameclock: no frame found (%d, %d reads, %.3f sorted)\n", rc, n, pur);
                    logged = mono_us();
                }
                sleep(2);
                continue;
            }
            printf("frameclock: frame %.3f us of ours (%.2f radio), %d reads %.3f sorted, found in %llu ms\n", Tm,
                   round(Tm / FE_TICK_US) * FE_TICK_US, n, pur, (unsigned long long)(mono_us() - acq_end) / 1000);
        }
        double Tt = round(Tm / FE_TICK_US) * FE_TICK_US, phi = 0;
        int64_t m = 0;
        int aligned = 0, onto = 0, good = 0, misses = 0, nap = 0, api = 0;
        if (carried && fabs(Tm - Tm_last) < 0.5) {
            const double k = (E - E_last) / Tm_last;
            if (fabs(k - round(k)) < 0.05) {                 // the same edges as before
                m = m_last + llround(k);
                phi = phi_last;
                aligned = 1;
                printf("frameclock: edges counted on from before (%.1f s)\n", (E - E_last) / 1e6);
            }
        }
        if (!aligned && carried && E - E_last < CARRY_US) {
            if (fabs(drift) < DRIFT_MAX) onto = 1;
            else printf("frameclock: the count is %.1f ms from the ms readings: counted from them again\n", drift / 1000);
        }
        uint64_t last_up = mono_us(), last_ap = 0, last_msg = 0, last_check = 0, last_log = 0;
        n = 0;
        for (;;) {
            if (__atomic_exchange_n(&reframe_req, 0, __ATOMIC_ACQ_REL)) {
                printf("frameclock: the radio's frame may have changed, finding it again\n");
                quick = 1;
                break;
            }
            const uint64_t now = mono_us();
            // One read a frame, aimed from 0.8 ms before the next edge to 0.2 ms
            // after it: a read takes 0.3-0.5 ms, so most bracket the edge.
            // Wider until the edge has been measured a few times.
            const double En = E + ceil(((double)now + 2000 - E) / Tm) * Tm;
            if (coarse_timer) usleep(500);
            else sleep_until((uint64_t)(good < 4 ? En - 1100 + rand() % 1600 : En - 800 + rand() % 1000));
            if (n < MAXR && read_state(&buf[n]) == 0) n++;
            const uint64_t t = mono_us();
            if (t - last_ap >= 100000) {
                uint32_t ms;
                const uint64_t a0 = mono_us();
                if (radio_wireless_time(&ms) == 0) {
                    const uint64_t a1 = mono_us();
                    if (a1 - a0 < 2000) {
                        ap_mid[api] = (double)(a0 + a1) / 2;
                        ap_us[api] = (double)ms * 1000 + 500;
                        api = (api + 1) % NAP;
                        if (nap < NAP) nap++;
                    }
                }
                last_ap = t;
            }
            if (t - last_up < UPDATE_US) continue;
            // The loop: E to the last edge before these reads, then the error.
            const double N = ((double)t - (double)last_up) / Tm;
            last_up = t;
            if (n) {
                const double j = floor(((double)buf[0].t0 - E) / Tm);
                if (j > 0) { E += j * Tm; m += (int64_t)j; }
            }
            double s = 0, w = 0;
            if (n && fe_measure(buf, n, E, Tm, &s, &w) == 0 && fabs(s) < (good < 4 ? 1000 : 400)) {
                E += KP * s;
                Tm += KI * s / N;
                good++;
                misses = 0;
            } else {
                misses++;
                if (misses == 2) {
                    double L = -1e9, U = 1e9;
                    int nl = 0, nu = 0;
                    for (int i = 0; i < n; i++) {
                        const double x0 = fe_wrap((double)buf[i].t0 - E, Tm), x1 = x0 + buf[i].rtt;
                        if (x0 < -FE_NEAR || x1 > FE_NEAR) continue;
                        if (buf[i].video) { nl++; if (x0 > L) L = x0; } else { nu++; if (x1 < U) U = x1; }
                    }
                    printf("frameclock: edge not measured: %d reads, %d video (last from %+.0f us), %d after (first back %+.0f us)\n",
                           n, nl, L, nu, U);
                }
            }
            n = 0;
            if (misses >= 2) {
                printf("frameclock: lost the frame, finding it again\n");
                quick = 1;
                break;
            }
            Tt = round(Tm / FE_TICK_US) * FE_TICK_US;
            const double k = Tt / Tm;
            // A new frame: the clock carried onto its edges - where its video
            // part ends on it, which edge is which - from the last count at
            // these edges' time. The ground does the same from its own.
            if (onto && good >= 3) {
                const double v = phi_last + (double)m_last * Tt_last + (E - E_last) * (Tt_last / Tm_last);
                phi = fmod(v, Tt);
                if (phi < 0) phi += Tt;
                m = llround((v - phi) / Tt);
                aligned = 1;
                onto = 0;
                gen++;
                last_msg = 0;
                printf("frameclock: the clock carried onto a new frame (%.2f -> %.2f us, video ends at %.1f)\n",
                       Tt_last, Tt, phi);
            }
            // Open8030: the radio's clock at its latest IRQ 5, on the edge
            // nearest it.
            int stamped = 0;
            if (o8030 && t >= info_quiet_until) {
                r_frame_info_t si;
                const uint64_t r0 = mono_us();
                const int rc = r_frame_info(&si);
                const uint64_t r1 = mono_us();
                if (rc == 0 && (si.flags & 2) && si.frame_ticks && r1 - r0 < 3000) {
                    info_fails = 0;
                    const double irq5 = (double)(r0 + r1) / 2 - (double)si.since_us;
                    const double kk = round((irq5 - E) / Tm);
                    const double g = irq5 - (E + kk * Tm);
                    const double sTt = si.frame_ticks * FE_TICK_US;
                    if (fabs(sTt - Tt) < 0.5 && fabs(g) < Tm / 2 - 300) {
                        // 32 bits of us (71.6 min), unwrapped.
                        if (!s_known) s64 = si.ap_us_lo;
                        else s64 += (uint64_t)(int64_t)(int32_t)(si.ap_us_lo - (uint32_t)s64);
                        s_known = 1;
                        const double vE = (double)s64 - kk * sTt;   // at our edge E
                        m = (int64_t)floor(vE / sTt);
                        Tt = sTt;
                        phi = vE - (double)m * Tt;
                        if (!aligned || !stamped_before) {
                            gen++;
                            last_msg = 0;
                            printf("frameclock: the radio clock from the radio's frame interrupts (frame %.2f us, IRQ 5 %+.0f us from our edge)\n",
                                   Tt, g);
                        }
                        aligned = 1;
                        onto = 0;
                        drift = 0;
                        stamped = 1;
                        g_avg = stamped_before ? g_avg + (g - g_avg) / 16 : g;
                    }
                } else if (rc != 0 && ++info_fails >= 3) {
                    info_fails = 0;
                    info_quiet_until = t + 30000000;
                }
            }
            stamped_before = stamped;
            // Which edge is which, from the ms readings: once. Later, only how
            // far those have drifted from the count (logged).
            if (good >= 6 && nap >= 20 && (!aligned || t - last_check >= 30000000)) {
                double a[NAP];
                for (int i = 0; i < nap; i++) a[i] = ap_us[i] - (ap_mid[i] - E) * k - (double)m * Tt;
                qsort(a, (size_t)nap, sizeof(double), cmp_d);
                const double off = a[nap / 2];                // radio clock at edge 0 of our count
                if (!aligned) {
                    phi = fmod(off, Tt);
                    if (phi < 0) phi += Tt;
                    m += llround((off - phi) / Tt);
                    aligned = 1;
                    onto = 0;
                    drift = 0;
                    gen++;
                    printf("frameclock: radio clock on the frame's edges (frame %.2f us, video ends at %.1f)\n", Tt, phi);
                } else {
                    drift = off - phi;
                    printf("frameclock: the radio clock's ms readings are %+.0f us from the edges' count\n", drift);
                }
                last_check = t;
            }
            if (aligned && misses == 0) {
                publish(1, E, Tm, Tt, phi, m, gen, (double)t + 3e6);
                carried = 1; E_last = E; Tm_last = Tm; Tt_last = Tt; m_last = m; phi_last = phi;
            }
            // Always: the ground's latency figures (and the screen lock's
            // captures) are on the radio clock through it - without it they
            // drift with the ground's coarse copy, ~10-20 us a second.
            if (aligned && t - last_msg >= 1000000) {
                send_frame(Tt, phi, gen);
                last_msg = t;
            }
            if (t - last_log >= 30000000) {
                printf("frameclock: frame %.3f us of ours, edge %+.1f us (bounds %.0f apart)%s\n", Tm, s, w,
                       stamped ? ", on the radio's frame interrupts" : aligned ? "" : ", not counted yet");
                if (stamped) printf("frameclock: the radio's IRQ 5 %+.1f us from our edge on average\n", g_avg);
                last_log = t;
            }
        }
        // The last count goes on while the frame is found again.
        if (quick && carried) hold(E_last + HOLD_US);
    }
    return NULL;
}

void frameclock_reframe(void) {
    __atomic_store_n(&reframe_req, 1, __ATOMIC_RELEASE);
}

void frameclock_start(void) {
    pthread_t t;
    if (pthread_create(&t, NULL, fc_thread, NULL) == 0) pthread_detach(t);
}

int frameclock_ap_us(uint64_t t, uint64_t *ap) {
    pthread_mutex_lock(&mtx);
    const int ok = st_valid && (double)t < st_until && (double)t + 3e6 > st_E;
    double v = 0;
    if (ok) v = st_phi + (double)st_m * st_Tt + ((double)t - st_E) * (st_Tt / st_Tm);
    pthread_mutex_unlock(&mtx);
    if (!ok || v < 0) return -1;
    *ap = (uint64_t)llround(v);
    return 0;
}

int frameclock_frame(double *period, double *phase, unsigned *gen) {
    pthread_mutex_lock(&mtx);
    const int ok = st_valid;
    *period = st_Tt;
    *phase = st_phi;
    *gen = st_gen;
    pthread_mutex_unlock(&mtx);
    return ok ? 0 : -1;
}
