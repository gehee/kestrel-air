// The video link's bandwidth: the ground's requests and cap, the automatic
// step to 40 MHz (KA_BW40=1), and undoing a change that lost the link.
#include "radio/internal.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "app/config.h"
#include "app/state.h"
#include "common/clock.h"
#include "app/settings.h"

// Ours: the video link's transmit bandwidth, at the goggle's request (sky cmd
// 0x24, gear + 1; 0 is stock's and means leave it). The video is the AP's
// broadcast link ("br" in bb_config_sky*.json, 5 MHz there), so slot AP,
// direction TX: BB_SET_BANDWIDTH {slot, dir, gear}, gear as the SDK's
// bb_bandwidth_e - 0..5 = 1.25, 2.5, 5, 10, 20, 40 MHz. The radio also widens
// the link by itself after link-up (bandwidth events 2 -> 3 -> 4, 2026-10-04).
// Not saved: a reboot is back on the config's. If the link is not back 3 s
// after a change, the change is undone here, since the goggle cannot reach us.

static int r_set_bandwidth(int gear) {
    uint8_t b[3] = { 0, 0, (uint8_t)gear };
    int r = r_set(0x0016, b, 3);
    printf("radio: video link bandwidth gear %d -> %d\n", gear, r);
    return r;
}

static int bw_max = 5;   // the goggle's cap (radio_set_max_bw); every request keeps to it
static uint64_t bw_up_backoff_until;   // a switch up that lost the link: not again before this
static uint64_t bw_up_backoff_ms = 60000;

void radio_bandwidth_request(int gear) {
    if (gear > bw_max) gear = bw_max;
    if (gear < 0 || gear > 5 || gear == r_bw_cur) return;
    int prev = r_bw_cur;
    if (r_set_bandwidth(gear) != 0) return;
    r_bw_prev = prev;
    r_bw_cur = gear;
    r_bw_at = mono_ms();
}

// Ours: the goggle's cap on the video link's bandwidth (sky cmd 0x41, MHz:
// 20, or 40 / 0 for none). At 20 the link never goes up to 40 on its own
// (bw_auto), and a link on 40 comes down at once. Not saved here: the goggle
// sends it again at every link-up.
void radio_set_max_bw(int mhz) {
    bw_max = (mhz > 0 && mhz < 40) ? 4 : 5;
    printf("radio: video link bandwidth cap %d MHz\n", bw_max == 4 ? 20 : 40);
    if (r_connected && r_bw_cur > bw_max) radio_bandwidth_request(bw_max);
}

// Ours: the goggle's cap on the video bitrate (sky cmd 0x40, kbps, 0 = none).
// Not saved here: the goggle sends it again at every link-up.
void radio_set_max_kbps(int kbps) {
    r_max_kbps = kbps > 0 ? kbps : 0;
    printf("radio: video bitrate cap %d kbps%s\n", r_max_kbps, r_max_kbps ? "" : " (none)");
}

// Ours: 40 MHz on its own. The radio widens the video link by itself up to the
// 20 MHz of its config (gear 4) and no further; at 40 MHz (gear 5) MCS 10 gives
// 34.5 Mbps against 25.9 (2026-10-05). Up to 40 MHz after 5 s at MCS 8 or more
// on 20; back to 20 after 1 s at MCS 3 or less on 40, and then not up again for
// 30 s. Each change costs ~3 s of picture while the MCS climbs back from the
// bottom, so 8 s after going up are left alone. Off unless KA_BW40=1 (2026-10-05:
// every step to or from 40 re-forms the link - about 0.7 s without picture on
// the goggle - so for now the link stays on the radio's own 20); the goggle's
// cap at 20 MHz (radio_set_max_bw) also keeps it there.
int radio_bw40_enabled(void) {
    static int on = -1;
    if (on < 0) on = env_int("KA_BW40", 0) == 1;
    return on;
}

static void r_bw_auto(uint64_t t) {
    static uint64_t good, bad, settle, cooldown;
    if (!radio_bw40_enabled() || !r_connected || r_bw_prev >= 0) { good = bad = 0; return; }
    // Above the goggle's cap however it got there: down to it.
    if (r_bw_cur > bw_max) { radio_bandwidth_request(bw_max); good = bad = 0; return; }
    if (r_bw_cur == 4) {
        bad = 0;
        if (r_mcs < 8 || t < cooldown || t < bw_up_backoff_until || bw_max < 5) { good = 0; return; }
        if (!good) good = t;
        if (t - good < 5000) return;
        printf("radio: MCS %d at 20 MHz for 5 s - up to 40 MHz\n", r_mcs);
        radio_bandwidth_request(5);
        settle = t + 8000;
        good = 0;
    } else if (r_bw_cur == 5) {
        good = 0;
        if (t < settle || r_mcs > 3) { bad = 0; return; }
        if (!bad) bad = t;
        if (t - bad < 1000) return;
        printf("radio: MCS %d at 40 MHz for 1 s - back to 20 MHz\n", r_mcs);
        radio_bandwidth_request(4);
        cooldown = t + 30000;
        bad = 0;
    }
}

// Every 100 ms (the timer thread): a change of ours that lost the link is
// undone after 3 s, then the automatic step to 40 MHz.
void r_bw_tick(uint64_t t) {
    if (r_bw_prev >= 0 && t - r_bw_at >= 3000) {
        const int tried = r_bw_cur;
        if (!r_connected) {
            // Back - but never above the goggle's cap (a cap-driven drop
            // that lost the link would otherwise return to 40).
            int back = r_bw_prev > bw_max ? bw_max : r_bw_prev;
            printf("radio: no link 3 s after the bandwidth change - back to gear %d\n", back);
            if (back != r_bw_cur && r_set_bandwidth(back) == 0) r_bw_cur = back;
            // A switch up that lost the link: not again for a while, longer
            // each time (60 s, 120 s, ... 16 min), or it cycles outages.
            if (tried > r_bw_prev) {
                bw_up_backoff_until = t + bw_up_backoff_ms;
                printf("radio: not up to 40 MHz again for %llu s\n",
                       (unsigned long long)(bw_up_backoff_ms / 1000));
                if (bw_up_backoff_ms < 960000) bw_up_backoff_ms *= 2;
            }
        } else if (tried > r_bw_prev) {
            bw_up_backoff_ms = 60000;   // it held: the next failure starts over
        }
        r_bw_prev = -1;
    }
    r_bw_auto(t);
}
