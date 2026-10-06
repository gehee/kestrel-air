// The air side of the AR8030 radio, as the stock air app
// runs it: the same radio requests with the same payloads in the same
// order, the same event handling and timers, the same periodic reads. Worked
// out by reverse-engineering it, and checked against a trace of its radio traffic.
//
// Boards: the Caddx Ascent Lite (stock board type 482, "prj 4") and Lite+ (472,
// "prj 7"), see unit/model.c; branches of the stock code for other boards
// are left out.
#include "video/video.h"
#include "camera/image.h"
#include "app/state.h"
#include "radio/radio.h"
#include "radio/internal.h"
#include "unit/model.h"

#include <fcntl.h>
#include <pthread.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "app/config.h"
#include "common/clock.h"

bbc_sock radio_video, radio_ctrl;
volatile int radio_init_failed;
volatile uint32_t radio_total_video_send;

static bbc_ioctl_conn io;
radio_hooks r_hooks;

static uint8_t own_mac[4];
int r_nfreq;
uint32_t r_freq[128];

volatile int r_connected, r_mcs;
static volatile int throughput, reconnect_count;
static int prev_mcs = 20;
static int fps_coef = 10;

// Frequencies and channel plan.
int r_is_hop, r_hop_en, r_freq_pinned;
uint32_t r_air_freq;
int r_gnd_n;
// The radio board's common work channel (RF board types 0x10/0x20/0x60).
const uint32_t r_common_list[] = { 5839000 };

// Retransmission pressure (event 13 and its 20 ms timer).
static volatile int retx_busy, retx_flag, busy_latched;
static uint64_t retx_flag_ts, t_ratio;

// Relay mode: the unit linked through a relay box, which reports its own
// downstream throughput, video backlog and radio ring (SG message 0x2d).
static volatile int relay_thr, relay_vbuf, relay_bb_left, relay_mcs;
static volatile uint64_t relay_t;

// Pairing in progress: the battery ADC is not read meanwhile.
volatile int r_pairing;

// Timers: the delayed disconnect after a link-down.
static uint64_t disconnect_at;
// Ours: the video link's bandwidth gear (bb_bandwidth_e, kept from the radio's
// own bandwidth events) and a change of ours waiting to prove itself; the
// goggle's cap on the video bitrate, kbps, 0 = none (see radio_tgt_bitrate).
volatile int r_bw_cur = 2, r_bw_prev = -1;
volatile uint64_t r_bw_at;
volatile int r_max_kbps;

// ADC (battery) and the register block values.
int r_adc_state;
static int have_ofs;
static int8_t ofs_a, ofs_b;

int r_get(uint16_t id, const void *in, uint32_t inlen, void *out, uint32_t outlen) {
    return bbc_ioctl(&io, 0x01000000 | id, in, inlen, out, outlen, NULL, IOCTL_MS);
}

int r_set(uint16_t id, const void *in, uint32_t inlen) {
    return bbc_ioctl(&io, 0x02000000 | id, in, inlen, NULL, 0, NULL, IOCTL_MS);
}

// BB_SET_PRJ_DISPATCH: a 256-byte command block, sub-command in byte 0.
int r_dispatch(uint8_t cmd, const uint8_t *args, int nargs) {
    uint8_t b[256];
    memset(b, 0, sizeof(b));
    b[0] = cmd;
    if (nargs) memcpy(b + 4, args, nargs);
    return r_set(0x00c8, b, sizeof(b));
}

void r_set_rf_path_b(int enable) {
    // Stock sends 256 bytes from a 17-byte buffer; the rest is heap
    // leftovers there, zeros here.
    uint8_t a[3] = { 0x01, 0x03, (uint8_t)enable };
    r_dispatch(0xcb, a, 3);
}

// The Lite+'s front-end module: high power (on) or low (off), through a GPIO
// of the radio chip (PRJ_CMD_SET_GPIO), as stock's "set fem ctrl".
void r_set_fem(int on) {
    uint8_t a[3] = { 0x83, 0x50, (uint8_t)(on ? 2 : 0) };
    if (r_dispatch(0x0e, a, 3)) printf("radio: set fem ctrl to %d failed\n", on);
    else printf("radio: set fem ctrl to %d\n", on);
}

void r_set_adc_meas(int chn, uint32_t period_ms) {
    uint8_t a[8] = { (uint8_t)chn, 0, 0, 0 };
    memcpy(a + 4, &period_ms, 4);
    r_dispatch(0x8a, a, 8);
}

int r_get_adc(int chn) {
    uint8_t in[256], out[256];
    memset(in, 0, sizeof(in));
    in[0] = 0x89;
    in[4] = (uint8_t)chn;
    if (r_get(0x00c8, in, sizeof(in), out, sizeof(out)) != 0) return 0;
    uint32_t v;
    memcpy(&v, out + 4, 4);
    return (int)v;
}

// BB_GET_MCS: reply [0] is the MCS plus 2, u32 [4] the throughput in kbps.
static int get_mcs(int *m, int *thr) {
    uint8_t in[2] = { 0, 0 }, r[8];
    if (r_get(0x0006, in, 2, r, 8) != 0) return -1;
    *m = r[0] - 2;
    if (thr) memcpy(thr, r + 4, 4);
    return 0;
}

void radio_set_mcs_policy(void) {
    // The MCS table, as stock reloads it: {u32 0, mcs, 2, snr_up, snr_dw, ldpc_up, ldpc_dw,
    // up_keep_ms, dw_keep_ms}
    static const uint8_t tab[3][16] = {
        { 0, 0, 0, 0, 1, 2, 0x42, 0, 0x2f, 0, 2, 4, 0xe8, 3, 0xf4, 1 },
        { 0, 0, 0, 0, 2, 2, 0x83, 0, 0x5d, 0, 2, 3, 0xf4, 1, 0x0a, 0 },
        { 0, 0, 0, 0, 5, 2, 0xee, 0, 0xa9, 0, 2, 4, 0xf4, 1, 0x1e, 0 },
    };
    int strategy = cfg_get("video_strategy", 0);
    for (int i = 0; i < 3; i++) r_set(0x0024, tab[i], 16);
    uint8_t range[4] = { 0, (uint8_t)(strategy == 2 ? 1 : 2), 2, 0 };
    r_set(0x001b, range, 4);
    uint8_t kikp = strategy == 2;
    r_dispatch(0x8c, &kikp, 1);
}

void radio_set_uart_mode(int mode) {
    uint8_t a[2] = { 2, (uint8_t)mode };
    r_dispatch(0x8f, a, 2);
}

// ---- events ---------------------------------------------------------------

// Stock pushes every event into one queue and handles them on one thread,
// in the order they came.
#define EVQ 32
static struct { int type; uint8_t d[136]; uint32_t len; } evq[EVQ];
static int evq_head, evq_tail;
static pthread_mutex_t evq_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t evq_cv = PTHREAD_COND_INITIALIZER;
static bbc_event ev[6];

static void on_event(const uint8_t *d, uint32_t len, void *arg) {
    pthread_mutex_lock(&evq_mtx);
    int n = (evq_tail + 1) % EVQ;
    if (n != evq_head) {
        evq[evq_tail].type = (int)(intptr_t)arg;
        evq[evq_tail].len = len < sizeof(evq[0].d) ? len : sizeof(evq[0].d);
        if (d) memcpy(evq[evq_tail].d, d, evq[evq_tail].len);
        evq_tail = n;
        pthread_cond_signal(&evq_cv);
    }
    pthread_mutex_unlock(&evq_mtx);
}

static void handle_event(int type, const uint8_t *d, uint32_t len) {
    int m, thr;
    switch (type) {
    case 0:   // link: {slot, state, previous}
        if (len < 2) break;
        relay_thr = 0;                        // relay mode off, on every link event
        printf("radio: link slot %d state %d\n", d[0], d[1]);
        if (d[1] == 2) {
            if (get_mcs(&m, &thr) == 0) {
                r_mcs = m;
                throughput = thr;
                prev_mcs = m;
            }
            disconnect_at = 0;
            if (r_air_freq) radio_set_freq(r_is_hop, r_air_freq);
            radio_total_video_send = 0;
            reconnect_count++;
            r_connected = 1;
            video_link(1);
        } else {
            if (!disconnect_at) disconnect_at = mono_ms() + 1000;   // connected stays set for up to 1 s
            video_link(0);
        }
        break;
    case 1:   // MCS changed
        if (get_mcs(&m, &thr) == 0) {
            r_mcs = m;
            throughput = thr;
            printf("radio: mcs=%d, throughput=%d kbps\n", m, thr);
            if (m < prev_mcs && r_hooks.set_bitrate) {   // down at once
                int kbps = (int)(radio_tgt_bitrate() * 0.7);
                r_hooks.set_bitrate(kbps < 128 ? 128 : kbps);
            }
            prev_mcs = m;
        }
        break;
    case 12:  // bandwidth changed
        if (get_mcs(&m, &thr) == 0) {
            r_mcs = m;
            throughput = thr;
            prev_mcs = m;
        }
        if (len >= 3 && d[0] == 0 && d[1] == 0 && d[2] <= 5) r_bw_cur = d[2];
        printf("radio: bandwidth changed (event %u bytes:", len);
        for (uint32_t i = 0; i < len && i < 8; i++) printf(" %02x", d[i]);
        printf("), mcs=%d, throughput=%d kbps\n", r_mcs, throughput);
        break;
    case 2:   // channel changed: {slot, dir, cur, prev}
        if (len >= 4 && !r_hop_en && r_in_list(r_air_freq, r_common_list, 1) && d[1] == 1)
            r_set_work_chan_list();
        break;
    case 13:  // retransmissions: [5] busy level, [6] 1 start / 0 end
        if (len >= 7) {
            if (d[6] == 1) {
                retx_flag = 1;
                retx_flag_ts = mono_ms();
            }
            retx_busy = d[5];
        }
        break;
    default:  // 9: ignored by stock
        break;
    }
}

static void *event_thread(void *arg) {
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&evq_mtx);
        while (evq_head == evq_tail) pthread_cond_wait(&evq_cv, &evq_mtx);
        int type = evq[evq_head].type;
        uint32_t len = evq[evq_head].len;
        uint8_t d[136];
        memcpy(d, evq[evq_head].d, len);
        evq_head = (evq_head + 1) % EVQ;
        pthread_mutex_unlock(&evq_mtx);
        handle_event(type, d, len);
    }
    return NULL;
}

// The delayed disconnect and the retransmission timer. Stock's timer thread
// wakes every 100 ms, so its "20 ms" timer and the 1 s delay both run on
// 100 ms ticks - as here.
static void *timer_thread(void *arg) {
    (void)arg;
    for (;;) {
        usleep(100000);
        uint64_t t = mono_ms();
        if (disconnect_at && t >= disconnect_at) {
            disconnect_at = 0;
            r_is_hop = 0;
            r_air_freq = 0;
            r_connected = 0;
            r_mcs = 0;
            relay_thr = 0;
            printf("radio: link down\n");
        }
        if (t - t_ratio >= 80) {
            busy_latched = retx_busy;
            t_ratio = t;
        }
        if (retx_flag == 1 && t - retx_flag_ts >= 40) retx_flag = 0;
        if (busy_latched == 1 && t - t_ratio >= 40) busy_latched = 0;
        r_bw_tick(t);
    }
    return NULL;
}

int radio_retx_too_many(void) {
    if (busy_latched) return 1;
    if (retx_flag) {
        retx_flag = 0;
        return 1;
    }
    return 0;
}

// ---- what the video path asks ---------------------------------------------

int radio_connected(void) { return r_connected; }
int radio_mcs(void) { return r_mcs; }
int radio_throughput(void) { return throughput; }
int radio_reconnect_count(void) { return reconnect_count; }
void radio_set_fps_coef(int coef) { fps_coef = coef; }

// The target bitrate: a share of the radio's throughput by MCS.
int radio_relay_mode(void) { return mono_ms() - relay_t <= 5000 && relay_thr != 0; }
int radio_relay_vbuf(void) { return relay_vbuf; }
int radio_relay_bb_left(void) { return relay_bb_left; }

// SG 0x2d sub 10 from a relay: its throughput (20 bits), queued video and
// radio ring (24 bits each), MCS.
void radio_relay_update(const uint8_t *b, int n) {
    relay_t = mono_ms();
    if (!r_connected || n < 10) {
        relay_thr = relay_vbuf = relay_bb_left = relay_mcs = 0;
        return;
    }
    relay_thr = b[0] | b[1] << 8 | (b[2] & 0x0f) << 16;
    relay_vbuf = b[3] | b[4] << 8 | b[5] << 16;
    relay_bb_left = b[6] | b[7] << 8 | b[8] << 16;
    relay_mcs = b[9];
}

// The target bitrate: a share of the radio's throughput by MCS
// (through a relay: of the weaker hop).
int radio_tgt_bitrate(void) {
    static const struct { int thr, pct; } tab[] = {
        { -2, 35 }, { -1, 50 }, { 0, 70 }, { 3, 70 }, { 5, 70 }, { 6, 70 }, { 8, 70 }, { 10, 70 },
    };
    int strategy = cfg_get("video_strategy", 0), kbps = 0, thr = throughput, m = r_mcs;
    int relay = radio_relay_mode() ? relay_thr : 0;
    if (!thr) puts("radio: no throughput figure");
    for (int i = (int)(sizeof(tab) / sizeof(tab[0])) - 1; i >= 0; i--) {
        int pct = tab[i].pct;
        if (strategy == 1) pct = tab[i].thr >= 0 ? 60 : tab[i].thr == -1 ? 40 : 30;
        if (strategy == 2 && tab[i].thr == -1) pct = 70;
        if (tab[i].thr <= m && thr) {
            kbps = thr * pct / 100;
            relay = relay * pct / 100;
            break;
        }
    }
    if (!kbps) kbps = 128;
    kbps = relay > 0 && relay < kbps ? relay * fps_coef / 10 : kbps * fps_coef / 10;
    if (strategy == 0) kbps = kbps * 82 / 100;
    if (r_max_kbps > 0 && kbps > r_max_kbps) kbps = r_max_kbps;   // ours: the goggle's cap
    return kbps > 20000 ? 20000 : kbps;
}

int radio_ringbuf_left(void) {
    uint8_t in[4] = { 0, 3, 0, 0 }, r[648];
    if (r_get(0x0011, in, 4, r, sizeof(r)) != 0) return -1;
    uint32_t v;
    memcpy(&v, r + 0x100, 4);
    return (int)v;
}

int radio_wireless_time(uint32_t *ms) {
    uint8_t r[4];
    if (r_get(0x000c, NULL, 0, r, 4) != 0) return -1;
    memcpy(ms, r, 4);
    return 0;
}

// ---- periodic reads (for the ground telemetry) ----------------------------

static int read_reg(uint16_t reg, uint8_t *reply) {
    uint8_t in[4] = { (uint8_t)reg, (uint8_t)(reg >> 8), 1, 0 };
    if (r_get(0x0064, in, 4, reply, 256) != 0) return -1;
    return reply[0];
}

// A register write: stock's request buffer holds, past the value, whatever
// the register read before it left there - so does this one.
static void write_reg(uint16_t reg, uint8_t v, const uint8_t *prev_reply) {
    uint8_t b[260];
    b[0] = (uint8_t)reg;
    b[1] = (uint8_t)(reg >> 8);
    b[2] = 1;
    b[3] = 0;
    b[4] = v;
    memcpy(b + 5, prev_reply + 1, 255);
    r_set(0x0064, b, sizeof(b));
}

void radio_period_info(radio_period *p) {
    uint8_t r[256];
    int v;
    memset(r, 0, sizeof(r));
    p->tssi_a = (uint8_t)read_reg(0x0e7c, r);
    p->tssi_b = (uint8_t)read_reg(0x0e7e, r);
    v = read_reg(0x080b, r);
    write_reg(0x080b, (uint8_t)(v & 0xf7), r);            // path A
    p->gain_a = (uint8_t)read_reg(0x0e5d, r) & 0x3f;
    v = read_reg(0x080b, r);
    write_reg(0x080b, (uint8_t)(v | 0x08), r);            // path B
    p->gain_b = (uint8_t)read_reg(0x0e5d, r) & 0x3f;
    if (!have_ofs) {
        uint8_t in[4] = { 2, 0, 0, 0 }, o[4];
        if (r_get(0x0071, in, 4, o, 4) == 0) {
            ofs_a = (int8_t)o[0];
            ofs_b = (int8_t)o[1];
            have_ofs = 1;
        }
    }
    p->ofs_a = ofs_a;
    p->ofs_b = ofs_b;
}

int radio_chan_info(uint8_t *reply, int max) {
    uint32_t got = 0;
    if (bbc_ioctl(&io, 0x0100000a, NULL, 0, reply, max, &got, IOCTL_MS) != 0) return -1;
    // The channel monitor: back to the ground's channel if the radio left it.
    if (!r_hop_en && r_connected && r_freq_pinned == 1 && reply[3] < r_nfreq && r_freq[reply[3]] != r_air_freq) {
        printf("radio: off the ground's channel - back to it\n");
        radio_set_freq(0, r_air_freq);
    }
    return (int)got;
}

// ---- start-up -------------------------------------------------------------

static int subscribe_all(void) {
    static const int order[6] = { 1, 0, 2, 9, 12, 13 };
    for (int i = 0; i < 6; i++)
        if (bbc_subscribe(&ev[i], order[i], on_event, (void *)(intptr_t)order[i])) return -1;
    return 0;
}

int radio_start(const radio_hooks *hk) {
    uint8_t st[300], ci[1028];
    pthread_t t;

    if (hk) r_hooks = *hk;
    cfg_load();

    while (bbc_hello() <= 0) sleep(1);
    if (bbc_ioctl_connect(&io)) return -1;

    uint8_t q[2] = { 0xff, 0x03 };
    while (r_get(0x0000, q, 2, st, sizeof(st)) != 0) usleep(100000);   // BB_GET_STATUS-1
    if (r_get(0x0000, q, 2, st, sizeof(st)) != 0) return -1;            // BB_GET_STATUS-2
    if (st[0] != 0) {
        fprintf(stderr, "radio: not the AP role (%d)\n", st[0]);
        return -1;
    }
    memcpy(own_mac, st + 6, 4);
    printf("radio: mac %02X %02X %02X %02X, link state %d\n",
           own_mac[0], own_mac[1], own_mac[2], own_mac[3], st[0xfc]);

    memset(ci, 0, sizeof(ci));
    if (bbc_ioctl(&io, 0x0100000a, NULL, 0, ci, sizeof(ci), NULL, IOCTL_MS) != 0) return -1;
    r_nfreq = ci[0] > 128 ? 128 : ci[0];
    memcpy(r_freq, ci + 4, r_nfreq * 4);
    printf("radio: %d channels, %s\n", r_nfreq, ci[1] ? "auto" : "manual");
    r_init_comm_work_chan();

    // Already linked: take the throughput too. Stock leaves it at 0 here, so
    // the bitrate sits at 128 kbps until the radio next reports an MCS change -
    // on a steady link, indefinitely (seen for minutes on the bench).
    if (st[0xfc] == 2) {
        int m, thr;
        if (get_mcs(&m, &thr) == 0) {
            r_connected = 1;
            r_mcs = m;
            throughput = thr;
            prev_mcs = m;
        }
    }

    pthread_create(&t, NULL, event_thread, NULL);
    pthread_detach(t);
    if (subscribe_all()) return -1;

    // Socket ports 3 (video) and 2 (ground messages): stock's buffer sizes.
    if (bbc_open(&radio_video, 0, 3, 5, 0x800, NULL, NULL)) return -1;
    if (bbc_open(&radio_ctrl, 0, 2, 2, 0x800, NULL, NULL)) return -1;

    r_set_rf_path_b(0);
    r_set_adc_meas(3, 200);
    r_adc_state = 1;
    if (model_lite_plus()) r_set_fem(0);

    // Retransmission events: window 10, stock's defaults.
    uint8_t retx[136];
    memset(retx, 0, sizeof(retx));
    retx[0] = 10; retx[1] = 6; retx[2] = 4; retx[3] = 2;
    int wv[5], nw = cfg_json_ints("/factory/fpv_debug_cfg.json", "retx_win", wv, 1);
    if (nw == 1 && wv[0]) {
        int pv[4];
        retx[0] = (uint8_t)wv[0];
        if (cfg_json_ints("/factory/fpv_debug_cfg.json", "retx_param", pv, 4) == 4)
            for (int i = 0; i < 4; i++) retx[1 + i] = (uint8_t)pv[i];
    }
    if (r_set(0x0026, retx, sizeof(retx)) == 0) {
        pthread_create(&t, NULL, timer_thread, NULL);
        pthread_detach(t);
    }

    radio_set_mcs_policy();

    // The paired ground's MAC, and the TX power offsets.
    uint8_t cand[402];
    int mac[4];
    memset(cand, 0, sizeof(cand));
    cand[1] = 1;
    if (cfg_json_ints("/factory/user_cfg.json", "bb_mac_addr_0", mac, 4) == 4) {
        int all0 = 1, allf = 1;
        for (int i = 0; i < 4; i++) {
            all0 &= (mac[i] & 0xff) == 0;
            allf &= (mac[i] & 0xff) == 0xff;
        }
        if (!all0 && !allf)
            for (int i = 0; i < 4; i++) cand[2 + i] = (uint8_t)mac[i];
    }
    r_set(0x0004, cand, sizeof(cand));

    int off[4] = { 0, 0, 0, 0 };
    if (cfg_json_ints("/usrdata/mp_cfg.json", "bb_power_offset", off, 4) < 2)
        cfg_json_ints("/factory/user_cfg.json", "bb_power_offset", off, 4);
    if (model_lite_plus()) {
        // Four offsets: two groups (0, 1) of the two paths.
        for (int g = 0; g < 2; g++)
            for (int path = 0; path < 2; path++) {
                uint8_t o[4] = { (uint8_t)g, (uint8_t)off[g * 2 + path], (uint8_t)path, 0 };
                r_set(0x006e, o, 4);
            }
    } else {
        // Two: one per path, group 2.
        for (int path = 0; path < 2; path++) {
            uint8_t o[4] = { 2, (uint8_t)off[path], (uint8_t)path, 0 };
            r_set(0x006e, o, 4);
        }
    }

    pthread_create(&t, NULL, r_standby_thread, NULL);
    pthread_detach(t);
    pthread_create(&t, NULL, r_adc_thread, NULL);
    pthread_detach(t);
    return 0;
}
