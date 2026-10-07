// Video, as the stock air app sends it: each slice the encoder hands out
// (encoder.c) packed in stock's framing into the local packet ring (ring.c),
// and a sender thread writing the ring to radio socket port 3, its pace and
// the encoder's rate following the radio. Reverse-engineered from the stock
// app and checked against its traces.
//
// The packet around each slice, and which header bytes are ours, are in
// protocol/kestrel_air.h. With KA_FEAT_APCLOCK the radio clock in the header is
// taken when the encoder handed the slice out (stock: whole ms when the packet
// was built), so the ground, which reads the same clock, has the capture on its
// own clock: the stamp minus the encoder time (KA_H_T_ENC).
// With --imu the last slice of each picture carries, after the slice, the IMU
// samples taken since the previous picture as suffix-SEI NAL units (imu.c).
#include "video/video.h"
#include "video/internal.h"
#include "video/ring.h"
#include "video/stats.h"
#include "camera/image.h"
#include "imu/imu.h"
#include "app/state.h"
#include "radio/radio.h"
#include "kestrel_air.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "sdk/cv610.h"
#include "radio/radio.h"
#include "app/config.h"
#include "common/clock.h"
#include "app/settings.h"

#define HDR (KA_PKT_START + KA_HDR_LEN)
#define EXT KA_EXT_LEN
#define PKT_MAX 0x80000

int video_verbose;
int video_window = 1;   // --bb-window: unacknowledged video writes allowed (1 = stock)
volatile int video_stream_flag = -1;   // 1 got a packet, 0 a 50 ms wait came up empty

int width = 1920, height = 1080, fps = 100, src_fps = 100;   // fps: encoded, src_fps: sensor
pthread_mutex_t venc_mtx = PTHREAD_MUTEX_INITIALIZER;
volatile int venc_running, venc_kbps, venc_fps;
volatile int venc_recreate;      // ours: tune_poll asks the read loop for a new channel
int roi_cache[4] = { -1, -1, -1, -1 };
volatile uint32_t left_frames;   // encoder backlog, as the drop thread sees it
volatile int busy;               // bit1 local backlog, bit2 link down
static volatile int send_enabled = 1;
volatile int cmd_req, cmd_enable;   // send start / stop, taken by the read thread
uint16_t fc_bitmap = 0xffff;     // flow-control bitmap
static int fps_coef = 10;


// The local backlog marks the send path
// busy (the drop thread then pauses the encoder) or trims the bitrate.
static void cache_monitor(void) {
    static int cnt5;
    int n = ring_count();
    if (n >= 16) {
        busy |= 2;
    } else if (n >= 10) {
        if (!(fc_bitmap & 8)) return;
        if (cnt5 == 0) {
            if (video_verbose) printf("video: bitrate down, %d pictures queued in the encoder\n", n);
            venc_down(1);
        }
        cnt5 = (cnt5 + 1) % 5;
    } else {
        busy &= ~2;
    }
    // Through a relay, its own video backlog counts too.
    if (radio_relay_mode()) {
        static int c;
        int fb = venc_fps ? radio_tgt_bitrate() * 128 / venc_fps : 0;
        int v = radio_relay_vbuf();
        if (fb && v > 3 * fb) {
            busy |= 2;
        } else if (fb && v > fb * 11 / 10) {
            if (video_verbose) printf("video: bitrate down, local backlog (%d)\n", c);
            if (c > 3) { c = 0; venc_down(1); }
            c++;
        }
    }
}

// The radio clock: now - offset, the offset (now - AP time) refreshed every
// 2 s and averaged over ten samples.
static uint32_t radio_offset(uint32_t now) {
    static uint32_t last, off, samples[10];
    static int n, idx, reconnects = -1;
    if (last && now - last < 2000) return off;
    last = now;
    if (reconnects != radio_reconnect_count()) { reconnects = radio_reconnect_count(); n = 0; }
    uint32_t ap;
    if (radio_wireless_time(&ap) != 0) return off;
    uint32_t o = now - ap;
    if (n < 3) {
        for (int i = 0; i < 10; i++) samples[i] = o;
        n++;
        off = o;
    } else {
        samples[++idx % 10] = o;
        uint64_t sum = 0;
        for (int i = 0; i < 10; i++) sum += samples[i];
        off = (uint32_t)(sum / 10);
    }
    return off;
}

// Ours (KA_FEAT_APCLOCK): the radio clock against CLOCK_MONOTONIC in us, for
// stamps finer than its whole ms. A sample is the middle of the request minus
// the middle of the ms it answered; a slow answer (over 2 ms) is skipped. One
// a second, averaged 1/8 per sample, started again on a jump of over 20 ms or
// a reconnect: a new link is a new clock. 0 until the first sample.
static int ap_valid;
static int64_t ap_off_us;
static void ap_track(void) {
    static uint64_t last;
    static int reconnects = -1;
    uint64_t t0 = mono_us();
    if (ap_valid && t0 - last < 1000000) return;
    last = t0;
    if (reconnects != radio_reconnect_count()) { reconnects = radio_reconnect_count(); ap_valid = 0; }
    uint32_t ap;
    if (radio_wireless_time(&ap) != 0) return;
    uint64_t t1 = mono_us();
    if (t1 - t0 > 2000) return;
    int64_t o = (int64_t)((t0 + t1) / 2) - ((int64_t)ap * 1000 + 500);
    if (!ap_valid || o - ap_off_us > 20000 || o - ap_off_us < -20000) {
        ap_off_us = o;
        ap_valid = 1;
    } else {
        ap_off_us += (o - ap_off_us) / 8;
    }
}

static void *read_thread(void *arg) {
    (void)arg;
    static uint8_t stash[0x7d000];
    uint32_t stash_len = 0, p_bytes = 0;
    uint16_t seq = 0;
    int cnt = 0, slice_idx = 0, first = 1;
    uint8_t *pkt = NULL;
    uint32_t coef_t = 0;

    for (;;) {
        if (cmd_req) {
            if (!cmd_enable) {
                if (venc_running) venc_stop_destroy();
                roi_cache[0] = -1;
                send_enabled = 0;
            } else {
                send_enabled = 1;
            }
            cmd_req = 0;
        }
        if (!send_enabled) { usleep(10000); continue; }
        if (!radio_connected()) {
            if (venc_running) {
                venc_stop_destroy();
                ring_flush(2500);
                shared.stream_pause = 1;
                roi_cache[0] = -1;
            }
            busy &= ~2;
            usleep(20000);
            continue;
        }
        if (!venc_running) {
            if (shared.stream_pause) { usleep(20000); continue; }
            ring_flush(1500);
            int tgt = radio_tgt_bitrate();
            tune_poll();       // ours: a corrected tune file counts at once
            venc_recreate = 0;
            if (venc_create((int)(tgt * 0.8)) == 0) {
                venc_start();
                roi_update(1);
            } else {
                usleep(100000);   // refused (a bad tune value?): not a busy loop
            }
            cnt = 0;
            first = 1;
            stash_len = 0;
            slice_idx = 0;        // ours: the new channel starts a new picture
            p_bytes = 0;
            continue;
        }

        tune_poll();
        if (venc_recreate && slice_idx == 0) {   // ours: a new channel, between pictures
            venc_recreate = 0;
            if (venc_running) {
                venc_stop_destroy();
                roi_cache[0] = -1;
            }
            continue;
        }

        // The rate: up by 512 kbps at a time towards the target, down to
        // 70% of it at once.
        int cur = venc_kbps, want = radio_tgt_bitrate();
        if (cur < want) {
            cnt++;
            if (cnt == 1 || cnt > (first ? 20 : 48)) {
                if (radio_retx_too_many()) {
                    int nk = (int)((double)cur - (double)want * 0.1);
                    if (nk >= want / 2) set_venc_bitrate(0, nk, 3);
                } else {
                    set_venc_bitrate(0, cur + 512 < want ? cur + 512 : want, 3);
                }
                cnt = 1;
            }
        } else if (want < cur) {
            set_venc_bitrate(0, (int)(radio_tgt_bitrate() * 0.7), 4);
            cnt = 0;
        } else {
            roi_update(1);
            cnt = 0;
        }

        if (!pkt && !(pkt = ring_reserve(PKT_MAX))) { usleep(2000); continue; }
        cache_monitor();
        vframe f;
        uint8_t *payload = pkt + HDR + EXT;
        int r = stream_get(payload + stash_len, PKT_MAX - HDR - EXT - 4 - stash_len, &f, 50);
        uint64_t t_got = mono_us();
        if (r < 1) {
            shared.venc_get_fail++;
            usleep(5000);
            continue;
        }
        first = 0;
        if (f.type == 3) {                   // parameter sets: in front of the keyframe
            if (stash_len + f.len < sizeof(stash)) {
                memcpy(stash + stash_len, payload + stash_len, f.len);
                stash_len += f.len;
            } else {
                stash_len = 0;
            }
            continue;
        }
        uint32_t len = f.len;
        if (f.type == 1 && stash_len) {
            memcpy(payload, stash, stash_len);
            len += stash_len;
        } else if (stash_len) {
            memmove(payload, payload + stash_len, f.len);
        }
        stash_len = 0;
        if (slice_idx == 0) seq++;

        // Ours: after the picture's last slice, the IMU samples as SEI NAL
        // units. plen is what goes on the radio; len stays the video's own.
        uint32_t plen = len;
        if (f.frame_end && imu_dev && len + 512 < PKT_MAX - HDR - EXT - 4)
            plen += imu_sei(payload + len, PKT_MAX - HDR - EXT - 4 - len, f.pts, seq);

        uint32_t now = raw_ms();
        uint8_t *h = pkt + 4;
        memset(pkt, 0, HDR + EXT);
        h[KA_H_TYPE] = (uint8_t)f.type;
        memcpy(h + KA_H_LEN, &plen, 4);
        h[KA_H_FPS] = (uint8_t)fps;
        memcpy(h + KA_H_PIC, &seq, 2);
        uint16_t w16 = (uint16_t)width, h16 = (uint16_t)height;
        memcpy(h + KA_H_WIDTH, &w16, 2);
        memcpy(h + KA_H_HEIGHT, &h16, 2);
        memcpy(h + KA_H_PTS, &f.pts, 8);
        uint32_t rt;
        if (lat_info()) {
            // Ours: the radio clock when the encoder handed the slice out, to 1/256 ms.
            ap_track();
            uint64_t ap = ap_valid ? t_got - (uint64_t)ap_off_us : 0;
            rt = (uint32_t)(ap / 1000);
            h[KA_H_APFRAC] = (uint8_t)(ap % 1000 * 256 / 1000);
            if (!ap_valid) rt = now - radio_offset(now), h[KA_H_APFRAC] = 128;
        } else {
            rt = now - radio_offset(now);
        }
        memcpy(h + KA_H_RADIO_MS, &rt, 4);
        h[KA_H_CAP_MS] = (uint8_t)(now - (uint32_t)(f.pts / 1000));
        h[KA_H_EXT_LEN] = EXT;
        h[KA_H_SLICE] = (uint8_t)(slice_idx & KA_SLICE_INDEX);
        if (!f.frame_end) {
            h[KA_H_SLICE] |= KA_SLICE_MORE;
            if (++slice_idx > KA_SLICE_INDEX) printf("video: slice index %d\n", slice_idx);
        } else {
            h[KA_H_SLICE] |= KA_SLICE_LAST;
            slice_idx = 0;
        }
        h[KA_H_SET_FPS] = (uint8_t)cfg_get("ch0_fps", 120);
        if (lat_info()) {
            uint64_t enc = t_got > f.pts ? (t_got - f.pts) / 10 : 0;
            ka_put_t10(h + KA_H_T_ENC, enc);
            h[KA_H_TAG] = KA_TAG;    // the version message is only sent in some states: say it here too
            h[KA_H_PROTO] = KA_PROTOCOL;
            h[KA_H_FEAT] = video_features();
        }
        uint32_t x = ka_xor32(payload, plen);
        uint8_t *e = pkt + HDR;
        e[0] = 6; e[1] = KA_EXT_XOR;
        memcpy(e + 2, &x, 4);
        ring_put(pkt, HDR + EXT + plen + 4, t_got);
        pkt = NULL;

        // fps_coef: at 50/60 fps, long exposures lower the target.
        int cf = cfg_get("ch0_fps", 120);
        if (f.frame_end && now - coef_t >= 100) {
            coef_t = now;
            if (cf != 50 && cf != 60) {
                fps_coef = 10;
            } else {
                image_isp_info isp;
                image_get_info(&isp);
                uint32_t lim = cf == 50 ? 20000 : 16667;
                if (isp.exp_time_us > lim + 5000 || fps_coef == 10) {
                    int c = (int)((isp.exp_time_us * 10 + lim / 20) / lim);
                    if (c >= 13 && c != fps_coef) {
                        fps_coef = c;
                        fc_bitmap = c < 15 ? 0xffff : 0;
                    }
                } else {
                    fps_coef = 10;
                    fc_bitmap = 0xffff;
                }
            }
            radio_set_fps_coef(fps_coef);
        }
        // An oversized P-frame (over 1.2x the average frame) trims the rate.
        if ((fc_bitmap & 2) && f.type != 1) {
            p_bytes += len;
            if (f.frame_end) {
                int tgt = radio_tgt_bitrate();
                uint32_t lim = (uint32_t)((int)(tgt / (fps * 8.0) * 1024.0)) * 12 / 10;
                if (p_bytes > lim) {
                    if (video_verbose) printf("video: bitrate down, picture of %u bytes over the target %u\n", p_bytes, lim);
                    venc_down(1);
                }
                p_bytes = 0;
            }
        }
    }
    return NULL;
}

// ---- sending ---------------------------------------------------------------

// With --bb-window above 1: send without waiting for this packet's
// acknowledgement (the radio acknowledges in order; only the window
// limit makes us wait).
static int stream_send_async(uint8_t *p, uint32_t n) {
    if (!radio_connected()) return -1;
    return bbc_write_async(&radio_video, p, n, video_window, 1500) == 0 ? (int)n : -1;
}

static int stream_send(uint8_t *p, uint32_t n) {
    uint32_t off = 0;
    while (off < n) {
        if (!radio_connected()) return -1;
        int w = bbc_write(&radio_video, p + off, n - off, 1500);
        if (w <= 0) return -1;
        off += (uint32_t)w;
    }
    return (int)n;
}

static void *send_thread(void *arg) {
    (void)arg;
    uint32_t last_check = 0;
    // Debug: KA_DUMP=file writes the first KA_DUMP_KB (default 8192) KiB of what goes on
    // the radio, slices in order, as an Annex-B stream. It adds a write to each send.
    int dump_fd = -1;
    uint32_t dump_left = 0;
    if (env_str("KA_DUMP")) {
        dump_fd = open(env_str("KA_DUMP"), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        dump_left = (uint32_t)env_int("KA_DUMP_KB", 8192) * 1024u;
    }
    for (;;) {
        uint32_t n;
        uint64_t t_got;
        uint8_t *p = ring_get(&n, &t_got, 1000);
        if (!p) continue;
        uint8_t *h = p + 4;
        uint32_t len;
        memcpy(&len, h + 4, 4);

        // The radio's own backlog on port 3 trims the rate.
        uint32_t now = raw_ms();
        if (cfg_get("video_strategy", 0) != 2 && radio_connected() && (fc_bitmap & 1) &&
            now - last_check > 0x31 && (h[KA_H_SLICE] >> 5) != 1) {
            last_check = now;
            int left = radio_ringbuf_left();
            if (left >= 0) {
                if (radio_relay_mode() && radio_relay_bb_left() > left) left = radio_relay_bb_left();
                int tgt = radio_tgt_bitrate();
                int lim = (int)((tgt / 8.0) * 1024.0 / 60.0);
                if (radio_mcs() <= 0) lim = lim * 16 / 10;
                else if (h[KA_H_FPS] <= 60) lim = lim * 15 / 10;
                if (left > lim) {
                    if (video_verbose) printf("video: bitrate down, radio queue limit %d, queued %d\n", lim, left);
                    set_venc_bitrate(0, (int)(venc_kbps * 0.8), 5);
                    usleep(2000);
                }
            }
        }

        // Ours: how long this slice waited here, the last write's time, the backlog.
        static uint64_t last_write_us;
        if (lat_info()) {
            uint64_t t_now = mono_us();
            ka_put_t10(h + KA_H_T_QUEUE, t_now > t_got ? (t_now - t_got) / 10 : 0);
            ka_put_t10(h + KA_H_T_WRITE, last_write_us / 10);
            int depth = ring_count();
            h[KA_H_DEPTH] = (uint8_t)(depth > 255 ? 255 : depth);
        }

        // Stock's framing: the byte count since link-up, the checksum, the trailer.
        uint32_t total = len + 50 + EXT;
        radio_total_video_send += total;
        uint32_t t = radio_total_video_send;
        uint8_t *e = p + HDR;
        for (int o = 0; o < EXT; o += e[o]) {
            if (e[o] == 0) {
                e[o] = 6; e[o + 1] = KA_EXT_TOTAL;
                memcpy(e + o + 2, &t, 4);
                break;
            }
        }
        p[0] = 0; p[1] = 0; p[2] = 0; p[3] = 1;
        h[KA_H_MAGIC] = 0x80;
        h[KA_H_SUM] = ka_hdr_sum(h);
        uint8_t *tr = p + HDR + EXT + len;
        tr[0] = 0xee; tr[1] = 0x29; tr[2] = 0x55; tr[3] = 0x9f;
        uint64_t t_start = mono_us();
        int sent = video_window > 1 ? stream_send_async(p, total) : stream_send(p, total);
        last_write_us = mono_us() - t_start;
        if (sent != (int)total) shared.vsend_fail++;
        if (dump_fd >= 0 && sent == (int)total && dump_left >= len) {
            if (write(dump_fd, p + HDR + EXT, len) == (ssize_t)len) dump_left -= len;
            else dump_left = 0;
            if (!dump_left) { close(dump_fd); dump_fd = -1; puts("video: KA_DUMP full"); }
        }
        uint64_t pts;
        memcpy(&pts, h + 18, 8);
        tstat_add(pts, t_got, t_start, mono_us(), (h[KA_H_SLICE] & KA_SLICE_LAST) != 0);
        ring_release();
    }
    return NULL;
}

// Pause the encoder's input while its output or
// the send path is backed up.
static void *drop_thread(void *arg) {
    (void)arg;
    for (;;) {
        while (!venc_running) usleep(10000);
        int enabled = 1;
        while (venc_running) {
            pthread_mutex_lock(&venc_mtx);
            if (venc_running) {
                if (left_frames >= 6 || busy) {
                    if (enabled) ss_mpi_venc_stop_chn(CHN);
                    enabled = 0;
                } else if (!enabled) {
                    ot_venc_start_param sp = { -1 };
                    ss_mpi_venc_start_chn(CHN, &sp);
                    enabled = 1;
                }
            }
            pthread_mutex_unlock(&venc_mtx);
            usleep(10000);
        }
    }
    return NULL;
}

void video_link(int up) {
    if (up) busy &= ~4;
    else busy |= 4;
}

int video_start(void) {
    pthread_t t;
    if (ring_init()) return -1;
    if (pthread_create(&t, NULL, send_thread, NULL)) return -1;
    pthread_detach(t);
    if (pthread_create(&t, NULL, read_thread, NULL)) return -1;
    pthread_detach(t);
    if (pthread_create(&t, NULL, drop_thread, NULL)) return -1;
    pthread_detach(t);
    return 0;
}

void video_stop(void) {
    video_send_stop();
}
