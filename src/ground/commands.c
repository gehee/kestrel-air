// The ground's commands: settings, the video's size and rate, keyframes,
// pairing, the radio's channels, bandwidth and caps.
#include "ground/internal.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "app/app.h"
#include "app/config.h"
#include "app/state.h"
#include "ground/ground.h"
#include "camera/image.h"
#include "common/clock.h"
#include "common/crc.h"
#include "kestrel_air.h"
#include "radio/radio.h"
#include "video/video.h"

static volatile int init_busy;
static uint8_t last_applied[49];
static int have_last;
static int iframe_req_cnt, iframe_reconnects = -1;

static int res_ok(int w, int h, int fps) {
    int r = (w == 1920 && h == 1080) || (w == 1280 && h == 720) || (w == 960 && h == 540) ||
            (w == 2560 && h == 1440);
    int f = fps == 25 || fps == 30 || fps == 50 || fps == 60 || fps == 100 || fps == 120;
    return r && f;
}

static int power_ok(int mw) {
    static const int tab[] = { 25, 100, 150, 200, 500, 101, 501 };
    for (unsigned i = 0; i < sizeof(tab) / sizeof(tab[0]); i++)
        if (tab[i] == mw) return mw;
    printf("ground: ground power setting not in the map, max_pwr=%d\n", 501);
    return 501;
}

// Apply what INIT_CFG changed.
static void *apply_thread(void *arg) {
    uint8_t *s = arg;
    uint16_t w0, h0, w1, h1;
    memcpy(&w0, s + 0x11, 2); memcpy(&h0, s + 0x13, 2);
    memcpy(&w1, s + 0x17, 2); memcpy(&h1, s + 0x19, 2);
    uint32_t t0 = mono_ms32();
    printf("ground: init cfg, start, t1=%u, bb: old=[%d,%d,%d], new=[%d,%d,%d]\n", t0,
           cfg_get("ch0_width", 0), cfg_get("ch0_height", 0), cfg_get("ch0_fps", 0), w0, h0, s[0x15]);
    // A new camera size or rate restarts the whole pipeline first.
    if (w0 != cfg_get("ch0_width", 0) || h0 != cfg_get("ch0_height", 0) || s[0x15] != cfg_get("ch0_fps", 0)) {
        int w = w0, h = h0, f = s[0x15];
        app_ch0_remap(&w, &h, &f);
        video_send_stop();
        app_set_sensor_res(w, h, f, cfg_get("angle", 0));
    }
    cfg_set("ch0_width", w0);
    cfg_set("ch0_height", h0);
    cfg_set("ch0_fps", s[0x15]);
    cfg_set("ch0_focus_en", s[0x16]);
    cfg_set("ch1_width", w1);
    cfg_set("ch1_height", h1);
    cfg_set("ch1_fps", s[0x1b]);
    cfg_set("rec_dev", s[0x1c]);
    cfg_set("rec_loop_dev", s[0x1d]);
    cfg_set("rec_auto_dev", s[0x1e]);
    cfg_set("rec_eis_en", s[0x24]);
    cfg_set("rec_color_mode", s[0x25]);
    cfg_set("rec_sharpness", s[0x26]);
    cfg_set("rec_sat", s[0x27]);
    cfg_set("rec_enc_format", s[0x28]);
    cfg_set("rec_pack_durtion", s[0x29]);
    cfg_set("cam_anti_flicker_en", s[0x2c]);
    uint16_t pw;
    memcpy(&pw, s + 0x2d, 2);
    cfg_set("bb_power_mw", power_ok(pw));
    cfg_set("video_strategy", s[0x2f] & 0xf);
    cfg_set("sys_standby_mode", s[0x30] & 1);
    cfg_save();
    image_apply_all();
    video_send_start();
    printf("ground: camera settings to send\n");
    shared.cam_flag = 1;
    printf("ground: init cfg, done in %u ms\n", mono_ms32() - t0);
    free(s);
    init_busy = 0;
    return NULL;
}

static void init_cfg(const uint8_t *p, int len) {
    printf("ground: init cfg received, busy %d, t=%u\n",
           init_busy, mono_ms32());
    if (init_busy || len < 50) return;
    init_busy = 1;
    const uint8_t *s = p + 1;
    uint16_t w0, h0, w1, h1;
    memcpy(&w0, s + 0x11, 2); memcpy(&h0, s + 0x13, 2);
    memcpy(&w1, s + 0x17, 2); memcpy(&h1, s + 0x19, 2);
    int err = !res_ok(w0, h0, s[0x15]) ? -1 : !res_ok(w1, h1, s[0x1b]) ? -2 :
              (s[0x24] >= 6 || (int8_t)s[0x16] >= 6 || s[0x2c] >= 6) ? -3 : 0;
    if (err) {
        printf("ground: init cfg invalid (%d), ignored\n", err);
        init_busy = 0;
        return;
    }
    for (int o = 0x32; o + 1 < len;) {     // TLVs: [len incl. these 2] [id] [data]
        int l = p[o], id = p[o + 1];
        if (l < 2) break;
        if (id == 0x15 && l >= 4) printf("ground: the ground's board type %d, hardware 0x%x\n", p[o + 2], p[o + 3]);
        else if (id != 0x10) printf("ground: init cfg, unknown field %d, length %d\n", id, l);
        o += l;
    }
    // What the settings would be after it, against what was applied last.
    uint8_t cur[49], next[49];
    put_cfg_struct(cur);
    memcpy(next, cur, 49);
    static const uint8_t taken[] = { 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
                                     0x1c, 0x1d, 0x1e, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2c };
    for (unsigned i = 0; i < sizeof(taken); i++) next[taken[i]] = s[taken[i]];
    uint16_t pw;
    memcpy(&pw, s + 0x2d, 2);
    pw = (uint16_t)power_ok(pw);
    memcpy(next + 0x2d, &pw, 2);
    next[0x2f] = (uint8_t)((next[0x2f] & 0xf0) | (s[0x2f] & 0x0f));
    next[0x30] = s[0x30] & 1;
    if (have_last) memcpy(last_applied, cur, 49);
    if (have_last && !memcmp(last_applied, next, 49)) {
        printf("ground: init cfg as the last one, nothing to do\n");
        video_send_start();
        shared.cam_flag = 1;
        init_busy = 0;
    } else {
        memcpy(last_applied, next, 49);
        have_last = 1;
        uint8_t *copy = malloc(49);
        memcpy(copy, s, 49);
        pthread_t t;
        pthread_create(&t, NULL, apply_thread, copy);
        pthread_detach(t);
    }
    printf("ground: init cfg applied, t=%u\n", mono_ms32());
}

// SG_MSG_ID_SET_SKY_CFG_RESET: forget the settings and
// the pairing, put the radio's config back from its .backup, reboot.
static void config_reset(int param) {
    char path[64] = "", backup[64], cmd[128];
    printf("ground: settings reset (%d)\n", param);
    if (system("rm -rf /factory/fpv_config.json") != 0) { /* as stock: carry on */ }
    if (system("rm /factory/user_cfg.json") != 0) { /* idem */ }
    FILE *f = fopen("/tmp/bb_cfg_path", "r");
    if (!f) {
        printf("ground: cannot open %s\n", "/tmp/bb_cfg_path");
    } else {
        if (!fgets(path, sizeof(path), f)) printf("ground: cannot read %s\n", "/tmp/bb_cfg_path");
        fclose(f);
        path[strcspn(path, "\n")] = 0;
        if (!path[0]) printf("ground: no config path in %s\n", "/tmp/bb_cfg_path");
        else printf("ground: config path from %s: %s\n", "/tmp/bb_cfg_path", path);
    }
    snprintf(backup, sizeof(backup), "%s%s", path, ".backup");
    snprintf(cmd, sizeof(cmd), "cp %s %s", backup, path);
    printf("ground: running %s\n", cmd);
    if (system("rm /factory/fpv_bb_freq.json") != 0) { /* idem */ }
    if (system(cmd) != 0) { /* idem */ }
    if (system("sync") != 0) { /* idem */ }
    if (system("reboot") != 0) { /* idem */ }
}

// SG_MSG_ID_SET_CHN_RES: channel 0 is the live view (a full pipeline
// restart at the new size and rate), channel 1 the recording (stored only).
static void set_chn_res(int chn, int w, int h, int fps) {
    printf("ground: output chn %d, %dx%d@%d, t=%u\n", chn, w, h, fps, mono_ms32());
    if (chn == 0) {
        if (w > 1920) { w = 1920; h = 1080; fps = 60; }     // the CV2004 is 1080p at most
        cfg_set("ch0_width", w);
        cfg_set("ch0_height", h);
        cfg_set("ch0_fps", fps);
        app_ch0_remap(&w, &h, &fps);
        video_send_stop();
        app_set_sensor_res(w, h, fps, cfg_get("angle", 0));
        video_send_start();
    } else if (chn == 1) {
        cfg_set("ch1_width", w);
        cfg_set("ch1_height", h);
        cfg_set("ch1_fps", fps);
    }
    cfg_save();
}

static int32_t u32at(const uint8_t *p) {
    int32_t v;
    memcpy(&v, p, 4);
    return v;
}

void dispatch(const uint8_t *p, int len) {
    int id = p[0];
    int32_t v = len >= 5 ? u32at(p + 1) : 0;
    switch (id) {
    case 0x01: if (len < 5) { puts("ground: argument missing"); break; }
               cfg_set("scenes", v); cfg_save(); image_set_scene(v); break;
    case 0x02: cfg_set("ev_x10", v); cfg_save(); image_set_ev(v); break;
    case 0x03: cfg_set("sat", v); cfg_save(); image_set_sat(v); break;
    case 0x04: cfg_set("sharpness", v); image_set_sharpness(v); cfg_save(); break;
    case 0x05: cfg_set("cct", (int16_t)v); cfg_save(); image_set_awb(v); break;
    case 0x06:
        // Stock only stores the angle and stops video: the sensor turns at
        // the next pipeline restart (SET_CHN_RES, a new INIT_CFG size, boot).
        printf("ground: camera angle %d\n", v);
        cfg_set("angle", (uint8_t)v);
        cfg_save();
        video_send_stop();
        break;
    case 0x08:
        if (len >= 7) set_chn_res(p[1], p[2] | p[3] << 8, p[4] | p[5] << 8, p[6]);
        break;
    case 0x0e: if (len >= 3 && p[1] == 0) { cfg_set("ch0_focus_en", p[2]); cfg_save(); } break;
    case 0x10: init_cfg(p, len); break;
    case 0x12: shared.cam_flag = 0; break;
    case 0x16: cfg_set("rec_pack_durtion", v); cfg_save(); break;
    case 0x1b: cfg_set("cam_3dnr_strength", v); cfg_save(); image_set_3dnr(v); break;
    case 0x1c: cfg_set("cam_2dnr_strength", v); cfg_save(); image_set_2dnr(v); break;
    case 0x20: {    // enable I-frames; the 6th request since a relink also starts video
        video_enable_idr(len >= 2 && p[1]);
        if (radio_reconnect_count() != iframe_reconnects) {
            iframe_reconnects = radio_reconnect_count();
            iframe_req_cnt = 0;
        }
        if (++iframe_req_cnt > 5 && shared.stream_pause) shared.stream_pause = 0;
        printf("ground: keyframe request %d, stream paused %d\n", iframe_req_cnt, shared.stream_pause);
        break;
    }
    case 0x21:
        if (len >= 7) {
            msg_is_hop = p[1];
            msg_slot = p[2];
            memcpy(&msg_freq, p + 3, 4);
            radio_set_freq(msg_is_hop, msg_freq);
        }
        break;
    case 0x22:
        if (len >= 3) {
            uint16_t mw;
            memcpy(&mw, p + 1, 2);
            cfg_set("bb_power_mw", mw);
            cfg_save();
            printf("ground: standby mode %d, TX power %d mW\n", cfg_get("sys_standby_mode", 1), mw);
        }
        break;
    case 0x23: if (len >= 2) { cfg_set("sys_standby_mode", p[1]); cfg_save(); } break;
    case 0x25: if (len >= 2) { cfg_set("video_strategy", p[1]); cfg_save(); radio_set_mcs_policy(); } break;
    case 0x26:
        if (len >= 2) video_scale_bitrate(p[1] == 1 ? 9 : p[1] == 2 ? 8 : 10, 10);
        break;
    case 0x27: cfg_set("contrast", v); cfg_save(); image_set_contrast(v); break;
    case 0x28: puts("ground: start the video"); shared.stream_pause = 0; break;
    case 0x29: {
        int n = (len - 1) / 4;
        if (n > 64) n = 64;
        for (int i = 0; i < n; i++) work_list[i] = (uint32_t)u32at(p + 1 + i * 4);
        work_n = n;
        printf("ground: work channel list, %d channels\n", n);
        radio_set_gnd_work_list(work_list, n);
        break;
    }
    case 0x2a: if (len >= 2 && (p[1] == 1 || p[1] == 2)) radio_set_uart_mode(p[1]); break;
    case 0x2c:
        if (len >= 2 && p[1] == 1) {
            uint8_t m[8] = { 0x08, 1, (uint8_t)work_n, 0 };
            uint32_t c = crc32c(0, (const uint8_t *)work_list, work_n * 4);
            memcpy(m + 4, &c, 4);
            ground_send(m, 8);
        }
        break;
    case 0x1e: config_reset(v); break;
    case 0x2b: printf("ground: audio chn %d on %d (no audio here)\n", len > 1 ? p[1] : 0, len > 2 ? p[2] : 0); break;
    case 0x2d: if (len >= 2 && p[1] == 10) radio_relay_update(p + 2, len - 2); break;
    // Ours. Stock ignores it, and stock's ground sends it with 0 at every
    // link-up, which must stay a no-op (the radio picks its bandwidth itself).
    // 1..6 is a gear (0..5) plus one, from kestrel-gnd.
    case 0x24: if (len >= 2 && p[1] >= 1 && p[1] <= 6) radio_bandwidth_request(p[1] - 1); break;
    case KA_CMD_MAX_KBPS: if (len >= 5) radio_set_max_kbps(v); break;
    case KA_CMD_MAX_BW:   if (len >= 2) radio_set_max_bw(p[1]); break;
    case 0x07: case 0x0f: case 0x11: case 0x13: case 0x14: case 0x15: case 0x17:
    case 0x18: case 0x19: case 0x1a: case 0x1d: case 0x1f:
        break;                          // logged only by stock
    default:
        // Recording (0x08-0x0c, needs the SD recorder), config reset, audio
        // and relay are not implemented.
        printf("ground: unknown command 0x%02x\n", id);
        break;
    }
}
