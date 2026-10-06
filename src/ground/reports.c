// The air's reports to the ground: version, settings, the periodic telemetry,
// the channel information, and the tick loop that sends them.
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
#include "kestrel_air.h"
#include "radio/radio.h"
#include "video/video.h"

static void send_rec_status(void) {
    uint8_t m[6] = { 0x01, 2, 0, 0, 0, 0 };   // 2: no recording path (no SD recorder)
    ground_send(m, 6);
}

// sg_video_cfg_trans_t, from the local settings.
void put_cfg_struct(uint8_t *s) {
    memset(s, 0, 49);
    s[0] = 1;
    s[0x09] = (uint8_t)cfg_get("ev_x10", 0);
    s[0x0a] = (uint8_t)cfg_get("scenes", 0);
    s[0x0b] = (uint8_t)cfg_get("sat", 0);
    s[0x0c] = (uint8_t)cfg_get("sharpness", 0);
    int16_t cct = (int16_t)cfg_get("cct", 0);
    memcpy(s + 0x0d, &cct, 2);
    s[0x0f] = (uint8_t)cfg_get("angle", 0);
    s[0x10] = (uint8_t)cfg_get("ratio", 1);
    uint16_t v;
    v = (uint16_t)cfg_get("ch0_width", 1920);  memcpy(s + 0x11, &v, 2);
    v = (uint16_t)cfg_get("ch0_height", 1080); memcpy(s + 0x13, &v, 2);
    s[0x15] = (uint8_t)cfg_get("ch0_fps", 120);
    s[0x16] = (uint8_t)cfg_get("ch0_focus_en", 0);
    v = (uint16_t)cfg_get("ch1_width", 1920);  memcpy(s + 0x17, &v, 2);
    v = (uint16_t)cfg_get("ch1_height", 1080); memcpy(s + 0x19, &v, 2);
    s[0x1b] = (uint8_t)cfg_get("ch1_fps", 60);
    s[0x1c] = (uint8_t)cfg_get("rec_dev", 3);
    s[0x1d] = (uint8_t)cfg_get("rec_loop_dev", 3);
    s[0x1e] = (uint8_t)cfg_get("rec_auto_dev", 1);
    v = (uint16_t)cfg_get("cam_exp", 0);       memcpy(s + 0x1f, &v, 2);
    v = (uint16_t)cfg_get("cam_max_iso", 0);   memcpy(s + 0x21, &v, 2);
    s[0x23] = (uint8_t)cfg_get("cam_3dnr_strength", 4);
    s[0x24] = (uint8_t)cfg_get("rec_eis_en", 0);
    s[0x25] = (uint8_t)cfg_get("rec_color_mode", 0);
    s[0x26] = (uint8_t)cfg_get("rec_sharpness", 5);
    s[0x27] = (uint8_t)cfg_get("rec_sat", 5);
    s[0x28] = (uint8_t)cfg_get("rec_enc_format", 0);
    s[0x29] = (uint8_t)cfg_get("rec_pack_durtion", 5);
    s[0x2a] = (uint8_t)cfg_get("cam_2dnr_strength", 1);
    s[0x2b] = (uint8_t)cfg_get("cam_max_iso_mode", 0);
    s[0x2c] = (uint8_t)cfg_get("cam_anti_flicker_en", 0);
    v = (uint16_t)cfg_get("bb_power_mw", 501);  memcpy(s + 0x2d, &v, 2);
    s[0x2f] = (uint8_t)((cfg_get("video_strategy", 0) & 0xf) | (cfg_get("contrast", 0) << 4));
    s[0x30] = (uint8_t)(cfg_get("sys_standby_mode", 1) & 1);
}

static void send_cam_setting(void) {
    uint8_t m[62];
    m[0] = 0x03;
    put_cfg_struct(m + 1);
    const uint8_t tlv[12] = { 3, 0x11, PRJ_TYPE, 3, 0x13, SENSOR_TYPE,
                              3, 0x12, (uint8_t)cfg_get("sys_standby_mode", 1), 3, 0x14, RF_HW_VER };
    memcpy(m + 50, tlv, 12);
    int r = ground_send(m, 62);
    printf("ground: camera settings to the ground, sndsize=62, ret=%d\n", r);
}

// The hardware version comes from a board-ID voltage on LSADC channel 1:
// model select, enable, start, five reads, stop, disable - stock's sequence.
static int hw_ver_mv(void) {
    int fd = open("/dev/ot_lsadc", O_RDWR), sum = 0;
    uint32_t model = 1, ch = 1;
    if (fd < 0) return 0;
    if (ioctl(fd, 0xc0044100, &model) >= 0 && ioctl(fd, 0x40044101, &ch) >= 0) {
        usleep(100);
        if (ioctl(fd, 0x4103) >= 0) {
            for (int i = 0; i < 5; i++) {
                usleep(100);
                sum += ioctl(fd, 0xc0044105, &ch);
            }
        }
    }
    ioctl(fd, 0x4104);
    ioctl(fd, 0x40044102, &ch);
    close(fd);
    int avg = sum / 5;
    return avg > 0x3ff ? 0 : (int)(avg * 3300.0 / 1023.0);
}

static void send_version(void) {
    static uint8_t m[16];
    static int done;
    if (!done) {
        int mv = hw_ver_mv(), a = 0, b = 0, c = 0;
        uint8_t hw = mv < 200 ? 0x10 : mv <= 449 ? 0x11 : mv <= 749 ? 0x12 : 0x13;
        FILE *f = fopen("/etc/app.version", "r");
        char line[128];
        while (f && fgets(line, sizeof(line), f))
            if (!strncmp(line, "APP_VERSION=", 12)) sscanf(line + 12, "%d.%d.%d", &a, &b, &c);
        if (f) fclose(f);
        m[0] = 0x04;
        // Bytes 1..4 and 7 are zero from the stock air app: ours says who it is.
        m[KA_VER_TAG0] = 'K'; m[KA_VER_TAG1] = 'A';
        m[KA_VER_PROTO] = KA_PROTOCOL; m[KA_VER_FEAT] = video_features();
        m[5] = hw;
        m[6] = RF_HW_VER;
        m[8] = 3;
        m[9] = (uint8_t)a; m[11] = (uint8_t)b; m[13] = (uint8_t)c;
        printf("ground: version %d.%d.%d, hardware %d mV (0x%x)\n", a, b, c, mv, hw);
        done = 1;
    }
    m[4] = video_features();   // the IMU is switched on after the first call
    m[15] = (uint8_t)cfg_get("sys_standby_mode", 1);
    ground_send(m, 16);
}

static uint8_t period_counter;

static void put16(uint8_t *p, int v) {
    uint16_t x = (uint16_t)(v < 0 ? 0 : v);
    memcpy(p, &x, 2);
}

static void send_period(void) {
    uint8_t m[39];
    radio_period r;
    image_isp_info isp;
    memset(m, 0, sizeof(m));
    m[0] = 0x05;
    int t = shared.cpu_temp_x100;
    m[1] = (uint8_t)(int8_t)(t >= 12800 ? 0x7f : (t / 100 < -100 ? -100 : t / 100));
    uint16_t pw = (uint16_t)cfg_get("bb_power_mw", 501);
    memcpy(m + 9, &pw, 2);
    uint32_t mv = (uint32_t)radio_batt_mv();
    memcpy(m + 11, &mv, 4);
    image_get_info(&isp);
    put16(m + 15, (int)(isp.iso * 100));
    put16(m + 17, (int)isp.color_temp);
    put16(m + 19, (int)(isp.r_gain * 1000));
    put16(m + 21, (int)(isp.b_gain * 1000));
    put16(m + 23, isp.pq_index);
    put16(m + 27, (int)isp.exp_time_us);
    m[29] = (uint8_t)((shared.low_power ? 1 : 0) | (shared.venc_running ? 2 : 0) | (shared.stream_pause ? 4 : 0));
    radio_period_info(&r);
    m[30] = r.tssi_a; m[31] = r.tssi_b; m[32] = r.gain_a; m[33] = r.gain_b;
    m[35] = (uint8_t)r.ofs_a; m[36] = (uint8_t)r.ofs_b;
    m[37] = (uint8_t)shared.venc_get_fail;     // encoder reads that came back empty
    m[38] = (uint8_t)((++period_counter << 4) | (shared.vsend_fail & 0xf));   // short radio writes
    // bytes 3..8: the last BB_SET_SKY_FREQ
    m[3] = msg_is_hop; m[4] = msg_slot;
    memcpy(m + 5, &msg_freq, 4);
    ground_send(m, 39);
}

uint8_t msg_is_hop, msg_slot;
uint32_t msg_freq;
uint32_t work_list[64];
int work_n;

static void send_chan_info(void) {
    static uint8_t r[1028];
    uint8_t m[43];
    int got = radio_chan_info(r, sizeof(r));
    memset(m, 0, sizeof(m));
    m[0] = 0x07;
    m[2] = 10;
    if (got > 0) {
        struct { uint16_t mhz; int16_t pwr; } e[128];
        int n = 0, cnt = r[0] > 128 ? 128 : r[0];
        for (int i = 0; i < cnt; i++) {
            uint32_t f;
            int32_t p;
            memcpy(&f, r + 4 + i * 4, 4);
            memcpy(&p, r + 0x204 + i * 4, 4);
            for (int k = 0; k < work_n; k++)
                if (work_list[k] == f) { e[n].mhz = (uint16_t)(f / 1000); e[n].pwr = (int16_t)p; n++; break; }
        }
        // Sorted by power; up to ten: all, or the five lowest and five highest.
        // Fewer than two leaves stock's ten empty entries.
        if (n >= 2) {
            for (int i = 1; i < n; i++)
                for (int j = i; j > 0 && e[j].pwr < e[j - 1].pwr; j--) {
                    __typeof__(e[0]) x = e[j]; e[j] = e[j - 1]; e[j - 1] = x;
                }
            int k = 0;
            for (int i = 0; i < n; i++) {
                if (n > 10 && i >= 5 && i < n - 5) continue;
                memcpy(m + 3 + k * 4, &e[i].mhz, 2);
                memcpy(m + 5 + k * 4, &e[i].pwr, 2);
                k++;
            }
            m[2] = (uint8_t)k;
        }
    }
    ground_send(m, 3 + 4 * m[2]);
}

// ---- the tick loop ---------------------------------------------------------

static volatile int restart_flag;

void *tick_thread(void *arg) {
    (void)arg;
    int cam_cnt = 0, last_rec = -1, send_cam = 0;
    for (;;) {
        for (int i = 0; i < 100; i++) {
            if (restart_flag) { restart_flag = 0; break; }
            if (!shared.cam_flag) {
                cam_cnt = 0;
            } else {
                cam_cnt++;
                if (cam_cnt == 1) { send_cam = 1; break; }
                if (cam_cnt > 30) cam_cnt = 0;
            }
            if (last_rec != 2) { last_rec = 2; break; }   // record state: no recorder
            usleep(50000);
            if (radio_connected()) {
                if (i == 42 || i == 82) send_version();
                if (i % 5 == 0) send_period();
                if (i && i % 13 == 0) send_chan_info();
            }
        }
        if (radio_connected()) {
            if (send_cam) send_cam_setting();
            send_rec_status();
            send_cam = 0;
        }
    }
    return NULL;
}
