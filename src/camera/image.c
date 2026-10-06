// Camera image settings, as the stock air app makes them: its PQ tuning
// bins (loaded with the SDK's PQ-bin library, switched between gain levels
// xg1..xg4 as ISO moves), then brightness, contrast and saturation in the
// CSC, white balance, sharpness, and the 3DNR strength on top.
//
// The tuning itself stays on the unit: the bins are the stock ones in
// /usrdata/fpv/tunning, and the few tables the stock app carries in its own
// binary (3DNR presets, the manual sharpen base, the CV2004 white-balance
// gains) are read from that binary on the unit, at offsets checked against
// its size.

#include "camera/image.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ot_common_3a.h"
#include "ot_common_isp.h"
#include "ot_common_video.h"
#include "ss_mpi_ae.h"
#include "ss_mpi_awb.h"
#include "ss_mpi_isp.h"
#include "ss_mpi_vi.h"
#include "app/config.h"

#define PIPE 0
#define TUNING_DIR "/usrdata/fpv/tunning"
#define SENSOR "cv2004"

// libbin.so (MPP PQ-bin import), the stock app's call.
typedef struct { uint32_t isp_enable, nr_enable, vi_pipe, rsv[3]; } pq_bin_param;
extern int OT_PQ_BIN_ImportBinData(pq_bin_param *p, uint8_t *buf, uint32_t size);

// Tables in the stock binary (build 18.21.10, 2486456 bytes): file offsets.
#define STOCK_APP        "/usrdata/fpv/ar_ldyhs_sky"
#define STOCK_APP_SIZE   2486456
#define NR_BODY          0x512
#define OFF_NR_LOW       0x254616
#define OFF_NR_MID       0x254b28
#define OFF_NR_HIGH      0x254104
#define OFF_SHARPEN_MAN  0x1a5d78     // ot_isp_sharpen_manual_attr, 0x1a4 bytes
#define OFF_WB_CV2004    0x2551b0     // 31 x {u32 cct, u16 r, gr, gb, b}

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static int have_stock;
unsigned image_max_exposure_us;   // 0: the tuning bin's own limit
static uint8_t nr_tab[3][NR_BODY];
static uint8_t sharpen_man[0x1a4];
static uint8_t wb_tab[31 * 12];

static int fps, angle, scene, level;
#define SWITCH_HOLD 10    // readings (100 ms apart) past a threshold before a bin switch
static volatile int reload_cnt;
static volatile int isp_running;     // uav ctx +0x6c: the ISP is up, the thread may query it
static int have_nr_bin, have_shp_bin;
static uint8_t nr_bin[NR_BODY];
static ot_isp_sharpen_attr shp_bin;
static image_isp_info info;

static void load_stock_tables(void) {
    FILE *f = fopen(STOCK_APP, "rb");
    if (!f) return;
    fseek(f, 0, SEEK_END);
    if (ftell(f) == STOCK_APP_SIZE) {
        static const long nr_off[3] = { OFF_NR_LOW, OFF_NR_MID, OFF_NR_HIGH };
        int ok = 1;
        for (int i = 0; i < 3; i++) {
            fseek(f, nr_off[i], SEEK_SET);
            ok &= fread(nr_tab[i], NR_BODY, 1, f) == 1;
        }
        fseek(f, OFF_SHARPEN_MAN, SEEK_SET);
        ok &= fread(sharpen_man, sizeof(sharpen_man), 1, f) == 1;
        fseek(f, OFF_WB_CV2004, SEEK_SET);
        ok &= fread(wb_tab, sizeof(wb_tab), 1, f) == 1;
        have_stock = ok;
    }
    fclose(f);
    if (!have_stock) printf("image: %s not the expected build: no 3DNR presets or manual sharpen\n", STOCK_APP);
}

// pq_bin_load: the tuning for gain level xg (0..3), frame rate, orientation
// and day (scene 0) or night (2).
static int pq_bin_load(int xg) {
    char path[160];
    snprintf(path, sizeof(path), "%s/cam_%s_%dfps_%d_xg%d_%s.bin", TUNING_DIR, SENSOR, fps,
             angle ? 180 : 0, xg + 1, scene == 2 ? "night" : "day");
    FILE *f = fopen(path, "rb");
    if (!f) {
        printf("image: cannot open %s\n", path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = size >= 10 ? malloc(size) : NULL;
    int ret = -1;
    if (buf && fread(buf, 1, size, f) == (size_t)size) {
        pq_bin_param p = { 1, 1, PIPE, { 0, 0, 0 } };
        if (OT_PQ_BIN_ImportBinData(&p, buf, (uint32_t)size) == 0) {
            // The bin's AE route: the only exposure limits stock sets.
            ot_isp_ae_route route;
            memset(&route, 0, sizeof(route));
            route.total_num = buf[0x200aa] > 15 ? 16 : buf[0x200aa];
            for (unsigned i = 0; i < route.total_num && 0x200ac + 12 * i + 8 <= (unsigned)size; i++) {
                memcpy(&route.route_node[i].int_time, buf + 0x200ac + 12 * i, 4);
                memcpy(&route.route_node[i].sys_gain, buf + 0x200ac + 12 * i + 4, 4);
            }
            // Optional cap on the exposure (--max-exposure-us): the route's
            // integration times are clipped to it, the gains kept, so AE
            // makes up the difference with gain. 0 leaves stock's route.
            if (image_max_exposure_us) {
                for (unsigned i = 0; i < route.total_num; i++)
                    if (route.route_node[i].int_time > image_max_exposure_us)
                        route.route_node[i].int_time = image_max_exposure_us;
            }
            ss_mpi_isp_set_ae_route_attr(PIPE, &route);

            // Keep the bin's own 3DNR and sharpen for the settings built on them.
            ot_3dnr_param nr;
            memset(&nr, 0, sizeof(nr));
            if (ss_mpi_vi_get_pipe_3dnr_param(PIPE, &nr) == 0 && nr.nr_version == 2 &&
                ((uint32_t *)&nr)[1] == 1) {
                memcpy(nr_bin, (uint8_t *)&nr + 8, NR_BODY);
                have_nr_bin = 1;
            }
            if (ss_mpi_isp_get_sharpen_attr(PIPE, &shp_bin) == 0) have_shp_bin = 1;

            ot_isp_csc_attr csc;
            if (ss_mpi_isp_get_csc_attr(PIPE, &csc) == 0) {
                csc.color_gamut = OT_COLOR_GAMUT_BT601;
                csc.limited_range_en = TD_FALSE;
                ss_mpi_isp_set_csc_attr(PIPE, &csc);
            }
            level = xg;
            reload_cnt++;
            ret = 0;
            printf("image: %s\n", path);
        }
    }
    free(buf);
    fclose(f);
    return ret;
}

// ---- the setters -----------------------------------------------------------

static void csc_byte(int which, int v) {
    ot_isp_csc_attr csc;
    if (ss_mpi_isp_get_csc_attr(PIPE, &csc) != 0) return;
    if (which == 0) csc.luma = (td_u8)v;
    else if (which == 1) csc.contr = (td_u8)v;
    else csc.satu = (td_u8)v;
    ss_mpi_isp_set_csc_attr(PIPE, &csc);
}

static int map_ev(int v) {
    switch (v) {
    case 0: return 50;  case 3: return 55;  case 7: return 65;  case 10: return 75;
    case -3: return 45; case -7: return 35; case -10: return 15;
    default: return v >= 1 && v <= 99 ? v : 50;
    }
}

static const int contrast_map[11] = { 50, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75 };
static const int sat_map[11] = { 50, 10, 30, 40, 45, 50, 55, 60, 70, 80, 90 };

void image_set_ev(int ev_x10) {
    int bri = map_ev(ev_x10);
    printf("image: brightness 0x%x\n", bri);
    csc_byte(0, bri);
}

void image_set_contrast(int menu) {
    int v = menu >= 0 && menu <= 10 ? contrast_map[menu] : 50;
    printf("image: contrast %d\n", v);
    csc_byte(1, v);
}

void image_set_sat(int menu) {
    int v = menu >= 0 && menu <= 10 ? sat_map[menu] : 50;
    printf("image: saturation %d\n", v);
    csc_byte(2, v);
}

void image_set_sharpness(int menu) {
    int v = menu >= 0 && menu <= 10 ? menu : 0;
    // Per level: detail_ctrl, detail_ctrl_threshold, max_sharp_gain.
    static const int lvl[10][3] = {
        { 25, 97, 20 }, { 90, 20, 44 }, { 105, 150, 818 }, { 127, 127, 1023 }, { 155, 100, 1230 },
        { 170, 88, 1330 }, { 180, 75, 1434 }, { 205, 50, 1638 }, { 215, 35, 1750 }, { 230, 25, 1843 },
    };
    static ot_isp_sharpen_attr a;
    printf("image: sharpness %d, weight %d\n", v, v);
    if (ss_mpi_isp_get_sharpen_attr(PIPE, &a) != 0) return;
    if (have_shp_bin) a = shp_bin;
    a.enable = TD_TRUE;
    a.motion_en = TD_FALSE;
    if (v == 0) {
        a.op_type = OT_OP_MODE_AUTO;
    } else {
        a.op_type = OT_OP_MODE_MANUAL;
        a.skin_umin = 0x26;
        a.skin_vmin = 0x2b;
        a.skin_umax = 0x74;
        a.skin_vmax = 0x74;
        if (have_stock) memcpy(&a.manual_attr, sharpen_man, sizeof(sharpen_man));
        if (v >= 1 && v <= 10) {
            a.manual_attr.detail_ctrl = (td_u8)lvl[v - 1][0];
            a.manual_attr.detail_ctrl_threshold = (td_u8)lvl[v - 1][1];
            a.manual_attr.max_sharp_gain = (td_u16)lvl[v - 1][2];
        }
    }
    ss_mpi_isp_set_sharpen_attr(PIPE, &a);
}

void image_set_awb(int cct) {
    static ot_isp_wb_attr wb;
    printf("image: white balance %d\n", cct);
    if (ss_mpi_isp_get_wb_attr(PIPE, &wb) != 0) return;
    if (cct == 0) {
        wb.op_type = OT_OP_MODE_AUTO;
    } else {
        td_u16 g[4] = { 0, 0, 0, 0 };
        int found = 0;
        for (int i = 0; have_stock && i < 31; i++) {
            uint32_t c;
            memcpy(&c, wb_tab + 12 * i, 4);
            if (c == (uint32_t)cct) {
                memcpy(g, wb_tab + 12 * i + 4, 8);
                found = 1;
                break;
            }
        }
        if (!found && ss_mpi_isp_cal_gain_by_temp(PIPE, &wb, (td_u16)cct, 0, g) != 0) return;
        wb.op_type = OT_OP_MODE_MANUAL;
        wb.manual_attr.r_gain = g[0];
        wb.manual_attr.gr_gain = g[1];
        wb.manual_attr.gb_gain = g[2];
        wb.manual_attr.b_gain = g[3];
    }
    ss_mpi_isp_set_wb_attr(PIPE, &wb);
}

// 3D noise reduction: 0 off, -1 the bin's, 1..3 stock's presets.
static void set_de3d(int s) {
    ot_3dnr_attr attr;
    static ot_3dnr_param p;
    printf("image: 3D noise reduction %d\n", s);
    if (ss_mpi_vi_get_pipe_3dnr_attr(PIPE, &attr) != 0) return;
    if (s == 0) {
        attr.enable = TD_FALSE;
        ss_mpi_vi_set_pipe_3dnr_attr(PIPE, &attr);
        return;
    }
    memset(&p, 0, sizeof(p));
    if (ss_mpi_vi_get_pipe_3dnr_param(PIPE, &p) != 0) return;
    const uint8_t *src = s == -1 ? (have_nr_bin ? nr_bin : NULL) :
                         (s >= 1 && s <= 3 && have_stock) ? nr_tab[s - 1] : NULL;
    if (!src) {
        printf("image: no 3DNR parameters\n");
        return;
    }
    memcpy((uint8_t *)&p + 8, src, NR_BODY);
    if (!attr.enable) {
        attr.enable = TD_TRUE;
        ss_mpi_vi_set_pipe_3dnr_attr(PIPE, &attr);
    }
    if (ss_mpi_vi_set_pipe_3dnr_param(PIPE, &p) == 0)
        printf("image: 3DNR enable %d, version %d, mode %d, sfs1 %d\n", attr.enable, ((uint32_t *)&p)[0],
               ((uint32_t *)&p)[1], ((uint8_t *)&p)[0x9a]);
}

void image_set_3dnr(int menu) {
    static const int map[5] = { 0, -1, 1, 2, 3 };
    set_de3d(menu >= 0 && menu <= 4 ? map[menu] : -1);
}

void image_set_2dnr(int menu) { (void)menu; }   // print-only in stock

void image_set_scene(int v) {
    printf("image: scene %d\n", v);
    if ((v & ~2) == 0) scene = v;    // 0 day, 2 night: the ISP thread reloads the bin
}

void image_low_power(int on) { (void)on; }       // print-only in stock

// All the image settings at once
void image_apply_all(void) {
    pthread_mutex_lock(&mtx);
    image_set_awb((int16_t)cfg_get("cct", 0));
    image_set_ev(cfg_get("ev_x10", 0));
    image_set_contrast(cfg_get("contrast", 0));
    image_set_sat(cfg_get("sat", 0));
    image_set_sharpness(cfg_get("sharpness", 0));
    image_set_3dnr(cfg_get("cam_3dnr_strength", 4));
    image_set_scene(cfg_get("scenes", 0));
    pthread_mutex_unlock(&mtx);
}

void image_get_info(image_isp_info *i) {
    pthread_mutex_lock(&mtx);
    *i = info;
    pthread_mutex_unlock(&mtx);
}

// Every 100 ms: switch gain level on ISO, with hysteresis; reload the bin
// on a switch or a scene change, and re-apply the settings after a load.
static void *isp_thread(void *arg) {
    (void)arg;
    static const int thr[4][2] = { { 8, 12 }, { 8, 36 }, { 24, 64 }, { 48, 64 } };
    static ot_isp_exp_info exp;
    int last_scene = scene, pending = 0;
    for (;;) {
        while (!isp_running) {        // a pipeline restart: back at xg1 after it
            usleep(200000);
            level = 0;
            pending = 0;
        }
        usleep(100000);
        ot_isp_wb_info wb;
        if (ss_mpi_isp_query_wb_info(PIPE, &wb) != 0 || ss_mpi_isp_query_exposure_info(PIPE, &exp) != 0)
            continue;
        int iso100 = (int)(exp.iso / 100), nxt = level;
        if (iso100 > thr[level][1]) nxt = level + 1;
        else if (iso100 < thr[level][0]) nxt = level - 1;
        nxt = nxt < 0 ? 0 : nxt > 3 ? 3 : nxt;
        int force = scene != last_scene;
        last_scene = scene;
        // Stock switches on the first reading past a threshold. With the
        // AE hunting (a blinking light in view is enough) that reloads the
        // bin every second or so, and each load re-applies every image
        // setting mid-stream. Here the new level has to hold for a second.
        pending = nxt != level ? pending + 1 : 0;
        if (force || pending >= SWITCH_HOLD) {
            if (nxt != level || force) pq_bin_load(nxt);
            level = nxt;
            pending = 0;
        }

        pthread_mutex_lock(&mtx);
        info.pq_index = level + 1;
        info.exp_time_us = exp.exp_time;
        info.iso = exp.iso / 100.0f;
        info.color_temp = wb.color_temp;
        info.r_gain = wb.r_gain / 256.0f;
        info.b_gain = wb.b_gain / 256.0f;
        pthread_mutex_unlock(&mtx);
        if (reload_cnt > 0) {
            reload_cnt--;
            image_apply_all();
        }
    }
    return NULL;
}

void image_pause(void) { isp_running = 0; }

// After a pipeline restart: the xg1 bin for the new rate and
// orientation; the ISP thread re-applies the settings once it is loaded.
void image_restart(int sensor_fps, int angle_deg) {
    fps = sensor_fps;
    angle = angle_deg;
    level = 0;
    pq_bin_load(0);
    isp_running = 1;
}

int image_start(int sensor_fps, int angle_deg) {
    pthread_t t;
    fps = sensor_fps;
    angle = angle_deg;
    scene = cfg_get("scenes", 0);
    load_stock_tables();
    pq_bin_load(0);
    isp_running = 1;
    if (pthread_create(&t, NULL, isp_thread, NULL)) return -1;
    pthread_detach(t);
    return 0;
}
