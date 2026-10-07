// The encoder channel: created only while the radio is linked and the ground
// has asked for video, its rate control, intra refresh and the other knobs
// (with live tuning through /tmp/ka-tune), and reading its slices.
#include "video/internal.h"
#include "video/slices.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <unistd.h>

#include "sdk/cv610.h"
#include "app/config.h"
#include "app/state.h"
#include "common/clock.h"
#include "imu/imu.h"
#include "kestrel_air.h"
#include "radio/radio.h"
#include "video/video.h"
#include "app/settings.h"

// set_venc_bitrate's memory of what it last applied; venc_create forgets it, so a
// new channel gets the current rate (ours).
static uint32_t sb_last_t;
static int sb_last_mcs = -100, sb_last_kbps = -1, sb_last_fps = -1;
static int venc_fd = -1;

// ---- the encoder -----------------------------------------------------------

// Ours: live tuning. /tmp/ka-tune holds "name=value" lines; the read thread
// looks at it twice a second and applies a changed file to the running
// encoder - for trying encoder settings while watching the picture, with no
// reboot (the camera pipeline cannot restart without one). It lives in RAM, so
// a reboot drops it. A name not in the file keeps its default:
//   ir=N        intra refresh rows per picture, 0 off (default KA_IR, 0)
//   ir_qp=N     intra refresh's request_i_qp (KA_IR_QP, 32)
//   gop=N       pictures between keyframes (KA_GOP; 1000 with intra refresh, else 25)
//   row_qp=N    rc row_qp_delta, how far a CTU row's QP may move (6)
//   i_prop=N    CBR max_i_proportion (3)
//   min_qp=N / max_qp=N   CBR QP range, P and I alike (encoder defaults)
//   cu=0|1      1: stock's CU costs, small blocks dearer; 0: the encoder's own
//   ir_mode=0|1 intra refresh by rows (0) or columns (1)
//   roi=0|1|2   the poor-link side regions: 0 never, 1 stock's (QP 51), 2 QP +6
//   fgp=0|1     the encoder's foreground protection
//   thr=0|1     rc texture thresholds: 0 ours (stock's), 1 the SDK's defaults
//   scene=0..3  ss_mpi_venc_set_scene_mode (1)
//   irlog=0|1   once a second in /tmp/air.log: intra area per picture
// ir, ir_qp, gop and ir_mode recreate the channel (a keyframe): changing the
// GOP of a running channel silently turns intra refresh off.
static const char *const tune_names[T_N] = { "ir", "ir_qp", "gop", "row_qp", "i_prop",
                                             "min_qp", "max_qp", "cu", "ir_mode", "roi",
                                             "fgp", "thr", "scene", "irlog" };
int tune[T_N] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
static int tune_or(int t, int def) { return tune[t] >= 0 ? tune[t] : def; }

// rc_param_apply: at creation and after every attribute change.
static void rc_param_apply(void) {
    static const td_u32 thr_ours[OT_VENC_TEXTURE_THRESHOLD_SIZE] =
        { 0, 0, 0, 0, 0, 8, 8, 8, 10, 10, 10, 15, 15, 20, 25, 30 };
    static const td_u32 thr_sdk[OT_VENC_TEXTURE_THRESHOLD_SIZE] =   // hi_rc.o's defaults
        { 0, 0, 0, 0, 3, 3, 5, 5, 8, 8, 8, 15, 15, 20, 25, 25 };
    const int sdk = tune_or(T_THR, 0) == 1;
    const td_u32 *thr = sdk ? thr_sdk : thr_ours;
    ot_venc_rc_adv_param adv;
    ot_venc_rc_param rc;
    memset(&adv, 0, sizeof(adv));
    ss_mpi_venc_set_rc_adv_param(CHN, &adv);
    if (ss_mpi_venc_get_rc_param(CHN, &rc) != TD_SUCCESS) return;
    memcpy(rc.threshold_i, thr, sizeof(thr_ours));
    memcpy(rc.threshold_p, thr, sizeof(thr_ours));
    rc.direction = sdk ? 8 : 5;
    rc.row_qp_delta = (td_u32)tune_or(T_ROW_QP, 6);
    rc.first_frame_start_qp = 45;
    rc.h265_cbr_param.max_i_proportion = (td_u32)tune_or(T_I_PROP, 3);
    rc.h265_cbr_param.max_reencode_times = 0;
    if (tune[T_MAX_QP] >= 0) rc.h265_cbr_param.max_qp = rc.h265_cbr_param.max_i_qp = (td_u32)tune[T_MAX_QP];
    if (tune[T_MIN_QP] >= 0) rc.h265_cbr_param.min_qp = rc.h265_cbr_param.min_i_qp = (td_u32)tune[T_MIN_QP];
    int e = ss_mpi_venc_set_rc_param(CHN, &rc);
    if (e) printf("video: rc param -> 0x%x\n", e);
}

// Intra refresh: CTU rows (or columns) per picture, 0 = off. On by default: no
// periodic keyframes to spike the latency (2026-09-30/10-01 on the daemon radio:
// max 47 ms against 82 ms; 2026-10-04 on ar_libre: capture -> picture on the
// ground p99 22-24 ms against 41-49 ms, the medians within 2 ms).
// The CV610 sweeps the picture ONCE PER GOP: its GOP-start picture becomes a P
// picture that restarts the refresh at the top, and the sweep switches off when
// it reaches the bottom until the next GOP start (hi_h265e.o, and our own
// streams, 2026-10-05). With GOP 1000 that was 17 pictures of refresh and then
// 9.8 s of none, and whatever the encoder left stale stayed - the patches. So
// the GOP is one sweep: ceil(CTU rows / rows per picture), 17 at 1080p with 2.
// The SDK refuses a GOP too short for one sweep (0xa0088007).
// On ar_libre two of the first nine starts with it wedged the radio's SDIO bus
// (a CMD53 timeout, then every write failed): the bus watchdog in
// radio/client.c turns that into a restart of the unit.
// A ground that (re)joins mid-stream is given a keyframe when it asks (IDR below).
static int ir_rows(void) { return tune_or(T_IR, env_int("KA_IR", 2)); }
static int ir_mode(void) { return tune_or(T_IR_MODE, env_int("KA_IR_MODE", 0)) == 1; }
static int ir_sweep(void) {   // pictures for one refresh sweep, 0 without intra refresh
    int ir = ir_rows();
    if (ir <= 0) return 0;
    int units = ir_mode() ? (width + 31) / 32 : (height + 31) / 32;
    return (units + ir - 1) / ir;
}
static int gop_len(void) {
    int g = tune_or(T_GOP, env_int("KA_GOP", ir_rows() > 0 ? ir_sweep() : 25));
    // Never shorter than a sweep with intra refresh (the SDK refuses it, and the
    // channel would not come back), and within what the SDK takes.
    if (ir_rows() > 0 && g < ir_sweep()) g = ir_sweep();
    return g < 1 ? 1 : g > 65536 ? 65536 : g;
}

static void ir_apply(void) {
    int ir = ir_rows();   // intra refresh: CTU rows per frame
    ot_venc_intra_refresh r = { ir > 0 ? TD_TRUE : TD_FALSE,
                                ir_mode() ? OT_VENC_INTRA_REFRESH_COLUMN : OT_VENC_INTRA_REFRESH_ROW,
                                (td_u32)(ir > 0 ? ir : 1), (td_u32)tune_or(T_IR_QP, env_int("KA_IR_QP", 32)) };
    int e = ss_mpi_venc_set_intra_refresh(CHN, &r);
    printf("video: intra refresh %d %s/picture, gop %d (one sweep %d), qp %u -> 0x%x\n", ir,
           ir_mode() ? "columns" : "rows", gop_len(), ir_sweep(), r.request_i_qp, e);
}

// The encoder's foreground protection, and its scene mode.
static void fgp_scene_apply(void) {
    ot_venc_fg_protect g;
    if (ss_mpi_venc_get_fg_protect(CHN, &g) == TD_SUCCESS) {
        // Always written: get_fg_protect does not report what was set (scene 1).
        const int want = tune_or(T_FGP, 0) == 1;
        g.enable = want ? TD_TRUE : TD_FALSE;
        const int e = ss_mpi_venc_set_fg_protect(CHN, &g);
        if (want || e) printf("video: fg protect %d -> 0x%x\n", want, e);
    }
    int scene = tune_or(T_SCENE, 1);
    if (scene < 0 || scene > 3) scene = 1;
    ss_mpi_venc_set_scene_mode(CHN, (ot_venc_scene_mode)scene);
}

// Stock's CU costs make 8x8 and 4x4 intra, 16x16 and 8x8 inter dearer.
static void cu_apply(void) {
    ot_venc_cu_pred cu;
    if (ss_mpi_venc_get_cu_pred(CHN, &cu) != TD_SUCCESS) return;
    if (tune_or(T_CU, 1)) {
        cu.pred_mode = OT_OP_MODE_MANUAL;
        cu.intra32_cost = 0;  cu.intra16_cost = 0;  cu.intra8_cost = 15;  cu.intra4_cost = 15;
        cu.inter64_cost = 0;  cu.inter32_cost = 0;  cu.inter16_cost = 15; cu.inter8_cost = 15;
    } else {
        cu.pred_mode = OT_OP_MODE_AUTO;
    }
    ss_mpi_venc_set_cu_pred(CHN, &cu);
}

// /tmp/ka-tune, if it changed since the last look (at most twice a second):
// read it and put it on the running encoder. Every name it does not hold goes
// back to its default, so removing a line undoes it.
void tune_poll(void) {
    static uint64_t last;
    static time_t mtime;
    static off_t size = -1;
    uint64_t now = mono_us();
    if (now - last < 500000) return;
    last = now;
    struct stat st;
    if (stat("/tmp/ka-tune", &st) != 0) { st.st_mtime = 0; st.st_size = -1; }
    if (st.st_mtime == mtime && st.st_size == size) return;
    mtime = st.st_mtime;
    size = st.st_size;

    int t[T_N];
    for (int i = 0; i < T_N; i++) t[i] = -1;
    FILE *f = st.st_size >= 0 ? fopen("/tmp/ka-tune", "r") : NULL;
    char line[64];
    while (f && fgets(line, sizeof(line), f)) {
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = 0;
        for (int i = 0; i < T_N; i++)
            if (!strcmp(line, tune_names[i])) t[i] = atoi(eq + 1);
    }
    if (f) fclose(f);
    if (!memcmp(t, tune, sizeof(t))) return;
    const int recreate = t[T_IR] != tune[T_IR] || t[T_GOP] != tune[T_GOP] ||
                         t[T_IR_MODE] != tune[T_IR_MODE] || t[T_IR_QP] != tune[T_IR_QP];
    memcpy(tune, t, sizeof(t));

    printf("video: tune:");
    for (int i = 0; i < T_N; i++) if (tune[i] >= 0) printf(" %s=%d", tune_names[i], tune[i]);
    printf("\n");
    if (recreate) {   // the read loop recreates the channel (venc_recreate)
        printf("video: tune: intra refresh or GOP changed - recreating the encoder channel\n");
        venc_recreate = 1;
        return;
    }
    pthread_mutex_lock(&venc_mtx);
    if (venc_running) {
        rc_param_apply();
        cu_apply();
        fgp_scene_apply();
        roi_cache[0] = -1;   // roi_update applies the roi knob on its next call
    }
    pthread_mutex_unlock(&venc_mtx);
}
// VPSS low delay: 64 lines, 128 with intra refresh. Every CTU row (32) was
// faster still (capture -> first slice out 6.0 against 6.3 ms at 64, 6.9 at
// 128), but at 1080p100 it made the encoder keep only every other picture at
// times - always with intra refresh or four slices, now and then without
// (2026-10-04/05: 50 fps out of 100 in, capture stamps 20 ms apart).
int video_low_delay_lines(void) { return ir_rows() > 0 ? 128 : 64; }
int lat_info(void) { static int v = -1; if (v < 0) v = env_int("KA_LATINFO", 1) != 0; return v; }

uint8_t video_features(void) {
    return (uint8_t)((lat_info() ? KA_FEAT_LATINFO | KA_FEAT_APCLOCK : 0) |
                     (ir_rows() > 0 ? KA_FEAT_INTRA_REFRESH : 0) |
                     (imu_dev ? KA_FEAT_IMU : 0) |
                     (radio_bw40_enabled() ? KA_FEAT_MAXBW : 0));
}

int venc_create(int kbps) {
    ot_venc_chn_attr a;
    memset(&a, 0, sizeof(a));
    a.venc_attr.type = OT_PT_H265;
    a.venc_attr.max_pic_width = width <= 1920 ? 1920 : 2560;
    a.venc_attr.max_pic_height = width <= 1920 ? 1080 : 1440;
    a.venc_attr.buf_size = a.venc_attr.max_pic_width * a.venc_attr.max_pic_height * 3 / 4;
    a.venc_attr.profile = 0;
    a.venc_attr.is_by_frame = TD_FALSE;
    a.venc_attr.pic_width = width;
    a.venc_attr.pic_height = height;
    a.venc_attr.h265_attr.rcn_ref_share_buf_en = TD_FALSE;
    a.venc_attr.h265_attr.frame_buf_ratio = 100;
    a.rc_attr.rc_mode = OT_VENC_RC_MODE_H265_CBR;
    a.rc_attr.h265_cbr.gop = (td_u32)gop_len();
    a.rc_attr.h265_cbr.stats_time = 1;
    a.rc_attr.h265_cbr.src_frame_rate = src_fps;
    a.rc_attr.h265_cbr.dst_frame_rate = fps;
    a.rc_attr.h265_cbr.bit_rate = kbps;
    a.gop_attr.gop_mode = OT_VENC_GOP_MODE_NORMAL_P;
    a.gop_attr.normal_p.ip_qp_delta = 2;
    printf("video: encoder create, chn=%d, w=%d, h=%d, fps=%d, kbps=%d\n", CHN, width, height, fps, kbps);
    if (ss_mpi_venc_create_chn(CHN, &a) != TD_SUCCESS) return -1;
    rc_param_apply();
    cu_apply();
    fgp_scene_apply();   // scene mode 1 unless tuned
    // Slices: a number of them (KA_SLICE_COUNT, else by size and rate - see
    // video/slices.c), as CTU rows each.
    const int slices = env_int("KA_SLICE_COUNT", slice_count_default(height, fps));
    const int rows = slice_rows_for(height, slices);
    ot_venc_slice_split split = { TD_TRUE, 1, (td_u32)rows, TD_TRUE };
    ss_mpi_venc_set_slice_split(CHN, &split);

    if (ir_rows() > 0) ir_apply();
    printf("video: gop %d, %d slices asked for: %d CTU rows a slice\n", a.rc_attr.h265_cbr.gop, slices, rows);

    ot_venc_h265_vui vui;
    if (ss_mpi_venc_get_h265_vui(CHN, &vui) == TD_SUCCESS) {
        vui.vui_video_signal.video_full_range_flag = 1;
        vui.vui_video_signal.colour_description_present_flag = 1;
        vui.vui_video_signal.colour_primaries = 6;
        vui.vui_video_signal.transfer_characteristics = 6;
        vui.vui_video_signal.matrix_coefficients = 6;
        ss_mpi_venc_set_h265_vui(CHN, &vui);
    }
    venc_kbps = kbps;
    venc_fps = fps;
    sb_last_kbps = sb_last_fps = -1;   // ours: the next set_venc_bitrate applies to this channel
    return 0;
}

void venc_start(void) {
    ot_venc_start_param sp = { -1 };
    ot_mpp_chn src = { OT_ID_VPSS, 0, 0 }, dst = { OT_ID_VENC, 0, CHN };
    ss_mpi_venc_start_chn(CHN, &sp);
    ss_mpi_sys_bind(&src, &dst);
    venc_fd = ss_mpi_venc_get_fd(CHN);
    venc_running = 1;
    shared.venc_running = 1;
}

void venc_stop_destroy(void) {
    ot_mpp_chn src = { OT_ID_VPSS, 0, 0 }, dst = { OT_ID_VENC, 0, CHN };
    pthread_mutex_lock(&venc_mtx);
    venc_running = 0;
    shared.venc_running = 0;
    ss_mpi_sys_unbind(&src, &dst);
    ss_mpi_venc_stop_chn(CHN);
    ss_mpi_venc_destroy_chn(CHN);
    venc_fd = -1;
    pthread_mutex_unlock(&venc_mtx);
}

// ROI: with a poor link (or on request) the outer quarters go to QP 51 - or,
// ours, never (roi=0) or only 6 coarser than the rest (roi=2): at QP 51 nothing
// there gets corrected, refresh rows included.
void roi_update(int en) {
    if ((width != 1920 && width != 1280) || !en || !venc_running) { roi_cache[0] = -1; return; }
    const int knob = tune_or(T_ROI, 1);
    int m = radio_mcs(), mode = cfg_get("ch0_focus_en", 0), neg;
    if (mode == 0) {
        if (m >= 1) en = 0, neg = 0;
        else neg = m < 0;
    } else {
        en = mode == 1;
        neg = m < 0;
    }
    if (knob == 0) en = 0;
    if (roi_cache[0] == width && roi_cache[1] == en && roi_cache[2] == neg && roi_cache[3] == knob)
        return;
    roi_cache[0] = width; roi_cache[1] = en; roi_cache[2] = neg; roi_cache[3] = knob;
    int qw = width == 1920 ? 480 : 320, rh = width == 1920 ? 1072 : 720;
    for (int i = 0; i < 2; i++) {
        ot_venc_roi_attr r;
        if (ss_mpi_venc_get_roi_attr(CHN, i, &r) != TD_SUCCESS) continue;
        r.idx = i;
        r.enable = en ? TD_TRUE : TD_FALSE;
        if (en) {
            r.is_abs_qp = knob == 2 ? TD_FALSE : TD_TRUE;
            r.qp = knob == 2 ? 6 : 51;
            r.rect.x = i ? width - qw : 0;
            r.rect.y = 0;
            r.rect.width = qw;
            r.rect.height = rh;
        }
        ss_mpi_venc_set_roi_attr(CHN, &r);
    }
}

// The encoder's parameters
static void param_set(int f, int kbps) {
    ot_venc_chn_attr a;
    pthread_mutex_lock(&venc_mtx);
    if (venc_running && ss_mpi_venc_get_chn_attr(CHN, &a) == TD_SUCCESS) {
        f = f == 0 ? src_fps : f < 10 ? 10 : f > src_fps ? src_fps : f;
        a.rc_attr.h265_cbr.bit_rate = kbps;
        a.rc_attr.h265_cbr.dst_frame_rate = f;
        ss_mpi_venc_set_chn_attr(CHN, &a);
        venc_kbps = kbps;
        venc_fps = f;
        rc_param_apply();
    }
    pthread_mutex_unlock(&venc_mtx);
}

// The bitrate: at most one change per 100 ms (or per MCS).
void set_venc_bitrate(int f, int kbps, int reason) {
    uint32_t now = raw_ms();
    int m = radio_mcs();
    (void)reason;
    if (now - sb_last_t < 100 && m == sb_last_mcs) return;
    sb_last_t = now;
    sb_last_mcs = m;
    if (f == 0) f = video_tgt_fps();
    if (cfg_get("video_strategy", 0) == 2) {
        int t = radio_tgt_bitrate();
        if (t <= 1024) {
            kbps = t;
            if (m < 0) f = 15;
            else if (m == 0) f = f / 2 + 3;
        }
    }
    if (kbps == sb_last_kbps && f == sb_last_fps) return;
    sb_last_kbps = kbps;
    sb_last_fps = f;
    int k = kbps < 128 ? 128 : kbps > 20000 ? 20000 : kbps;
    if (video_verbose) printf("video: encoder chn %d, %d fps, %d kbps (reason %d)\n", CHN, f, k, reason);
    param_set(f, k);
    roi_update(1);
}

void venc_down(int reason) { set_venc_bitrate(0, (int)((unsigned)venc_kbps * 8 / 10), reason); }

void video_set_bitrate(int kbps) { param_set(video_tgt_fps(), kbps); }   // MCS drop: no limiter
void video_scale_bitrate(int num, int den) {
    if (fc_bitmap & 4) set_venc_bitrate(0, venc_kbps * num / den, 2);
}
void video_enable_idr(int en) {
    ss_mpi_venc_enable_idr(CHN, en ? TD_TRUE : TD_FALSE);
    // Experiment: with intra refresh there are no regular IDRs, so a goggle
    // that (re)joins mid-stream asks for one and would otherwise never start.
    if (en && ir_rows() > 0) {
        int e = ss_mpi_venc_request_idr(CHN, TD_TRUE);
        printf("video: keyframe asked for by the ground -> 0x%x\n", e);
    }
}
int video_tgt_fps(void) { return fps; }
// Send start / stop: a request the read thread takes on its next
// pass (stopping destroys the encoder); the caller waits up to 2 s for it.
static int send_cmd(int enable) {
    if (cmd_req) {
        printf("video: send %s, warning: request busy\n", enable ? "start" : "stop");
        return -1;
    }
    cmd_enable = enable;
    cmd_req = 1;
    for (int i = 0; i < 100 && cmd_req; i++) usleep(20000);
    if (cmd_req) printf("video: send %s, fail\n", enable ? "start" : "stop");
    return 0;
}

int video_send_start(void) { return send_cmd(1); }
int video_send_stop(void) { return send_cmd(0); }

void video_set_format(int w, int h, int f, int sensor_fps) {
    width = w;
    height = h;
    fps = f;
    src_fps = sensor_fps;
}

// ---- reading the encoder ---------------------------------------------------


// The next packet (a slice) into dst.
// Ours: what a NAL type makes of the packet: 1 an IDR slice, 0 any other slice,
// 3 no slice (parameter sets, SEI).
static int nal_kind(int t) {
    return t == OT_VENC_H265_NALU_IDR_SLICE || t == 20 ? 1 : t < 32 ? 0 : 3;
}

// Ours (irlog=1): the intra area of each picture, in 32x32 units, from the
// encoder's own block counts - a refresh band of 2 CTU rows is ~120-180 at
// 1080p, a picture without one far less. Once a second: min/max per picture,
// how many pictures carried a band, and the QP.
static void irlog_add(const ot_venc_h265_stream_info *h, int frame_end) {
    static uint32_t pic, n, band, mn = ~0u, mx, last_end_units, qp_sum;
    static uint64_t t0;
    // The counts look like the whole picture's, repeated per slice: taken from
    // the picture's last slice only.
    if (!frame_end) return;
    last_end_units = h->intra32x32_cu_num + h->intra16x16_cu_num / 4 +
                     h->intra8x8_cu_num / 16 + h->intra4x4_cu_num / 64;
    pic = last_end_units;
    n++;
    if (pic >= 100) band++;
    if (pic < mn) mn = pic;
    if (pic > mx) mx = pic;
    qp_sum += h->mean_qp;
    pic = 0;
    uint64_t t = mono_us();
    if (!t0) t0 = t;
    if (t - t0 < 1000000) return;
    if (tune_or(T_IRLOG, 0) == 1)
        printf("video: irlog: %u pictures, intra 32x32 units per picture min %u max %u (last slice %u), "
               "%u with a refresh band, mean qp %u\n", n, mn, mx, last_end_units, band,
               n ? qp_sum / n : 0);
    n = band = mx = qp_sum = 0;
    mn = ~0u;
    t0 = t;
}

int stream_get(uint8_t *dst, uint32_t room, vframe *out, int timeout_ms) {
    static ot_venc_pack packs[16];
    ot_venc_chn_status st;
    ot_venc_stream s;
    fd_set fds;
    struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };

    if (venc_fd < 0) return 0;
    FD_ZERO(&fds);
    FD_SET(venc_fd, &fds);
    if (select(venc_fd + 1, &fds, NULL, NULL, &tv) <= 0) {
        left_frames = 0;
        video_stream_flag = 0;
        return 0;
    }
    if (ss_mpi_venc_query_status(CHN, &st) != TD_SUCCESS) return -1;
    if (!st.cur_packs) { left_frames = 0; return 0; }
    video_stream_flag = 1;
    if (video_verbose && st.left_stream_frames > 1)
        printf("video: encoder backlog: %u pictures, %u bytes, %u frames, %u packs\n", st.left_pics, st.left_stream_bytes, st.left_stream_frames, st.cur_packs);
    left_frames = st.left_stream_bytes > 0x32000 ? 8 : st.left_stream_frames;

    memset(&s, 0, sizeof(s));
    s.pack = packs;
    s.pack_cnt = st.cur_packs < 16 ? st.cur_packs : 16;
    if (ss_mpi_venc_get_stream(CHN, &s, -1) != TD_SUCCESS) return -1;
    uint32_t total = 0;
    for (unsigned i = 0; i < s.pack_cnt; i++) total += packs[i].len - packs[i].offset;
    // Ours: typed by its slices, not by its first NAL. With intra refresh every
    // GOP start is a P picture with VPS/SPS/PPS and SEIs ahead of slice 0 in one
    // get; typed by packs[0] it went to the parameter-set stash and was dropped
    // with it (2026-10-05). The parameter sets ride along to the ground, which
    // checks and caches them.
    int kind = 3;
    for (unsigned i = 0; i < s.pack_cnt; i++) {
        int k = nal_kind(packs[i].data_type.h265_type);
        for (unsigned j = 0; j < packs[i].data_num && j < OT_VENC_MAX_PACK_INFO_NUM; j++) {
            int kj = nal_kind(packs[i].pack_info[j].pack_type.h265_type);
            if (kj < k) k = kj;
        }
        if (k == 1 || (k == 0 && kind == 3)) kind = k;
    }
    out->type = kind;
    out->pts = packs[0].pts;
    out->len = 0;
    out->frame_end = 1;
    if (total > room) {
        printf("video: slice too large, dropped, %u bytes\n", total);
    } else {
        uint32_t o = 0;
        for (unsigned i = 0; i < s.pack_cnt; i++) {
            memcpy(dst + o, packs[i].addr + packs[i].offset, packs[i].len - packs[i].offset);
            o += packs[i].len - packs[i].offset;
            out->frame_end = packs[i].is_frame_end != 0;
        }
        out->len = total;
    }
    irlog_add(&s.h265_info, out->frame_end);
    ss_mpi_venc_release_stream(CHN, &s);
    return 1;
}

