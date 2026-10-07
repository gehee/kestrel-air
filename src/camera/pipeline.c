// Hi3516CV610 HAL: MIPI -> VI -> ISP -> VPSS -> VENC on the Caddx Ascent air
// unit (CV2004 sensor), set up the way the stock air app runs it - the whole
// chain online and low-delay, the encoder handing out each slice as it is
// done. The bring-up order and the board's values (VI device 1, 2+2 MIPI
// lanes on port 1, the fusion group, the early-end interrupts, VPSS low
// delay) are the stock firmware's.
//
// Runs on the stock air firmware in place of the stock air app: the vendor's
// kernel modules stay loaded, and the MPI libraries are the same
// HI3516CV610_MPP_V1.0.2.0 B051 build the modules are. The sensor
// drivers (libsns_*.so) are installed next to kestrel-air.

#include "kestrel_air.h"
#include "camera/pipeline.h"

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "sdk/cv610.h"
#include "common/clock.h"
#include "app/settings.h"


// The sensor: Ascents ship with either of two, both 1920x1080 on I2C 0x36
// with 16-bit register addresses. The capture is always the full 1080p;
// VPSS scales to the encoded size. They are told apart as the stock app
// does, by two ID registers, the CV2004 first.
typedef struct {
    const char *name, *lib, *obj;
    int id;                       // the ISP's sensor id (what the driver registers)
    unsigned id_reg;              // two consecutive ID registers ...
    unsigned char id_val[2];      // ... and what they read
    int type;                     // the sensor type the stock app reports to the ground
    int angle_tuning;             // the tuning bins' names carry the orientation
    int bayer_follows_angle;      // the ISP's bayer order turns with the orientation
} cv610_sensor;
static const cv610_sensor sensors[] = {
    { "cv2004",  "libsns_cv2004.so",  "g_sns_cv2004_obj",  2004, 0x3002, { 0x04, 0x20 }, 1, 1, 1 },
    // Stock leaves the ISP's order at BGGR at both orientations.
    { "os02k10", "libsns_os02k10.so", "g_sns_os02k10_obj", 1480, 0x300a, { 0x53, 0x02 }, 7, 0, 0 },
};
static const cv610_sensor *sns = &sensors[0];
const char *cv610_sensor_name(void) { return sns->name; }
int cv610_sensor_type(void) { return sns->type; }
int cv610_sensor_angle_tuning(void) { return sns->angle_tuning; }
#define SNS_I2C    0
#define SNS_ADDR   0x36
#define CAP_W      1920
#define CAP_H      1080

#define MIPI_DEV   1          // combo device 1: the 2+2 split's port 1, lanes {1,3}
#define VI_DEV     1
#define VI_PIPE    0
#define VI_CHN     0
#define VPSS_GRP   0
#define VPSS_CHN   0
// Stock encodes the live view on channel 1. Channel 0 is left alone.
#define VENC_CHN   1

#ifndef LANE_DIVIDE_MODE_1    // missing from the SDK's mipi header, present in the kernel's
#define LANE_DIVIDE_MODE_1 ((lane_divide_mode_t)1)
#endif

#define CHECK(expr) do { \
        td_s32 _r = (expr); \
        if (_r != TD_SUCCESS) { \
            fprintf(stderr, "camera: %s failed: 0x%x\n", #expr, _r); \
            return _r; \
        } \
    } while (0)

static short out_w, out_h;
static int fps;
static void *sns_handle;
static ot_isp_sns_obj *sns_obj;
static ot_isp_3a_alg_lib ae_lib  = { .id = VI_PIPE, .lib_name = "ot_ae_lib" };
static ot_isp_3a_alg_lib awb_lib = { .id = VI_PIPE, .lib_name = "ot_awb_lib" };
static pthread_t isp_thread;
static int isp_running;

static int sns_angle = -1;   // orientation to set before sensor init, -1: leave

// The sensor's orientation, as stock sets it: before the sensor is
// initialised, 0 normal, else mirror+flip.
void cv610_set_sensor_angle(int angle) { sns_angle = angle; }

static int ld_lines = 64;    // VPSS low delay, see vpss_setup
void cv610_set_low_delay_lines(int lines) { ld_lines = lines; }

// Kernel-side MPP state outlives the process that made it. The vendor app we
// replace - or a run of ours that died - leaves SYS, VB and ISP set up, and
// nothing below can be configured until they are released.
static void release_previous(void) {
    ss_mpi_venc_stop_chn(VENC_CHN);
    ss_mpi_venc_destroy_chn(VENC_CHN);
    ss_mpi_vpss_stop_grp(VPSS_GRP);
    ss_mpi_vpss_disable_chn(VPSS_GRP, VPSS_CHN);
    ss_mpi_vpss_destroy_grp(VPSS_GRP);
    ss_mpi_isp_exit(VI_PIPE);
    ss_mpi_vi_disable_chn(VI_PIPE, VI_CHN);
    ss_mpi_vi_stop_pipe(VI_PIPE);
    ss_mpi_vi_destroy_pipe(VI_PIPE);
    ss_mpi_vi_unbind(VI_DEV, VI_PIPE);
    ss_mpi_vi_disable_dev(VI_DEV);
    ss_mpi_sys_exit();
    ss_mpi_vb_exit();
}

static int sys_setup(void) {
    ot_vb_cfg vb;
    ot_vb_supplement_cfg sup;
    ot_vi_vpss_mode mode;

    release_previous();

    // MPP timestamps on CLOCK_MONOTONIC, as stock sets them: the capture
    // time carried to the ground is then comparable with anything else
    // stamped on this unit.
    CHECK(ss_mpi_sys_init_pts_base(mono_us()));

    // Stock's pools: online VI -> VPSS needs no raw frames, only six
    // 1080p YUV 4:2:0 pictures.
    memset(&vb, 0, sizeof(vb));
    vb.max_pool_cnt = 128;
    vb.common_pool[0].blk_size = (td_u64)CAP_W * CAP_H * 3 / 2;
    vb.common_pool[0].blk_cnt  = 6;
    if (out_w * out_h < CAP_W * CAP_H) {
        vb.common_pool[1].blk_size = (td_u64)out_w * out_h * 3 / 2;
        vb.common_pool[1].blk_cnt  = 4;
    }
    CHECK(ss_mpi_vb_set_cfg(&vb));
    // Motion data for the 3DNR, as stock asks for it.
    memset(&sup, 0, sizeof(sup));
    sup.supplement_cfg = OT_VB_SUPPLEMENT_MOTION_DATA_MASK | OT_VB_SUPPLEMENT_BNR_MOT_MASK;
    CHECK(ss_mpi_vb_set_supplement_cfg(&sup));
    CHECK(ss_mpi_vb_init());
    CHECK(ss_mpi_sys_init());

    memset(&mode, 0, sizeof(mode));
    for (int i = 0; i < OT_VI_MAX_PIPE_NUM; i++) mode.mode[i] = OT_VI_OFFLINE_VPSS_OFFLINE;
    mode.mode[VI_PIPE] = OT_VI_ONLINE_VPSS_ONLINE;
    CHECK(ss_mpi_sys_set_vi_vpss_mode(&mode));
    CHECK(ss_mpi_sys_set_vi_aiisp_mode(VI_PIPE, OT_VI_AIISP_MODE_DEFAULT));
    return 0;
}

static int mipi_setup(void) {
    combo_dev_attr_t attr;
    combo_dev_t dev = MIPI_DEV, clk_dev = 0;
    sns_clk_source_t sns_clk = 0;
    sns_rst_source_t sns_rst = 0;
    lane_divide_mode_t hs = LANE_DIVIDE_MODE_1;
    ext_data_type_t ext;
    int fd, ok = 1;

    memset(&attr, 0, sizeof(attr));
    attr.devno = dev;
    attr.input_mode = INPUT_MODE_MIPI;
    attr.data_rate = MIPI_DATA_RATE_X1;
    attr.img_rect.width = CAP_W;
    attr.img_rect.height = CAP_H;
    attr.mipi_attr.input_data_type = DATA_TYPE_RAW_10BIT;
    attr.mipi_attr.wdr_mode = OT_MIPI_WDR_MODE_NONE;
    for (int i = 0; i < MIPI_LANE_NUM; i++)
        attr.mipi_attr.lane_id[i] = (i < 2) ? (short)(1 + i * 2) : (short)-1;
    memset(&ext, 0, sizeof(ext));
    ext.devno = dev;
    ext.num = 3;
    for (int i = 0; i < 3; i++) {
        ext.ext_data_bit_width[i] = 12;
        ext.ext_data_type[i] = 0x2c;
    }

    if ((fd = open("/dev/ot_mipi_rx", O_RDWR)) < 0) {
        fprintf(stderr, "camera: open /dev/ot_mipi_rx: %s\n", strerror(errno));
        return -1;
    }
#define MIPI(req, arg) \
    if (ok && ioctl(fd, (req), (arg)) < 0) { \
        fprintf(stderr, "camera: mipi %s: %s\n", #req, strerror(errno)); ok = 0; }
    // Stock's sequence, request for request: the sensor clock and reset on
    // source 0, the MIPI clock and reset on combo device 0 (the 2+2 split's
    // two ports share it), the attributes on device 1, and the embedded
    // data types the CV2004 sends alongside the picture.
    MIPI(OT_MIPI_SET_HS_MODE, &hs);
    MIPI(OT_MIPI_ENABLE_SENSOR_CLOCK, &sns_clk);
    MIPI(OT_MIPI_RESET_SENSOR, &sns_rst);
    MIPI(OT_MIPI_UNRESET_SENSOR, &sns_rst);
    MIPI(OT_MIPI_ENABLE_MIPI_CLOCK, &clk_dev);
    MIPI(OT_MIPI_RESET_MIPI, &clk_dev);
    MIPI(OT_MIPI_SET_DEV_ATTR, &attr);
    MIPI(OT_MIPI_SET_EXT_DATA_TYPE, &ext);
    MIPI(OT_MIPI_UNRESET_MIPI, &clk_dev);
#undef MIPI
    close(fd);
    return ok ? 0 : -1;
}

static int vi_setup(void) {
    ot_vi_dev_attr dev;
    ot_vi_pipe_attr pipe;
    ot_vi_chn_attr chn;
    ot_vi_wdr_fusion_grp_attr fusion;
    ot_frame_interrupt_attr irq;
    ot_3dnr_attr nr;
    ot_vi_pipe_param param;

    memset(&dev, 0, sizeof(dev));
    dev.intf_mode = OT_VI_INTF_MODE_MIPI;
    dev.work_mode = OT_VI_WORK_MODE_MULTIPLEX_1;
    dev.component_mask[0] = 0xFFF00000;
    dev.scan_mode = OT_VI_SCAN_PROGRESSIVE;
    for (int i = 0; i < 4; i++) dev.ad_chn_id[i] = -1;
    dev.data_seq = OT_VI_DATA_SEQ_YVYU;
    dev.sync_cfg.vsync_valid = OT_VI_VSYNC_VALID_SIG;
    dev.data_type = OT_VI_DATA_TYPE_RAW;
    dev.in_size.width = CAP_W;
    dev.in_size.height = CAP_H;
    dev.data_rate = OT_DATA_RATE_X1;
    CHECK(ss_mpi_vi_set_dev_attr(VI_DEV, &dev));
    CHECK(ss_mpi_vi_enable_dev(VI_DEV));
    CHECK(ss_mpi_vi_bind(VI_DEV, VI_PIPE));

    // Online VI -> VPSS needs the pipe named as a (single-pipe, linear)
    // fusion group, or VPSS faults every frame it is handed.
    memset(&fusion, 0, sizeof(fusion));
    fusion.wdr_mode = OT_WDR_MODE_NONE;
    fusion.cache_line = CAP_H;
    fusion.pipe_id[0] = VI_PIPE;
    CHECK(ss_mpi_vi_set_wdr_fusion_grp_attr(VI_PIPE, &fusion));

    memset(&pipe, 0, sizeof(pipe));
    pipe.pipe_bypass_mode = OT_VI_PIPE_BYPASS_NONE;
    pipe.size.width = CAP_W;
    pipe.size.height = CAP_H;
    pipe.pixel_format = OT_PIXEL_FORMAT_RGB_BAYER_10BPP;
    pipe.compress_mode = OT_COMPRESS_MODE_NONE;
    pipe.frame_rate_ctrl.src_frame_rate = OT_VI_INVALID_FRAME_RATE;
    pipe.frame_rate_ctrl.dst_frame_rate = OT_VI_INVALID_FRAME_RATE;
    CHECK(ss_mpi_vi_create_pipe(VI_PIPE, &pipe));
    // Stock's pixel clock for the online pipe, and its plain (not
    // "advanced") noise-reduction effect.
    CHECK(ss_mpi_vi_set_pipe_online_clock(VI_PIPE, 264000000));
    CHECK(ss_mpi_vi_get_pipe_param(VI_PIPE, &param));
    param.nr_effect_mode = OT_VI_NR_EFFECT_MODE_NORM;
    CHECK(ss_mpi_vi_set_pipe_param(VI_PIPE, &param));

    // Hand the frame on 100 lines before its end, as stock does: the next
    // stage starts while the last rows are still arriving.
    memset(&irq, 0, sizeof(irq));
    irq.interrupt_type = OT_FRAME_INTERRUPT_EARLY_END;
    irq.early_line = CAP_H - 100;
    CHECK(ss_mpi_vi_set_pipe_frame_interrupt_attr(VI_PIPE, &irq));
    CHECK(ss_mpi_vi_start_pipe(VI_PIPE));

    memset(&chn, 0, sizeof(chn));
    chn.size.width = CAP_W;
    chn.size.height = CAP_H;
    chn.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;   // the only 420 VI takes
    chn.dynamic_range = OT_DYNAMIC_RANGE_SDR8;
    chn.video_format = OT_VIDEO_FORMAT_LINEAR;
    chn.compress_mode = OT_COMPRESS_MODE_NONE;
    chn.frame_rate_ctrl.src_frame_rate = OT_VI_INVALID_FRAME_RATE;
    chn.frame_rate_ctrl.dst_frame_rate = OT_VI_INVALID_FRAME_RATE;
    CHECK(ss_mpi_vi_set_chn_attr(VI_PIPE, VI_CHN, &chn));
    CHECK(ss_mpi_vi_enable_chn(VI_PIPE, VI_CHN));

    memset(&nr, 0, sizeof(nr));
    nr.enable = TD_TRUE;
    nr.nr_type = OT_NR_TYPE_VIDEO_NORM;
    nr.compress_mode = OT_COMPRESS_MODE_NONE;
    nr.nr_motion_mode = OT_NR_MOTION_MODE_NORM;
    CHECK(ss_mpi_vi_set_pipe_3dnr_attr(VI_PIPE, &nr));
    return 0;
}

static void *isp_run(void *arg) {
    (void)arg;
    ss_mpi_isp_run(VI_PIPE);   // returns only after ss_mpi_isp_exit()
    return NULL;
}

// The sensor, found the way the stock app does: each candidate's two ID
// registers, in order. A unit with neither is not supported: the camera does
// not start on it.
static int sensor_read(int fd, unsigned reg) {
    unsigned char a[2] = { reg >> 8, reg & 0xff }, v = 0;
    if (write(fd, a, 2) != 2 || read(fd, &v, 1) != 1) return -1;
    return v;
}

static int sensor_detect(void) {
    char dev[16];
    int fd, id0 = -1, id1 = -1;

    snprintf(dev, sizeof(dev), "/dev/i2c-%d", SNS_I2C);
    if ((fd = open(dev, O_RDWR)) < 0) return 1;
    if (ioctl(fd, 0x0706 /* I2C_SLAVE_FORCE */, SNS_ADDR) == 0) {
        for (unsigned i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
            const cv610_sensor *c = &sensors[i];
            id0 = sensor_read(fd, c->id_reg);
            id1 = sensor_read(fd, c->id_reg + 1);
            printf("camera: sensor id 0x%x/%x = %02x %02x: %s\n", c->id_reg, c->id_reg + 1,
                   id0 & 0xff, id1 & 0xff, (id0 == c->id_val[0] && id1 == c->id_val[1]) ? c->name : "not it");
            if (id0 == c->id_val[0] && id1 == c->id_val[1]) {
                sns = c;
                close(fd);
                return 1;
            }
        }
        fprintf(stderr, "camera: an image sensor kestrel-air does not know\n");
        close(fd);
        return 0;
    }
    close(fd);
    return 1;
}

static int isp_setup(void) {
    ot_isp_sns_commbus bus;
    ot_isp_bind_attr bind;
    ot_isp_pub_attr pub;
    ot_isp_ctrl_param ctrl;

    if (!sensor_detect()) return -1;
    if (!(sns_handle = dlopen(sns->lib, RTLD_NOW | RTLD_GLOBAL))) {
        fprintf(stderr, "camera: %s\n", dlerror());
        return -1;
    }
    if (!(sns_obj = (ot_isp_sns_obj *)dlsym(sns_handle, sns->obj))) {
        fprintf(stderr, "camera: %s\n", dlerror());
        return -1;
    }
    // Stock's order: the orientation (the driver only records it; the
    // sensor init writes it), the sensor's callbacks, its bus, then AE/AWB.
    if (sns_angle >= 0 && sns_obj->pfn_mirror_flip)
        sns_obj->pfn_mirror_flip(VI_PIPE, sns_angle ? ISP_SNS_MIRROR_FLIP : ISP_SNS_NORMAL);
    CHECK(sns_obj->pfn_register_callback(VI_PIPE, &ae_lib, &awb_lib));
    if (sns_obj->pfn_set_bus_info) {
        bus.i2c_dev = SNS_I2C;
        CHECK(sns_obj->pfn_set_bus_info(VI_PIPE, bus));
    }
    CHECK(ss_mpi_ae_register(VI_PIPE, &ae_lib));
    CHECK(ss_mpi_awb_register(VI_PIPE, &awb_lib));

    memset(&bind, 0, sizeof(bind));
    bind.sns_id = sns->id;
    bind.ae_lib = ae_lib;
    bind.awb_lib = awb_lib;
    CHECK(ss_mpi_isp_set_bind_attr(VI_PIPE, &bind));
    // Stock reads the control parameters back and sets them unchanged.
    CHECK(ss_mpi_isp_get_ctrl_param(VI_PIPE, &ctrl));
    CHECK(ss_mpi_isp_set_ctrl_param(VI_PIPE, &ctrl));
    CHECK(ss_mpi_isp_mem_init(VI_PIPE));

    memset(&pub, 0, sizeof(pub));
    pub.wnd_rect.width = CAP_W;
    pub.wnd_rect.height = CAP_H;
    pub.sns_size.width = CAP_W;
    pub.sns_size.height = CAP_H;
    pub.frame_rate = fps;          // the sensor driver picks its mode from this
    // Read out turned by 180 degrees, the CV2004's RGGB starts on blue.
    pub.bayer_format = (!sns->bayer_follows_angle || sns_angle > 0) ? OT_ISP_BAYER_BGGR : OT_ISP_BAYER_RGGB;
    pub.wdr_mode = OT_WDR_MODE_NONE;
    pub.mipi_crop_attr.mipi_crop_offset.width = CAP_W;    // unused (crop off), as stock fills it
    pub.mipi_crop_attr.mipi_crop_offset.height = CAP_H;
    CHECK(ss_mpi_isp_set_pub_attr(VI_PIPE, &pub));
    CHECK(ss_mpi_isp_init(VI_PIPE));
    return 0;
}

static int vpss_setup(void) {
    ot_frame_interrupt_attr irq;
    ot_vpss_grp_attr grp;
    ot_vpss_chn_attr chn;
    ot_low_delay_info ld;
    ot_mpp_chn src = { OT_ID_VI, VI_PIPE, VI_CHN };
    ot_mpp_chn dst = { OT_ID_VPSS, VPSS_GRP, 0 };

    memset(&irq, 0, sizeof(irq));
    irq.interrupt_type = OT_FRAME_INTERRUPT_EARLY_END;
    irq.early_line = CAP_H - 100;
    CHECK(ss_mpi_vpss_set_grp_frame_interrupt_attr(VPSS_GRP, &irq));

    memset(&grp, 0, sizeof(grp));
    grp.max_width = CAP_W;
    grp.max_height = CAP_H;
    grp.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    grp.dynamic_range = OT_DYNAMIC_RANGE_SDR8;
    grp.dei_mode = OT_VPSS_DEI_MODE_OFF;
    grp.frame_rate.src_frame_rate = fps;
    grp.frame_rate.dst_frame_rate = fps;
    CHECK(ss_mpi_vpss_create_grp(VPSS_GRP, &grp));

    memset(&chn, 0, sizeof(chn));
    chn.width = out_w;
    chn.height = out_h;
    chn.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    chn.dynamic_range = OT_DYNAMIC_RANGE_SDR8;
    chn.video_format = OT_VIDEO_FORMAT_LINEAR;
    chn.compress_mode = OT_COMPRESS_MODE_NONE;
    chn.chn_mode = OT_VPSS_CHN_MODE_USER;
    chn.frame_rate.src_frame_rate = -1;
    chn.frame_rate.dst_frame_rate = -1;
    chn.border_attr.bottom_width = 1;   // border off; stock's value
    CHECK(ss_mpi_vpss_set_chn_attr(VPSS_GRP, VPSS_CHN, &chn));
    CHECK(ss_mpi_vpss_enable_chn(VPSS_GRP, VPSS_CHN));
    CHECK(ss_mpi_vpss_start_grp(VPSS_GRP));

    // The encoder hears about a picture's lines every ld_lines out of VPSS, not
    // once the whole picture is. 32 is one CTU row, the least it can start on:
    // stock's 128 held each slice back ~0.5 ms (2026-10-04, by the radio clock:
    // capture -> first slice out 6.9 -> 6.2 ms, last 11.4 -> 10.9 ms, at
    // 1080p100). But at 32 the encoder sometimes kept only every other picture
    // (50 fps) - always with intra refresh - so the caller sets 64, or 128 with
    // intra refresh (cv610_set_low_delay_lines; 64: first slice out 6.3 ms).
    // KA_LD_LINES overrides it.
    memset(&ld, 0, sizeof(ld));
    ld.enable = TD_TRUE;
    ld.line_cnt = env_int("KA_LD_LINES", ld_lines);
    CHECK(ss_mpi_vpss_set_chn_low_delay(VPSS_GRP, VPSS_CHN, &ld));

    CHECK(ss_mpi_sys_bind(&src, &dst));
    return 0;
}

int cv610_pipeline_create(char sensor, short width, short height, char framerate, int slice_count) {
    (void)sensor; (void)slice_count;
    out_w = width;
    out_h = height;
    fps = (unsigned char)framerate;
    printf("camera: capture %dx%d@%d -> %dx%d\n", CAP_W, CAP_H, fps, out_w, out_h);

    if (sys_setup() || mipi_setup() || vi_setup() || isp_setup() || vpss_setup())
        return -1;
    if (pthread_create(&isp_thread, NULL, isp_run, NULL))
        return -1;
    isp_running = 1;
    return 0;
}

void cv610_pipeline_destroy(void) {
    ot_mpp_chn src = { OT_ID_VI, VI_PIPE, VI_CHN };
    ot_mpp_chn dst = { OT_ID_VPSS, VPSS_GRP, 0 };

    ss_mpi_sys_unbind(&src, &dst);
    ss_mpi_vpss_stop_grp(VPSS_GRP);
    ss_mpi_vpss_disable_chn(VPSS_GRP, VPSS_CHN);
    ss_mpi_vpss_destroy_grp(VPSS_GRP);
    ss_mpi_isp_exit(VI_PIPE);
    if (isp_running) {
        pthread_join(isp_thread, NULL);
        isp_running = 0;
    }
    ss_mpi_awb_unregister(VI_PIPE, &awb_lib);
    ss_mpi_ae_unregister(VI_PIPE, &ae_lib);
    if (sns_obj && sns_obj->pfn_un_register_callback)
        sns_obj->pfn_un_register_callback(VI_PIPE, &ae_lib, &awb_lib);
    ss_mpi_vi_disable_chn(VI_PIPE, VI_CHN);
    ss_mpi_vi_stop_pipe(VI_PIPE);
    ss_mpi_vi_destroy_pipe(VI_PIPE);
    ss_mpi_vi_unbind(VI_DEV, VI_PIPE);
    ss_mpi_vi_disable_dev(VI_DEV);
    if (sns_handle) {
        dlclose(sns_handle);
        sns_handle = NULL;
        sns_obj = NULL;
    }
}

void cv610_system_deinit(void) {
    ss_mpi_sys_exit();
    ss_mpi_vb_exit();
}
