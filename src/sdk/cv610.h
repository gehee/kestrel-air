// The Hi3516CV610 media libraries (MPI, ISP, AE, AWB) and the MIPI RX driver as
// kestrel-air uses them: the functions it calls, the constants it passes and
// the structures it exchanges with them. Members it does not touch are
// reserved space, named rsv_<offset>; enumerations are 32-bit integers with
// the values it uses. The reserved space keeps the shape of what it covers
// (plain fields, arrays, unions, 8-byte-aligned runs) so GCC lays out the
// variables that hold these structures the same way.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/ioctl.h>

// --- Basic types and status --------------------------------------------------

typedef uint8_t  td_u8;
typedef uint16_t td_u16;
typedef uint32_t td_u32;
typedef uint64_t td_u64;
typedef int32_t  td_s32;

typedef uint32_t td_bool;
enum { TD_FALSE = 0, TD_TRUE = 1 };

#define TD_SUCCESS 0     // what every call returns when it worked

// --- Common to the modules ---------------------------------------------------

typedef struct {
    uint32_t width;
    uint32_t height;
} ot_size;
_Static_assert(sizeof(ot_size) == 8, "ot_size");

typedef struct {
    int32_t x;
    int32_t y;
    uint32_t width;
    uint32_t height;
} ot_rect;
_Static_assert(sizeof(ot_rect) == 16, "ot_rect");

typedef struct {
    int32_t src_frame_rate;
    int32_t dst_frame_rate;
} ot_frame_rate_ctrl;
_Static_assert(sizeof(ot_frame_rate_ctrl) == 8, "ot_frame_rate_ctrl");

typedef struct {
    uint32_t rsv_0;
    uint32_t bottom_width;
    uint32_t rsv_8, rsv_12, rsv_16;
} ot_border;
_Static_assert(sizeof(ot_border) == 20, "ot_border");

typedef uint32_t ot_pixel_format;
enum {
    OT_PIXEL_FORMAT_RGB_BAYER_10BPP = 24,
    OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420 = 38,
};

typedef uint32_t ot_dynamic_range;
enum { OT_DYNAMIC_RANGE_SDR8 = 0 };

typedef uint32_t ot_video_format;
enum { OT_VIDEO_FORMAT_LINEAR = 0 };

typedef uint32_t ot_compress_mode;
enum { OT_COMPRESS_MODE_NONE = 0 };

typedef uint32_t ot_wdr_mode;
enum { OT_WDR_MODE_NONE = 0 };

typedef uint32_t ot_data_rate;
enum { OT_DATA_RATE_X1 = 0 };

typedef uint32_t ot_op_mode;
enum { OT_OP_MODE_AUTO = 0, OT_OP_MODE_MANUAL = 1 };

typedef uint32_t ot_payload_type;
enum { OT_PT_H265 = 265 };

// An interrupt ahead of the frame's end (VI pipe, VPSS group).
typedef uint32_t ot_frame_interrupt_type;
enum { OT_FRAME_INTERRUPT_EARLY_END = 2 };

typedef struct {
    ot_frame_interrupt_type interrupt_type;
    uint32_t early_line;
} ot_frame_interrupt_attr;
_Static_assert(sizeof(ot_frame_interrupt_attr) == 8, "ot_frame_interrupt_attr");

typedef struct {
    td_bool enable;
    uint32_t line_cnt;
    uint32_t rsv_8;
} ot_low_delay_info;
_Static_assert(sizeof(ot_low_delay_info) == 12, "ot_low_delay_info");

// A module's channel, for binding one module's output to another's input.
typedef uint32_t ot_mod_id;
enum { OT_ID_VPSS = 7, OT_ID_VENC = 8, OT_ID_VI = 16 };

typedef struct {
    ot_mod_id mod_id;
    int32_t dev_id;
    int32_t chn_id;
} ot_mpp_chn;
_Static_assert(sizeof(ot_mpp_chn) == 12, "ot_mpp_chn");

// --- MIPI RX driver (ioctls on its device) -----------------------------------

#define MIPI_LANE_NUM 4

typedef uint32_t combo_dev_t;
typedef uint32_t sns_clk_source_t;
typedef uint32_t sns_rst_source_t;

typedef uint32_t lane_divide_mode_t;
enum { LANE_DIVIDE_MODE_1 = 1 };

typedef uint32_t input_mode_t;
enum { INPUT_MODE_MIPI = 0 };

typedef uint32_t mipi_data_rate_t;
enum { MIPI_DATA_RATE_X1 = 0 };

typedef uint32_t data_type_t;
enum { DATA_TYPE_RAW_10BIT = 1 };

typedef uint32_t mipi_wdr_mode_t;
enum { OT_MIPI_WDR_MODE_NONE = 0 };

typedef struct {
    uint32_t rsv_0, rsv_4;
    uint32_t width;
    uint32_t height;
} img_rect_t;
_Static_assert(sizeof(img_rect_t) == 16, "img_rect_t");

typedef struct {
    data_type_t input_data_type;
    mipi_wdr_mode_t wdr_mode;
    int16_t lane_id[MIPI_LANE_NUM];
    union { uint8_t rsv_16[8]; };
} mipi_dev_attr_t;
_Static_assert(sizeof(mipi_dev_attr_t) == 24, "mipi_dev_attr_t");

typedef struct {
    combo_dev_t devno;
    input_mode_t input_mode;
    mipi_data_rate_t data_rate;
    img_rect_t img_rect;
    union {                          // by the input mode
        mipi_dev_attr_t mipi_attr;
        uint8_t rsv_28[172];
    };
} combo_dev_attr_t;
_Static_assert(sizeof(combo_dev_attr_t) == 200, "combo_dev_attr_t");

typedef struct {
    combo_dev_t devno;
    uint32_t num;
    uint32_t ext_data_bit_width[3];
    uint32_t ext_data_type[3];
} ext_data_type_t;
_Static_assert(sizeof(ext_data_type_t) == 32, "ext_data_type_t");

#define OT_MIPI_SET_DEV_ATTR        _IOW('m', 0x01, combo_dev_attr_t)
#define OT_MIPI_RESET_SENSOR        _IOW('m', 0x05, sns_rst_source_t)
#define OT_MIPI_UNRESET_SENSOR      _IOW('m', 0x06, sns_rst_source_t)
#define OT_MIPI_RESET_MIPI          _IOW('m', 0x07, combo_dev_t)
#define OT_MIPI_UNRESET_MIPI        _IOW('m', 0x08, combo_dev_t)
#define OT_MIPI_SET_HS_MODE         _IOW('m', 0x0b, lane_divide_mode_t)
#define OT_MIPI_ENABLE_MIPI_CLOCK   _IOW('m', 0x0c, combo_dev_t)
#define OT_MIPI_ENABLE_SENSOR_CLOCK _IOW('m', 0x10, sns_clk_source_t)
#define OT_MIPI_SET_EXT_DATA_TYPE   _IOW('m', 0x12, ext_data_type_t)

// --- System and video buffers ------------------------------------------------

#define OT_VB_SUPPLEMENT_MOTION_DATA_MASK 0x2
#define OT_VB_SUPPLEMENT_BNR_MOT_MASK     0x8

typedef struct {
    uint64_t blk_size;
    uint32_t blk_cnt;
    uint8_t rsv_12[36];
} ot_vb_pool_cfg;
_Static_assert(sizeof(ot_vb_pool_cfg) == 48, "ot_vb_pool_cfg");

typedef struct {
    uint32_t max_pool_cnt;
    uint8_t rsv_4[4];
    ot_vb_pool_cfg common_pool[16];
} ot_vb_cfg;
_Static_assert(sizeof(ot_vb_cfg) == 776, "ot_vb_cfg");

typedef struct {
    uint32_t supplement_cfg;     // OT_VB_SUPPLEMENT_*_MASK bits
} ot_vb_supplement_cfg;
_Static_assert(sizeof(ot_vb_supplement_cfg) == 4, "ot_vb_supplement_cfg");

// --- Video input (VI) --------------------------------------------------------

#define OT_VI_MAX_PIPE_NUM 4
#define OT_VI_INVALID_FRAME_RATE (-1)

typedef int32_t ot_vi_dev;
typedef int32_t ot_vi_pipe;
typedef int32_t ot_vi_chn;
typedef int32_t ot_vi_grp;

// How each pipe hands its frames to the VPSS.
typedef uint32_t ot_vi_vpss_mode_type;
enum { OT_VI_OFFLINE_VPSS_OFFLINE = 0, OT_VI_ONLINE_VPSS_ONLINE = 3 };

typedef struct {
    ot_vi_vpss_mode_type mode[OT_VI_MAX_PIPE_NUM];
} ot_vi_vpss_mode;
_Static_assert(sizeof(ot_vi_vpss_mode) == 16, "ot_vi_vpss_mode");

typedef uint32_t ot_vi_aiisp_mode;
enum { OT_VI_AIISP_MODE_DEFAULT = 0 };

typedef uint32_t ot_vi_intf_mode;
enum { OT_VI_INTF_MODE_MIPI = 4 };

typedef uint32_t ot_vi_work_mode;
enum { OT_VI_WORK_MODE_MULTIPLEX_1 = 0 };

typedef uint32_t ot_vi_scan_mode;
enum { OT_VI_SCAN_PROGRESSIVE = 0 };

typedef uint32_t ot_vi_data_seq;
enum { OT_VI_DATA_SEQ_YVYU = 5 };

typedef uint32_t ot_vi_data_type;
enum { OT_VI_DATA_TYPE_RAW = 0 };

typedef uint32_t ot_vi_vsync_valid;
enum { OT_VI_VSYNC_VALID_SIG = 1 };

typedef struct {
    uint32_t rsv_0, rsv_4, rsv_8, rsv_12;
    ot_vi_vsync_valid vsync_valid;
    uint32_t rsv_20, rsv_24, rsv_28, rsv_32, rsv_36;
    uint32_t rsv_40, rsv_44, rsv_48, rsv_52, rsv_56;
} ot_vi_sync_cfg;
_Static_assert(sizeof(ot_vi_sync_cfg) == 60, "ot_vi_sync_cfg");

typedef struct {
    ot_vi_intf_mode intf_mode;
    ot_vi_work_mode work_mode;
    uint32_t component_mask[2];
    ot_vi_scan_mode scan_mode;
    int32_t ad_chn_id[4];
    ot_vi_data_seq data_seq;
    ot_vi_sync_cfg sync_cfg;
    ot_vi_data_type data_type;
    uint8_t rsv_104[4];
    ot_size in_size;
    ot_data_rate data_rate;
} ot_vi_dev_attr;
_Static_assert(sizeof(ot_vi_dev_attr) == 120, "ot_vi_dev_attr");

typedef struct {
    ot_wdr_mode wdr_mode;
    uint32_t cache_line;
    ot_vi_pipe pipe_id[2];
    uint8_t rsv_16[4];
} ot_vi_wdr_fusion_grp_attr;
_Static_assert(sizeof(ot_vi_wdr_fusion_grp_attr) == 20, "ot_vi_wdr_fusion_grp_attr");

typedef uint32_t ot_vi_pipe_bypass_mode;
enum { OT_VI_PIPE_BYPASS_NONE = 0 };

typedef struct {
    ot_vi_pipe_bypass_mode pipe_bypass_mode;
    uint32_t rsv_4;
    ot_size size;
    ot_pixel_format pixel_format;
    ot_compress_mode compress_mode;
    ot_frame_rate_ctrl frame_rate_ctrl;
} ot_vi_pipe_attr;
_Static_assert(sizeof(ot_vi_pipe_attr) == 32, "ot_vi_pipe_attr");

typedef uint32_t ot_nr_effect_mode;
enum { OT_VI_NR_EFFECT_MODE_NORM = 0 };

typedef struct {
    uint32_t rsv_0, rsv_4, rsv_8, rsv_12, rsv_16, rsv_20;
    ot_nr_effect_mode nr_effect_mode;
    uint32_t rsv_28;
} ot_vi_pipe_param;
_Static_assert(sizeof(ot_vi_pipe_param) == 32, "ot_vi_pipe_param");

typedef struct {
    ot_size size;
    ot_pixel_format pixel_format;
    ot_dynamic_range dynamic_range;
    ot_video_format video_format;
    ot_compress_mode compress_mode;
    uint32_t rsv_24, rsv_28, rsv_32;
    ot_frame_rate_ctrl frame_rate_ctrl;
} ot_vi_chn_attr;
_Static_assert(sizeof(ot_vi_chn_attr) == 44, "ot_vi_chn_attr");

// The pipe's 3D noise reduction.
typedef uint32_t ot_nr_type;
enum { OT_NR_TYPE_VIDEO_NORM = 0 };

typedef uint32_t ot_nr_motion_mode;
enum { OT_NR_MOTION_MODE_NORM = 0 };

typedef struct {
    td_bool enable;
    ot_nr_type nr_type;
    ot_compress_mode compress_mode;
    ot_nr_motion_mode nr_motion_mode;
} ot_3dnr_attr;
_Static_assert(sizeof(ot_3dnr_attr) == 16, "ot_3dnr_attr");

// The parameters proper are handled as raw bytes after the version.
typedef uint32_t ot_nr_version;

typedef struct {
    ot_nr_version nr_version;
    union { uint8_t rsv_4[1316]; };  // by the version
} ot_3dnr_param;
_Static_assert(sizeof(ot_3dnr_param) == 1320, "ot_3dnr_param");

// --- Image signal processor (ISP), with its AE and AWB libraries -------------

typedef int32_t ot_sensor_id;

// A 3A algorithm library, registered by name.
typedef struct {
    int32_t id;
    char lib_name[20];
} ot_isp_3a_alg_lib;
_Static_assert(sizeof(ot_isp_3a_alg_lib) == 24, "ot_isp_3a_alg_lib");

typedef struct {
    ot_sensor_id sns_id;
    ot_isp_3a_alg_lib ae_lib;
    uint8_t rsv_28[24];
    ot_isp_3a_alg_lib awb_lib;
} ot_isp_bind_attr;
_Static_assert(sizeof(ot_isp_bind_attr) == 76, "ot_isp_bind_attr");

// The sensor driver: its bus, and the entry points of its library's object.
typedef union {
    int8_t i2c_dev;
} ot_isp_sns_commbus;
_Static_assert(sizeof(ot_isp_sns_commbus) == 1, "ot_isp_sns_commbus");

typedef uint32_t ot_isp_sns_mirrorflip_type;
enum { ISP_SNS_NORMAL = 0, ISP_SNS_MIRROR_FLIP = 3 };

typedef struct {
    int32_t (*pfn_register_callback)(ot_vi_pipe, ot_isp_3a_alg_lib *, ot_isp_3a_alg_lib *);
    int32_t (*pfn_un_register_callback)(ot_vi_pipe, ot_isp_3a_alg_lib *, ot_isp_3a_alg_lib *);
    int32_t (*pfn_set_bus_info)(ot_vi_pipe, ot_isp_sns_commbus);
    void *rsv_12, *rsv_16, *rsv_20;
    void (*pfn_mirror_flip)(ot_vi_pipe, ot_isp_sns_mirrorflip_type);
    void *rsv_28, *rsv_32, *rsv_36, *rsv_40, *rsv_44;
} ot_isp_sns_obj;
_Static_assert(sizeof(ot_isp_sns_obj) == 48, "ot_isp_sns_obj");

typedef uint32_t ot_isp_bayer_format;
enum { OT_ISP_BAYER_RGGB = 0, OT_ISP_BAYER_BGGR = 3 };

typedef struct {
    uint32_t rsv_0;
    ot_rect mipi_crop_offset;
} ot_mipi_crop_attr;
_Static_assert(sizeof(ot_mipi_crop_attr) == 20, "ot_mipi_crop_attr");

typedef struct {
    ot_rect wnd_rect;
    ot_size sns_size;
    float frame_rate;
    ot_isp_bayer_format bayer_format;
    ot_wdr_mode wdr_mode;
    uint32_t rsv_36, rsv_40, rsv_44;
    ot_mipi_crop_attr mipi_crop_attr;
} ot_isp_pub_attr;
_Static_assert(sizeof(ot_isp_pub_attr) == 68, "ot_isp_pub_attr");

// Read and written back unchanged.
typedef struct {
    uint32_t rsv_0, rsv_4, rsv_8, rsv_12, rsv_16, rsv_20, rsv_24;
    uint32_t rsv_28, rsv_32, rsv_36, rsv_40, rsv_44, rsv_48;
} ot_isp_ctrl_param;
_Static_assert(sizeof(ot_isp_ctrl_param) == 52, "ot_isp_ctrl_param");

// Auto exposure: the route of exposure time against gain.
typedef struct {
    uint32_t int_time;
    uint32_t sys_gain;
    uint32_t rsv_8, rsv_12;
} ot_isp_ae_route_node;
_Static_assert(sizeof(ot_isp_ae_route_node) == 16, "ot_isp_ae_route_node");

typedef struct {
    uint32_t total_num;
    ot_isp_ae_route_node route_node[16];
} ot_isp_ae_route;
_Static_assert(sizeof(ot_isp_ae_route) == 260, "ot_isp_ae_route");

typedef struct {
    uint32_t exp_time;
    uint8_t rsv_4[4];
    uint8_t rsv_8[4152];
    uint32_t iso;
    uint8_t rsv_4164[1320];
} ot_isp_exp_info;
_Static_assert(sizeof(ot_isp_exp_info) == 5484, "ot_isp_exp_info");

// White balance.
typedef struct {
    uint16_t r_gain;
    uint16_t gr_gain;
    uint16_t gb_gain;
    uint16_t b_gain;
} ot_isp_mwb_attr;
_Static_assert(sizeof(ot_isp_mwb_attr) == 8, "ot_isp_mwb_attr");

typedef struct {
    uint8_t rsv_0[8];
    ot_op_mode op_type;
    ot_isp_mwb_attr manual_attr;
    uint8_t rsv_20[1280];
} ot_isp_wb_attr;
_Static_assert(sizeof(ot_isp_wb_attr) == 1300, "ot_isp_wb_attr");

typedef struct __attribute__((aligned(4))) {
    uint16_t r_gain;
    uint8_t rsv_2[4];
    uint16_t b_gain;
    uint8_t rsv_8[2];
    uint16_t color_temp;
    uint8_t rsv_12[44];
} ot_isp_wb_info;
_Static_assert(sizeof(ot_isp_wb_info) == 56, "ot_isp_wb_info");

// Colour space conversion.
typedef uint32_t ot_color_gamut;
enum { OT_COLOR_GAMUT_BT601 = 0 };

typedef struct {
    uint8_t rsv_0[4];
    ot_color_gamut color_gamut;
    uint8_t rsv_8[1];
    uint8_t luma;
    uint8_t contr;
    uint8_t satu;
    td_bool limited_range_en;
    uint8_t rsv_16[4];
    uint8_t rsv_20[36];
} ot_isp_csc_attr;
_Static_assert(sizeof(ot_isp_csc_attr) == 56, "ot_isp_csc_attr");

// Sharpening.
typedef struct {
    uint8_t rsv_0[302];
    uint8_t detail_ctrl;
    uint8_t detail_ctrl_threshold;
    uint8_t rsv_304[6];
    uint16_t max_sharp_gain;
    uint8_t rsv_312[108];
} ot_isp_sharpen_manual_attr;
_Static_assert(sizeof(ot_isp_sharpen_manual_attr) == 420, "ot_isp_sharpen_manual_attr");

typedef struct {
    td_bool enable;
    td_bool motion_en;
    uint8_t rsv_8[6];
    uint8_t skin_umin;
    uint8_t skin_vmin;
    uint8_t skin_umax;
    uint8_t skin_vmax;
    uint8_t rsv_18[2];
    ot_op_mode op_type;
    uint8_t rsv_24[4];
    ot_isp_sharpen_manual_attr manual_attr;
    uint8_t rsv_448[6720];
} ot_isp_sharpen_attr;
_Static_assert(sizeof(ot_isp_sharpen_attr) == 7168, "ot_isp_sharpen_attr");

// --- Video processing (VPSS) -------------------------------------------------

typedef int32_t ot_vpss_grp;
typedef int32_t ot_vpss_chn;

typedef uint32_t ot_vpss_dei_mode;
enum { OT_VPSS_DEI_MODE_OFF = 0 };

typedef struct {
    uint32_t rsv_0, rsv_4, rsv_8, rsv_12;
    uint32_t max_width;
    uint32_t max_height;
    uint32_t rsv_24, rsv_28;
    ot_dynamic_range dynamic_range;
    ot_pixel_format pixel_format;
    ot_vpss_dei_mode dei_mode;
    uint32_t rsv_44;
    ot_frame_rate_ctrl frame_rate;
} ot_vpss_grp_attr;
_Static_assert(sizeof(ot_vpss_grp_attr) == 56, "ot_vpss_grp_attr");

typedef uint32_t ot_vpss_chn_mode;
enum { OT_VPSS_CHN_MODE_USER = 1 };

typedef struct {
    uint32_t rsv_0, rsv_4, rsv_8;
    uint32_t width;
    uint32_t height;
    uint32_t rsv_20;
    ot_vpss_chn_mode chn_mode;
    ot_video_format video_format;
    ot_dynamic_range dynamic_range;
    ot_pixel_format pixel_format;
    ot_compress_mode compress_mode;
    ot_frame_rate_ctrl frame_rate;
    ot_border border_attr;
    uint32_t rsv_72, rsv_76, rsv_80, rsv_84, rsv_88, rsv_92;
} ot_vpss_chn_attr;
_Static_assert(sizeof(ot_vpss_chn_attr) == 96, "ot_vpss_chn_attr");

// --- Video encoder (VENC), H.265 ---------------------------------------------

#define OT_VENC_TEXTURE_THRESHOLD_SIZE 16
#define OT_VENC_MAX_PACK_INFO_NUM 8

typedef int32_t ot_venc_chn;

// The channel: encoder, rate control (CBR) and GOP.
typedef struct {
    td_bool rcn_ref_share_buf_en;
    uint32_t frame_buf_ratio;
} ot_venc_h265_attr;
_Static_assert(sizeof(ot_venc_h265_attr) == 8, "ot_venc_h265_attr");

typedef struct {
    ot_payload_type type;
    uint32_t max_pic_width;
    uint32_t max_pic_height;
    uint32_t buf_size;
    uint32_t profile;
    td_bool is_by_frame;
    uint32_t pic_width;
    uint32_t pic_height;
    union {                          // by the codec
        ot_venc_h265_attr h265_attr;
        uint8_t rsv_32[28];
    };
} ot_venc_attr;
_Static_assert(sizeof(ot_venc_attr) == 60, "ot_venc_attr");

typedef uint32_t ot_venc_rc_mode;
enum { OT_VENC_RC_MODE_H265_CBR = 14 };

typedef struct {
    uint32_t gop;
    uint32_t stats_time;
    uint32_t src_frame_rate;
    uint32_t dst_frame_rate;
    uint32_t bit_rate;
} ot_venc_h265_cbr;
_Static_assert(sizeof(ot_venc_h265_cbr) == 20, "ot_venc_h265_cbr");

typedef struct {
    ot_venc_rc_mode rc_mode;
    union {                          // by the mode
        ot_venc_h265_cbr h265_cbr;
        struct {
            uint32_t rsv_4, rsv_8, rsv_12, rsv_16, rsv_20, rsv_24, rsv_28;
            uint32_t rsv_32, rsv_36, rsv_40, rsv_44, rsv_48, rsv_52, rsv_56;
        } rsv_4;
    };
} ot_venc_rc_attr;
_Static_assert(sizeof(ot_venc_rc_attr) == 60, "ot_venc_rc_attr");

typedef uint32_t ot_venc_gop_mode;
enum { OT_VENC_GOP_MODE_NORMAL_P = 0 };

typedef struct {
    int32_t ip_qp_delta;
} ot_venc_gop_normal_p;
_Static_assert(sizeof(ot_venc_gop_normal_p) == 4, "ot_venc_gop_normal_p");

typedef struct {
    ot_venc_gop_mode gop_mode;
    union {                          // by the mode
        ot_venc_gop_normal_p normal_p;
        struct {
            uint32_t rsv_4, rsv_8, rsv_12, rsv_16, rsv_20, rsv_24;
        } rsv_4;
    };
} ot_venc_gop_attr;
_Static_assert(sizeof(ot_venc_gop_attr) == 28, "ot_venc_gop_attr");

typedef struct {
    ot_venc_attr venc_attr;
    ot_venc_rc_attr rc_attr;
    ot_venc_gop_attr gop_attr;
} ot_venc_chn_attr;
_Static_assert(sizeof(ot_venc_chn_attr) == 148, "ot_venc_chn_attr");

typedef struct {
    int32_t recv_pic_num;
} ot_venc_start_param;
_Static_assert(sizeof(ot_venc_start_param) == 4, "ot_venc_start_param");

// Rate control parameters.
typedef struct {
    uint32_t max_i_proportion;
    uint32_t rsv_4;
    uint32_t max_qp;
    uint32_t min_qp;
    uint32_t max_i_qp;
    uint32_t min_i_qp;
    uint32_t rsv_24, rsv_28;
    int32_t max_reencode_times;
    uint32_t rsv_36, rsv_40;
} ot_venc_h265_cbr_param;
_Static_assert(sizeof(ot_venc_h265_cbr_param) == 44, "ot_venc_h265_cbr_param");

typedef struct {
    uint32_t threshold_i[OT_VENC_TEXTURE_THRESHOLD_SIZE];
    uint32_t threshold_p[OT_VENC_TEXTURE_THRESHOLD_SIZE];
    uint8_t rsv_128[64];
    uint32_t direction;
    uint32_t row_qp_delta;
    int32_t first_frame_start_qp;
    uint8_t rsv_204[8];
    union {                          // by the mode
        ot_venc_h265_cbr_param h265_cbr_param;
        uint8_t rsv_212[68];
    };
} ot_venc_rc_param;
_Static_assert(sizeof(ot_venc_rc_param) == 280, "ot_venc_rc_param");

// Cleared and set as a whole.
typedef struct {
    uint32_t rsv_0;
} ot_venc_rc_adv_param;
_Static_assert(sizeof(ot_venc_rc_adv_param) == 4, "ot_venc_rc_adv_param");

typedef uint32_t ot_venc_scene_mode;

// Encoding tools.
typedef uint32_t ot_venc_intra_refresh_mode;
enum { OT_VENC_INTRA_REFRESH_ROW = 0, OT_VENC_INTRA_REFRESH_COLUMN = 1 };

typedef struct {
    td_bool enable;
    ot_venc_intra_refresh_mode mode;
    uint32_t refresh_num;
    uint32_t request_i_qp;
} ot_venc_intra_refresh;
_Static_assert(sizeof(ot_venc_intra_refresh) == 16, "ot_venc_intra_refresh");

typedef struct {
    td_bool enable;
    uint32_t split_mode;
    uint32_t split_size;
    td_bool slice_output_en;
} ot_venc_slice_split;
_Static_assert(sizeof(ot_venc_slice_split) == 16, "ot_venc_slice_split");

typedef struct {
    uint32_t idx;
    td_bool enable;
    td_bool is_abs_qp;
    int32_t qp;
    ot_rect rect;
} ot_venc_roi_attr;
_Static_assert(sizeof(ot_venc_roi_attr) == 32, "ot_venc_roi_attr");

typedef struct {
    ot_op_mode pred_mode;
    uint32_t intra32_cost;
    uint32_t intra16_cost;
    uint32_t intra8_cost;
    uint32_t intra4_cost;
    uint32_t inter64_cost;
    uint32_t inter32_cost;
    uint32_t inter16_cost;
    uint32_t inter8_cost;
} ot_venc_cu_pred;
_Static_assert(sizeof(ot_venc_cu_pred) == 36, "ot_venc_cu_pred");

typedef struct {
    td_bool enable;
    uint8_t rsv_4[4];
    uint8_t rsv_8[136];
} ot_venc_fg_protect;
_Static_assert(sizeof(ot_venc_fg_protect) == 144, "ot_venc_fg_protect");

// The VUI's video signal description.
typedef struct {
    uint8_t rsv_0, rsv_1;
    uint8_t video_full_range_flag;
    uint8_t colour_description_present_flag;
    uint8_t colour_primaries;
    uint8_t transfer_characteristics;
    uint8_t matrix_coefficients;
} ot_venc_vui_video_signal;
_Static_assert(sizeof(ot_venc_vui_video_signal) == 7, "ot_venc_vui_video_signal");

typedef struct {
    uint32_t rsv_0, rsv_4, rsv_8, rsv_12, rsv_16, rsv_20;
    ot_venc_vui_video_signal vui_video_signal;
    uint8_t rsv_31;
} ot_venc_h265_vui;
_Static_assert(sizeof(ot_venc_h265_vui) == 32, "ot_venc_h265_vui");

// The stream: packs (NAL units) and the frame's statistics.
typedef uint32_t ot_venc_h265_nalu_type;
enum { OT_VENC_H265_NALU_IDR_SLICE = 19 };

typedef union {
    ot_venc_h265_nalu_type h265_type;
} ot_venc_data_type;
_Static_assert(sizeof(ot_venc_data_type) == 4, "ot_venc_data_type");

typedef struct {
    ot_venc_data_type pack_type;
    uint32_t rsv_4, rsv_8;
} ot_venc_pack_info;
_Static_assert(sizeof(ot_venc_pack_info) == 12, "ot_venc_pack_info");

typedef struct {
    uint8_t rsv_0[4];
    uint8_t *addr;
    uint32_t len;
    uint8_t rsv_12[4];
    uint64_t pts;
    td_bool is_frame_end;
    uint8_t rsv_28[12];
    ot_venc_data_type data_type;
    uint32_t offset;
    uint32_t data_num;
    ot_venc_pack_info pack_info[OT_VENC_MAX_PACK_INFO_NUM];
    uint8_t rsv_148[4];
} ot_venc_pack;
_Static_assert(sizeof(ot_venc_pack) == 152, "ot_venc_pack");

typedef struct {
    uint32_t rsv_0, rsv_4, rsv_8, rsv_12, rsv_16;
    uint32_t intra32x32_cu_num;
    uint32_t intra16x16_cu_num;
    uint32_t intra8x8_cu_num;
    uint32_t intra4x4_cu_num;
    uint32_t rsv_36, rsv_40, rsv_44;
    uint32_t mean_qp;
    uint32_t rsv_52;
} ot_venc_h265_stream_info;
_Static_assert(sizeof(ot_venc_h265_stream_info) == 56, "ot_venc_h265_stream_info");

typedef struct {
    ot_venc_pack *pack;
    uint32_t pack_cnt;
    uint8_t rsv_8[4];
    union {                          // by the codec
        ot_venc_h265_stream_info h265_info;
        uint8_t rsv_12[68];
    };
    union { uint64_t rsv_80[55]; };  // by the codec too
} ot_venc_stream;
_Static_assert(sizeof(ot_venc_stream) == 520, "ot_venc_stream");

typedef struct {
    uint32_t left_pics;
    uint32_t left_stream_bytes;
    uint32_t left_stream_frames;
    uint32_t cur_packs;
    uint64_t rsv_16, rsv_24, rsv_32, rsv_40, rsv_48, rsv_56, rsv_64;
    uint64_t rsv_72, rsv_80, rsv_88, rsv_96, rsv_104, rsv_112;
} ot_venc_chn_status;
_Static_assert(sizeof(ot_venc_chn_status) == 120, "ot_venc_chn_status");

// --- Functions, by library ---------------------------------------------------
// Each returns TD_SUCCESS or an error code, but ss_mpi_venc_get_fd: an fd.

// libss_mpi: system, buffers, VI, VPSS, VENC
int32_t ss_mpi_sys_init(void);
int32_t ss_mpi_sys_exit(void);
int32_t ss_mpi_sys_init_pts_base(uint64_t pts_base_us);
int32_t ss_mpi_sys_set_vi_vpss_mode(const ot_vi_vpss_mode *mode);
int32_t ss_mpi_sys_set_vi_aiisp_mode(ot_vi_pipe pipe, ot_vi_aiisp_mode mode);

int32_t ss_mpi_vb_set_cfg(const ot_vb_cfg *cfg);
int32_t ss_mpi_vb_set_supplement_cfg(const ot_vb_supplement_cfg *cfg);
int32_t ss_mpi_vb_init(void);
int32_t ss_mpi_vb_exit(void);

int32_t ss_mpi_vi_set_dev_attr(ot_vi_dev dev, const ot_vi_dev_attr *attr);
int32_t ss_mpi_vi_enable_dev(ot_vi_dev dev);
int32_t ss_mpi_vi_disable_dev(ot_vi_dev dev);
int32_t ss_mpi_vi_bind(ot_vi_dev dev, ot_vi_pipe pipe);
int32_t ss_mpi_vi_unbind(ot_vi_dev dev, ot_vi_pipe pipe);
int32_t ss_mpi_vi_set_wdr_fusion_grp_attr(ot_vi_grp grp, const ot_vi_wdr_fusion_grp_attr *attr);
int32_t ss_mpi_vi_create_pipe(ot_vi_pipe pipe, const ot_vi_pipe_attr *attr);
int32_t ss_mpi_vi_destroy_pipe(ot_vi_pipe pipe);
int32_t ss_mpi_vi_start_pipe(ot_vi_pipe pipe);
int32_t ss_mpi_vi_stop_pipe(ot_vi_pipe pipe);
int32_t ss_mpi_vi_set_pipe_online_clock(ot_vi_pipe pipe, uint32_t clock_hz);
int32_t ss_mpi_vi_get_pipe_param(ot_vi_pipe pipe, ot_vi_pipe_param *param);
int32_t ss_mpi_vi_set_pipe_param(ot_vi_pipe pipe, const ot_vi_pipe_param *param);
int32_t ss_mpi_vi_set_pipe_frame_interrupt_attr(ot_vi_pipe pipe, const ot_frame_interrupt_attr *attr);
int32_t ss_mpi_vi_get_pipe_3dnr_attr(ot_vi_pipe pipe, ot_3dnr_attr *attr);
int32_t ss_mpi_vi_set_pipe_3dnr_attr(ot_vi_pipe pipe, const ot_3dnr_attr *attr);
int32_t ss_mpi_vi_get_pipe_3dnr_param(ot_vi_pipe pipe, ot_3dnr_param *param);
int32_t ss_mpi_vi_set_pipe_3dnr_param(ot_vi_pipe pipe, const ot_3dnr_param *param);
int32_t ss_mpi_vi_set_chn_attr(ot_vi_pipe pipe, ot_vi_chn chn, const ot_vi_chn_attr *attr);
int32_t ss_mpi_vi_enable_chn(ot_vi_pipe pipe, ot_vi_chn chn);
int32_t ss_mpi_vi_disable_chn(ot_vi_pipe pipe, ot_vi_chn chn);

int32_t ss_mpi_vpss_create_grp(ot_vpss_grp grp, const ot_vpss_grp_attr *attr);
int32_t ss_mpi_vpss_destroy_grp(ot_vpss_grp grp);
int32_t ss_mpi_vpss_start_grp(ot_vpss_grp grp);
int32_t ss_mpi_vpss_stop_grp(ot_vpss_grp grp);
int32_t ss_mpi_vpss_set_grp_frame_interrupt_attr(ot_vpss_grp grp, const ot_frame_interrupt_attr *attr);
int32_t ss_mpi_vpss_set_chn_attr(ot_vpss_grp grp, ot_vpss_chn chn, const ot_vpss_chn_attr *attr);
int32_t ss_mpi_vpss_set_chn_low_delay(ot_vpss_grp grp, ot_vpss_chn chn, const ot_low_delay_info *info);
int32_t ss_mpi_vpss_enable_chn(ot_vpss_grp grp, ot_vpss_chn chn);
int32_t ss_mpi_vpss_disable_chn(ot_vpss_grp grp, ot_vpss_chn chn);

int32_t ss_mpi_venc_create_chn(ot_venc_chn chn, const ot_venc_chn_attr *attr);
int32_t ss_mpi_venc_destroy_chn(ot_venc_chn chn);
int32_t ss_mpi_venc_get_chn_attr(ot_venc_chn chn, ot_venc_chn_attr *attr);
int32_t ss_mpi_venc_set_chn_attr(ot_venc_chn chn, const ot_venc_chn_attr *attr);
int32_t ss_mpi_venc_start_chn(ot_venc_chn chn, const ot_venc_start_param *param);
int32_t ss_mpi_venc_stop_chn(ot_venc_chn chn);
int32_t ss_mpi_venc_get_fd(ot_venc_chn chn);
int32_t ss_mpi_venc_query_status(ot_venc_chn chn, ot_venc_chn_status *status);
int32_t ss_mpi_venc_get_stream(ot_venc_chn chn, ot_venc_stream *stream, int32_t timeout_ms);
int32_t ss_mpi_venc_release_stream(ot_venc_chn chn, const ot_venc_stream *stream);
int32_t ss_mpi_venc_request_idr(ot_venc_chn chn, td_bool instant);
int32_t ss_mpi_venc_enable_idr(ot_venc_chn chn, td_bool enable);
int32_t ss_mpi_venc_get_rc_param(ot_venc_chn chn, ot_venc_rc_param *param);
int32_t ss_mpi_venc_set_rc_param(ot_venc_chn chn, const ot_venc_rc_param *param);
int32_t ss_mpi_venc_set_rc_adv_param(ot_venc_chn chn, const ot_venc_rc_adv_param *param);
int32_t ss_mpi_venc_set_scene_mode(ot_venc_chn chn, ot_venc_scene_mode mode);
int32_t ss_mpi_venc_set_intra_refresh(ot_venc_chn chn, const ot_venc_intra_refresh *refresh);
int32_t ss_mpi_venc_set_slice_split(ot_venc_chn chn, const ot_venc_slice_split *split);
int32_t ss_mpi_venc_get_roi_attr(ot_venc_chn chn, uint32_t idx, ot_venc_roi_attr *attr);
int32_t ss_mpi_venc_set_roi_attr(ot_venc_chn chn, const ot_venc_roi_attr *attr);
int32_t ss_mpi_venc_get_cu_pred(ot_venc_chn chn, ot_venc_cu_pred *pred);
int32_t ss_mpi_venc_set_cu_pred(ot_venc_chn chn, const ot_venc_cu_pred *pred);
int32_t ss_mpi_venc_get_fg_protect(ot_venc_chn chn, ot_venc_fg_protect *protect);
int32_t ss_mpi_venc_set_fg_protect(ot_venc_chn chn, const ot_venc_fg_protect *protect);
int32_t ss_mpi_venc_get_h265_vui(ot_venc_chn chn, ot_venc_h265_vui *vui);
int32_t ss_mpi_venc_set_h265_vui(ot_venc_chn chn, const ot_venc_h265_vui *vui);

// libss_mpi_isp
int32_t ss_mpi_isp_mem_init(ot_vi_pipe pipe);
int32_t ss_mpi_isp_init(ot_vi_pipe pipe);
int32_t ss_mpi_isp_run(ot_vi_pipe pipe);
int32_t ss_mpi_isp_exit(ot_vi_pipe pipe);
int32_t ss_mpi_isp_set_bind_attr(ot_vi_pipe pipe, const ot_isp_bind_attr *attr);
int32_t ss_mpi_isp_set_pub_attr(ot_vi_pipe pipe, const ot_isp_pub_attr *attr);
int32_t ss_mpi_isp_get_ctrl_param(ot_vi_pipe pipe, ot_isp_ctrl_param *param);
int32_t ss_mpi_isp_set_ctrl_param(ot_vi_pipe pipe, const ot_isp_ctrl_param *param);
int32_t ss_mpi_isp_get_csc_attr(ot_vi_pipe pipe, ot_isp_csc_attr *attr);
int32_t ss_mpi_isp_set_csc_attr(ot_vi_pipe pipe, const ot_isp_csc_attr *attr);
int32_t ss_mpi_isp_get_sharpen_attr(ot_vi_pipe pipe, ot_isp_sharpen_attr *attr);
int32_t ss_mpi_isp_set_sharpen_attr(ot_vi_pipe pipe, const ot_isp_sharpen_attr *attr);

// libss_mpi_ae
int32_t ss_mpi_ae_register(ot_vi_pipe pipe, const ot_isp_3a_alg_lib *lib);
int32_t ss_mpi_ae_unregister(ot_vi_pipe pipe, const ot_isp_3a_alg_lib *lib);
int32_t ss_mpi_isp_set_ae_route_attr(ot_vi_pipe pipe, const ot_isp_ae_route *route);
int32_t ss_mpi_isp_query_exposure_info(ot_vi_pipe pipe, ot_isp_exp_info *info);

// libss_mpi_awb
int32_t ss_mpi_awb_register(ot_vi_pipe pipe, const ot_isp_3a_alg_lib *lib);
int32_t ss_mpi_awb_unregister(ot_vi_pipe pipe, const ot_isp_3a_alg_lib *lib);
int32_t ss_mpi_isp_get_wb_attr(ot_vi_pipe pipe, ot_isp_wb_attr *attr);
int32_t ss_mpi_isp_set_wb_attr(ot_vi_pipe pipe, const ot_isp_wb_attr *attr);
int32_t ss_mpi_isp_query_wb_info(ot_vi_pipe pipe, ot_isp_wb_info *info);
int32_t ss_mpi_isp_cal_gain_by_temp(ot_vi_pipe pipe, const ot_isp_wb_attr *attr,
                                    uint16_t color_temp, int16_t shift, uint16_t *gains);

// libss_mpi_sysbind
int32_t ss_mpi_sys_bind(const ot_mpp_chn *src, const ot_mpp_chn *dst);
int32_t ss_mpi_sys_unbind(const ot_mpp_chn *src, const ot_mpp_chn *dst);
