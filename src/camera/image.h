// The camera settings the ground can change, and the ISP state the period
// telemetry reports (see image.c).
#pragma once
#include <stdint.h>

typedef struct {
    int pq_index;            // pq bin index + 1
    uint32_t exp_time_us;
    float iso;               // exp_info +0x1040 / 100
    uint32_t color_temp;
    float r_gain, b_gain;    // wb gains / 256
} image_isp_info;
extern unsigned image_max_exposure_us;   // --max-exposure-us, 0 = stock
int  image_start(int sensor_fps, int angle_deg);
void image_pause(void);                  // around a pipeline restart
void image_restart(int sensor_fps, int angle_deg);
void image_get_info(image_isp_info *i);
void image_apply_all(void);
void image_set_scene(int v);
void image_set_ev(int ev_x10);
void image_set_sat(int v);
void image_set_sharpness(int v);
void image_set_awb(int cct);
void image_set_contrast(int v);
void image_set_3dnr(int menu);
void image_set_2dnr(int menu);
void image_low_power(int on);
// Ours: the cap on the auto exposure, changed while it runs (KA_CMD_MAX_EXP),
// where the backend can (image_max_exposure_live; its info then says maxexp=1).
int  image_max_exposure_live(void);
int  image_set_max_exposure(unsigned us);   // 0: none; -1 where not live
// Ours: the camera settings this backend applies, by the goggle's names
// (rotate, scene, ev, wb, sat, contrast, sharpness, 3dnr, focus), comma
// separated, for its info (cam=); NULL: all of them.
const char *image_settings(void);
