// What video.c (packets, ring, sender) and encoder.c (the encoder channel)
// share. Not for other modules: they use video/video.h.
#pragma once
#include <pthread.h>
#include <stdint.h>

#define CHN 1   // the encoder channel

extern int width, height, fps, src_fps;      // fps: encoded, src_fps: sensor
extern pthread_mutex_t venc_mtx;
extern volatile int venc_running, venc_kbps, venc_fps;
extern volatile int venc_recreate;           // tune_poll asks the read loop for a new channel
extern int roi_cache[4];
extern volatile uint32_t left_frames;        // encoder backlog, as the drop thread sees it
extern volatile int busy;                    // bit1 local backlog, bit2 link down
extern volatile int cmd_req, cmd_enable;     // send start / stop, taken by the read thread
extern uint16_t fc_bitmap;                   // flow-control bitmap

// Live tuning (/tmp/ka-tune, see encoder.c).
enum { T_IR, T_IR_QP, T_GOP, T_ROW_QP, T_I_PROP, T_MIN_QP, T_MAX_QP, T_CU, T_IR_MODE, T_ROI,
       T_FGP, T_THR, T_SCENE, T_IRLOG, T_N };
extern int tune[T_N];

typedef struct {
    int type;           // 0 P, 1 IDR, 3 parameter sets / SEI
    uint64_t pts;
    uint32_t len;
    int frame_end;
} vframe;

int  lat_info(void);
void tune_poll(void);
int  venc_create(int kbps);
void venc_start(void);
void venc_stop_destroy(void);
void roi_update(int en);
void set_venc_bitrate(int f, int kbps, int reason);
void venc_down(int reason);
// The next packet (a slice) into dst: 1 got one, 0 none within timeout_ms, -1 error.
int  stream_get(uint8_t *dst, uint32_t room, vframe *out, int timeout_ms);
