// The video path: the encoder channel, the packets around each slice, the
// local ring and the sender to the radio (see video.c).
#pragma once
#include <stdint.h>

int  video_start(void);                  // the video read and send threads
void video_stop(void);
void video_link(int up);                 // radio link events mark the send path busy
extern int video_verbose;                // --bb-verbose
extern int video_window;                 // --bb-window: unacknowledged video writes allowed
extern volatile int video_stream_flag;   // 1 got a packet, 0 a 50 ms wait came up empty
void video_set_bitrate(int kbps);
void video_scale_bitrate(int num, int den);
void video_enable_idr(int enable);
int video_tune_armed(void);  // /tmp/ka-tune armed=1: the FC taken as armed (bench)
int video_tune_pwr(void);    // /tmp/ka-tune pwr=N: TX power N dBm (13..26) while armed, -1 none
int video_tune_pathb(void);  // /tmp/ka-tune pathb=0: the second TX path off while armed
int  video_kbps(void);      // the encoder's bitrate now
int  video_tgt_fps(void);
int  video_send_start(void);
int  video_send_stop(void);
void video_set_format(int w, int h, int fps, int sensor_fps);
int  video_low_delay_lines(void);        // for camera_set_low_delay_lines
// Strips (KA_CMD_STRIPS): the strips a picture is to be encoded in (0 or 1:
// whole pictures), taken at the next channel; how many it is in now and the
// most this backend encodes (1: none).
void video_set_strips(int n);
void video_strips(int *now, int *max);

// What this air app can do (KA_FEAT_* in protocol/kestrel_air.h), announced
// in the version message and every slice header.
uint8_t video_features(void);
