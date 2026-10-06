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
int  video_tgt_fps(void);
int  video_send_start(void);
int  video_send_stop(void);
void video_set_format(int w, int h, int fps, int sensor_fps);
int  video_low_delay_lines(void);        // for cv610_set_low_delay_lines

// What this air app can do (KA_FEAT_* in protocol/kestrel_air.h), announced
// in the version message and every slice header.
uint8_t video_features(void);
