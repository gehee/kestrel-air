// What the files of the radio module share. Not for other modules: they use
// radio/radio.h.
#pragma once
#include <stdint.h>

#include "radio/radio.h"

#define IOCTL_MS 1000

extern radio_hooks r_hooks;
extern volatile int r_connected, r_mcs;
extern volatile int r_pairing;               // pairing in progress: the battery ADC waits
extern int r_adc_state;

// The channel plan (channels.c).
extern int r_nfreq;
extern uint32_t r_freq[128];
extern int r_is_hop, r_hop_en, r_freq_pinned;
extern uint32_t r_air_freq;
extern int r_gnd_n;
extern const uint32_t r_common_list[1];      // the radio board's common work channel
int  r_in_list(uint32_t f, const uint32_t *l, int n);
void r_init_comm_work_chan(void);
void r_set_work_chan_list(void);

// The video link's bandwidth (bandwidth.c).
extern volatile int r_bw_cur, r_bw_prev;     // gear now; before our last change (-1: none pending)
extern volatile uint64_t r_bw_at;            // when we changed it
extern volatile int r_max_kbps;              // the ground's bitrate cap, 0 = none
void r_bw_tick(uint64_t t);

// Requests to the radio (radio.c).
int  r_get(uint16_t id, const void *in, uint32_t inlen, void *out, uint32_t outlen);
int  r_set(uint16_t id, const void *in, uint32_t inlen);
// Open8030's frame info (request 0x0c5a0035, its ar_rf_frame_info): the AP's
// clock at its latest frame interrupt (IRQ 5). Only asked of Open8030.
typedef struct {
    uint32_t version, flags, frames, frame_ticks, since_us, ap_us_lo, ap_us_hi;
    uint32_t taken, rejected;
    int32_t residual_us;
    uint32_t width, frame_mode, sent;
} r_frame_info_t;
int  r_frame_info(r_frame_info_t *out);
// The radio runs Open8030 (the firmware start.sh had arlink load).
int  r_is_open8030(void);
int  r_dispatch(uint8_t cmd, const uint8_t *args, int nargs);
void r_set_rf_path_b(int enable);
void r_set_adc_meas(int chn, uint32_t period_ms);
void r_set_fem(int on);                     // the Lite+'s high-power front end
int  r_get_adc(int chn);

void *r_adc_thread(void *arg);               // adc.c
void *r_standby_thread(void *arg);           // power.c
