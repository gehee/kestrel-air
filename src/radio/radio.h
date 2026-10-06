#pragma once
#include <stdint.h>

#include "radio/client.h"

// The air side of the AR8030 radio, run the way the stock air app runs it
// (see radio.c): its radio setup, event handling, periodic register and ADC
// reads, TX power, and what it tells the video path.

typedef struct {
    // Called on an MCS drop, as stock lowers the encoder at once.
    void (*set_bitrate)(int kbps);
    // The encoder's frame rate target.
    int (*tgt_fps)(void);
    // Whether the flight controller reports armed (TX power leaves low power).
    int (*flying)(void);
} radio_hooks;

int  radio_start(const radio_hooks *hooks);   // blocks until set up
extern bbc_sock radio_video, radio_ctrl;      // socket ports 3 and 2 of slot 0

int  radio_connected(void);
int  radio_mcs(void);
int  radio_throughput(void);
int  radio_reconnect_count(void);
int  radio_retx_too_many(void);
int  radio_tgt_bitrate(void);             // kbps
void radio_set_fps_coef(int coef);        // x10; 10 unless exposure-limited
int  radio_ringbuf_left(void);            // bytes queued in the radio for port 3, -1 on error
int  radio_wireless_time(uint32_t *ms);   // the radio's AP clock
extern volatile uint32_t radio_total_video_send;   // reset on each link-up

// Values the ground telemetry reports.
typedef struct {
    uint8_t tssi_a, tssi_b, gain_a, gain_b;
    int8_t ofs_a, ofs_b;
} radio_period;
void radio_period_info(radio_period *p);     // the register block, ~every 270 ms
int  radio_chan_info(uint8_t *reply, int max);   // GET_CHAN_INFO + channel monitor
int  radio_batt_mv(void);                 // filtered ADC, adc_offset applied (see radio.c)

// Pairing (the key, MSP 190, debug command 0x86); relay mode (SG 0x2d).
int  radio_match(int timeout_ms, uint8_t mac[4]);
int  radio_relay_mode(void);
int  radio_relay_vbuf(void);
int  radio_relay_bb_left(void);
void radio_relay_update(const uint8_t *b, int n);
void radio_set_max_kbps(int kbps);         // the goggle's cap on the video bitrate, 0 = none (sky cmd 0x40)
void radio_bandwidth_request(int gear);   // video link TX bandwidth, 0..5 = 1.25..40 MHz (sky cmd 0x24)
void radio_set_max_bw(int mhz);           // the goggle's cap on the video link bandwidth, 20 or 40 MHz (sky cmd 0x41)
int radio_bw40_enabled(void);   // the step up to 40 MHz is on (KA_BW40=1)
extern volatile int radio_init_failed;   // the radio did not start (LED pattern)

// Ground commands that reach the radio.
int  radio_set_freq(int is_hop, uint32_t freq_khz);
void radio_set_gnd_work_list(const uint32_t *freqs, int n);
void radio_set_mcs_policy(void);           // after video_strategy changes
void radio_set_uart_mode(int mode);        // ground msg 0x29
