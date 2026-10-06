// What the modules share: flags set in one thread and read in others.
#pragma once
#include <stdatomic.h>

typedef struct {
    atomic_int stream_pause;     // 1 at boot and after a link loss; the ground's TRIG clears it
    atomic_int venc_running;     // the encoder channel exists
    atomic_int low_power;        // the standby low-power mode is on
    atomic_int cam_flag;         // camera settings to resend to the ground
    atomic_int cpu_temp_x100;    // the SoC's temperature, C x 100
    atomic_int cpu_hot;          // 90..110 C: the LED warning
    atomic_uint venc_get_fail;   // encoder reads that failed
    atomic_uint vsend_fail;      // radio writes that came up short
} app_state;

extern app_state shared;
