// The radio chip's ADC: the battery voltage, and the SoC temperature.
#include "radio/internal.h"

#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "app/config.h"
#include "platform/platform.h"
#include "app/state.h"
#include "common/clock.h"

static int adc_n, adc_filt, adc_offset;

// The battery voltage, every 500 ms: ten ADC reads on channel 3
// calibrate an offset (to 900), then channel 0 is the battery.
static void update_batt(void) {
    if (r_adc_state == 1) {
        int v = r_get_adc(3);
        if (v > 100) {
            adc_n++;
            adc_filt = adc_filt ? (adc_filt * (0x400 - 0x100) + v * 0x100) >> 10 : v;
        }
        if (adc_n > 9) {
            r_adc_state = 2;
            adc_offset = 900 - adc_filt;
            r_dispatch(0x8b, NULL, 0);
            r_set_adc_meas(0, 300);
            adc_filt = 0;
            printf("radio: adc offset=%d\n", adc_offset);
        }
    } else if (r_adc_state == 2 && r_connected && !r_pairing) {
        int v = r_get_adc(0);
        adc_filt = adc_filt ? (adc_filt * 0x300 + v * 0x100) >> 10 : v;
    }
}


int radio_batt_mv(void) {
    int mv = platform_supply_mv();   // the SoC's own ADC, where the board has it there (P1)
    if (mv >= 0) return mv;
    if (r_adc_state != 2) return 0;
    return ((adc_offset + 900) * adc_filt / 900) * 16 + 200;
}


// The SoC's temperature sensor, read through /dev/mem as stock reads it.
// The SoC temperature, every 500 ms: the SoC temperature (at
// 110 C stock reboots the unit, and so does this), then the battery ADC.
void *r_adc_thread(void *arg) {
    (void)arg;
    for (;;) {
        int t = platform_cpu_temp_x100();
        shared.cpu_temp_x100 = t;
        if (t != INT_MIN && t < 11000) shared.cpu_hot = t >= 9000;   // the LED warning, nothing else
        if (t != INT_MIN && t >= 11000) {
            printf("radio: SoC at %d.%02d C: rebooting\n", t / 100, t % 100);
            if (system("reboot") != 0) { /* nothing more to do */ }
            sleep(2);
        }
        update_batt();
        usleep(500000);
    }
    return NULL;
}
