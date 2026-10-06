// Transmit power and the standby low-power mode.
#include "radio/internal.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "app/config.h"
#include "app/state.h"
#include "camera/image.h"
#include "common/clock.h"

// mW to dBm on this board; values under 20 are taken as dBm.
static int power_dbm(int p) {
    static const int tab[][2] = {
        { 25, 13 }, { 100, 19 }, { 150, 21 }, { 200, 22 }, { 500, 26 }, { 101, 19 }, { 501, 26 },
    };
    if (p < 20) return p;
    for (unsigned i = 0; i < sizeof(tab) / sizeof(tab[0]); i++)
        if (tab[i][0] == p) return tab[i][1];
    return -1;
}

static void set_local_power(int p) {
    int dbm = power_dbm(p);
    if (dbm < 0) {
        printf("radio: no power level for %d\n", p);
        return;
    }
    uint8_t a[3] = { 2, (uint8_t)dbm, (uint8_t)dbm }, b[2] = { 8, (uint8_t)dbm };
    while (r_set(0x0009, a, 3) != 0) usleep(10000);
    while (r_set(0x0008, b, 2) != 0) usleep(10000);
}

void *r_standby_thread(void *arg) {
    (void)arg;
    const int low = 10;
    int cur = -1, low_power = 0, first = 1;
    for (;;) {
        if (!first && !r_connected) {
            while (!r_connected) usleep(30000);
            usleep(100000);
        }
        first = 0;
        int want = cfg_get("bb_power_mw", 25);
        int flying = r_hooks.flying && r_hooks.flying();
        int normal;
        if (flying) normal = low_power || want != cur ? 1 : -1;
        else if (cfg_get("sys_standby_mode", 1) == 0) normal = want != cur || low_power ? 1 : -1;
        else normal = !(cur == low && low_power) ? 0 : -1;

        if (normal == 0) {            // into low power, the sensor too
            printf("radio: low power, target %d mW, now %d mW\n", low, cur);
            image_low_power(1);
            shared.low_power = 1;
            uint8_t a[3] = { 3, 0, 0 };
            r_set(0x0009, a, 3);
            set_local_power(low);
            usleep(5000);
            r_set_rf_path_b(0);
            cur = low;
            low_power = 1;
        } else if (normal == 1) {     // out of it
            image_low_power(0);
            shared.low_power = 0;
            r_set_rf_path_b(1);
            usleep(100000);
            uint8_t a[3] = { 3, 0, 1 };
            r_set(0x0009, a, 3);
            set_local_power(want);
            cur = want;
            low_power = 0;
        }
        usleep(30000);
    }
    return NULL;
}
