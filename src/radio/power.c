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
#include "radio/power_levels.h"
#include "unit/model.h"
#include "video/video.h"

// mW to the radio's level on this board: stock's table, its labels 1 dB
// above the level (kept: the board's gain after the chip is not known; a
// "nearest dBm" table would run it about 1 dB over its labels). Values under
// 20 are taken as dBm. Nothing above 500 mW (26): the 630 / 790 mW TEST
// levels (27 / 28) are gone - at 27 the SNR at the goggle was 1.5-4.7 dB under
// 200 mW's (flight 2026-10-30, Lite+): the amplifier distorts past ~26.
static int power_dbm(int p) {
    if (p < 20) return p;
    if (p >= 1013 && p <= 1026) return p - 1000;   // /tmp/ka-tune pwr=N (bench): N dBm
    const struct power_level *l = power_level_of(p);
    return l ? l->dbm : -1;
}

static volatile int applied_dbm = -1;
int radio_tx_dbm(void) { return applied_dbm; }

static void set_local_power(int p) {
    int dbm = power_dbm(p);
    if (dbm < 0) {
        printf("radio: no power level for %d\n", p);
        return;
    }
    // Each unit's top level (model_max_dbm): the Ascent Lite has no
    // high-power front end, 100 mW (19 dBm) as stock offers it; the P1 Air
    // stops at 23 as stock's radio config does. A 500 mW request (the
    // ground's stock 500mW Auto, or any value not in the map, taken as that)
    // asked them for 26 dBm.
    if (dbm > model_max_dbm()) {
        printf("radio: %d dBm is above this unit's %d dBm: %d dBm\n", dbm, model_max_dbm(), model_max_dbm());
        dbm = model_max_dbm();
    }
    // AUTO (stock's N+1: 101, 501, and ours 151, 201, 317): the radio takes
    // the ground's power requests between 25 mW and the level (an Open8030
    // radio: SET 9 {2, max, min} with min below max); a fixed level is a
    // range of one.
    const struct power_level *lvl = power_level_of(p);
    const int automatic = lvl && lvl->automatic;
    const int min_dbm = automatic && dbm > 13 ? 13 : dbm;
    uint8_t a[3] = { 2, (uint8_t)dbm, (uint8_t)min_dbm }, b[2] = { 8, (uint8_t)dbm };
    // Retried until the radio takes them; a refusal is said once, with its
    // code, instead of looping silently (a level the radio's calibration
    // has no row for is refused for good).
    int e, tries = 0;
    while ((e = r_set(0x0009, a, 3)) != 0) {
        if (tries++ == 0) printf("radio: TX power %d dBm: SET 9 refused (%d), retrying\n", dbm, e);
        usleep(10000);
    }
    tries = 0;
    while ((e = r_set(0x0008, b, 2)) != 0) {
        if (tries++ == 0) printf("radio: TX power %d dBm: SET 8 refused (%d), retrying\n", dbm, e);
        usleep(10000);
    }
    printf("radio: TX power %d dBm (%d mW asked)%s\n", dbm, p, automatic ? ", AUTO from 13 dBm" : "");
    applied_dbm = dbm;
    // The Lite+'s front end goes to high power above 23 dBm (stock: 500 mW).
    if (model_lite_plus()) {
        r_set_fem(dbm > 23);
        if (dbm > 23) usleep(50000);
    }
}

void *r_standby_thread(void *arg) {
    (void)arg;
    // Standby's level, in dBm. 13, not 10: at 10 dBm the radio's TX power
    // reading (TSSI, which its closed loop follows) swings +-12% and the
    // ground's gain jumps 6 dB or more from frame to frame - 3.6% of frames
    // failed and 4.7% were sent again on the bench (40 MHz MCS 8, ~1 m),
    // against 1.3% / 1.3% from 13 dBm up (2026-11-02).
    const int low = 13;
    int cur = -1, low_power = 0, first = 1, cur_pathb = 1;
    for (;;) {
        if (!first && !r_connected) {
            while (!r_connected) usleep(30000);
            usleep(100000);
        }
        first = 0;
        int want = cfg_get("bb_power_mw", 25);
        int flying = r_hooks.flying && r_hooks.flying();
        // Bench sweeps (/tmp/ka-tune): a power in dBm and the second path.
        if (flying && video_tune_pwr() > 0) want = 1000 + video_tune_pwr();
        const int pathb = flying ? video_tune_pathb() : 1;
        int normal;
        if (flying) normal = low_power || want != cur || pathb != cur_pathb ? 1 : -1;
        else if (cfg_get("sys_standby_mode", 1) == 0) normal = want != cur || low_power ? 1 : -1;
        else normal = !(cur == low && low_power) ? 0 : -1;

        if (normal == 0) {            // into low power, the sensor too
            printf("radio: low power, %d dBm\n", low);
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
            printf("radio: normal power, %d mW (%s)\n", want, flying ? "flying" : "standby mode off");
            image_low_power(0);
            shared.low_power = 0;
            r_set_rf_path_b(pathb);
            usleep(100000);
            uint8_t a[3] = { 3, 0, 1 };
            r_set(0x0009, a, 3);
            set_local_power(want);
            if (pathb != 1) printf("radio: second TX path off (bench)\n");
            cur = want; cur_pathb = pathb;
            low_power = 0;
        }
        usleep(30000);
    }
    return NULL;
}
