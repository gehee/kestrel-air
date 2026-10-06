// Pairing with a ground in pairing mode.
#include "radio/internal.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "app/config.h"
#include "app/state.h"
#include "common/clock.h"

// Look for a ground in pairing mode for timeout_ms; on success hand the
// radio its MAC as the candidate and return it in mac. The whole request
// sequence is stock's, including what it restores (only on failure).
int radio_match(int timeout_ms, uint8_t mac[4]) {
    uint8_t q[2] = { 0, 0 }, st[300], wl[129], r[164], cmd[14];
    uint8_t prev[8][4];
    int count[8], saved_gnd_n = r_gnd_n, get_failed = 0, found = 0, ok = 0;
    const char *err = NULL;

    if (r_get(0x0000, q, 2, st, sizeof(st)) != 0) {
        puts("radio: pairing: err-2");
        return -1;
    }
    printf("radio: pairing: start, slot bitmap 0x01, role %s\n", st[0] == 0 ? "AP" : "DEV");
    r_pairing = 1;
    memset(wl, 0, sizeof(wl));
    r_get(0x0012, NULL, 0, wl, sizeof(wl));
    r_init_comm_work_chan();

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 1; cmd[1] = 1;
    if (r_set(0x0002, cmd, sizeof(cmd)) != 0) { err = "radio: pairing: err-3"; goto restore; }
    r_gnd_n = 0;
    printf("radio: pairing: listening for %d ms\n", timeout_ms);

    memset(prev, 0, sizeof(prev));
    memset(count, 0, sizeof(count));
    memset(r, 0, sizeof(r));
    for (int remaining = timeout_ms; remaining > 0 && !found; remaining -= 20) {
        if (r_get(0x0001, NULL, 0, r, sizeof(r)) != 0) {
            puts("radio: pairing: err-4");
            get_failed = 1;
            break;
        }
        for (int i = 0; i < 8; i++) {
            if (!(r[0] >> i & 1)) continue;
            if (!memcmp(prev[i], r + 1 + 4 * i, 4)) count[i]++;
            else { memcpy(prev[i], r + 1 + 4 * i, 4); count[i] = 1; }
            printf("radio: pairing: slot %d heard %d times\n", i, count[i]);
            if (i == 0 && count[0] > 10) { found = 1; break; }
        }
        usleep(20000);
    }
    if (found) usleep(500000);

    cmd[0] = 0;                               // exit pairing: only byte 0 changes
    if (r_set(0x0002, cmd, sizeof(cmd)) != 0) { err = "radio: pairing: err-5"; goto restore; }
    r_pairing = 0;
    if (get_failed) { err = "radio: pairing: err-6"; goto restore; }

    for (int i = 0; i < 8; i++) {
        if (!(r[0] >> i & 1)) continue;
        printf("radio: pairing: slot %d heard %d times\n", i, count[i]);
        if (count[i] <= 3) continue;
        const uint8_t *m = r + 1 + 4 * i, *info = r + 36 + 16 * i;
        printf("radio: pairing: found ground %02X%02X%02X%02X in slot %u csi: (%u, %u, %u, %u, %u)\n", m[0], m[1], m[2], m[3], i,
               info[0] | info[1] << 8, info[6], info[7], info[2] | info[3] << 8, info[4] | info[5] << 8);
        uint8_t c[402];
        memset(c, 0, sizeof(c));
        c[0] = (uint8_t)i;
        c[1] = 1;
        memcpy(c + 2, m, 4);
        if (r_set(0x0004, c, sizeof(c)) != 0) { err = "radio: pairing: err-7"; goto restore; }
        memcpy(mac, m, 4);
        ok = 1;
    }
    if (!ok) { err = "radio: pairing: err-9"; goto restore; }
    r_air_freq = 0;
    r_freq_pinned = 0;
    puts("radio: pairing: done");
    return 0;

restore:
    puts(err);
    r_gnd_n = saved_gnd_n;
    puts("radio: pairing: restoring the work channel list");
    if (r_set(0x0023, wl, sizeof(wl)) != 0) puts("radio: pairing: restoring the work channel list failed");
    return -1;
}
