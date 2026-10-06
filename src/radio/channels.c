// The channel plan: the work channel list, the frequency, the ground's list.
#include "radio/internal.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "app/config.h"
#include "app/state.h"
#include "common/clock.h"

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static uint32_t r_gnd_list[64];

int r_in_list(uint32_t f, const uint32_t *l, int n) {
    for (int i = 0; i < n; i++)
        if (l[i] == f) return 1;
    return 0;
}

static int chan_index(uint32_t f) {
    for (int i = 0; i < r_nfreq; i++)
        if (r_freq[i] == f) return i;
    return -1;
}

void r_init_comm_work_chan(void) {
    uint8_t b[129];
    memset(b, 0, sizeof(b));
    for (int i = 0; i < r_nfreq && b[0] < 128; i++)
        if (r_in_list(r_freq[i], r_common_list, 1)) b[1 + b[0]++] = (uint8_t)i;
    r_set(0x0023, b, sizeof(b));
}

void r_set_work_chan_list(void) {
    uint8_t b[129];
    memset(b, 0, sizeof(b));
    for (int i = 0; i < r_nfreq && b[0] < 128; i++) {
        uint32_t f = r_freq[i];
        if (!r_in_list(f, r_gnd_list, r_gnd_n)) continue;
        if (r_hop_en && (f >= 7000000 || (f >= 4000000 && f <= 4200000))) continue;
        if (!r_hop_en && r_in_list(f, r_common_list, 1)) continue;
        b[1 + b[0]++] = (uint8_t)i;
    }
    if (b[0]) r_set(0x0023, b, sizeof(b));
}

int radio_set_freq(int hop, uint32_t f) {
    pthread_mutex_lock(&mtx);
    r_is_hop = hop;
    pthread_mutex_unlock(&mtx);
    int idx = f ? chan_index(f) : -1;
    if (idx < 0) return -1;
    if (!hop && !r_in_list(f, r_gnd_list, r_gnd_n)) {   // deferred until the list has it
        r_hop_en = 0;
        r_freq_pinned = 1;
        r_air_freq = f;
        return 0;
    }
    uint8_t m = hop ? 1 : 0;
    usleep(200000);
    r_set(0x0005, &m, 1);
    r_hop_en = hop;
    usleep(150000);
    r_set_work_chan_list();
    usleep(150000);
    r_freq_pinned = 1;
    if (!hop) {
        uint8_t c[2] = { 1, (uint8_t)idx };
        r_set(0x0006, c, 2);
        r_air_freq = f;
    }
    return 0;
}

void radio_set_gnd_work_list(const uint32_t *f, int n) {
    if (n > 64) n = 64;
    memcpy(r_gnd_list, f, n * sizeof(*f));
    r_gnd_n = n;
    if (r_hop_en || r_in_list(r_air_freq, r_gnd_list, r_gnd_n)) r_set_work_chan_list();
}
