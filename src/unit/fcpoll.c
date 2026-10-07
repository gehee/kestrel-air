#include "unit/fcpoll.h"

#include <string.h>

// Requests a second; they add up to FCPOLL_SLOTS_PER_SEC. All change slowly: the HUD
// leans with the camera's IMU, not with the flight controller's attitude. (Stock polls
// only status, the two versions and analog.)
static const struct { uint8_t cmd, per_sec; } rows[] = {
    { FCP_MSP_STATUS,        14 },
    { FCP_MSP_BATTERY_STATE,  6 },
    { FCP_MSP_ALTITUDE,       6 },
    { FCP_MSP_RAW_GPS,        5 },
    { FCP_MSP_COMP_GPS,       5 },
    { FCP_MSP_ANALOG,         4 },
    { FCP_MSP_BOXIDS,         4 },
    { FCP_MSP_FC_VERSION,     3 },
    { FCP_MSP_FC_VARIANT,     3 },
};
#define NROWS ((int)(sizeof(rows) / sizeof(rows[0])))

void fcpoll_init(fcpoll_t *p) { memset(p, 0, sizeof(*p)); }

// Each slot every row earns its rate in credit; the one with the most is asked and
// pays a whole second's worth of slots. Rates that sum to the slots in a second keep
// every credit bounded, and a row asked n times a second is asked every 1/n.
uint8_t fcpoll_next(fcpoll_t *p) {
    int best = 0;
    for (int i = 0; i < NROWS; i++) {
        p->credit[i] += rows[i].per_sec;
        if (p->credit[i] > p->credit[best]) best = i;
    }
    p->credit[best] -= FCPOLL_SLOTS_PER_SEC;
    return rows[best].cmd;
}
