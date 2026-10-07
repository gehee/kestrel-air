#include "unit/fcpoll.h"

#include <string.h>

// Requests a second; each table adds up to FCPOLL_SLOTS_PER_SEC. (Stock polls only
// status, the two versions and analog.) With the camera's IMU the HUD moves with it and
// everything here changes slowly.
typedef struct { uint8_t cmd, per_sec; } row_t;
static const row_t rows_cam_imu[] = {
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
// Without one, the flight controller's gyro moves the HUD: asked often, the rest at the
// rates they had when attitude was polled.
static const row_t rows_fc_imu[] = {
    { FCP_MSP_RAW_IMU,       20 },
    { FCP_MSP_STATUS,        10 },
    { FCP_MSP_BATTERY_STATE,  4 },
    { FCP_MSP_ALTITUDE,       4 },
    { FCP_MSP_RAW_GPS,        3 },
    { FCP_MSP_COMP_GPS,       3 },
    { FCP_MSP_ANALOG,         2 },
    { FCP_MSP_BOXIDS,         2 },
    { FCP_MSP_FC_VERSION,     1 },
    { FCP_MSP_FC_VARIANT,     1 },
};
#define N(t) ((int)(sizeof(t) / sizeof(t[0])))

void fcpoll_init(fcpoll_t *p, int fc_imu) {
    memset(p, 0, sizeof(*p));
    p->fc_imu = fc_imu;
}

// Each slot every row earns its rate in credit; the one with the most is asked and
// pays a whole second's worth of slots. Rates that sum to the slots in a second keep
// every credit bounded, and a row asked n times a second is asked every 1/n.
uint8_t fcpoll_next(fcpoll_t *p) {
    const row_t *rows = p->fc_imu ? rows_fc_imu : rows_cam_imu;
    const int n = p->fc_imu ? N(rows_fc_imu) : N(rows_cam_imu);
    int best = 0;
    for (int i = 0; i < n; i++) {
        p->credit[i] += rows[i].per_sec;
        if (p->credit[i] > p->credit[best]) best = i;
    }
    p->credit[best] -= FCPOLL_SLOTS_PER_SEC;
    return rows[best].cmd;
}
