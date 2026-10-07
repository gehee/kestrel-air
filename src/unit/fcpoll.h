// What the air unit asks the flight controller (Betaflight, MSP) for, and when.
#pragma once
#include <stdint.h>

// MSP commands polled. The ground decodes the responses (kestrel-gnd).
enum {
    FCP_MSP_FC_VARIANT = 2, FCP_MSP_FC_VERSION = 3,
    FCP_MSP_STATUS = 101,           // armed, flight mode flags
    FCP_MSP_RAW_IMU = 102,          // the FC's accelerometer and gyro: the HUD's motion without a camera IMU
    FCP_MSP_RAW_GPS = 106, FCP_MSP_COMP_GPS = 107,
    FCP_MSP_ALTITUDE = 109,
    FCP_MSP_ANALOG = 110,           // RSSI
    FCP_MSP_BOXIDS = 119,           // which modes the mode flags are, bit by bit
    FCP_MSP_BATTERY_STATE = 130,    // cells, voltage, current, mAh drawn
};

// One request every 20 ms (50 a second), spread evenly: no request waits for a burst
// of the others. Each is asked for as often as its row in fcpoll.c says.
#define FCPOLL_SLOTS_PER_SEC 50

typedef struct { int credit[16]; int fc_imu; } fcpoll_t;

// fc_imu: the IMU data comes from the flight controller (a unit with no IMU on its
// camera, the Lite+): its IMU is asked for, 20 times a second.
void    fcpoll_init(fcpoll_t *p, int fc_imu);
uint8_t fcpoll_next(fcpoll_t *p);   // the MSP command for this slot
