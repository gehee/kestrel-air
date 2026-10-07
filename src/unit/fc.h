// The flight controller link (see fc.c).
#pragma once

// Set before fc_start: poll the flight controller's IMU (MSP_RAW_IMU) for the ground,
// the IMU data of a unit with none on its camera (KA_FEAT_FC_IMU).
extern int fc_imu;

int  fc_start(const char *dev);
int  fc_flying(void);
