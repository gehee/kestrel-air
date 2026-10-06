#pragma once

// HiSilicon Hi3516CV610 - the Caddx Ascent air unit's SoC, CV2004 sensor.
// It links the SDK's MPI libraries directly (see the Makefile).

int  cv610_pipeline_create(char sensor, short width, short height, char framerate, int slice_count);
void cv610_pipeline_destroy(void);
void cv610_system_deinit(void);
void cv610_set_sensor_angle(int angle);      // before cv610_pipeline_create
void cv610_set_low_delay_lines(int lines);   // VPSS -> encoder, before cv610_pipeline_create
