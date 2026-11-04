#pragma once
#include <stdint.h>

// Camera backend selected by BACKEND at build time. Device wiring and module
// loading belong to the OS; only the HiSilicon backend links MPI libraries.

int  camera_pipeline_create(char sensor, short width, short height, char framerate, int slice_count);
void camera_pipeline_destroy(void);
void camera_system_deinit(void);
void camera_set_sensor_angle(int angle);      // before camera_pipeline_create
void camera_set_low_delay_lines(int lines);   // VPSS -> encoder, before camera_pipeline_create

// The image sensor found at the last camera_pipeline_create (the CV2004 before the first one).
const char *camera_sensor_name(void);         // "cv2004", "os02k10", "ps5280"
int  camera_sensor_type(void);                // the type the stock app reports to the ground
int  camera_sensor_angle_tuning(void);        // the tuning bins' names carry the orientation
// Ours: the sensor's output now (1920x1080, or 1536x864 at 120 fps), and
// whether the camera does 864p120 (its 1536x864 window at 120 fps).
void camera_capture_size(int *w, int *h);
int  camera_has_864p120(void);
int  camera_has_864p144(void);                // ours: and 864p144 (the OS02K10's 1536x864 window at 144 fps)
int  camera_sensor_probe(void);               // ours: find the sensor before the first pipeline
uint32_t camera_readout_us(void);             // ours: the sensor's readout of the picture's rows, us
