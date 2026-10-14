#pragma once

// HiSilicon Hi3516CV610 - the Caddx Ascent air unit's SoC, CV2004 or OS02K10 sensor.
// It links the SDK's MPI libraries directly (see the Makefile).

int  cv610_pipeline_create(char sensor, short width, short height, char framerate, int slice_count);
void cv610_pipeline_destroy(void);
void cv610_system_deinit(void);
void cv610_set_sensor_angle(int angle);      // before cv610_pipeline_create
void cv610_set_low_delay_lines(int lines);   // VPSS -> encoder, before cv610_pipeline_create

// The image sensor found at the last cv610_pipeline_create (the CV2004 before the first one).
const char *cv610_sensor_name(void);         // "cv2004", "os02k10": also the tuning bins' names
int  cv610_sensor_type(void);                // the type the stock app reports to the ground
int  cv610_sensor_angle_tuning(void);        // the tuning bins' names carry the orientation
// Ours: the sensor's output now (1920x1080, or 1536x864 at 120 fps), and
// whether the camera does 864p120 (its 1536x864 window at 120 fps).
void cv610_capture_size(int *w, int *h);
int  cv610_has_864p120(void);
int  cv610_has_864p144(void);                // ours: and 864p144 (the OS02K10's 1536x864 window at 144 fps)
int  cv610_sensor_probe(void);               // ours: find the sensor before the first pipeline
uint32_t cv610_readout_us(void);             // ours: the sensor's readout of the picture's rows, us
