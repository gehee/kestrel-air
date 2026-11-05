#pragma once
#include <stddef.h>
#include <stdint.h>
int ar9311_node(const char *name, char *path, size_t size);
int ar9311_controls_start(void);
int ar9311_controls_alive(void);
int ar9311_camera_set(unsigned command, int value);
// The sensor's mode as its driver has it: a line's time (ps), its rows, and
// the exposure now (lines). 0 when read, -1 when not.
int ar9311_sensor_timing(uint32_t *line_ps, uint32_t *rows, uint32_t *exposure_lines);
// How long after row 0's capture the capture driver hands its buffer over
// (us), the stamp it puts on it; 0 when not known.
uint32_t ar9311_capture_lag_us(void);
