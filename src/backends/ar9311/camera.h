#pragma once
#include <stddef.h>
int ar9311_node(const char *name, char *path, size_t size);
int ar9311_controls_start(void);
int ar9311_controls_alive(void);
int ar9311_camera_set(unsigned command, int value);
