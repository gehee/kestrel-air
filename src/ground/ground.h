// Messages with the ground over radio socket port 2 (see ground.c).
#pragma once
#include <stdint.h>

int  ground_start(void);
int  ground_send(const uint8_t *payload, int len);   // one message to the ground
