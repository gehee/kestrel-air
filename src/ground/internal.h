// What the files of the ground module share. Not for other modules: they use
// ground/ground.h.
#pragma once
#include <stdint.h>

#include "ground/frame.h"

#define SENSOR_TYPE   cv610_sensor_type()   // 1 CV2004, 7 OS02K10
#define RF_HW_VER     0x10    // this unit's RF board, as stock reports it

int  write_frame(const uint8_t *f, int n, int timeout_ms, int tries);   // ground.c

// The ground's channel settings, kept for the reports (commands.c sets them,
// reports.c sends them back).
extern uint8_t msg_is_hop, msg_slot;
extern uint32_t msg_freq;
extern uint32_t work_list[64];
extern int work_n;

void  put_cfg_struct(uint8_t *s);            // reports.c: the settings block
void *tick_thread(void *arg);                // reports.c: the periodic reports
void  dispatch(const uint8_t *p, int len);   // commands.c: one message from the ground
