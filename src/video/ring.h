// The local packet ring between the read and send threads (see ring.c).
#pragma once
#include <stdint.h>

int      ring_init(void);                                   // 0 ok, -1 out of memory
int      ring_count(void);                                  // packets queued
uint8_t *ring_reserve(uint32_t n);                          // room for n bytes, contiguous; NULL if none
void     ring_put(uint8_t *p, uint32_t len, uint64_t t_got);   // queue what was reserved
uint8_t *ring_get(uint32_t *len, uint64_t *t_got, int timeout_ms);   // the oldest; NULL on timeout
void     ring_release(void);                                // done with what ring_get gave
void     ring_flush(int ms);                                // wait for the sender, then drop all
