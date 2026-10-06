// The clocks kestrel-air reads.
#pragma once
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

// CLOCK_MONOTONIC: the MPP's own clock (capture times) and everything timed here.
static inline uint64_t mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}
static inline uint64_t mono_ms(void) { return mono_us() / 1000; }
// The same, in ms wrapping at 32 bits, for code that keeps u32 times.
static inline uint32_t mono_ms32(void) { return (uint32_t)mono_ms(); }

// CLOCK_MONOTONIC_RAW in ms, as stock times its packets.
static inline uint32_t raw_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// Wall-clock ms, wrapping at 32 bits.
static inline uint32_t wall_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}
