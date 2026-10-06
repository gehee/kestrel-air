// The unit's IMU (an ICM-40609-D on SPI0 CS0), woken and sampled here; the
// video path carries its samples to the ground as H.265 SEI messages.
#pragma once
#include <stdint.h>

extern const char *imu_dev;              // --imu: the spidev node, NULL = off
int  imu_start(const char *spidev);      // 0 ok; the video simply goes without on failure
void imu_stop(void);
// Appends the samples taken since the last call, as SEI NAL units for the
// picture `pic` captured at `pts` (MPP clock, us). Returns the bytes written.
uint32_t imu_sei(uint8_t *dst, uint32_t room, uint64_t pts, uint16_t pic);
// For tools: the newest samples (time in us on the MPP clock; gx gy gz ax ay az, raw).
int  imu_latest(uint64_t *t_us, int16_t (*v)[6], int max);
// For tools: sample, overflow and SEI counts.
void imu_stats(uint64_t *samples, uint32_t *overflows, uint32_t *lost, double *period_us,
               uint64_t *sei_msgs, uint64_t *sei_bytes);
