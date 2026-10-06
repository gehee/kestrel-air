// --bb-verbose timing of the video path, printed every 5 s.
#include "video/stats.h"

#include <stdio.h>
#include <stdlib.h>

#include "camera/image.h"
#include "imu/imu.h"
#include "radio/radio.h"
#include "video/video.h"

// --bb-verbose: where each slice spends its time on the air, every 5 s.
// capture (MPP PTS, CLOCK_MONOTONIC) -> out of the encoder -> the sender
// picks it up -> the radio has taken it (write acknowledged).
#define TSTAT 4096
static struct { int first[TSTAT], last[TSTAT], queue[TSTAT], write[TSTAT], nf, nl, nq; uint64_t t0; } ts;

static int cmp_int(const void *a, const void *b) { return *(const int *)a - *(const int *)b; }

static void tstat_line(const char *name, int *v, int n) {
    if (!n) return;
    qsort(v, n, sizeof(int), cmp_int);
    printf("  %-26s n=%4d p10 %6.2f  med %6.2f  p90 %6.2f  p99 %6.2f ms\n", name, n, v[n / 10] / 1000.0,
           v[n / 2] / 1000.0, v[n * 9 / 10] / 1000.0, v[n * 99 / 100] / 1000.0);
}

void tstat_add(uint64_t pts, uint64_t got, uint64_t start, uint64_t end, int last_slice) {
    if (!video_verbose) return;
    int a = (int)(got - pts);
    if (last_slice) { if (ts.nl < TSTAT) ts.last[ts.nl++] = a; }
    else if (ts.nf < TSTAT) ts.first[ts.nf++] = a;
    if (ts.nq < TSTAT) { ts.queue[ts.nq] = (int)(start - got); ts.write[ts.nq++] = (int)(end - start); }
    if (!ts.t0) ts.t0 = end;
    if (end - ts.t0 >= 5000000) {
        image_isp_info isp;
        image_get_info(&isp);
        printf("video: timing (5 s): exposure %u us, iso %.0f\n", isp.exp_time_us, isp.iso * 100);
        tstat_line("capture -> first slice out", ts.first, ts.nf);
        tstat_line("capture -> last slice out", ts.last, ts.nl);
        tstat_line("slice out -> send start", ts.queue, ts.nq);
        tstat_line(video_window > 1 ? "send call (window wait)" : "send start -> radio ack", ts.write, ts.nq);
        if (video_window > 1) {
            static int lat[1024];
            uint32_t partial = 0, lost = 0;
            int n = bbc_take_latencies(&radio_video, lat, 1024, &partial, &lost);
            tstat_line("write -> radio ack", lat, n);
            if (partial || lost) printf("  %u partial acknowledgements, %u writes never acknowledged\n", partial, lost);
        }
        if (imu_dev) {
            static uint64_t last_samples, last_sei_bytes;
            uint64_t n, sn, sb;
            uint32_t ov, lost;
            double per;
            imu_stats(&n, &ov, &lost, &per, &sn, &sb);
            printf("  imu: %llu samples in 5 s, period %.1f us, %u FIFO overflows, %u packets lost, SEI %llu B/s\n",
                   (unsigned long long)(n - last_samples), per, ov, lost, (unsigned long long)(sb - last_sei_bytes) / 5);
            last_samples = n;
            last_sei_bytes = sb;
        }
        ts.nf = ts.nl = ts.nq = 0;
        ts.t0 = end;
    }
}
