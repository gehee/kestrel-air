// The unit's IMU: an InvenSense/TDK ICM-40609-D on SPI0 chip select 0
// (/dev/spidev0.0). The stock app never wakes it; this does, reads its FIFO
// and hands the samples, stamped on the MPP clock (CLOCK_MONOTONIC), to the
// video path, which carries them to the ground as H.265 SEI messages.
//
// - SPI mode 3 (or 1): the PL022's own chip select drops between bytes in
//   modes 0 and 2, so the chip would see single-byte transfers there.
// - Gyro +-2000 dps (16.4 LSB/dps), accelerometer +-32 g (1024 LSB/g, this
//   part's power-on range: at rest it reads 1.00 g), both low-noise at 1 kHz,
//   in the chip's own axes. The FIFO holds the 16-byte accel+gyro packets; a
//   thread drains it every ~3.7 ms.
// - Sample times. The chip's clock is its own (an RC oscillator), and a FIFO
//   read says only "the newest sample is no older than now". So each sample
//   gets a number k, every read gives a point (k of its newest sample, read
//   time), and the sample times are the line t = a + b*k through those
//   points: b the slope of a least-squares fit over the last ~2 s, a the
//   lower envelope (the read that came right after a sample). Reads land at
//   every phase of the 1 ms sample period, so the envelope sits within about
//   the SPI latency of the true time. The chip's 16-bit packet timestamp only
//   detects lost packets.
//
// SEI payload (user_data_unregistered, little-endian), one message per run of
// samples at most 60 ms long:
//   UUID "kestrel-air-IMU1" | u8 version 1 | u8 flags (1: samples were
//   dropped before these) | u16 picture number | u64 frame capture PTS (us) |
//   u64 time of the first sample (us) | u16 n | u16 gyro LSB per 0.1 dps |
//   u16 accel LSB per g | n x { u16 us after the first sample, i16 gx gy gz
//   ax ay az }

#include "imu/imu.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include "common/clock.h"

const char *imu_dev;

// ICM-4x bank 0.
#define R_DEVICE_CONFIG 0x11
#define R_INT_STATUS    0x2D
#define R_FIFO_COUNTH   0x2E
#define R_FIFO_DATA     0x30
#define R_SIGNAL_RESET  0x4B
#define R_PWR_MGMT0     0x4E
#define R_GYRO_CONFIG0  0x4F
#define R_ACCEL_CONFIG0 0x50
#define R_FIFO_CONFIG   0x16
#define R_FIFO_CONFIG1  0x5F
#define R_WHO_AM_I      0x75
#define WHO_ICM40609D   0x3B

#define PKT        16           // FIFO packet: header, accel 6, gyro 6, temp 1, timestamp 2
#define ODR_HZ     1000
#define PERIOD_US  (1000000 / ODR_HZ)
#define POLL_US    3700         // not a multiple of the sample period: the phase sweeps
#define SPI_HZ     4000000
#define GYRO_LSB_X10  164       // +-2000 dps
#define ACCEL_LSB     1024      // +-32 g

#define NS     2048             // sample ring
#define MFIT   512              // reads in the time fit
#define MAXN   100              // samples per SEI message
#define SEI_SPAN_US 60000

typedef struct { uint64_t t; int16_t v[6]; } smp;

static int fd = -1;
static pthread_t thr;
static volatile int run;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static smp ring[NS];
static uint64_t produced, consumed;     // sample counts; consumed is the SEI path's
static uint32_t overflows, lost_pkts;
static double fit_b = PERIOD_US;
static uint64_t sei_msgs, sei_bytes;

static int xfer(const uint8_t *tx, uint8_t *rx, unsigned n) {
    struct spi_ioc_transfer t = { .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx,
                                  .len = n, .speed_hz = SPI_HZ, .bits_per_word = 8 };
    return ioctl(fd, SPI_IOC_MESSAGE(1), &t) < 0 ? -1 : 0;
}

static int rd(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(0x80 | reg), 0 }, rx[2];
    return xfer(tx, rx, 2) ? -1 : rx[1];
}

static void wr(uint8_t reg, uint8_t v) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7f), v }, rx[2];
    xfer(tx, rx, 2);
}

// The chip as the datasheet's start-up sequence has it: reset, configure,
// FIFO on, sensors on, 50 ms for the gyro to settle, FIFO flushed.
static int imu_setup(void) {
    int who = rd(R_WHO_AM_I);
    if (who != WHO_ICM40609D) {
        fprintf(stderr, "imu: WHO_AM_I %02x, expected %02x (ICM-40609-D) on %s\n", who, WHO_ICM40609D, imu_dev);
        return -1;
    }
    wr(R_DEVICE_CONFIG, 0x01);                  // soft reset
    usleep(5000);
    rd(R_INT_STATUS);                           // clears RESET_DONE
    wr(R_GYRO_CONFIG0, 0x06);                   // +-2000 dps, 1 kHz
    wr(R_ACCEL_CONFIG0, 0x06);                  // +-32 g, 1 kHz
    wr(R_FIFO_CONFIG1, 0x0b);                   // accel + gyro + timestamp into the FIFO
    wr(R_FIFO_CONFIG, 0x40);                    // stream mode: the oldest is overwritten
    wr(R_PWR_MGMT0, 0x0f);                      // gyro and accel, low-noise
    usleep(50000);
    wr(R_SIGNAL_RESET, 0x02);                   // FIFO flush: drop the settling samples
    usleep(1000);
    printf("imu: ICM-40609-D on %s: PWR_MGMT0 %02x GYRO_CONFIG0 %02x ACCEL_CONFIG0 %02x FIFO_CONFIG1 %02x\n",
           imu_dev, rd(R_PWR_MGMT0), rd(R_GYRO_CONFIG0), rd(R_ACCEL_CONFIG0), rd(R_FIFO_CONFIG1));
    return 0;
}

// ---- sample times -----------------------------------------------------------

static struct { double k, t; } fitv[MFIT];
static int fit_n, fit_pos;
static double fit_t0;
static uint64_t last_t;

// Adds one read (k of its newest sample, its time) and returns the line.
static void fit_add(double k, double t, double *a, double *b) {
    if (!fit_n) fit_t0 = t;
    fitv[fit_pos].k = k;
    fitv[fit_pos].t = t - fit_t0;
    fit_pos = (fit_pos + 1) % MFIT;
    if (fit_n < MFIT) fit_n++;
    double bb = PERIOD_US;
    int newest = (fit_pos + MFIT - 1) % MFIT, oldest = fit_n < MFIT ? 0 : fit_pos;
    if (fit_n >= 64 && fitv[newest].t - fitv[oldest].t > 500000) {
        double sk = 0, st = 0, skk = 0, skt = 0;
        double k0 = fitv[oldest].k;
        for (int i = 0; i < fit_n; i++) {
            double x = fitv[i].k - k0, y = fitv[i].t;
            sk += x; st += y; skk += x * x; skt += x * y;
        }
        double d = fit_n * skk - sk * sk;
        if (d > 0) {
            bb = (fit_n * skt - sk * st) / d;
            if (bb < PERIOD_US * 0.9 || bb > PERIOD_US * 1.1) bb = PERIOD_US;
        }
    }
    double aa = 1e30;
    for (int i = 0; i < fit_n; i++) {
        double v = fitv[i].t - bb * fitv[i].k;
        if (v < aa) aa = v;
    }
    *a = aa + fit_t0;
    *b = bb;
}

static void imu_flush_resync(void) {
    wr(R_SIGNAL_RESET, 0x02);
    fit_n = fit_pos = 0;
}

static void *imu_thread(void *arg) {
    (void)arg;
    static uint8_t buf[PKT * 240 + 1], tx[PKT * 240 + 1];
    uint64_t k = 0;                      // packets so far (lost ones included)
    uint16_t prev_ts = 0;
    int have_prev = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (run) {
        next.tv_nsec += POLL_US * 1000L;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        uint8_t ctx[3] = { 0x80 | R_FIFO_COUNTH, 0, 0 }, crx[3];
        if (xfer(ctx, crx, 3)) continue;
        uint64_t t_c = mono_us();
        unsigned bytes = (unsigned)crx[1] << 8 | crx[2];
        unsigned n = bytes / PKT;
        if (bytes > 1900) {                           // about to overwrite: lost data
            overflows++;
            imu_flush_resync();
            have_prev = 0;
            continue;
        }
        if (!n) continue;

        smp got[240];
        int ng = 0;
        for (unsigned done = 0; done < n;) {
            unsigned c = n - done < 240 ? n - done : 240;
            memset(tx, 0, c * PKT + 1);
            tx[0] = 0x80 | R_FIFO_DATA;
            if (xfer(tx, buf, c * PKT + 1)) break;
            for (unsigned i = 0; i < c; i++) {
                const uint8_t *p = buf + 1 + i * PKT;
                if ((p[0] & 0xe0) != 0x60) { overflows++; imu_flush_resync(); have_prev = 0; ng = 0; done = n; break; }
                uint16_t ts = (uint16_t)(p[14] << 8 | p[15]);
                if (have_prev) {
                    unsigned d = (uint16_t)(ts - prev_ts);
                    // The chip's own microseconds are ~1000 apart; two or more
                    // periods mean packets went missing.
                    if (d > PERIOD_US * 3 / 2 && d < 40000) {
                        unsigned miss = (d + PERIOD_US / 2) / PERIOD_US - 1;
                        k += miss;
                        lost_pkts += miss;
                    }
                }
                prev_ts = ts;
                have_prev = 1;
                k++;
                smp *s = &got[ng++];
                s->t = k;                              // the number for now; the time below
                int16_t raw[6];
                for (int j = 0; j < 6; j++) raw[j] = (int16_t)(p[1 + 2 * j] << 8 | p[2 + 2 * j]);
                // FIFO order is accel then gyro; the SEI carries gyro first.
                s->v[0] = raw[3]; s->v[1] = raw[4]; s->v[2] = raw[5];
                s->v[3] = raw[0]; s->v[4] = raw[1]; s->v[5] = raw[2];
            }
            done += c;
        }
        if (!ng) continue;

        double a, b;
        fit_add((double)k, (double)t_c, &a, &b);
        fit_b = b;
        pthread_mutex_lock(&mtx);
        for (int i = 0; i < ng; i++) {
            uint64_t t = (uint64_t)(a + b * (double)got[i].t + 0.5);
            if (t <= last_t) t = last_t + 1;
            last_t = t;
            got[i].t = t;
            ring[produced++ % NS] = got[i];
        }
        pthread_mutex_unlock(&mtx);
    }
    return NULL;
}

int imu_start(const char *dev) {
    imu_dev = dev;
    fd = open(dev, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "imu: %s: %s\n", dev, strerror(errno));
        return -1;
    }
    uint8_t mode = SPI_MODE_3, bpw = 8;
    uint32_t hz = SPI_HZ;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
        fprintf(stderr, "imu: %s: SPI setup: %s\n", dev, strerror(errno));
        close(fd);
        fd = -1;
        return -1;
    }
    if (imu_setup()) { close(fd); fd = -1; return -1; }
    run = 1;
    if (pthread_create(&thr, NULL, imu_thread, NULL)) { run = 0; close(fd); fd = -1; return -1; }
    return 0;
}

void imu_stop(void) {
    if (fd < 0) return;
    run = 0;
    pthread_join(thr, NULL);
    wr(R_PWR_MGMT0, 0x00);                      // back to sleep, as the stock firmware leaves it
    close(fd);
    fd = -1;
}

int imu_latest(uint64_t *t_us, int16_t (*v)[6], int max) {
    pthread_mutex_lock(&mtx);
    int n = produced < (uint64_t)max ? (int)produced : max;
    if (n > NS) n = NS;
    for (int i = 0; i < n; i++) {
        const smp *s = &ring[(produced - n + i) % NS];
        t_us[i] = s->t;
        memcpy(v[i], s->v, sizeof(s->v));
    }
    pthread_mutex_unlock(&mtx);
    return n;
}

void imu_stats(uint64_t *samples, uint32_t *overflow, uint32_t *lost, double *period_us,
                   uint64_t *sei_n, uint64_t *sei_b) {
    pthread_mutex_lock(&mtx);
    *samples = produced; *overflow = overflows; *lost = lost_pkts; *period_us = fit_b;
    *sei_n = sei_msgs; *sei_b = sei_bytes;
    pthread_mutex_unlock(&mtx);
}

// ---- the SEI ------------------------------------------------------------------

static void put16(uint8_t *p, unsigned v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }

// One suffix-SEI NAL (type 40) with a user_data_unregistered message of n
// samples; emulation prevention applied. Returns its size, or 0 if no room.
static uint32_t sei_nal(uint8_t *dst, uint32_t room, const smp *s, int n, uint64_t pts, uint16_t pic,
                        int flags) {
    uint8_t body[16 + 26 + MAXN * 14];
    memcpy(body, "kestrel-air-IMU1", 16);
    unsigned o = 16;
    body[o++] = 1;
    body[o++] = (uint8_t)flags;
    put16(body + o, pic); o += 2;
    put64(body + o, pts); o += 8;
    put64(body + o, s[0].t); o += 8;
    put16(body + o, (unsigned)n); o += 2;
    put16(body + o, GYRO_LSB_X10); o += 2;
    put16(body + o, ACCEL_LSB); o += 2;
    for (int i = 0; i < n; i++) {
        put16(body + o, (unsigned)(s[i].t - s[0].t)); o += 2;
        for (int j = 0; j < 6; j++) { put16(body + o, (uint16_t)s[i].v[j]); o += 2; }
    }
    // rbsp: payloadType 5, payloadSize, the body, rbsp_trailing_bits
    uint8_t rbsp[sizeof(body) + 8];
    unsigned r = 0;
    rbsp[r++] = 5;
    for (unsigned sz = o; ; sz -= 255) {
        if (sz >= 255) rbsp[r++] = 255;
        else { rbsp[r++] = (uint8_t)sz; break; }
    }
    memcpy(rbsp + r, body, o);
    r += o;
    rbsp[r++] = 0x80;

    uint32_t w = 0;
    if (room < 6 + r + r / 2) return 0;
    dst[w++] = 0; dst[w++] = 0; dst[w++] = 0; dst[w++] = 1;
    dst[w++] = 40 << 1; dst[w++] = 1;            // suffix SEI, layer 0, temporal id 0
    int zeros = 0;
    for (unsigned i = 0; i < r; i++) {
        if (zeros >= 2 && rbsp[i] <= 3) { dst[w++] = 3; zeros = 0; }
        dst[w++] = rbsp[i];
        zeros = rbsp[i] == 0 ? zeros + 1 : 0;
    }
    return w;
}

// Appends the samples taken since the last call to dst as SEI NAL units, to
// go after the last slice of picture `pic` (captured at `pts`). Returns the
// bytes written, 0 for none.
uint32_t imu_sei(uint8_t *dst, uint32_t room, uint64_t pts, uint16_t pic) {
    if (fd < 0) return 0;
    smp batch[MAXN];
    uint32_t w = 0;
    int flags = 0;
    for (int msg = 0; msg < 3; msg++) {
        int n = 0;
        pthread_mutex_lock(&mtx);
        if (produced - consumed > 400) { consumed = produced - MAXN; flags = 1; }   // a stale backlog
        while (consumed < produced && n < MAXN) {
            const smp *s = &ring[consumed % NS];
            if (n && s->t - batch[0].t > SEI_SPAN_US) break;
            batch[n++] = *s;
            consumed++;
        }
        pthread_mutex_unlock(&mtx);
        if (!n) break;
        uint32_t z = sei_nal(dst + w, room - w, batch, n, pts, pic, flags);
        if (!z) break;
        w += z;
        sei_msgs++;
        sei_bytes += z;
        flags = 0;
    }
    return w;
}
