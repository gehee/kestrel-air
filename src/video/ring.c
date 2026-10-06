// The local packet ring: the read thread puts each packet in, the send
// thread takes them out in order. One writer, one reader; 2 MiB, 512 packets.
#include "video/ring.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define RING (2u << 20)
#define QMAX 512
static uint8_t *ring;

int ring_init(void) { return (ring = malloc(RING)) ? 0 : -1; }
static struct { uint32_t off, len; uint64_t t_got; } q[QMAX];
static int qh, qt;
static uint32_t wr;                     // next write offset
static pthread_mutex_t rmtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t rcv = PTHREAD_COND_INITIALIZER;
static volatile int sender_busy, flushing;

int ring_count(void) { return (qt - qh + QMAX) % QMAX; }

// Room for a packet of up to n bytes, contiguous; NULL if there is none.
uint8_t *ring_reserve(uint32_t n) {
    pthread_mutex_lock(&rmtx);
    uint32_t rd = ring_count() ? q[qh].off : wr;
    uint8_t *p = NULL;
    if ((qt + 1) % QMAX != qh) {
        if (!ring_count()) wr = 0, rd = 0;
        if (wr >= rd) {
            if (RING - wr >= n) p = ring + wr;
            else if (rd > n) { wr = 0; p = ring; }
        } else if (rd - wr > n) {
            p = ring + wr;
        }
    }
    pthread_mutex_unlock(&rmtx);
    return p;
}

void ring_put(uint8_t *p, uint32_t len, uint64_t t_got) {
    pthread_mutex_lock(&rmtx);
    q[qt].off = (uint32_t)(p - ring);
    q[qt].len = len;
    q[qt].t_got = t_got;
    qt = (qt + 1) % QMAX;
    wr = q[(qt + QMAX - 1) % QMAX].off + len;
    pthread_cond_signal(&rcv);
    pthread_mutex_unlock(&rmtx);
}

uint8_t *ring_get(uint32_t *len, uint64_t *t_got, int timeout_ms) {
    struct timespec until;
    clock_gettime(CLOCK_REALTIME, &until);
    until.tv_sec += timeout_ms / 1000;
    until.tv_nsec += (long)(timeout_ms % 1000) * 1000000;
    if (until.tv_nsec >= 1000000000) { until.tv_sec++; until.tv_nsec -= 1000000000; }
    pthread_mutex_lock(&rmtx);
    while (!ring_count() || flushing)
        if (pthread_cond_timedwait(&rcv, &rmtx, &until)) { pthread_mutex_unlock(&rmtx); return NULL; }
    uint8_t *p = ring + q[qh].off;
    *len = q[qh].len;
    *t_got = q[qh].t_got;
    sender_busy = 1;
    pthread_mutex_unlock(&rmtx);
    return p;
}

void ring_release(void) {
    pthread_mutex_lock(&rmtx);
    qh = (qh + 1) % QMAX;
    sender_busy = 0;
    pthread_mutex_unlock(&rmtx);
}

// Flush: wait (up to ms) for the sender to finish the
// packet it is on, then drop everything queued.
void ring_flush(int ms) {
    flushing = 1;
    int ok = 0;
    for (int t = 0; t <= ms; t += 10) {
        pthread_mutex_lock(&rmtx);
        if (!sender_busy) {
            qh = qt = 0;
            wr = 0;
            ok = 1;
        }
        pthread_mutex_unlock(&rmtx);
        if (ok) break;
        usleep(10000);
    }
    flushing = 0;
    printf("video: ring flush, ok=%d, left_ms=%d\n", ok, ms);
}
