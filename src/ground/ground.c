// Messages with the ground over radio socket port 2, as the stock air app
// exchanges them ("SG" messages): the framing, the acknowledgements, the
// ground's commands and the air's periodic reports. Reverse-engineered;
// the framing rebuilt every frame of three captured sessions byte for byte.
//
// The frame is in frame.c, the air's reports in reports.c, the ground's
// commands in commands.c; here: sending, receiving and acknowledging.

#include "ground/ground.h"
#include "ground/internal.h"
#include "ground/frame.h"
#include "video/video.h"
#include "camera/image.h"
#include "app/app.h"
#include "app/state.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "radio/radio.h"
#include "app/config.h"
#include "kestrel_air.h"
#include "common/clock.h"
#include "common/crc.h"



static pthread_mutex_t tx_mtx = PTHREAD_MUTEX_INITIALIZER;
static uint8_t tx_seq;

// Up to three tries of 200 ms, a few ms apart, as stock writes.
int write_frame(const uint8_t *f, int n, int timeout_ms, int tries) {
    for (int t = 0; t < tries; t++) {
        int off = 0;
        while (off < n) {
            int w = bbc_write(&radio_ctrl, f + off, n - off, timeout_ms);
            if (w <= 0) break;
            off += w;
        }
        if (off == n) return n;
        usleep(5000 + (rand() % 8) * 1000);
    }
    return -1;
}

int ground_send(const uint8_t *payload, int len) {
    uint8_t f[4096 + 11];
    if (len > 0xffa) return 0;
    pthread_mutex_lock(&tx_mtx);
    uint8_t seq = ++tx_seq;
    pthread_mutex_unlock(&tx_mtx);
    int n = frame_build(f, seq, 0, payload, len);
    return write_frame(f, n, 200, 3);
}

// ---- receive --------------------------------------------------------------

static uint8_t rxbuf[0x1400];
static int rxlen;
static pthread_mutex_t rx_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t rx_cv = PTHREAD_COND_INITIALIZER;

static void on_rx(const uint8_t *d, uint32_t n, void *arg) {
    (void)arg;
    pthread_mutex_lock(&rx_mtx);
    if (rxlen + (int)n > (int)sizeof(rxbuf)) {
        puts("ground: receive buffer full, reset");
        rxlen = 0;
    }
    if (n <= sizeof(rxbuf)) {
        memcpy(rxbuf + rxlen, d, n);
        rxlen += n;
    }
    pthread_cond_signal(&rx_cv);
    pthread_mutex_unlock(&rx_mtx);
}

static void *rx_thread(void *arg) {
    (void)arg;
    static uint8_t buf[0x1400];
    int have = 0;
    uint32_t last_seq = 0xffffffff;
    for (;;) {
        pthread_mutex_lock(&rx_mtx);
        while (!rxlen) pthread_cond_wait(&rx_cv, &rx_mtx);
        int n = rxlen < (int)sizeof(buf) - have ? rxlen : (int)sizeof(buf) - have;
        memcpy(buf + have, rxbuf, n);
        memmove(rxbuf, rxbuf + n, rxlen - n);
        rxlen -= n;
        pthread_mutex_unlock(&rx_mtx);
        have += n;

        int i = 0;
        while (have - i >= 6) {
            uint8_t *b = buf + i;
            int len = 0, r = frame_parse(b, have - i, &len);
            if (r == FRAME_MORE) break;               // the rest comes later
            if (r == FRAME_BAD_SUM) puts("ground: frame header checksum wrong");
            if (r == FRAME_BAD_CRC) puts("ground: frame CRC wrong");
            if (r != FRAME_OK) { i++; continue; }
            if (!(b[4] & 0x08) && (b[4] & 7) == 0) {
                uint8_t ack[16];
                int an = frame_build(ack, b[3], 1, b + 6, len ? 1 : 0);
                write_frame(ack, an, 10, 1);
                if (b[3] == last_seq) {
                    printf("ground: frame %d again, cmd 0x%x, t=%u\n", b[3], len ? b[6] : 0, mono_ms32());
                } else {
                    last_seq = b[3];
                    if (len) dispatch(b + 6, len);
                }
            }
            i += len + FRAME_OVERHEAD;
        }
        memmove(buf, buf + i, have - i);
        have -= i;
    }
    return NULL;
}

int ground_start(void) {
    pthread_t t;
    crc32c_init();
    bbc_set_rx(&radio_ctrl, on_rx, NULL);
    if (pthread_create(&t, NULL, rx_thread, NULL)) return -1;
    pthread_detach(t);
    if (pthread_create(&t, NULL, tick_thread, NULL)) return -1;
    pthread_detach(t);
    return 0;
}
