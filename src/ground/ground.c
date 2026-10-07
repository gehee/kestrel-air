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

// One writer at a time. The reports (the tick thread), the flight controller's
// relay (fc.c) and the acknowledgements (the receive thread) all write to the one
// control socket; a frame goes out in as many writes as the radio takes, and
// another thread's frame in between corrupts both - and the socket's byte
// count, after which every write fails and the ground hears nothing more.
static pthread_mutex_t write_mtx = PTHREAD_MUTEX_INITIALIZER;

static int write_frame_locked(const uint8_t *f, int n, int timeout_ms, int tries);

static void on_rx(const uint8_t *d, uint32_t n, void *arg);

// The socket counts the bytes written to it, and the goggle's end counts from
// zero each time its app starts: when the goggle's app restarts (the link stays
// up, so there is no link event) every write after fails. Writing again does not
// help; a new socket counts from zero too. Three reports in a row that the
// radio would not take, and it is reopened - at most every 3 s.
static void reopen_ctrl(void) {
    static uint32_t last_ms;
    uint32_t now = mono_ms32();
    if (last_ms && now - last_ms < 3000) return;
    last_ms = now;
    printf("ground: the control socket takes no writes - reopening it\n");
    bbc_close(&radio_ctrl);
    if (bbc_open(&radio_ctrl, 0, 2, 2, 0x800, on_rx, NULL)) puts("ground: the control socket would not reopen");
}

// Up to three tries of 200 ms, a few ms apart, as stock writes.
int write_frame(const uint8_t *f, int n, int timeout_ms, int tries) {
    static int failed_in_a_row;
    pthread_mutex_lock(&write_mtx);
    int r = write_frame_locked(f, n, timeout_ms, tries);
    if (tries >= 2) {                        // the reports; an acknowledgement's one quick try is not a verdict
        if (r >= 0) failed_in_a_row = 0;
        else if (++failed_in_a_row >= 3) { failed_in_a_row = 0; reopen_ctrl(); }
    }
    pthread_mutex_unlock(&write_mtx);
    return r;
}

static int write_frame_locked(const uint8_t *f, int n, int timeout_ms, int tries) {
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
