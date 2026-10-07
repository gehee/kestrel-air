// The flight controller link, as the stock air app runs it:
// Betaflight MSP at 115200 on /dev/ttyAMA1.
//
// - It polls the FC every 20 ms. Stock asks MSP_STATUS fifty times, then FC_VERSION,
//   FC_VARIANT and ANALOG, and nothing else. Ours asks for what the HUD shows -
//   attitude, battery, altitude, GPS, the flight modes - see fcpoll.c.
// - Every complete *response* goes to the ground as SG message 0x02 while
//   the radio is linked, raw, in chunks of up to 172 bytes - except
//   DisplayPort (182): its strings are kept as a screen, and each "draw"
//   sends the whole screen as one rebuilt $X> 182 frame (subcommand 0x35).
// - MSP_STATUS / STATUS_EX give the armed state (TX power leaves low power);
//   MSP 190 with {1, x, 1} asks for pairing, as the pairing key does.
//
// The stock app also carries an on-screen menu of its own driven by the
// sticks, but nothing in it starts that menu: it is not here either.

#include "unit/fc.h"
#include "ground/ground.h"
#include "unit/board.h"
#include "unit/fcpoll.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "radio/radio.h"
#include "common/crc.h"

static int fd = -1;
static char dev[64];
static volatile int flying;
static pthread_mutex_t uart_mtx = PTHREAD_MUTEX_INITIALIZER;

int fc_flying(void) { return flying; }

// ---- FC -> ground queue (msp_ringbuffer, 4096 bytes) ----------------------

static uint8_t ring[4096];
static volatile uint32_t rd, wr;    // free-running; one writer, one reader, as stock

static uint32_t ring_space(void) { return sizeof(ring) - (wr - rd); }

static void ring_put(const uint8_t *p, uint32_t n) {
    if (n > ring_space()) n = ring_space();
    for (uint32_t i = 0; i < n; i++) ring[(wr + i) % sizeof(ring)] = p[i];
    wr += n;
}

// Every 10 ms: whatever is queued (at least 4 bytes), 172 bytes a message.
static void *fwd_thread(void *arg) {
    (void)arg;
    uint8_t m[1 + 172];
    m[0] = 0x02;
    for (;;) {
        while (wr - rd < 4) usleep(10000);
        uint32_t n = wr - rd;
        if (n > 172) n = 172;
        for (uint32_t i = 0; i < n; i++) m[1 + i] = ring[(rd + i) % sizeof(ring)];
        rd += n;
        if (radio_connected()) ground_send(m, (int)n + 1);   // else the bytes are dropped
    }
    return NULL;
}

// ---- DisplayPort screen ---------------------------------------------------

static uint8_t disp[4096];
static uint32_t disp_idx = 11;
static uint8_t disp_seq;
// 0 relay raw, 1 drop, 2 send the rebuilt screen. Stock sets 1 on any
// DisplayPort response and returns to 0 only after sending a screen, so
// between a DisplayPort message and the next draw nothing is relayed.
static int fwd_mode;

static void displayport(const uint8_t *pl, int n) {
    fwd_mode = 1;
    if (n < 1) return;
    int sub = pl[0];
    if (sub == 1 || sub == 2) {                  // release, clear: no strings
        disp_idx = 11;
    } else if (sub == 3 && n >= 4) {             // write string: row, col, attr, text
        uint8_t row = pl[1], col = pl[2], attr = pl[3];
        uint8_t L = (uint8_t)(n - 4 + 4);
        if (disp_idx + L >= sizeof(disp) - 15) return;
        // The latest string at a place (same length and attribute) replaces the last one.
        for (uint32_t p = 11; p < disp_idx;) {
            uint8_t e = disp[p];
            if (e == L && disp[p + 1] == row && disp[p + 2] == col && disp[p + 3] == attr) {
                memmove(disp + p, disp + p + L, disp_idx - p - L);
                disp_idx -= L;
                break;
            }
            if (e == 0) break;
            p += e;
        }
        disp[disp_idx] = L;
        disp[disp_idx + 1] = row;
        disp[disp_idx + 2] = col;
        disp[disp_idx + 3] = attr;
        memcpy(disp + disp_idx + 4, pl + 4, L - 4);
        disp_idx += L;
    } else if (sub == 4) {                       // draw: the whole screen, one frame
        uint16_t size = (uint16_t)(disp_idx - 8);
        disp[0] = '$'; disp[1] = 'X'; disp[2] = '>'; disp[3] = 0;
        disp[4] = 182; disp[5] = 0;
        disp[6] = (uint8_t)size; disp[7] = (uint8_t)(size >> 8);
        disp[8] = 0x35;
        disp[9] = ++disp_seq;
        disp[10] = 0;
        uint8_t c = 0;
        for (uint32_t i = 3; i < disp_idx; i++) c = crc8_dvb_s2(c, disp[i]);
        disp[disp_idx] = c;
        fwd_mode = 2;
    }
}

// ---- responses ------------------------------------------------------------

static void on_response(int cmd, const uint8_t *pl, int n, const uint8_t *raw, uint32_t raw_len) {
    if ((cmd == 101 || cmd == 150) && n >= 10) {     // armed: flight mode flags bit 0
        uint32_t modes;
        memcpy(&modes, pl + 6, 4);
        int armed = modes & 1;
        if (armed && !flying) { flying = 1; puts("fc: flying"); }
        else if (!armed && flying) { flying = 0; puts("fc: not flying"); }
    } else if (cmd == 182) {
        displayport(pl, n);
    } else if (cmd == 190 && n >= 3 && pl[0] == 1 && pl[2] == 1) {
        board_trigger_pairing();
    }

    if (!radio_connected()) return;                 // not queued while unlinked
    if (cmd == 182) {
        // DisplayPort: the screen goes as one frame, at each draw; the pieces before it do not.
        if (fwd_mode == 2 && disp_idx <= ring_space()) ring_put(disp, disp_idx + 1);
        fwd_mode = 0;
    } else if (raw_len <= ring_space()) {
        // Everything else, as it comes. (Stock held these back from the first piece of
        // a screen to its draw; polled values - attitude, battery, GPS - would then
        // be lost often.)
        ring_put(raw, raw_len);
    }
}

// The stock byte parser: MSP v1 ($M), v2 ($X) and v2 inside v1 (cmd 255).
static void *rx_thread(void *arg) {
    (void)arg;
    static uint8_t raw[1024], pl[192];
    uint8_t hdr[8], chunk[256], x = 0, crc = 0;
    uint32_t raw_len = 0;
    int state = 0, dir = 0, i = 0, size = 0, cmd = 0;

    for (;;) {
        int got = (int)read(fd, chunk, sizeof(chunk));
        if (got <= 0) { usleep(4000); continue; }
        for (int k = 0; k < got; k++) {
            uint8_t b = chunk[k];
            if (state == 0) raw_len = 0;
            if (raw_len < sizeof(raw)) raw[raw_len++] = b;
            // err: why a frame was dropped - 1 not a start, 2/3 not a direction,
            // 4 too long, 5/7 an MSP v2-in-v1 frame too short/long, 6 the v1
            // checksum, 8 the v2-in-v1 CRC, 9 a v2 frame too long, 10 the v2 CRC.
            int complete = 0, err = 0;
            switch (state) {
            case 0: if (b == '$') state = 1; break;
            case 1:
                x = 0; crc = 0; i = 0;
                if (b == 'M') state = 2; else if (b == 'X') state = 3; else err = 1;
                break;
            case 2: case 3:
                if (b == '<' || b == '>') { dir = b == '>'; state = state == 2 ? 4 : 10; }
                else err = state == 2 ? 2 : 3;
                break;
            case 4:                                         // v1 size, cmd
                hdr[i++] = b; x ^= b;
                if (i == 2) {
                    size = hdr[0]; cmd = hdr[1];
                    if (size > 192) err = 4;
                    else if (cmd == 0xff) { if (size < 6) err = 5; else state = 7; }
                    else { i = 0; state = size ? 5 : 6; }
                }
                break;
            case 5: pl[i++] = b; x ^= b; if (i == size) state = 6; break;
            case 6: if (b != x) err = 6; else complete = 1; break;
            case 7:                                         // v2 header inside v1
                hdr[i++] = b; x ^= b; crc = crc8_dvb_s2(crc, b);
                if (i == 7) {
                    size = hdr[5] | hdr[6] << 8;
                    cmd = hdr[3] | hdr[4] << 8;
                    i = 0;
                    if (size > 192) err = 7; else state = size ? 8 : 9;
                }
                break;
            case 8: crc = crc8_dvb_s2(crc, b); x ^= b; pl[i++] = b; if (i == size) state = 9; break;
            case 9: x ^= b; if (b == crc) state = 6; else err = 8; break;
            case 10:                                        // v2 flag, cmd, size
                hdr[i++] = b; crc = crc8_dvb_s2(crc, b);
                if (i == 5) {
                    cmd = hdr[1] | hdr[2] << 8;
                    size = hdr[3] | hdr[4] << 8;
                    i = 0;
                    // Stock has no check here and overruns its buffer; this one rejects.
                    if (size > 192) err = 9; else state = size ? 11 : 12;
                }
                break;
            case 11: crc = crc8_dvb_s2(crc, b); pl[i++] = b; if (i == size) state = 12; break;
            case 12: if (b == crc) complete = 1; else err = 10; break;
            }
            if (err) {
                printf("fc: MSP frame dropped (%d)\n", err);
                state = 0;
            } else if (complete) {
                if (dir == 1) on_response((int16_t)cmd, pl, size, raw, raw_len);
                state = 0;
            }
        }
    }
    return NULL;
}

int fc_imu;

static void *poll_thread(void *arg) {
    (void)arg;
    fcpoll_t poll;
    fcpoll_init(&poll, fc_imu);
    if (fc_imu) puts("fc: polling the flight controller's IMU (MSP_RAW_IMU, 20 a second)");
    for (;;) {
        uint8_t cmd = fcpoll_next(&poll);
        uint8_t req[6] = { '$', 'M', '<', 0, cmd, cmd };
        pthread_mutex_lock(&uart_mtx);
        if (write(fd, req, 6) < 0) { /* the FC is not there: keep polling */ }
        pthread_mutex_unlock(&uart_mtx);
        usleep(20000);
    }
    return NULL;
}

// Blocks until the UART opens, retrying every second, as stock's main does.
int fc_start(const char *d) {
    pthread_t t;
    struct termios tio;
    struct serial_struct ss;

    snprintf(dev, sizeof(dev), "%s", d ? d : "/dev/ttyAMA1");
    if (pthread_create(&t, NULL, fwd_thread, NULL)) return -1;
    pthread_detach(t);
    for (;;) {
        printf("fc: open %s", dev);
        fd = open(dev, O_RDWR | O_NOCTTY);
        if (fd < 0) {
            puts(" failed");
            sleep(1);
            continue;
        }
        puts(": 115200 8N1");
        memset(&tio, 0, sizeof(tio));
        tio.c_iflag = IGNPAR;
        tio.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &tio);
        if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
            ss.xmit_fifo_size = 0x80000;
            if (ioctl(fd, TIOCSSERIAL, &ss) != 0) printf("fc: UART FIFO size not changed\n");
        }
        break;
    }
    puts("fc: UART ready");
    if (pthread_create(&t, NULL, rx_thread, NULL)) return -1;
    pthread_detach(t);
    if (pthread_create(&t, NULL, poll_thread, NULL)) return -1;
    pthread_detach(t);
    return 0;
}
