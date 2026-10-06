#pragma once
#include <pthread.h>
#include <stdint.h>

// The radio interface kestrel-air drives the AR8030 through: the calls the
// stock air app makes, in its order and with its arguments (reverse-engineered
// from its traffic). client.c implements them on ar_libre's client library,
// with no radio daemon in between.

// Incoming data on a socket (e.g. the ground's commands on port 2).
typedef void (*bbc_rx_cb)(const uint8_t *data, uint32_t len, void *arg);

typedef struct {
    int fd;
    int slot, port;
    volatile int alive;
    pthread_t reader;
    pthread_mutex_t mtx;
    pthread_cond_t cv;
    int acked;          // bytes the radio took, -1 on error
    int have_ack;
    int stale;          // acknowledgements still due for writes that timed out
    // Pipelined writes (bbc_write_async): sent, not yet acknowledged, in order.
    int inflight;
    uint32_t pend_len[64];
    uint64_t pend_t[64];
    int ph, pt;
    uint32_t partial;   // acknowledgements for fewer bytes than were written
    uint32_t lost;      // writes never acknowledged (dropped by the radio)
    int lat_us[1024];   // write -> acknowledgement, collected for statistics
    int nlat;
    bbc_rx_cb rx;
    void *rx_arg;
} bbc_sock;

// The three calls the stock app opens with, each on its own connection:
// connectivity test, device list, device select. Returns the number of
// basebands there are, or -1.
int bbc_hello(void);

// Open a baseband socket the way stock does: flags 0x23 (RX, TX and bit 5,
// which the vendor client keeps and the open SDK strips - without it the
// socket crawls), stock's buffer sizes. Blocks until the radio answers.
int bbc_open(bbc_sock *s, int slot, int bb_port,
             uint32_t tx_buf, uint32_t rx_buf, bbc_rx_cb rx, void *rx_arg);

// Set (or change) the incoming-data callback of an open socket.
void bbc_set_rx(bbc_sock *s, bbc_rx_cb rx, void *rx_arg);

// One write, one acknowledgement, as stock does it. Returns the bytes the
// radio took, or -1 on a timeout or a dead connection.
int bbc_write(bbc_sock *s, const void *data, uint32_t len, int timeout_ms);

// The ioctl session: one connection of its own for every get (0x01xxxxxx)
// and set (0x02xxxxxx) request, as stock keeps it. bbc_ioctl returns the
// reply's status (>= 0) or -1 on a timeout or a dead connection; the reply
// payload is copied into out (up to outlen), its full length into *got.
typedef struct {
    int fd;
    pthread_mutex_t mtx;
} bbc_ioctl_conn;

int bbc_ioctl_connect(bbc_ioctl_conn *c);
int bbc_ioctl(bbc_ioctl_conn *c, uint32_t req, const void *in, uint32_t inlen,
              void *out, uint32_t outlen, uint32_t *got, int timeout_ms);

// Pipelined writes: send without waiting for this write's acknowledgement,
// waiting only while `window` writes are already unacknowledged (up to
// timeout_ms). Acknowledgements are matched in order. Returns 0 when sent,
// -1 on a timeout or a dead connection (the data was not sent). Stock
// never does this: it waits for every acknowledgement (bbc_write).
int bbc_write_async(bbc_sock *s, const void *data, uint32_t len, int window, int timeout_ms);
// Copy out and reset the write -> ack latencies (us) gathered so far.
int bbc_take_latencies(bbc_sock *s, int *out, int max, uint32_t *partial, uint32_t *lost);

// Baseband events (class 3): a connection of their own, asked for with
// op 1 and pushed with op 3, as stock subscribes to them. Event 0 is the
// link state: {slot, current, previous}, 0 idle, 1 locked, 2 connected.
typedef void (*bbc_event_cb)(const uint8_t *data, uint32_t len, void *arg);
typedef struct {
    int fd;
    pthread_t reader;
    bbc_event_cb cb;
    void *arg;
} bbc_event;

int  bbc_subscribe(bbc_event *e, int event, bbc_event_cb cb, void *arg);
void bbc_unsubscribe(bbc_event *e);

// Just closes the socket - what the radio sees when the stock app exits,
// and demonstrably survives.
void bbc_close(bbc_sock *s);
