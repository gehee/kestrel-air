// The bbclient interface on ar_libre's client library
// (https://github.com/gehee/ar_libre): the radio straight through arlink.ko's
// /dev/arlink0 or the stock /dev/artosyn_sdio, with no radio daemon.
//
// The library connects once for the whole process and subscribes to every
// event itself; the calls here map onto it one for one. Pipelined writes
// (bbc_write_async, --bb-window above 1) are written synchronously for now.
#include "radio/client.h"

void bbc_bus_watchdog_start(void);   // below
#include <arlink.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "common/clock.h"

static pthread_mutex_t conn_mtx = PTHREAD_MUTEX_INITIALIZER;
static bb_host_t *bb_host;
static bb_dev_handle_t *bb_dev;

// Connect on first use; 0 once connected.
static int ensure_connected(void) {
    pthread_mutex_lock(&conn_mtx);
    if (!bb_dev) {
        bb_dev_list_t *list = NULL;
        if (!bb_host && bb_host_connect(&bb_host, NULL, 0) != 0) {
            bb_host = NULL;
        } else if (bb_dev_getlist(bb_host, &list) > 0) {
            bb_dev = bb_dev_open(list[0]);
            bb_dev_freelist(list);
            if (bb_dev) printf("radio: radio through %s\n", arlink_version());
        }
    }
    int r = bb_dev ? 0 : -1;
    pthread_mutex_unlock(&conn_mtx);
    return r;
}

int bbc_hello(void) {
    static int watchdog_started;
    if (!watchdog_started++) bbc_bus_watchdog_start();
    return ensure_connected() ? -1 : 1;           // one baseband
}

// Incoming data, handed to the socket's callback as it comes.
static void *reader(void *arg) {
    bbc_sock *s = arg;
    static __thread uint8_t buf[16384];
    while (s->alive) {
        const uint64_t t0 = mono_us();
        int n = bb_socket_read(s->fd, buf, sizeof(buf), 100);
        if (n <= 0) {
            // Nothing at once rather than after the wait: the radio has gone
            // (the bus watchdog below restarts the unit). No spinning meanwhile.
            if (mono_us() - t0 < 20000) usleep(100000);
            continue;
        }
        pthread_mutex_lock(&s->mtx);
        bbc_rx_cb rx = s->rx;
        void *rx_arg = s->rx_arg;
        pthread_mutex_unlock(&s->mtx);
        if (rx) rx(buf, (uint32_t)n, rx_arg);
    }
    return NULL;
}

int bbc_open(bbc_sock *s, int slot, int bb_port,
             uint32_t tx_buf, uint32_t rx_buf, bbc_rx_cb rx, void *rx_arg) {
    memset(s, 0, sizeof(*s));
    s->fd = -1;
    s->slot = slot;
    s->port = bb_port;
    s->rx = rx;
    s->rx_arg = rx_arg;
    if (ensure_connected()) return -1;
    bb_sock_opt_t opt = { tx_buf, rx_buf };
    int fd = bb_socket_open(bb_dev, slot, (uint32_t)bb_port, 0x23, &opt);   // stock's flags
    if (fd < 0) {
        fprintf(stderr, "radio: socket %d/%d refused (%d)\n", slot, bb_port, fd);
        return -1;
    }
    s->fd = fd;
    pthread_mutex_init(&s->mtx, NULL);
    pthread_cond_init(&s->cv, NULL);
    s->alive = 1;
    if (pthread_create(&s->reader, NULL, reader, s)) {
        s->alive = 0;
        bb_socket_close(fd);
        s->fd = -1;
        return -1;
    }
    return 0;
}

void bbc_set_rx(bbc_sock *s, bbc_rx_cb rx, void *rx_arg) {
    pthread_mutex_lock(&s->mtx);
    s->rx_arg = rx_arg;
    s->rx = rx;
    pthread_mutex_unlock(&s->mtx);
}

int bbc_write(bbc_sock *s, const void *data, uint32_t len, int timeout_ms) {
    if (!s->alive) return -1;
    uint64_t t0 = mono_us();
    int w = bb_socket_write(s->fd, data, len, timeout_ms);
    pthread_mutex_lock(&s->mtx);
    if (w > 0) {
        if (s->nlat < 1024) s->lat_us[s->nlat++] = (int)(mono_us() - t0);
        if ((uint32_t)w < len) s->partial++;
    } else {
        s->lost++;
    }
    pthread_mutex_unlock(&s->mtx);
    return w > 0 ? w : -1;
}

int bbc_ioctl_connect(bbc_ioctl_conn *c) {
    memset(c, 0, sizeof(*c));
    c->fd = -1;
    pthread_mutex_init(&c->mtx, NULL);
    return ensure_connected();
}

int bbc_ioctl(bbc_ioctl_conn *c, uint32_t req, const void *in, uint32_t inlen,
              void *out, uint32_t outlen, uint32_t *got, int timeout_ms) {
    (void)c;
    int r = arlink_request(req, in, inlen, out, outlen, got, timeout_ms);
    return r < 0 ? -1 : r;
}

// Synchronous for now: the whole buffer written before returning.
int bbc_write_async(bbc_sock *s, const void *data, uint32_t len, int window, int timeout_ms) {
    (void)window;
    const uint8_t *p = data;
    uint32_t off = 0;
    while (off < len) {
        int w = bbc_write(s, p + off, len - off, timeout_ms);
        if (w <= 0) return -1;
        off += (uint32_t)w;
    }
    return 0;
}

int bbc_take_latencies(bbc_sock *s, int *out, int max, uint32_t *partial, uint32_t *lost) {
    pthread_mutex_lock(&s->mtx);
    int n = s->nlat < max ? s->nlat : max;
    memcpy(out, s->lat_us, n * sizeof(int));
    s->nlat = 0;
    if (partial) { *partial = s->partial; s->partial = 0; }
    if (lost) { *lost = s->lost; s->lost = 0; }
    pthread_mutex_unlock(&s->mtx);
    return n;
}

// The library already has every event subscribed; this only registers the
// callback, which has the same signature. e->fd keeps the event number.
int bbc_subscribe(bbc_event *e, int event, bbc_event_cb cb, void *arg) {
    memset(e, 0, sizeof(*e));
    e->fd = -1;
    e->cb = cb;
    e->arg = arg;
    if (ensure_connected() || arlink_subscribe(event, cb, arg)) return -1;
    e->fd = event;
    return 0;
}

void bbc_unsubscribe(bbc_event *e) {
    if (e->fd < 0) return;
    arlink_subscribe(e->fd, NULL, NULL);
    e->fd = -1;
}

void bbc_close(bbc_sock *s) {
    if (s->alive) {
        s->alive = 0;
        pthread_join(s->reader, NULL);
    }
    if (s->fd >= 0) bb_socket_close(s->fd);
    s->fd = -1;
}

// ---- the radio bus watchdog ------------------------------------------------
// If arlink.ko reports a failed transfer on the radio's bus (bus_errors in its
// stats; a plain loss of the RF link does not count), the chip is wedged: every
// later write fails until the unit is rebooted (seen 2026-10-04 under a steady
// stream, as a CMD53 timeout). Nor does the radio come back to this process if
// the chip resets (a brown-out): it drops to its boot ROM, arlink.ko uploads
// the firmware again and a new device appears, while the library's connection
// stays on the old one and every call fails. Seen here as the stats file going
// away or the upload count moving. Better a clean restart than a stream that
// never comes back: exit with status 2, which air/run-kestrel-air.sh answers
// with a reboot.

#define ARLINK_STATS "/sys/class/misc/arlink0/stats"

static void radio_lost(const char *why) {
    printf("radio: %s: exiting so the unit restarts\n", why);
    fflush(stdout);
    _exit(2);
}

static void *bus_watchdog(void *arg) {
    (void)arg;
    int seen = 0;                       // the running chip's stats were read once
    unsigned long long uploads0 = 0;
    for (;;) {
        sleep(1);
        FILE *f = fopen(ARLINK_STATS, "r");
        if (!f) {
            // Never there: the stock driver, or a radio not up yet.
            if (seen) radio_lost("the radio's device went away (the chip reset?)");
            continue;
        }
        char line[96];
        unsigned long long n, errors = 0, uploads = 0;
        int have_uploads = 0;
        while (fgets(line, sizeof(line), f)) {
            if (sscanf(line, "bus_errors %llu", &n) == 1) errors = n;
            else if (sscanf(line, "uploads %llu", &n) == 1) { uploads = n; have_uploads = 1; }
        }
        fclose(f);
        if (errors) {
            char why[64];
            snprintf(why, sizeof(why), "the radio bus failed (%llu errors)", errors);
            radio_lost(why);
        }
        if (!have_uploads) continue;
        if (!seen) {
            seen = 1;
            uploads0 = uploads;
        } else if (uploads != uploads0) {
            radio_lost("the radio's firmware was uploaded again (the chip reset)");
        }
    }
    return NULL;
}

void bbc_bus_watchdog_start(void) {
    pthread_t t;
    if (!pthread_create(&t, NULL, bus_watchdog, NULL)) pthread_detach(t);
}
