// The flight logs (flightlog.h). Nothing that prints waits on the flash: a
// thread takes the output from a pipe, passes it on to the original stdout
// (/tmp/air.log) and keeps it in memory - the last 10 s before arming, then
// the flight; another writes the flight to the flash every 5 s.
#include "app/flightlog.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "common/clock.h"
#include "unit/fc.h"

#define LOG_DIR      "/usrdata/fpv/air/logs"
#define KEEP         5                    // flight logs kept
#define AROUND_US    10000000ull          // before arming and after disarming
#define FLUSH_US     5000000ull
#define RING_BYTES   (64 * 1024)          // 10 s of output, with room to spare
#define PEND_MAX     (512 * 1024)         // a flash this far behind loses text

static int out_fd = -1;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
// Before arming: chunks with their time, oldest dropped past 10 s.
static struct { uint64_t t; uint32_t len; } ring_idx[512];
static char ring[RING_BYTES];
static unsigned ring_n, ring_bytes;
// The flight: text waiting for the flash.
static char *pend;
static size_t pend_len;
static int recording, file_fd = -1;
static uint64_t disarmed_at;
static int64_t epoch_off;                 // goggle time - monotonic seconds, 0 = unknown
static unsigned flight_seq;
static volatile int enabled;              // the goggle's Debug Info, off by default

void flightlog_set_enabled(int on) {
    if (on != enabled) printf("flightlog: flight logs %s\n", on ? "on (the goggle's Debug Info)" : "off");
    enabled = on;
}

void flightlog_set_time(uint32_t epoch) {
    if (epoch < 1600000000u) return;      // a goggle without a set clock
    epoch_off = (int64_t)epoch - (int64_t)(mono_us() / 1000000ull);
}

// The newest KEEP - 1 stay; the one about to be made is the KEEP-th.
static int name_cmp(const void *a, const void *b) { return strcmp(*(char *const *)a, *(char *const *)b); }
static void prune(void) {
    DIR *d = opendir(LOG_DIR);
    if (!d) return;
    char *names[256];
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) && n < 256) {
        size_t l = strlen(e->d_name);
        if (strncmp(e->d_name, "fpvOS_", 6) == 0 && l > 4 && strcmp(e->d_name + l - 4, ".log") == 0)
            names[n++] = strdup(e->d_name);
    }
    closedir(d);
    qsort(names, n, sizeof names[0], name_cmp);
    for (int i = 0; i < n; i++) {
        if (i + (KEEP - 1) < n) {
            char p[300];
            snprintf(p, sizeof p, LOG_DIR "/%s", names[i]);
            unlink(p);
        }
        free(names[i]);
    }
}

static char flight_name[64];              // named when it arms

static void name_flight(uint64_t now) {
    if (epoch_off) {
        time_t t = (time_t)(epoch_off + (int64_t)(now / 1000000ull));
        struct tm tm;
        gmtime_r(&t, &tm);
        strftime(flight_name, sizeof flight_name, "fpvOS_%Y%m%d_%H%M%S.log", &tm);
    } else {
        // No goggle time yet: after the timed ones in the listing, in order.
        snprintf(flight_name, sizeof flight_name, "fpvOS_notime_%u_%u.log", (unsigned)getpid(), ++flight_seq);
    }
}

static void open_flight(const char *name) {
    mkdir(LOG_DIR, 0755);
    prune();
    char path[128];
    snprintf(path, sizeof path, LOG_DIR "/%s", name);
    file_fd = open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
}

static void pend_add(const char *b, size_t n) {
    if (!pend) pend = malloc(PEND_MAX);
    if (pend && pend_len + n <= PEND_MAX) { memcpy(pend + pend_len, b, n); pend_len += n; }
}

static void ring_add(const char *b, size_t n, uint64_t now) {
    if (n > RING_BYTES) { b += n - RING_BYTES; n = RING_BYTES; }
    // Older than 10 s, or no room: drop from the front.
    while (ring_n && (now - ring_idx[0].t > AROUND_US || ring_bytes + n > RING_BYTES ||
                      ring_n == sizeof ring_idx / sizeof ring_idx[0])) {
        uint32_t l = ring_idx[0].len;
        memmove(ring, ring + l, ring_bytes - l);
        ring_bytes -= l;
        memmove(ring_idx, ring_idx + 1, (ring_n - 1) * sizeof ring_idx[0]);
        ring_n--;
    }
    memcpy(ring + ring_bytes, b, n);
    ring_bytes += n;
    ring_idx[ring_n].t = now;
    ring_idx[ring_n].len = (uint32_t)n;
    ring_n++;
}

// Armed or not, once a chunk of output came in or a second went by.
static void step(uint64_t now) {
    const int armed = enabled && fc_flying();
    if (!recording && armed) {
        recording = 1;
        disarmed_at = 0;
        name_flight(now);
        pend_len = 0;
        pend_add(ring, ring_bytes);          // the 10 s before
        ring_n = ring_bytes = 0;
    } else if (recording && armed) {
        disarmed_at = 0;
    } else if (recording && !armed) {
        if (!disarmed_at) disarmed_at = now;
        else if (now - disarmed_at >= AROUND_US) recording = 2;   // the writer closes it
    }
}

static void *reader(void *arg) {
    int fd = (int)(intptr_t)arg;
    char buf[4096];
    for (;;) {
        ssize_t n = read(fd, buf, sizeof buf);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) break;
        for (ssize_t w = 0; w < n;) {        // /tmp/air.log first, as before
            ssize_t r = write(out_fd, buf + w, (size_t)(n - w));
            if (r <= 0) { if (r < 0 && errno == EINTR) continue; break; }
            w += r;
        }
        uint64_t now = mono_us();
        pthread_mutex_lock(&mtx);
        step(now);
        if (recording == 1) pend_add(buf, (size_t)n);
        else if (!recording) ring_add(buf, (size_t)n, now);
        pthread_mutex_unlock(&mtx);
    }
    return NULL;
}

static void *writer(void *arg) {
    (void)arg;
    uint64_t last = mono_us();
    for (;;) {
        usleep(200000);
        uint64_t now = mono_us();
        pthread_mutex_lock(&mtx);
        step(now);                           // arming and the 10 s after disarming, without output too
        int closing = recording == 2;
        int due = recording && (closing || now - last >= FLUSH_US);
        char *chunk = NULL, name[sizeof flight_name];
        size_t len = 0;
        memcpy(name, flight_name, sizeof name);
        if (due && pend_len) {
            chunk = malloc(pend_len);
            if (chunk) { memcpy(chunk, pend, pend_len); len = pend_len; }
            pend_len = 0;
        }
        if (closing) recording = 0;
        pthread_mutex_unlock(&mtx);
        if (!due) continue;
        last = now;
        if (file_fd < 0) open_flight(name);
        if (file_fd >= 0 && len) {
            for (size_t w = 0; w < len;) {
                ssize_t r = write(file_fd, chunk + w, len - w);
                if (r <= 0) { if (r < 0 && errno == EINTR) continue; break; }
                w += (size_t)r;
            }
            fsync(file_fd);
        }
        free(chunk);
        if (closing && file_fd >= 0) { close(file_fd); file_fd = -1; }
    }
    return NULL;
}

void flightlog_start(void) {
    int p[2];
    if (pipe2(p, O_CLOEXEC) != 0) return;
    fflush(stdout);
    fflush(stderr);
    out_fd = dup(STDOUT_FILENO);
    if (out_fd < 0 || dup2(p[1], STDOUT_FILENO) < 0 || dup2(p[1], STDERR_FILENO) < 0) return;
    close(p[1]);
    pthread_t t;
    pthread_create(&t, NULL, reader, (void *)(intptr_t)p[0]);
    pthread_detach(t);
    pthread_create(&t, NULL, writer, NULL);
    pthread_detach(t);
}
