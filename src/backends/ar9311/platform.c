// P1 wiring belongs to Linux's device tree, not HiSilicon MMIO or GPIO numbers.
#include "platform/platform.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

void platform_sensor_boot(void) { /* Clock/reset owned by the sensor driver. */ }
void platform_record_init(void) { /* Storage is mounted by the OS. */ }
void platform_format(int *w, int *h, int *fps) { *w = 1920; *h = 1080; *fps = 60; }
int platform_fixed_format(void) { return 1; }
int platform_sensor_fps(int fps) { (void)fps; return 60; }
const char *platform_fc_device(void) { return NULL; } // P1 FC UART not validated yet
int platform_cpu_temp_x100(void) { return INT_MIN; } // No validated thermal provider yet
int platform_hw_ver_mv(void) { return -1; }
// Linux owns GPIO polarity and debounce; never rely on event numbering.
int platform_key_open(void) {
    DIR *d = opendir("/dev/input");
    struct dirent *e;
    int found = -1;
    if (!d) return -1;
    while ((e = readdir(d))) {
        char path[320], name[128] = {0};
        if (strncmp(e->d_name, "event", 5)) continue;
        snprintf(path, sizeof(path), "/dev/input/%s", e->d_name);
        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0 && !strcmp(name, "p1-air-bind")) {
            if (found >= 0) { close(fd); close(found); closedir(d); errno = EEXIST; return -1; }
            found = fd;
        } else close(fd);
    }
    closedir(d);
    return found;
}
int platform_key_read(int fd, char *value) {
    unsigned char keys[(KEY_MAX + 8) / 8] = {0};
    // Current state remains trustworthy even if events accumulated during pairing.
    if (ioctl(fd, EVIOCGKEY(sizeof(keys)), keys) < 0) return -1;
    *value = (keys[KEY_WPS_BUTTON / 8] & (1u << (KEY_WPS_BUTTON % 8))) ? '0' : '1';
    return 1;
}
int platform_key_upgrade(void) { return 0; }
void platform_led_set(int red, int on) {
    const char *path = red ? "/sys/class/leds/p1-air:status-0/brightness" :
                             "/sys/class/leds/p1-air:status-1/brightness";
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return;
    if (write(fd, on ? "1" : "0", 1) < 0) { /* LED faults must not stop video */ }
    close(fd);
}

int platform_radio_board_controls(void) { return 0; }
// The radio clock off the radio's frame (radio/frameclock.c), KA_FRAMECLOCK=1.
// Off by default: on the P1's SDIO radio it found no frame (2026-11-04: "no
// frame found", 300 reads a try, again and again), so the stamps stay the
// radio clock's own.
int platform_frameclock(void) {
    static int v = -1;
    if (v < 0) { const char *e = getenv("KA_FRAMECLOCK"); v = e && *e && atoi(e) == 1; }
    return v;
}
