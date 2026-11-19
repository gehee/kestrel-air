// P1 wiring belongs to Linux's device tree, not HiSilicon MMIO or GPIO numbers.
#include "platform/platform.h"
#include "camera/pipeline.h"
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
// The SC231AI's modes (fpvOS sc231ai): 1080p at 60, and its centre 720 rows at
// 90, of which the encoder codes the centre 1280 columns. A 720p request above
// 60 fps (90, or a goggle's "720p100", 120 on the wire: its lines take 90 at
// most, 2026-11-17) runs 720p90 where the driver has it (camera_has_720p90).
// Anything else runs 1080p60, the mode the AE and ISP tuning were validated
// in - a 720p60 request too: the 720 rows at 60 are a crop of the picture the
// encoder cannot scale (no scaler on the way to it either: the CVISP's output
// size only windows it, 2026-11-18), and 1080p60 shows all of it.
void platform_format(int *w, int *h, int *fps) {
    if (*w == 1280 && *h == 720 && *fps > 60 && camera_has_720p90()) { *fps = 90; return; }
    *w = 1920; *h = 1080; *fps = 60;
}
int platform_fixed_format(void) { return 1; }
int platform_sensor_fps(int fps) { return fps == 90 ? 90 : 60; }
// The modes it runs, as a goggle asks for them: 1080p60, and with fpvOS
// sc231ai's 720p modes 720p90. Not 1280x720@120 (the Ascents' "720p100"),
// which it runs at 90, nor 720p60, which it runs as 1080p60: a goggle offers
// no other.
const char *platform_modes(void) {
    return camera_has_720p90() ? "1920x1080@60,1280x720@90" : "1920x1080@60";
}
// The flight controller's UART: stock's ttyS1 (serial@1504000), which fpvOS's
// device tree names serial1. None where the kernel has no such port (an image
// before it: one 8250 port), since fc_start() waits for the port to open.
const char *platform_fc_device(void) {
    static const char dev[] = "/dev/ttyS1";
    static int said;
    if (access(dev, F_OK) == 0) return dev;
    if (!said++) printf("platform: no %s - no flight controller link\n", dev);
    return NULL;
}
// fpvOS's ar9311-temp hwmon driver, looked up by its name once: one of its
// values, 0 when it has none.
static int ar9311_hwmon(const char *attr, long *v) {
    static char dir[48];
    static int looked;
    if (!looked) {
        looked = 1;
        for (int i = 0; i < 16 && !dir[0]; i++) {
            char p[64], name[32] = "";
            snprintf(p, sizeof(p), "/sys/class/hwmon/hwmon%d/name", i);
            FILE *f = fopen(p, "r");
            if (!f) continue;
            if (fgets(name, sizeof(name), f) && !strncmp(name, "ar9311_temp", 11))
                snprintf(dir, sizeof(dir), "/sys/class/hwmon/hwmon%d", i);
            fclose(f);
        }
        if (!dir[0]) puts("platform: no ar9311_temp hwmon sensor - no SoC temperature or supply voltage");
    }
    if (!dir[0]) return 0;
    char p[80];
    snprintf(p, sizeof(p), "%s/%s", dir, attr);
    FILE *f = fopen(p, "r");
    int ok = f && fscanf(f, "%ld", v) == 1;
    if (f) fclose(f);
    return ok;
}
// The SoC temperature (the vendor's conversion of the SoC sensor's code).
int platform_cpu_temp_x100(void) {
    long mdeg;
    return ar9311_hwmon("temp1_input", &mdeg) ? (int)(mdeg / 10) : INT_MIN;
}
int platform_hw_ver_mv(void) { return -1; }
// The board's supply, its in0 (SAR ADC channel and divider from the device
// tree; the P1's: channel 0, x10.09, the FC connector's 5 V), when the
// kernel's driver has it.
int platform_supply_mv(void) {
    long mv;
    return ar9311_hwmon("in0_input", &mv) ? (int)mv : -1;
}
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
// The radio clock off the radio's frame (radio/frameclock.c); KA_FRAMECLOCK=0
// turns it off. With Open8030 the P1's radio is found and followed (2026-11-18:
// the frame in 0.4-0.7 s, its edge within some 35 us, never lost; 2026-11-04's
// slow reads found none). Its frame interrupts are read only from fast replies
// (frameclock.c FAST_RTT_US: over SDIO here half take 0.75 ms or more, which
// put the IRQ up to a millisecond late and restarted the clock every few
// seconds): then IRQ 5 sits +217 to +228 us from the edge, one generation in
// five minutes.
int platform_frameclock(void) {
    static int v = -1;
    if (v < 0) { const char *e = getenv("KA_FRAMECLOCK"); v = !(e && *e && atoi(e) == 0); }
    return v;
}
