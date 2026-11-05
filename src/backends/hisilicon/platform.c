#include "platform/platform.h"
#include "camera/pipeline.h"
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
void platform_sensor_boot(void) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd >= 0) {
        volatile uint32_t *r = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x11010000);
        if (r != MAP_FAILED) {
            uint32_t v = r[0x8440 / 4];
            if (((v >> 12) & 0xf) != 4) r[0x8440 / 4] = (v & ~0xf000u) | 0x4000;
            munmap((void *)r, 0x10000);
        }
        close(fd);
    }
}

void platform_record_init(void) {
    if (access("/record/", F_OK) && system("mkdir /record/") != 0) { /* as stock */ }
    if (system("mountpoint /record/") == 0) return;
    if (system("mount /dev/mmcblk1p1 /record/") == 0) return;
    if (system("mount /dev/mmcblk1 /record/") != 0)
        puts("app: mounting the SD card on /record failed");
}

void platform_format(int *w, int *h, int *fps) {
    if (*w == 1536 && *h == 864 && *fps >= 144 && camera_has_864p144()) {
        *fps = 144;
        return;
    }
    if (*w == 1536 && *h == 864 && *fps >= 120 && camera_has_864p120()) {
        *fps = 120;
        return;
    }
    int known = (*w == 1920 && *h == 1080) || (*w == 1280 && *h == 720) || (*w == 2560 && *h == 1440);
    if ((known && *fps > 99) || *fps > 100) *fps = 100;
}

int platform_fixed_format(void) { return 0; }

int platform_sensor_fps(int fps) { return fps == 60 ? 60 : fps == 50 ? 50 : fps == 120 ? 120 : fps == 144 ? 144 : 100; }

int platform_cpu_temp_x100(void) {
    static volatile uint32_t *regs;
    static int tried;
    if (!regs && !tried) {
        tried = 1;
        int fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (fd >= 0) {
            void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x1102a000);
            if (m != MAP_FAILED) regs = m;
            close(fd);
        }
    }
    if (!regs) return INT_MIN;
    double raw = regs[2] & 0x3ff;
    return (int)(float)(((raw - 127) * 165 / 784 - 40) * 100);
}

int platform_supply_mv(void) { return -1; }   // the battery is the radio chip's ADC (radio/adc.c)

int platform_hw_ver_mv(void) {
    int fd = open("/dev/ot_lsadc", O_RDWR), sum = 0;
    uint32_t model = 1, ch = 1;
    if (fd < 0) return -1;
    if (ioctl(fd, 0xc0044100, &model) >= 0 && ioctl(fd, 0x40044101, &ch) >= 0) {
        usleep(100);
        if (ioctl(fd, 0x4103) >= 0) {
            for (int i = 0; i < 5; i++) {
                usleep(100);
                sum += ioctl(fd, 0xc0044105, &ch);
            }
        }
    }
    ioctl(fd, 0x4104);
    ioctl(fd, 0x40044102, &ch);
    close(fd);
    int avg = sum / 5;
    return avg > 0x3ff ? -1 : (int)(avg * 3300.0 / 1023.0);
}


const char *platform_fc_device(void) { return "/dev/ttyAMA1"; }
int platform_key_open(void) { return open("/sys/class/gpio/gpio0/value", O_RDONLY | O_CLOEXEC); }
int platform_key_read(int fd, char *value) {
    lseek(fd, 0, SEEK_SET);
    return (int)read(fd, value, 1);
}
int platform_key_upgrade(void) { return 1; }
void platform_led_set(int red, int on) {
    char path[80];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", red ? 63 : 8);
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return;
    if (write(fd, on ? "1" : "0", 1) < 0) { /* retain legacy best effort */ }
    close(fd);
}

int platform_radio_board_controls(void) { return 1; }
int platform_frameclock(void) { return 1; } // camera frame lock runs on it
