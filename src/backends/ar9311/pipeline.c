// Linux owns sensor clocks, reset, CSI, ISP and capture buffers. The launcher
// loads those drivers; this backend verifies the supported camera before use.
#include "camera/pipeline.h"
#include "backends/ar9311/camera.h"
#include <dirent.h>
#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

int ar9311_node(const char *name, char *path, size_t size) {
    DIR *d = opendir("/sys/class/video4linux");
    struct dirent *e;
    int n = 0;
    if (!d) return -1;
    while ((e = readdir(d))) {
        char p[512], v[128];
        snprintf(p, sizeof(p), "/sys/class/video4linux/%s/name", e->d_name);
        FILE *f = fopen(p, "r");
        if (!f) continue;
        if (fgets(v, sizeof(v), f)) {
            v[strcspn(v, "\r\n")] = 0;
            if (!strcmp(v, name) || (!strcmp(name, "sc231ai") && !strncmp(v, "sc231ai ", 8))) {
                if (snprintf(path, size, "/dev/%s", e->d_name) >= (int)size) n = 2;
                else n++;
            }
        }
        fclose(f);
    }
    closedir(d);
    if (n != 1) fprintf(stderr, "camera: expected one %s node, found %d\n", name, n);
    return n == 1 ? 0 : -1;
}
int camera_sensor_probe(void) { char p[128]; return ar9311_node("sc231ai", p, sizeof(p)); }
int camera_pipeline_create(char sensor, short w, short h, char rate, int slices) {
    char p[128];
    (void)sensor; (void)slices;
    if (w != 1920 || h != 1080 || rate != 60) return -1;
    return camera_sensor_probe() || ar9311_node("ar-cvisp", p, sizeof(p)) ||
        ar9311_node("C&M Wave5 VPU encoder", p, sizeof(p)) ? -1 : 0;
}
void camera_pipeline_destroy(void) { }
void camera_system_deinit(void) { }
void camera_set_sensor_angle(int angle) { if (angle) fprintf(stderr, "camera: rotation unsupported on P1\n"); }
void camera_set_low_delay_lines(int lines) { (void)lines; }
const char *camera_sensor_name(void) { return "sc231ai"; }
int camera_sensor_type(void) { return 0; } // No verified stock protocol sensor ID
int camera_sensor_angle_tuning(void) { return 0; }
void camera_capture_size(int *w, int *h) { *w = 1920; *h = 1080; }
int camera_has_864p120(void) { return 0; }
int camera_has_864p144(void) { return 0; }

// A line is the active width and the horizontal blanking at the pixel rate
// (SC231AI: 2240 / 151.2 MHz, 14.8 us), all read from the sensor's subdevice
// with the exposure the AE service last set.
static uint32_t frame_lines;   // rows + vertical blanking, as last read

int ar9311_sensor_timing(uint32_t *line_ps, uint32_t *rows, uint32_t *exposure_lines) {
    char path[128];
    if (ar9311_node("sc231ai", path, sizeof(path))) return -1;
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    struct v4l2_subdev_format f = { .which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = 0 };
    struct v4l2_ext_control c[4] = {
        { .id = V4L2_CID_PIXEL_RATE }, { .id = V4L2_CID_HBLANK }, { .id = V4L2_CID_EXPOSURE },
        { .id = V4L2_CID_VBLANK },
    };
    struct v4l2_ext_controls cs = { .which = V4L2_CTRL_WHICH_CUR_VAL, .count = 4, .controls = c };
    int ok = !ioctl(fd, VIDIOC_SUBDEV_G_FMT, &f) && !ioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) &&
             c[0].value64 > 0 && c[1].value >= 0 && c[2].value > 0 && f.format.height > 0;
    close(fd);
    if (!ok) return -1;
    *line_ps = (uint32_t)((f.format.width + (uint64_t)c[1].value) * 1000000000000ull / (uint64_t)c[0].value64);
    *rows = f.format.height;
    *exposure_lines = (uint32_t)c[2].value;
    frame_lines = f.format.height + (uint32_t)(c[3].value > 0 ? c[3].value : 0);
    return 0;
}

// The capture driver (fpvOS ar-cvisp) completes a buffer `depth` ticks after
// arming it and stamps it then. Measured (rows marked before each queue and
// timed as they were overwritten, 2026-11-04): at depth 1 the stamp comes as
// the frame starts landing - row 0 0.1 ms before it, row 1079 15.6 ms after,
// a row every 14.6 us; at depth 2 every row is in place, the last 1 ms before
// it. So row 0 is (depth - 1) frames and 0.1 ms before the stamp: 16.8 ms at
// 1080p60 with depth 2. (The time the ISP takes before a row lands is not in
// it: it is not measured, and short against a frame.)
uint32_t ar9311_capture_lag_us(void) {
    uint32_t line, rows, exp, depth = 0;
    FILE *f = fopen("/sys/module/ar_cvisp/parameters/depth", "r");
    if (f) { if (fscanf(f, "%u", &depth) != 1) depth = 0; fclose(f); }
    if (!depth || ar9311_sensor_timing(&line, &rows, &exp)) return 0;
    return (uint32_t)(((uint64_t)line * (depth - 1) * frame_lines + 500000) / 1000000) + 100;
}
// The sensor reading out the picture's rows, top to bottom: 16.0 ms at 1080p60.
uint32_t camera_readout_us(void) {
    uint32_t line, rows, exp;
    return ar9311_sensor_timing(&line, &rows, &exp) ? 0 : (uint32_t)(((uint64_t)line * rows + 500000) / 1000000);
}
