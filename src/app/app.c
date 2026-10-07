// The stock-compatible air runtime: brings up the camera pipeline, then the
// radio, video, ground messages and flight controller link, in the order
// the stock air app does, and runs until asked to stop.

#include "app/app.h"
#include "video/video.h"
#include "camera/image.h"
#include "imu/imu.h"
#include "ground/ground.h"
#include "unit/board.h"
#include "unit/model.h"
#include "unit/fc.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include "camera/pipeline.h"
#include "radio/radio.h"
#include "app/config.h"
#include "app/settings.h"

static int flying(void) { return fc_flying(); }

// Before the sensor is looked for: its 24 MHz clock select in bits 15:12 of
// 0x11018440 (the same for the CV2004 and the OS02K10).
static void sensor_boot(void) {
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

// /tmp/devinfo2.txt, as stock writes it once the sensor has answered.
static void sensor_devinfo(void) {
    char cmd[80];
    snprintf(cmd, sizeof(cmd), "echo \"sensor_name=%s\n\" > /tmp/devinfo2.txt", cv610_sensor_name());
    if (system(cmd) != 0) { /* as stock */ }
}

// The recording path and the SD card, mounted if present.
// Recording itself is a stub in the stock build too.
static void record_init(void) {
    if (access("/record/", F_OK) && system("mkdir /record/") != 0) { /* as stock */ }
    if (system("mountpoint /record/") == 0) return;
    if (system("mount /dev/mmcblk1p1 /record/") == 0) return;
    if (system("mount /dev/mmcblk1 /record/") != 0)
        puts("app: mounting the SD card on /record failed");
}

// Output sizes: the sensors run 1080p, 720p and 1440p requests at
// up to 100 fps; anything else passes through.
void app_ch0_remap(int *w, int *h, int *fps) {
    int known = (*w == 1920 && *h == 1080) || (*w == 1280 && *h == 720) || (*w == 2560 && *h == 1440);
    if (known && *fps > 99) *fps = 100;
}

// The sensor's mode for a rate: its 60 and 50 fps modes (both sensors have 50, 60 and 100) when asked for
// exactly those, its 100 fps mode otherwise (the encoder drops the rest).
static int sensor_fps(int fps) { return fps == 60 ? 60 : fps == 50 ? 50 : 100; }

// A new output size or rate: tear the whole pipeline down and bring it up
// again at the new output size and rate, with the sensor turned for angle.
// The encoder has to be stopped first (video_send_stop).
int app_set_sensor_res(int w, int h, int fps, int angle) {
    printf("app: new output %dx%d@%d, angle %d\n", w, h, fps, angle);
    int sf = sensor_fps(fps);
    image_pause();
    cv610_pipeline_destroy();
    cv610_system_deinit();
    cv610_set_sensor_angle(angle);
    cv610_set_low_delay_lines(video_low_delay_lines());
    if (cv610_pipeline_create(0, w, h, (char)sf, 0)) {
        fprintf(stderr, "app: camera pipeline restart failed\n");
        return -1;
    }
    video_set_format(w, h, fps, sf);
    image_restart(sf, angle);
    puts("app: new output size, done");
    return 0;
}

int app_run(volatile char *keep_running) {
    printf("app: kestrel-air %s\n", KA_VERSION);
    settings_log();
    if (model_init(BOARD_TYPE_FILE)) {
        // air/start.sh only starts kestrel-air on a Lite or Lite+. Exit 0: no reboot.
        fprintf(stderr, "app: not an air unit kestrel-air runs on\n");
        return 0;
    }
    board_leds_boot();
    if (cfg_load()) printf("app: no %s, wrote the defaults\n", CFG_PATH);
    sensor_boot();
    int w = cfg_get("ch0_width", 1920), h = cfg_get("ch0_height", 1080), f = cfg_get("ch0_fps", 120);
    int angle = cfg_get("angle", 0);
    app_ch0_remap(&w, &h, &f);
    int sf = sensor_fps(f);

    // VI/ISP/VPSS with the sensor turned as configured, the
    // tuning bin and the image settings. The encoder waits for the ground.
    cv610_set_sensor_angle(angle);
    cv610_set_low_delay_lines(video_low_delay_lines());
    if (cv610_pipeline_create(0, w, h, (char)sf, 0)) {
        fprintf(stderr, "app: camera pipeline failed\n");
        return 1;
    }
    sensor_devinfo();
    video_set_format(w, h, f, sf);
    image_start(sf, angle);
    image_apply_all();

    record_init();

    // Ours, not stock's: wake the IMU. Without it the video simply carries no SEI.
    // IMU data (--imu), from where this board has it: the camera's own IMU (cam-imu, the
    // Lite), else the flight controller's, polled over MSP (fc-imu, the Lite+). A camera IMU
    // that does not answer falls back to the flight controller's.
    if (imu_dev && !model_cam_imu()) {
        printf("app: IMU data from the flight controller (%s: no IMU on the camera)\n", model_name());
        imu_dev = NULL;
        fc_imu = 1;
    } else if (imu_dev && imu_start(imu_dev)) {
        puts("app: the camera's IMU does not answer: IMU data from the flight controller instead");
        imu_dev = NULL;
        fc_imu = 1;
    }

    // The radio. A radio that will not come up leaves the unit running,
    // with the LEDs saying so, as stock does.
    radio_hooks hooks = { video_set_bitrate, video_tgt_fps, flying };
    if (radio_start(&hooks)) {
        fprintf(stderr, "app: radio setup failed\n");
        radio_init_failed = 1;
    } else {
        video_start();
        ground_start();
    }
    fc_start("/dev/ttyAMA1");
    board_start();

    while (*keep_running) usleep(100000);

    printf("app: stop streaming\n");
    video_stop();
    imu_stop();
    cv610_pipeline_destroy();
    cv610_system_deinit();
    return 0;
}
