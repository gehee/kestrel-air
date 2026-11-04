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
#include "platform/platform.h"

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include "camera/pipeline.h"
#include "radio/radio.h"
#include "app/config.h"
#include "app/settings.h"

/* Optional OS readiness marker: camera initialization and radio startup have
 * completed. It does not claim an RF link or a frame has been transmitted. */
static void ready(int available) {
    const char *path = getenv("KA_READY_FILE");
    if (!path || !*path) return;
    if (!available) { unlink(path); return; }
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (fd < 0) { perror("app readiness"); return; }
    char text[32];
    int n = snprintf(text, sizeof(text), "%ld\n", (long)getpid());
    if (write(fd, text, n) != n) { perror("app readiness"); unlink(path); }
    close(fd);
}

static int flying(void) { return fc_flying(); }

// /tmp/devinfo2.txt, as stock writes it once the sensor has answered.
static void sensor_devinfo(void) {
    char cmd[80];
    snprintf(cmd, sizeof(cmd), "echo \"sensor_name=%s\n\" > /tmp/devinfo2.txt", camera_sensor_name());
    if (system(cmd) != 0) { /* as stock */ }
}

// Output sizes: the sensors run 1080p, 720p and 1440p requests at
// up to 100 fps; anything else passes through, at up to 100 fps too. Ours:
// 864p120 (1536x864 at 120, KA_FEAT_864P120) and 864p144 (at 144, the info's
// 864p144=1) where the camera has them.
void app_ch0_remap(int *w, int *h, int *fps) { platform_format(w, h, fps); }

// The sensor's mode for a rate: its 60 and 50 fps modes (both sensors have 50, 60 and 100) when asked for
// exactly those, its 100 fps mode otherwise (the encoder drops the rest). Ours: 120, a window,
// only for 864p120, and 144, the OS02K10's window, only for 864p144 (app_ch0_remap).
// A new output size or rate: tear the whole pipeline down and bring it up
// again at the new output size and rate, with the sensor turned for angle.
// The encoder has to be stopped first (video_send_stop).
int app_set_sensor_res(int w, int h, int fps, int angle) {
    printf("app: new output %dx%d@%d, angle %d\n", w, h, fps, angle);
    int sf = platform_sensor_fps(fps);
    image_pause();
    camera_pipeline_destroy();
    camera_system_deinit();
    camera_set_sensor_angle(angle);
    camera_set_low_delay_lines(video_low_delay_lines());
    if (camera_pipeline_create(0, w, h, (char)sf, 0)) {
        fprintf(stderr, "app: camera pipeline restart failed\n");
        return -1;
    }
    video_set_format(w, h, fps, sf);
    image_restart(sf, angle);
    puts("app: new output size, done");
    return 0;
}

int app_run(volatile sig_atomic_t *keep_running) {
    printf("app: kestrel-air %s\n", KA_VERSION);
    settings_log();
    if (model_init(BOARD_TYPE_FILE)) {
        // air/start.sh only starts kestrel-air on a Lite, Lite+ or GT. Exit 0: no reboot.
        fprintf(stderr, "app: not an air unit kestrel-air runs on\n");
        return 0;
    }
    board_leds_boot();
    if (cfg_load()) printf("app: no %s, wrote the defaults\n", CFG_PATH);
    platform_sensor_boot();
    // Ours: the sensor before the first pipeline, so that a stored 864p120 is
    // run the way this sensor runs it (the pipeline fails on an unknown one).
    camera_sensor_probe();
    int w = cfg_get("ch0_width", 1920), h = cfg_get("ch0_height", 1080), f = cfg_get("ch0_fps", 120);
    int angle = cfg_get("angle", 0);
    app_ch0_remap(&w, &h, &f);
    // A backend with one mode stores it, so that the goggle is told the mode
    // that runs. The others keep the request and clamp it again at each start:
    // a 1080p120 request stays 120, an 864p144 one stays for a camera that has it.
    if (platform_fixed_format()) { cfg_set("ch0_width", w); cfg_set("ch0_height", h); cfg_set("ch0_fps", f); }
    int sf = platform_sensor_fps(f);

    // VI/ISP/VPSS with the sensor turned as configured, the
    // tuning bin and the image settings. The encoder waits for the ground.
    camera_set_sensor_angle(angle);
    camera_set_low_delay_lines(video_low_delay_lines());
    int cam_ok = camera_pipeline_create(0, w, h, (char)sf, 0) == 0;
    if (!cam_ok) {
        // Keep the radio up. Exit 1 used to reboot-loop the GT when the
        // PS5280 had no matching driver yet.
        fprintf(stderr, "app: camera pipeline failed, radio only\n");
    } else {
        sensor_devinfo();
        video_set_format(w, h, f, sf);
        if (image_start(sf, angle)) {
            fprintf(stderr, "app: camera pipeline failed, radio only\n");
            camera_pipeline_destroy();
            camera_system_deinit();
            cam_ok = 0;
        } else {
            image_apply_all();
        }
    }

    platform_record_init();

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
        if (cam_ok) video_start();
        ground_start();
    }
    if (platform_fc_device()) fc_start(platform_fc_device());
    board_start();
    if (!radio_init_failed) ready(1);

    while (*keep_running) usleep(100000);

    ready(0);
    printf("app: stop streaming\n");
    if (cam_ok) video_stop();
    imu_stop();
    if (cam_ok) {
        image_pause();
        camera_pipeline_destroy();
        camera_system_deinit();
    }
    return 0;
}
