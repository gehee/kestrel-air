#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unit/model.h"
#include "platform/platform.h"
#include "camera/pipeline.h"
static int has_720p90;   // fpvOS sc231ai's mode parameter, which the camera looks for
int camera_has_720p90(void) { return has_720p90; }
/* Its info's modes= names each mode it runs, each run as asked: a goggle
 * offers no other. */
static void check_modes(void) {
    const char *m = platform_modes();
    assert(m && *m);
    for (const char *p = m; p; p = strchr(p, ',') ? strchr(p, ',') + 1 : NULL) {
        int mw, mh, mf, w, h, f;
        assert(sscanf(p, "%dx%d@%d", &mw, &mh, &mf) == 3);
        w = mw; h = mh; f = mf;
        platform_format(&w, &h, &f);
        assert(w == mw && h == mh && f == mf);
        assert(platform_sensor_fps(f) == f);
    }
}
int main(void) {
    /* A stale Lite board file/environment must never activate its GPIOs,
     * factory defaults or memory-mapped temperature sensor on an AR9311. */
    setenv("KA_BOARD_TYPE", "472", 1);
    assert(model_init("/nonexistent") == 0);
    assert(!strcmp(model_name(), "BETAFPV P1 Air"));
    assert(!model_lite_plus() && !model_cam_imu() && !model_radio_defaults());
    assert(!platform_key_upgrade());
    assert(!platform_fc_device() || !strcmp(platform_fc_device(), "/dev/ttyS1"));
    assert(platform_cpu_temp_x100() == INT_MIN && platform_hw_ver_mv() < 0 &&
           platform_supply_mv() < 0);
    int w=1536,h=864,f=144;
    platform_format(&w,&h,&f);
    assert(w==1920 && h==1080 && f==60 && platform_sensor_fps(f)==60);
    check_modes();
    assert(!strcmp(platform_modes(), "1920x1080@60"));
    w = 1280; h = 720; f = 120;
    platform_format(&w, &h, &f);
    assert(w == 1920 && h == 1080 && f == 60);   // without the driver's 720p modes
    has_720p90 = 1;
    check_modes();
    assert(!strcmp(platform_modes(), "1920x1080@60,1280x720@90"));
    /* 720p60 is a crop of the picture: 1080p60 shows all of it. */
    w = 1280; h = 720; f = 60;
    platform_format(&w, &h, &f);
    assert(w == 1920 && h == 1080 && f == 60);
    /* A goggle that asks for the Ascents' 720p100 (1280x720@120) gets 720p90. */
    w = 1280; h = 720; f = 120;
    platform_format(&w, &h, &f);
    assert(w == 1280 && h == 720 && f == 90 && platform_sensor_fps(f) == 90);
    w = 1920; h = 1080; f = 120;
    platform_format(&w, &h, &f);
    assert(w == 1920 && h == 1080 && f == 60);
    return 0;
}
