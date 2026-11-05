#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include "unit/model.h"
#include "platform/platform.h"
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
    return 0;
}
