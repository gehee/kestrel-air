#include <assert.h>
#include <errno.h>
#include <linux/input.h>
#include <stdarg.h>
#include <string.h>
#include "platform/platform.h"
#include "camera/pipeline.h"
static int pressed, failed;
int camera_has_720p90(void) { return 0; }   // platform_format's: the camera is not linked
int __wrap_ioctl(int fd, unsigned long request, ...) {
    assert(fd == 42);
    assert(request == EVIOCGKEY((KEY_MAX + 8) / 8));
    if (failed) { errno = ENODEV; return -1; }
    va_list ap;
    va_start(ap, request);
    unsigned char *keys = va_arg(ap, unsigned char *);
    va_end(ap);
    memset(keys, 0, (KEY_MAX + 8) / 8);
    if (pressed) keys[KEY_WPS_BUTTON / 8] |= 1u << (KEY_WPS_BUTTON % 8);
    return 0;
}
int main(void) {
    char value = '?';
    assert(platform_key_read(42, &value) == 1 && value == '1');
    pressed = 1;
    assert(platform_key_read(42, &value) == 1 && value == '0');
    pressed = 0;
    assert(platform_key_read(42, &value) == 1 && value == '1');
    failed = 1;
    value = '?';
    assert(platform_key_read(42, &value) < 0 && value == '?');
    assert(!platform_key_upgrade());
}
