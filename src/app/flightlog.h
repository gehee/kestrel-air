// The flight logs: what kestrel-air prints, kept on the flash for each armed
// flight (from 10 s before arming to 10 s after disarming), the newest five,
// in /usrdata/fpv/air/logs. /tmp/air.log is in RAM and gone at power-off.
// Off until the goggle turns them on (its Debug Info, KA_CMD_LOGS at every
// link-up); not kept across a reboot.
#pragma once
#include <stdint.h>

// Takes over stdout and stderr (still passed on to where they went before).
void flightlog_start(void);
// The goggle's clock (seconds since 1970, from its config frame): names the
// logs fpvOS_YYYYMMDD_HHMMSS.log, as the goggle names its own.
void flightlog_set_time(uint32_t epoch);
// On or off; a flight being kept when turned off is closed as at disarming.
void flightlog_set_enabled(int on);
