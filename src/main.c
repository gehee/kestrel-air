// kestrel-air: the Caddx Ascent air unit's video application.
//
// It runs the unit as the stock air app does - camera, encoder, the AR8030
// radio and the ground's messages (src/*/) - with the radio reached through
// ar_libre's client library, with no radio daemon.
#include "video/video.h"
#include "camera/image.h"
#include "imu/imu.h"
#include "app/app.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


char keepRunning = 1;

static void usage(void) {
  printf(
    "\n\t\tkestrel-air %s\n"
    "\n"
    "  Usage:\n"
    "    kestrel-air [Arguments]\n"
    "\n"
    "    --tx-mode ar8030          - Accepted for older start scripts; the only mode\n"
    "    --bb-window N             - Video writes in flight to the radio, unacknowledged\n"
    "                                (Default: 1, as stock)\n"
    "    --max-exposure-us US      - Cap the sensor exposure (Default: 0 = the tuning's)\n"
    "    --imu                     - IMU data for the ground, from where the board has it: the\n"
    "                                camera's IMU (Lite: ICM-40609-D on /dev/spidev0.0), its\n"
    "                                samples in the video as H.265 SEI messages; else the flight\n"
    "                                controller's (Lite+: MSP_RAW_IMU, 20 a second)\n"
    "    --imu-dev PATH            - --imu on another spidev node\n"
    "    --bb-verbose              - Per-stage timing and radio events in the log\n"
    "    --help                    - This text\n"
    "\n"
    "  Picture size, frame rate and camera angle come from the unit's own\n"
    "  /factory/fpv_config.json, as with the stock app.\n"
    "\n", KA_VERSION);
}

static void on_signal(int sig) {
  (void)sig;
  keepRunning = 0;
}

int main(int argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    const char *a = argv[i];
    const char *v = i + 1 < argc ? argv[i + 1] : NULL;
    int takes_value = !strcmp(a, "--tx-mode") ||
                      !strcmp(a, "--bb-window") || !strcmp(a, "--max-exposure-us") ||
                      !strcmp(a, "--imu-dev");

    if (takes_value && !v) {
      fprintf(stderr, "kestrel-air: %s needs a value\n", a);
      return 2;
    }
    if (!strcmp(a, "--tx-mode")) {
      if (strcmp(v, "ar8030")) {
        fprintf(stderr, "kestrel-air: --tx-mode %s: ar8030 is the only mode\n", v);
        return 2;
      }
    } else if (!strcmp(a, "--bb-window")) {
      int w = atoi(v);
      video_window = w < 1 ? 1 : w;
    } else if (!strcmp(a, "--max-exposure-us")) {
      image_max_exposure_us = (unsigned)atoi(v);
    } else if (!strcmp(a, "--imu")) {
      if (!imu_dev) imu_dev = "/dev/spidev0.0";
    } else if (!strcmp(a, "--imu-dev")) {
      imu_dev = v;
    } else if (!strcmp(a, "--bb-verbose")) {
      video_verbose = 1;
    } else if (!strcmp(a, "--help") || !strcmp(a, "-h")) {
      usage();
      return 0;
    } else {
      fprintf(stderr, "kestrel-air: unknown argument %s\n", a);
      usage();
      return 2;
    }
    if (takes_value) i++;
  }

  signal(SIGINT, on_signal);
  // A peer that goes away is an error to report, not a reason to die
  // mid-write.
  signal(SIGPIPE, SIG_IGN);
  setvbuf(stdout, NULL, _IOLBF, 0);

  return app_run(&keepRunning);
}
