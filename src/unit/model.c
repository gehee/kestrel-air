// Which air unit this is. Both run the same stock firmware, which tells them
// apart in its board script (fpv_run_by_type.sh) by a voltage on LSADC
// channel 1: 0.75-1.05 V is board type 472, the Lite+, 1.5-1.8 V is 482, the
// Lite (flag files can force a type). The script writes the result to
// /usrdata/fpv/fpv_board_type and hands it on, and air/start.sh passes it to
// kestrel-air as KA_BOARD_TYPE.
//
// Where the stock air app treats the Lite+ (its project number 7, the Lite's
// is 4) differently, and kestrel-air does the same:
//  - the project number in the camera settings sent to the ground
//    (ground/reports.c);
//  - four TX power offsets, two groups of two paths, instead of two
//    (radio/radio.c);
//  - a front-end module (FEM) switched to high power above 23 dBm, low power
//    otherwise and at start (radio/power.c).
// Not done: the Lite+ also sends audio, which kestrel-air does not.
// The radio chip firmware and config (bb_demo_sky_cx472.img and
// bb_config_sky_cx472*.json on the Lite+) are picked by air/start.sh.
#include "unit/model.h"

#include "app/settings.h"

#include <stdio.h>
#include <stdlib.h>

static const struct { int board_type, prj; const char *name; } models[] = {
    { 482, 4, "Ascent Lite" },
    { 472, 7, "Ascent Lite+" },
};
static int cur;

int model_init(const char *board_type_file) {
    int bt = env_int("KA_BOARD_TYPE", 0);
    const char *from = "KA_BOARD_TYPE";
    if (!bt) {
        FILE *f = fopen(board_type_file, "r");
        if (f) {
            if (fscanf(f, "%d", &bt) != 1) bt = 0;
            fclose(f);
        }
        from = board_type_file;
    }
    if (!bt) {
        printf("model: no board type (KA_BOARD_TYPE, %s): taken as a Lite\n", board_type_file);
        cur = 0;
        return 0;
    }
    for (unsigned i = 0; i < sizeof(models) / sizeof(models[0]); i++)
        if (models[i].board_type == bt) {
            cur = (int)i;
            printf("model: %s (board type %d, from %s)\n", models[i].name, bt, from);
            return 0;
        }
    printf("model: board type %d (from %s) is not a Lite or Lite+\n", bt, from);
    return -1;
}

int model_board_type(void) { return models[cur].board_type; }
int model_prj(void) { return models[cur].prj; }
int model_lite_plus(void) { return models[cur].board_type == 472; }
const char *model_name(void) { return models[cur].name; }
