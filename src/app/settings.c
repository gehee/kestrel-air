#include "app/settings.h"
#include "unit/model.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Every KA_* setting, where it is read and what its default is.
static const struct { const char *name, *def, *what; } known[] = {
    { "KA_BOARD_TYPE", BOARD_TYPE_FILE, "stock's board type: 482 Lite, 472 Lite+; set by air/start.sh (unit/model.c)" },
    { "KA_IR",         "2",            "intra refresh, CTU rows (or columns) per picture; 0: periodic keyframes (video/encoder.c)" },
    { "KA_IR_MODE",    "0",            "intra refresh by rows (0) or columns (1)" },
    { "KA_IR_QP",      "32",           "intra refresh's request_i_qp" },
    { "KA_GOP",        "one sweep",    "GOP length; with intra refresh one sweep (17 at 1080p), else 25" },
    { "KA_SLICE_COUNT", "by size",     "slices a picture (2 above 60 fps, 4 at 60 and below); rows each from the height (video/slices.c)" },
    { "KA_LATINFO",    "1",            "0: leave the header bytes that are ours as the stock app sends them" },
    { "KA_LD_LINES",   "64 / 128",     "VPSS low delay, lines (128 with intra refresh) (camera/pipeline.c)" },
    { "KA_BW40",       "0",            "1: widen the video link to 40 MHz on its own (radio/bandwidth.c)" },
    { "KA_STRATEGY",   "the ground's", "video_strategy, whatever the ground's INIT_CFG says (app/config.c)" },
    { "KA_DUMP",       "off",          "file: the first KA_DUMP_KB KiB sent, as an Annex-B stream (video/video.c)" },
    { "KA_DUMP_KB",    "8192",         "how much KA_DUMP keeps" },
};
#define NKNOWN ((int)(sizeof(known) / sizeof(known[0])))

const char *env_str(const char *name) {
    const char *v = getenv(name);
    return v && *v ? v : NULL;
}

int env_int(const char *name, int def) {
    const char *v = env_str(name);
    return v ? atoi(v) : def;
}

void settings_log(void) {
    extern char **environ;
    for (char **e = environ; *e; e++) {
        if (strncmp(*e, "KA_", 3)) continue;
        size_t n = strcspn(*e, "=");
        int k = 0;
        while (k < NKNOWN && (strlen(known[k].name) != n || strncmp(*e, known[k].name, n))) k++;
        if (k < NKNOWN) printf("settings: %s (default %s): %s\n", *e, known[k].def, known[k].what);
        else printf("settings: %.*s: not a setting kestrel-air knows - ignored\n", (int)n, *e);
    }
}
