// The stock air app's settings files, read and written the way it does.
//
// /factory/fpv_config.json: stock's
// defaults first, then every key present in the file on top; a missing
// file is written with the defaults. Every save writes all keys, in stock's
// order, jansson-style (four-space indent, no trailing newline), then syncs.
//
// Other JSON files (/factory/user_cfg.json, /usrdata/mp_cfg.json) are read
// with cfg_json_ints and updated with cfg_json_set, which keeps every other
// member and its order, as jansson's load-modify-dump does.
#include "kestrel_air.h"
#include "app/config.h"
#include "common/json.h"
#include "app/settings.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static struct { const char *key; int val; } keys[] = {
    { "ev_x10", 0 }, { "scenes", 0 }, { "sat", 0 }, { "contrast", 0 }, { "sharpness", 0 },
    { "cct", 0 }, { "angle", 0 }, { "ratio", 1 },
    { "ch0_width", 1280 }, { "ch0_height", 720 }, { "ch0_fps", 60 }, { "ch0_focus_en", 0 },
    { "ch1_width", 1920 }, { "ch1_height", 1080 }, { "ch1_fps", 60 },
    { "rec_dev", 0 }, { "rec_loop_dev", 0 }, { "rec_auto_dev", 0 },
    { "cam_exp", 0 }, { "cam_max_iso", 0 }, { "cam_3dnr_strength", 1 }, { "rec_eis_en", 0 },
    { "rec_color_mode", 0 }, { "rec_sharpness", 5 }, { "rec_sat", 5 }, { "rec_enc_format", 0 },
    { "rec_pack_durtion", 5 }, { "cam_2dnr_strength", 1 }, { "cam_max_iso_mode", 0 },
    { "cam_anti_flicker_en", 0 }, { "bb_power_mw", 25 }, { "sys_standby_mode", 1 },
    { "video_strategy", 0 },
};
#define NKEYS ((int)(sizeof(keys) / sizeof(keys[0])))
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

// ---- fpv_config ----------------------------------------------------------

int cfg_load(void) {
    char *doc = json_slurp(CFG_PATH);
    if (!doc) {                      // a fresh unit: write the defaults
        cfg_save();
        return -1;
    }
    const char *p = doc;
    json_node *root = json_parse(&p);
    pthread_mutex_lock(&mtx);
    for (json_node *k = root->type == 'o' ? root->kids : NULL; k; k = k->next)
        for (int i = 0; i < NKEYS; i++)
            if (k->key && !strcmp(k->key, keys[i].key) && k->type == 'n') keys[i].val = atoi(k->text);
    pthread_mutex_unlock(&mtx);
    json_free(root);
    free(doc);
    return 0;
}

int cfg_get(const char *key, int def) {
    int v = def;
    // Experiment: KA_STRATEGY forces video_strategy (0 race/low delay, 2 wing),
    // whatever the ground's INIT_CFG says.
    if (!strcmp(key, "video_strategy") && env_str("KA_STRATEGY")) return env_int("KA_STRATEGY", 0);
    pthread_mutex_lock(&mtx);
    for (int i = 0; i < NKEYS; i++)
        if (!strcmp(keys[i].key, key)) { v = keys[i].val; break; }
    pthread_mutex_unlock(&mtx);
    return v;
}

void cfg_set(const char *key, int val) {
    pthread_mutex_lock(&mtx);
    for (int i = 0; i < NKEYS; i++)
        if (!strcmp(keys[i].key, key)) { keys[i].val = val; break; }
    pthread_mutex_unlock(&mtx);
}

int cfg_save(void) {
    FILE *f = fopen(CFG_PATH, "w");
    if (!f) return -1;
    pthread_mutex_lock(&mtx);
    fprintf(f, "{\n");
    for (int i = 0; i < NKEYS; i++)
        fprintf(f, "    \"%s\": %d%s\n", keys[i].key, keys[i].val, i + 1 < NKEYS ? "," : "");
    fprintf(f, "}");
    pthread_mutex_unlock(&mtx);
    fflush(f);
    fclose(f);
    sync();
    return 0;
}

// ---- other files ----------------------------------------------------------

int cfg_json_ints(const char *path, const char *key, int *v, int max) {
    char *doc = json_slurp(path);
    int n = -1;
    if (!doc) return -1;
    const char *p = doc;
    json_node *root = json_parse(&p);
    for (json_node *k = root->type == 'o' ? root->kids : NULL; k; k = k->next) {
        if (!k->key || strcmp(k->key, key)) continue;
        if (k->type == 'a') {
            n = 0;
            for (json_node *e = k->kids; e && n < max; e = e->next)
                if (e->type != 'o' && e->type != 'a') v[n++] = json_num(e);
        } else if (max > 0 && (k->type == 'n' || k->type == 's')) {
            v[0] = json_num(k);
            n = 1;
        }
        break;
    }
    json_free(root);
    free(doc);
    return n;
}

// Set key to a raw JSON value, in place if present, appended if not; every
// other member stays as it was. A missing file starts an empty object.
int cfg_json_set(const char *path, const char *key, const char *value_json) {
    char *doc = json_slurp(path);
    const char *p = doc ? doc : "{}";
    json_node *root = json_parse(&p);
    free(doc);
    if (root->type != 'o') { json_free(root); root = calloc(1, sizeof(*root)); root->type = 'o'; }
    const char *vp = value_json;
    json_node *val = json_parse(&vp), **pk = &root->kids;
    for (; *pk; pk = &(*pk)->next)
        if ((*pk)->key && !strcmp((*pk)->key, key)) break;
    val->key = strdup(key);
    if (*pk) {
        val->next = (*pk)->next;
        (*pk)->next = NULL;
        json_free(*pk);
    }
    *pk = val;
    int r = json_write(path, root);
    json_free(root);
    return r;
}
