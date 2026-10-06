#pragma once

// The stock air app's settings (see config.c).
#define CFG_PATH "/factory/fpv_config.json"

int  cfg_load(void);                        // -1 if the file is missing
int  cfg_get(const char *key, int def);
void cfg_set(const char *key, int val);
int  cfg_save(void);

// Integers under key in any JSON file: one number or an array (numbers or
// quoted "0x.." strings). Returns how many were read, -1 if none.
int  cfg_json_ints(const char *path, const char *key, int *v, int max);
// Set key (a raw JSON value) in any JSON file, keeping the other members.
int  cfg_json_set(const char *path, const char *key, const char *value_json);
