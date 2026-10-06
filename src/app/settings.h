// Settings from the environment: KA_<NAME>=value, for trying things on a unit
// without a new build. Every name is listed, with its default, in settings.c.
// Unset or empty means the default.
#pragma once

int         env_int(const char *name, int def);
const char *env_str(const char *name);   // NULL if unset or empty
void        settings_log(void);          // at start-up: the ones set, and unknown KA_* names
