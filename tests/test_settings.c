#include <stdlib.h>
#include <string.h>

#include "app/settings.h"
#include "check.h"

void test_settings(void) {
    unsetenv("KA_IR");
    CHECK(env_int("KA_IR", 2) == 2);           // unset: the default
    setenv("KA_IR", "", 1);
    CHECK(env_int("KA_IR", 2) == 2);           // empty: the default too
    CHECK(env_str("KA_IR") == NULL);
    setenv("KA_IR", "0", 1);
    CHECK(env_int("KA_IR", 2) == 0);
    setenv("KA_DUMP", "/tmp/x", 1);
    CHECK(env_str("KA_DUMP") && !strcmp(env_str("KA_DUMP"), "/tmp/x"));
    setenv("KA_NOT_A_SETTING", "1", 1);
    settings_log();                            // names it, and does not stop
    unsetenv("KA_IR"); unsetenv("KA_DUMP"); unsetenv("KA_NOT_A_SETTING");
}
