#include "check.h"
#include "radio/power_levels.h"

// The levels a ground may ask for (command 0x22): stock's, and ours.
void test_power(void) {
    // Stock's lists, as stock sends them.
    CHECK(power_level_of(25) && power_level_of(25)->dbm == 13 && !power_level_of(25)->automatic);
    CHECK(power_level_of(100) && power_level_of(100)->dbm == 19);
    CHECK(power_level_of(101) && power_level_of(101)->dbm == 19 && power_level_of(101)->automatic);
    CHECK(power_level_of(200) && power_level_of(200)->dbm == 22);
    CHECK(power_level_of(500) && power_level_of(500)->dbm == 26 && !power_level_of(500)->automatic);
    CHECK(power_level_of(501) && power_level_of(501)->dbm == 26 && power_level_of(501)->automatic);

    // 316 mW is 24 dBm, and its AUTO has the same top.
    CHECK(power_level_of(316) && power_level_of(316)->dbm == 24 && !power_level_of(316)->automatic);
    CHECK(power_level_of(317) && power_level_of(317)->dbm == 24 && power_level_of(317)->automatic);
    // 400 mW is 25 dBm, between 316 and 500.
    CHECK(power_level_of(400) && power_level_of(400)->dbm == 25 && !power_level_of(400)->automatic);
    CHECK(power_level_of(401) && power_level_of(401)->dbm == 25 && power_level_of(401)->automatic);

    // Every AUTO value is its level + 1, with the level's dBm.
    for (int i = 0; i < POWER_LEVELS_N; i++) {
        const struct power_level *l = &power_levels[i];
        if (!l->automatic) continue;
        const struct power_level *fixed = power_level_of(l->p - 1);
        CHECK(fixed && !fixed->automatic && fixed->dbm == l->dbm);
    }

    // Levels rise with the value, and nothing is above 500 mW (26 dBm: the
    // amplifier distorts past it).
    for (int i = 0; i < POWER_LEVELS_N; i++) CHECK(power_levels[i].dbm <= 26);
    CHECK(power_level_of(630) == 0 && power_level_of(790) == 0);   // the removed TEST levels
    CHECK(power_level_of(0) == 0 && power_level_of(24) == 0);
}
