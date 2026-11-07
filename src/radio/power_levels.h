// The TX power values a ground may ask for (ground command 0x22), and the
// radio level each one is.
//
// Stock's encoding: N mW holds level N, N + 1 is "AUTO, up to N" (the radio
// takes the ground's power requests between 25 mW and N). Stock's lists are
// 25 / 100 / 101 (Lite) and 25 / 100 / 200 / 500 / 501 (Lite+). The 150, 200
// and 316 mW levels and their AUTO (151, 201, 317) are ours: a ground that
// has seen KA_FEAT_PWRLVL offers them, so AUTO's top can be set below 500 mW.
// 400 mW (25 dBm) and its AUTO (401) too, offered where the info says
// pwr400=1 (the header's feature byte is full).
//
// dbm: the board's level for it, stock's table (labels 1 dB above the level:
// kept, the board's gain after the chip is not known). 316 mW is 24 dBm: what
// the radio sat at in AUTO on the 2026-11-03 flight, where it never asked for
// 26.
#ifndef POWER_LEVELS_H
#define POWER_LEVELS_H

struct power_level { int p; int dbm; int automatic; };

static const struct power_level power_levels[] = {
    {  25, 13, 0 }, { 100, 19, 0 }, { 150, 21, 0 }, { 200, 22, 0 }, { 316, 24, 0 }, { 400, 25, 0 },
    { 500, 26, 0 },
    { 101, 19, 1 }, { 151, 21, 1 }, { 201, 22, 1 }, { 317, 24, 1 }, { 401, 25, 1 }, { 501, 26, 1 },
};
#define POWER_LEVELS_N ((int)(sizeof(power_levels) / sizeof(power_levels[0])))

// The row for p, or 0.
static inline const struct power_level *power_level_of(int p) {
    for (int i = 0; i < POWER_LEVELS_N; i++)
        if (power_levels[i].p == p) return &power_levels[i];
    return 0;
}

#endif
