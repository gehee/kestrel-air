#include "check.h"
#include "video/spectator.h"
void test_spectator(void) {
    uint8_t out[64], rb[64], mac[4] = {0, 0, 1, 2};
    CHECK(!spectator_sei(out, 55, mac, 1920, 1080, 60));
    size_t n = spectator_sei(out, sizeof(out), mac, 1920, 1080, 60);
    CHECK(n >= 39 && n <= 56);
    CHECK(out[3] == 1 && out[4] == 80 && out[5] == 1);
    size_t j = 0; int zeros = 0;
    for (size_t i = 6; i < n; ++i) {
        if (zeros == 2 && out[i] == 3) { zeros = 0; continue; }
        rb[j++] = out[i]; zeros = out[i] == 0 ? zeros + 1 : 0;
    }
    CHECK(j == 33 && rb[0] == 5 && rb[1] == 30);
    CHECK(memcmp(rb + 2, "kestrel-spec-v1", 15) == 0 && rb[17] == 0);
    CHECK(rb[18] == 1 && rb[19] == 1 && memcmp(rb + 20, mac, 4) == 0);
    CHECK((rb[24] | rb[25] << 8) == 1920);
    CHECK((rb[26] | rb[27] << 8) == 1080);
    CHECK((rb[30] | rb[31] << 8) == 2000 && rb[32] == 128);
    spectator_recovery s = {0};
    CHECK(!spectator_recovery_due(&s, 0, 1, 0));
    CHECK(!spectator_recovery_due(&s, 1, 1, 100));
    CHECK(!spectator_recovery_due(&s, 1, 1, 2000099));
    CHECK(!spectator_recovery_due(&s, 1, 0, 2000100));
    CHECK(spectator_recovery_due(&s, 1, 1, 2000100));
    CHECK(spectator_recovery_due(&s, 1, 1, 20000100));
    CHECK(!spectator_recovery_due(&s, 1, 1, 20000101));
    CHECK(!spectator_recovery_due(&s, 0, 1, 22000100));
    CHECK(!spectator_recovery_due(&s, 1, 1, 24000100));
    CHECK(spectator_recovery_due(&s, 1, 1, 26000100));
}
