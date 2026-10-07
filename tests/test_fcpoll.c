#include <string.h>

#include "check.h"
#include "unit/fcpoll.h"

void test_fcpoll(void) {
    fcpoll_t p;
    fcpoll_init(&p);
    int count[256] = { 0 }, last[256], gap_max[256] = { 0 };
    memset(last, -1, sizeof(last));
    // Ten seconds of slots.
    for (int slot = 0; slot < 10 * FCPOLL_SLOTS_PER_SEC; slot++) {
        uint8_t c = fcpoll_next(&p);
        count[c]++;
        if (last[c] >= 0 && slot - last[c] > gap_max[c]) gap_max[c] = slot - last[c];
        last[c] = slot;
    }
    // Each as often as its row says, over ten seconds.
    CHECK(count[FCP_MSP_STATUS] == 140);
    CHECK(count[FCP_MSP_BATTERY_STATE] == 60 && count[FCP_MSP_ALTITUDE] == 60);
    CHECK(count[FCP_MSP_RAW_GPS] == 50 && count[FCP_MSP_COMP_GPS] == 50);
    CHECK(count[FCP_MSP_ANALOG] == 40 && count[FCP_MSP_BOXIDS] == 40);
    CHECK(count[FCP_MSP_FC_VERSION] == 30 && count[FCP_MSP_FC_VARIANT] == 30);
    CHECK(count[108] == 0);                         // attitude is not asked for
    // Spread: status (14 a second) is never more than 5 slots apart, battery (6, every 8.3 slots on average) not more than 12.
    CHECK(gap_max[FCP_MSP_STATUS] <= 5);
    CHECK(gap_max[FCP_MSP_BATTERY_STATE] <= 12);
    // Nothing else is ever asked.
    int total = 0;
    for (int i = 0; i < 256; i++) total += count[i];
    CHECK(total == 10 * FCPOLL_SLOTS_PER_SEC);
}
