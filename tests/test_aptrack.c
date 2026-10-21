#include "video/aptrack.h"
#include "check.h"

void test_aptrack(void) {
    const uint64_t s = 1000000;
    // The fallback without a sample: every stamp asks.
    CHECK(ap_track_due(0, 0, 5 * s, 5 * s));
    CHECK(ap_track_due(0, 0, 5 * s + 10, 5 * s));
    // With a sample: once a second, frame clock good or not.
    CHECK(!ap_track_due(1, 0, 5 * s + s - 1, 5 * s));
    CHECK(ap_track_due(1, 0, 6 * s, 5 * s));
    CHECK(!ap_track_due(1, 1, 5 * s + s - 1, 5 * s));
    CHECK(ap_track_due(1, 1, 6 * s, 5 * s));
    // Frame clock good, no sample yet (start, or a reconnect): still at most
    // one read a second, the first at once.
    CHECK(ap_track_due(0, 1, 5 * s, 0));
    CHECK(!ap_track_due(0, 1, 5 * s + 1000, 5 * s));
    CHECK(ap_track_due(0, 1, 6 * s, 5 * s));
}
