#include "video/slices.h"
#include "check.h"

void test_slices(void) {
    // The defaults give the rows used before the count existed.
    CHECK(slice_rows_for(1080, slice_count_default(1080, 100)) == 17);
    CHECK(slice_rows_for(1080, slice_count_default(1080, 60)) == 9);
    CHECK(slice_rows_for(720, slice_count_default(720, 120)) == 12);
    CHECK(slice_rows_for(720, slice_count_default(720, 60)) == 6);
    CHECK(slice_rows_for(900, slice_count_default(900, 90)) == 8);
    CHECK(slice_rows_for(1440, slice_count_default(1440, 60)) == 12);
    // Three slices of 1080p: 12, 12, 10 rows.
    CHECK(slice_rows_for(1080, 3) == 12);
    CHECK(slice_rows_for(1080, 1) == 34);
    CHECK(slice_rows_for(1080, 0) == 34);      // clamped to one
    CHECK(slice_rows_for(1080, 99) == 5);      // clamped to eight: 34 / 8 rounded up
}
