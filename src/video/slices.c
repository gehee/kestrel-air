// How a picture is cut into slices: a number of slices (KA_SLICE_COUNT, or the
// default for the size and rate), turned into the encoder's CTU rows per slice.
//
// Fewer rows a slice means each part of the picture leaves the encoder sooner
// after its rows are read; but at 1080p100 four slices made the encoder skip
// every other picture (2026-10-04: 50 fps out of 100 in), three kept up
// (2026-10-06). So two above 60 fps, four at 60 and below.
#include "video/slices.h"

#define CTU 32          // the encoder's H.265 CTU, pixels

int slice_count_default(int height, int fps) {
    if (height == 900 || height == 1440) return 4;
    return fps > 60 ? 2 : 4;
}

// ceil(CTU rows / count): the last slice takes what is left, never more rows
// than the others. Counts outside 1..8 are clamped.
int slice_rows_for(int height, int count) {
    const int rows = (height + CTU - 1) / CTU;
    if (count < 1) count = 1;
    if (count > 8) count = 8;
    if (count > rows) count = rows;
    return (rows + count - 1) / count;
}
