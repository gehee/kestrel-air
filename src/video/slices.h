// How a picture is cut into slices (see slices.c). No SDK: the host tests use it.
#pragma once

int slice_count_default(int height, int fps);          // slices a picture by default
int slice_rows_for(int height, int count);             // CTU rows per slice for that many slices
