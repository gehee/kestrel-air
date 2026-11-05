// White balance by colour temperature, from the SC231AI tuning file's AWB
// block: the vendor's own lookup (libmpp_service.so 0x277c40, which stock's
// manual white balance never reaches - it freezes the auto gains instead).
#pragma once
#include <stddef.h>
#include <stdint.h>

#define AR9311_WB_BLOCK      0xbbe94   // the AWB block's offset in the tuning file
#define AR9311_WB_BLOCK_SIZE 0x110

// Q8 red and blue gains (green 256) for kelvin: 0, or -1 when the block holds
// no usable reference points.
int ar9311_wb_gains(const uint8_t *block, size_t size, int kelvin, unsigned *red, unsigned *blue);
// The same from the tuning file at path.
int ar9311_wb_gains_file(const char *path, int kelvin, unsigned *red, unsigned *blue);
