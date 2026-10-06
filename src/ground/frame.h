// The frame around every message with the ground (see frame.c).
#pragma once
#include <stdint.h>

#define FRAME_OVERHEAD 11   // header 6, the 00 after the payload, CRC 4

// Builds a frame around len bytes of payload into f (room for len + 11);
// returns its length. ack: an acknowledgement of the frame numbered seq.
int frame_build(uint8_t *f, uint8_t seq, int ack, const uint8_t *payload, int len);

// What starts at b, with have bytes there: FRAME_OK with the payload's length
// in *len (the frame is *len + FRAME_OVERHEAD bytes, b[3] its number, b[4] its
// flags, the payload at b + 6); FRAME_MORE if it is not all there yet; else
// not a frame at b (skip a byte and look again).
enum { FRAME_OK = 0, FRAME_MORE, FRAME_NOT, FRAME_BAD_SUM, FRAME_BAD_CRC };
int frame_parse(const uint8_t *b, int have, int *len);
