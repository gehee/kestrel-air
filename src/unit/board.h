// The board: the pairing key, the LEDs, debug commands (see board.c).
#pragma once

void board_trigger_pairing(void);
int  board_start(void);                  // the button thread, LED timer, debug commands
void board_leds_boot(void);              // red off, green on
