// Which air unit this is: the Ascent Lite or the Ascent Lite+ (see model.c).
#pragma once

#define BOARD_TYPE_FILE "/usrdata/fpv/fpv_board_type"

int  model_init(const char *board_type_file);  // 0, or -1: a board kestrel-air does not run on
int  model_board_type(void);    // stock's board type: 482 (Lite) or 472 (Lite+)
int  model_prj(void);           // stock's project number for it: 4 or 7
int  model_lite_plus(void);     // the Lite+: four power offsets, a high-power FEM
const char *model_name(void);
