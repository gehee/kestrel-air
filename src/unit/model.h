// Which air unit this is: the Ascent Lite, the Ascent Lite+, or the Ascent GT (see model.c).
#pragma once

#define BOARD_TYPE_FILE "/usrdata/fpv/fpv_board_type"

int  model_init(const char *board_type_file);  // 0, or -1: a board kestrel-air does not run on
int  model_board_type(void);    // stock's board type: 482 (Lite), 472 (Lite+), 4861 (GT)
int  model_prj(void);           // stock's project number for it: 4, 7 or 5
int  model_lite_plus(void);     // the Lite+: four power offsets, a high-power FEM
int  model_gt(void);            // the GT: board 4861, PS5280, the Lite's 3v3 radio
int  model_cam_imu(void);       // an IMU on the camera head: the Lite yes, the Lite+ and the GT no
const char *model_name(void);

int model_radio_defaults(void); // Legacy power-offset files and empty peer defaults
