#ifndef __IT_H
#define __IT_H

#ifdef __cplusplus
extern "C" {
#endif
#include "ti_msp_dl_config.h"
#include "motorControl.h"

extern float angleData[];
extern uint8_t imu_ready;

void IMU_SendCalibrationCmd(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_CALLBACKS_H */
