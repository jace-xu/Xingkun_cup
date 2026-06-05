#ifndef TRACKLINE_H
#define TRACKLINE_H

#include "motorControl.h"
#include "ti_msp_dl_config.h"
#include "pid.h"

typedef enum {
    TRACK = 0,
    LOST,
    NEXT
} TRACK_LINE_ENUM;

// 循迹功能函数
void Track_Init(void);
TRACK_LINE_ENUM Track_Run(void); // 返回值改为枚举类型
void Track_SetSpeed(float speed);

uint8_t Track_IsLineDetected(void);


#endif
