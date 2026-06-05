#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "ti_msp_dl_config.h"
#include "pid.h"
#include "filter.h"
#include <math.h>

#define DEG2RAD(deg)    ((deg) * 3.14159265358979 / 180.0f)
#define RAD2DEG(rad)    ((rad) * 180.0f / 3.14159265358979 )
#define      PI      3.14159265358979

// 电机状态枚举
typedef enum{
	PID_CONTROL = 0,      // 正常PID控制
	BRAKE = 1,            // 拉高双引脚硬件制动
} Motor_State;

// 电机结构体定义
typedef struct {
    float omiga;            // 单位：弧度
    Motor_State state;
    float output;
}Motor;

// 定义电机测试结构体
extern Motor set_motor0;
extern Motor set_motor1;
extern Motor measure_motor0;
extern Motor measure_motor1;

// 定义PID结构体
extern PID pid0_omiga;
extern PID pid1_omiga;

// 定义编码器记录值
extern int32_t encoder0;
extern int32_t encoder1;

// 里程计
extern float motordis;      // 里程
extern uint8_t disFlag;     // 标志位

// 对外接口函数
void testPWM(float pwm0,float pwm1);
void Motor_Init(void);
void loopControl(void);
void set_omiga(float omiga0, float omiga1);
void motor_stop(void);
void motor_release(void);
// 里程计
void reset_dis(void);
void setDis(uint8_t a);


#endif
