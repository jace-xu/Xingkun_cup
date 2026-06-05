#ifndef TASK_H
#define TASK_H

#include <stdint.h>
#include "pid.h"
#include "trackLine.h"     
#include "motorControl.h" 

// 定义状态机状态
typedef enum {
    TASK_IDLE = 0,               
    TASK_TURN_TO_ANGLE,          
    TASK_STRAIGHT_UNTIL_LINE,    
    TASK_LINE_TRACKING,          
    TASK_STOP                    
} TaskState_t;

// 状态机全局结构体
// 在 TaskManager_t 结构体中新增两个速度变量
typedef struct {
    TaskState_t state;
    
    // --- 任务配置参数 ---
    uint8_t target_rounds;       
    uint8_t task_type;           
    float target_angles[6];      
    float dist_threshold;        
    
    // --- 新增：动态速度参数 ---
    float straight_speed;        // 走直线时的速度
    float track_speed;           // 巡线时的速度
    
    // --- 运行状态变量 ---
    uint8_t round_count;         
    uint8_t half_step;           
    uint8_t start_trigger;       
} TaskManager_t;

extern TaskManager_t carTask;

void Task_Init(void);
void Task_Run(void);             

// 修改后的配置接口：直接暴露6次调角的独立参数
void Task_SetConfig(uint8_t rounds, uint8_t task_type, 
                    float a1, float a2, float a3, 
                    float a4, float a5, float a6);

void Task_Control(uint8_t enable);


// 声光提示
void beep_led_tip(void);
void run_beep_led(void);

#endif