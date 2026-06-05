#include "task.h"
#include "ti_msp_dl_config.h"

#define TASK_TICK_MS            20       

#define ANGLE_KP                0.5f     
#define ANGLE_KI                0.0f     
#define ANGLE_KD                0.0f     
#define ANGLE_PID_LIMIT         2.0f    
#define ANGLE_DEADZONE          1.0f     

// === 速度宏定义区 ===
#define V_BASE_TURN             0.0f     

// 任务1、2使用的基础慢速
#define V_BASE_STRAIGHT         6.0f    
#define V_BASE_TRACK            3.14f     // 对应 trackLine.c 里的 TRACK_SPEED 默认值

// 任务3使用的极速模式 
#define V_FAST_STRAIGHT         6.28f    
#define V_FAST_TRACK            6.28f     
// ====================

TaskManager_t carTask;           
PID angle_pid;                   

extern uint8_t imu_ready;        
extern float angleData[3];       

extern float motordis;
extern void reset_dis(void);
extern void setDis(uint8_t a);


// 配置接口：将6个角度依次存入数组，并根据任务类型分配速度
void Task_SetConfig(uint8_t rounds, uint8_t task_type, 
                    float a1, float a2, float a3, 
                    float a4, float a5, float a6)
{
    carTask.target_rounds = rounds;
    carTask.task_type = task_type;
    
    carTask.target_angles[0] = a1; // 第1圈上半圈
    carTask.target_angles[1] = a2; // 第1圈下半圈
    carTask.target_angles[2] = a3; // 第2圈上半圈
    carTask.target_angles[3] = a4; // 第2圈下半圈
    carTask.target_angles[4] = a5; // 第3圈上半圈
    carTask.target_angles[5] = a6; // 第3圈下半圈

    // --- 核心修改：动态分配阈值与速度 ---
    if (task_type == 1) {
        carTask.dist_threshold = 0.8f;  
        carTask.straight_speed = V_BASE_STRAIGHT; 
        carTask.track_speed    = V_BASE_TRACK;    
    } 
    else if (task_type == 2) {
        carTask.dist_threshold = 1.0f;  
        carTask.straight_speed = V_BASE_STRAIGHT; 
        carTask.track_speed    = V_BASE_TRACK;    
    } 
    else if (task_type == 3) {
        // 任务3：类似任务1走直线，但速度全开！
        carTask.dist_threshold = 0.8f;  
        carTask.straight_speed = V_FAST_STRAIGHT; 
        carTask.track_speed    = V_FAST_TRACK;    
    }
    else {
        // 兜底保护
        carTask.dist_threshold = 0.8f;  
        carTask.straight_speed = V_BASE_STRAIGHT; 
        carTask.track_speed    = V_BASE_TRACK;    
    }
}

void Task_Control(uint8_t enable)
{
    if (enable == 1) {
        carTask.start_trigger = 1;
    } 
    else if (enable == 0) {
        carTask.start_trigger = 0;
        carTask.state = TASK_IDLE;      
        carTask.round_count = 0;        
        carTask.half_step = 0;          
        
        set_omiga(0.0f, 0.0f);          
        setDis(0);                      
    }
}

void Task_Init(void)
{
    carTask.state = TASK_IDLE;
    carTask.round_count = 0;
    carTask.half_step = 0;
    carTask.start_trigger = 0; 
    
    // 默认全0防错，走任务1
    Task_SetConfig(1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    PID_Init(&angle_pid, ANGLE_KP, ANGLE_KI, ANGLE_KD, 0.0f, ANGLE_PID_LIMIT);
}

void Task_Run(void)
{
    float current_yaw = angleData[2]; 
    float steer = 0.0f;
    float left_speed = 0.0f, right_speed = 0.0f;
    
    uint8_t step_index = carTask.round_count * 2 + carTask.half_step;
    if (step_index > 5) step_index = 5; // 防止数组越界保护
    float target_yaw = carTask.target_angles[step_index];

    switch (carTask.state) 
    {
        case TASK_IDLE:
            if (imu_ready == 1 && carTask.start_trigger == 1) {
                carTask.state = TASK_TURN_TO_ANGLE;
                ResetPID(&angle_pid); 
            }
            break;

        case TASK_TURN_TO_ANGLE:
            PID_SetTarget(&angle_pid, target_yaw);
            
            if (current_yaw < target_yaw + ANGLE_DEADZONE && current_yaw > target_yaw - ANGLE_DEADZONE) {
                carTask.state = TASK_STRAIGHT_UNTIL_LINE; 
                ResetPID(&angle_pid); 
                
                reset_dis(); 
                setDis(1);
            } else {
                // 原地调角不加速，依然用 V_BASE_TURN
                steer = PID_compute(&angle_pid, current_yaw, TASK_TICK_MS / 1000.0f);
                left_speed  = V_BASE_TURN - steer;
                right_speed = V_BASE_TURN + steer;
                set_omiga(left_speed, right_speed);
            }
            break;

        case TASK_STRAIGHT_UNTIL_LINE:
            PID_SetTarget(&angle_pid, target_yaw);
            
            if (motordis >= carTask.dist_threshold && Track_IsLineDetected()) {
                carTask.state = TASK_LINE_TRACKING;
                beep_led_tip();
                Track_Init(); 
                
                // 【核心修改】：Track_Init 内部会把速度重置为宏定义 4.0f，我们必须在此处用任务设定的专属巡线速度把它覆盖掉！
                Track_SetSpeed(carTask.track_speed); 
                
            } else {
                steer = PID_compute(&angle_pid, current_yaw, TASK_TICK_MS / 1000.0f);
                // 【核心修改】：把 V_BASE_STRAIGHT 替换为结构体中动态配置的 straight_speed
                left_speed  = carTask.straight_speed - steer;
                right_speed = carTask.straight_speed + steer;
                set_omiga(left_speed, right_speed);
            }
            break;

        case TASK_LINE_TRACKING:
            {
                TRACK_LINE_ENUM track_status = Track_Run();
                
                if (track_status == NEXT && motordis >= carTask.dist_threshold) {
                    
                    setDis(0); 
                    carTask.half_step++; // 跑完半圈
                    
                    if (carTask.half_step >= 2) {
                        carTask.half_step = 0;   
                        carTask.round_count++;   
                    }
                    
                    if (carTask.round_count >= carTask.target_rounds) {
                        carTask.state = TASK_STOP;
                        beep_led_tip(); 
                    } else {
                        carTask.state = TASK_TURN_TO_ANGLE;
                        beep_led_tip(); 
                        ResetPID(&angle_pid);
                    }
                }
            }
            break;

        case TASK_STOP:
            set_omiga(0.0f, 0.0f); 
            break;

        default:
            break;
    }
}


// 声光提示函数
uint8_t ticks = 0;
void beep_led_tip(void){
    ticks = 50;
}

void run_beep_led(void){
    if (ticks) {
        DL_GPIO_setPins(TIP_PORT, TIP_beep_PIN);
        DL_GPIO_clearPins(TIP_PORT, TIP_led_PIN);
    }
    else {
        DL_GPIO_clearPins(TIP_PORT, TIP_beep_PIN);
        DL_GPIO_setPins(TIP_PORT, TIP_led_PIN);       
    }
    if (ticks > 0) {
        ticks--;
    }
}