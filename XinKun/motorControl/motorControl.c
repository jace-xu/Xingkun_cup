#include "motorControl.h"

// 电机0
#define  Kp_omiga_0    			 			60.0f
#define  Ki_omiga_0    			 			100.0f
#define  Kd_omiga_0          			    0.0f
#define  omiga_integral_limit_0    			6.28f
#define  omiga_output_limit_0      			1000.0f
// 电机1
#define  Kp_omiga_1    			 			60.0f
#define  Ki_omiga_1    			 			100.0f
#define  Kd_omiga_1    			 			0.0f
#define  omiga_integral_limit_1    			6.28f
#define  omiga_output_limit_1      			1000.0f

// 电机宏定义常数
// 编码器每圈脉冲数
#define ENCODER_PULSE_PER_TURN 13
// 二倍频
#define ENCODER_FREQ_TIMES    2
// 电机减速比
#define REDUCTION_RATIO        28.0f
// 速度计算周期（毫秒）
#define SPEED_CALC_INTERVAL 0.02f

Motor set_motor0 = {0,0,0};
Motor set_motor1 = {0,0,0};
Motor measure_motor0 = {0,0,0};
Motor measure_motor1 = {0,0,0};

int32_t encoder0 = 0;
int32_t encoder1 = 0;
static int32_t encoder0_last = 0;
static int32_t encoder1_last = 0;

PID pid0_omiga = {0};
PID pid1_omiga = {0};

LowPassfilter l_filter0 = {0,1};
LowPassfilter l_filter1 = {0,1};

float motordis = 0;
uint8_t disFlag = 0;

// 里程计相关
// rst里程计
void reset_dis(void){
    motordis = 0;
}

// 开启/关闭里程计
void setDis(uint8_t a){
    if(a == 0)
        disFlag = 0;
    else if (a == 1)
        disFlag = 1;
    else
        return;
}



// 设置速度
void set_omiga(float omiga0,float omiga1){
    set_motor0.omiga = omiga0;
    set_motor1.omiga = omiga1;
}

// 测试pwm
void testPWM(float pwm0, float pwm1){
    // 处理电机0 - 已交换AIN1和AIN2电平
    if(pwm0 > 0.0f){
        // 正转
        DL_GPIO_clearPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN1_PIN);
        DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, pwm0, GPIO_Motor_PWM_C0_IDX);
    }
    else if(pwm0 < 0.0f){
        // 反转
        DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN1_PIN);
        DL_GPIO_clearPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, -pwm0, GPIO_Motor_PWM_C0_IDX);
    }
    else{
        // 停止（刹车模式）
        DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN1_PIN);
        DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, 0, GPIO_Motor_PWM_C0_IDX);
    }

    // 处理电机1 - 已交换BIN1和BIN2电平
    if(pwm1 > 0.0f){
        // 正转
        DL_GPIO_clearPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN1_PIN);
        DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, pwm1, GPIO_Motor_PWM_C1_IDX);
    }
    else if(pwm1 < 0.0f){
        // 反转
        DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN1_PIN);
        DL_GPIO_clearPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, -pwm1, GPIO_Motor_PWM_C1_IDX);
    }
    else{
        // 停止（刹车模式）
        DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN1_PIN);
        DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, 0, GPIO_Motor_PWM_C1_IDX);
    }
}

// 设置方向
static void setDir(void){
    if(set_motor0.output > 0){
        DL_GPIO_clearPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN1_PIN);
        DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN2_PIN);        
    }
    else if (set_motor0.output < 0) {
        DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN1_PIN);
        DL_GPIO_clearPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN2_PIN);       
    }

    if(set_motor1.output > 0){
        DL_GPIO_clearPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN1_PIN);
        DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN2_PIN);  
    }
    else if (set_motor1.output < 0) {
        DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN1_PIN);
        DL_GPIO_clearPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN2_PIN);    
    }
}

// 电机停止
void motor_stop(void){
    DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN1_PIN);
    DL_GPIO_setPins(Motor0_Dir_PORT, Motor0_Dir_Motor0_Dir_AIN2_PIN); 
    DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN1_PIN );
    DL_GPIO_setPins(Motor1_Dir_PORT, Motor1_Dir_Motor1_Dir_BIN2_PIN );
    set_motor0.state = BRAKE;
    set_motor1.state = BRAKE;
    measure_motor0.state = BRAKE;
    measure_motor1.state = BRAKE;      
}

// 计算速度
static void compute_omiga(){
    // 电机0 速度计算 + 一阶低通滤波
    int32_t delta0 = encoder0 - encoder0_last;
    float raw_omiga0 = (delta0 * 2.0f * PI) / (ENCODER_PULSE_PER_TURN * REDUCTION_RATIO * SPEED_CALC_INTERVAL * ENCODER_FREQ_TIMES);
    measure_motor0.omiga = LowPassfilter_First(&l_filter0, raw_omiga0);
    encoder0_last = encoder0;

    // 电机1 速度计算 + 一阶低通滤波
    int32_t delta1 = encoder1 - encoder1_last;
    float raw_omiga1 = (delta1 * 2.0f * PI) / (ENCODER_PULSE_PER_TURN * REDUCTION_RATIO * SPEED_CALC_INTERVAL * ENCODER_FREQ_TIMES);
    measure_motor1.omiga = LowPassfilter_First(&l_filter1, raw_omiga1);
    encoder1_last = encoder1;

    if(disFlag == 1){
        float average_delta = 0.5f * (delta0 + delta1); // 取左右轮脉冲增量的平均值
        float step_distance = PI * 0.065f * average_delta / (ENCODER_PULSE_PER_TURN * ENCODER_FREQ_TIMES * REDUCTION_RATIO);
        
        motordis += step_distance; 
    }
}

// 速度环控制
void loopControl(void){

    // 计算速度
    compute_omiga();
    
    // PID控制
    if(set_motor0.state == PID_CONTROL){
        // 电机0
        PID_SetTarget(&pid0_omiga, set_motor0.omiga);
        set_motor0.output = PID_compute(&pid0_omiga,measure_motor0.omiga,SPEED_CALC_INTERVAL);

        // 电机1
        PID_SetTarget(&pid1_omiga, set_motor1.omiga);
        set_motor1.output = PID_compute(&pid1_omiga,measure_motor1.omiga,SPEED_CALC_INTERVAL);

        // 设置方向
        setDir();

        // 设置pwm
        if (set_motor0.output > 0)
            DL_TimerG_setCaptureCompareValue(Motor_PWM_INST,set_motor0.output , GPIO_Motor_PWM_C0_IDX);
        else
            DL_TimerG_setCaptureCompareValue(Motor_PWM_INST,-set_motor0.output , GPIO_Motor_PWM_C0_IDX);
        if(set_motor1.output >= 0)
            DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, set_motor1.output, GPIO_Motor_PWM_C1_IDX);
        else
            DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, -set_motor1.output, GPIO_Motor_PWM_C1_IDX);

    }
    else {
        motor_stop();
    }
 
    // testPWM(100,100);
}

// 初始化
void Motor_Init(void){
    // 开启pwm
    DL_TimerG_startCounter(Motor_PWM_INST);
    
    // 设置占空比为0
    DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, 0, GPIO_Motor_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_INST, 0, GPIO_Motor_PWM_C1_IDX);

    // 开启编码器中断
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(Motor1_Encoder_INT_IRQN);
    // 设置pid参数
    PID_Init(&pid0_omiga,Kp_omiga_0,Ki_omiga_0,Kd_omiga_0,omiga_integral_limit_0,omiga_output_limit_0);
    PID_Init(&pid1_omiga,Kp_omiga_1,Ki_omiga_1,Kd_omiga_1,omiga_integral_limit_1,omiga_output_limit_1);
}


// 电机释放
void motor_release(){
    set_motor0.state = PID_CONTROL;
    set_motor1.state = PID_CONTROL;
    measure_motor0.state = PID_CONTROL;
    measure_motor1.state = PID_CONTROL;     
}


