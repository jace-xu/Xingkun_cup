#include "ti_msp_dl_config.h"
#include "motorControl.h"
#include "vofaSend.h"
#include "it.h"
#include "task.h"
#include "trackLine.h"

int main(void)
{
    SYSCFG_DL_init();
    // -------------------------关于串口-----------------------
    // IMU中断
    NVIC_ClearPendingIRQ(IMU_UART_INST_INT_IRQN);// 清除标志位
    NVIC_EnableIRQ(IMU_UART_INST_INT_IRQN); // 使能串口中断

    // vofa调式中断
    NVIC_ClearPendingIRQ(VOFA_UART_INST_INT_IRQN);// 清除标志位
    NVIC_EnableIRQ(VOFA_UART_INST_INT_IRQN); // 使能串口中断   
    // --------------------------------------------------------


    // 等待陀螺仪开机
    for(volatile int i = 0;i<15000000;i++){
        
    }
    IMU_SendCalibrationCmd(); // 陀螺仪校准
    // 等待校准
    for(volatile int i = 0;i<12000000;i++){
        
    }
    imu_ready = 1;
    
    Motor_Init();
    Task_Init();
    Track_Init();

    // // 读取引脚配置任务
    // uint8_t task = 0;
    // uint8_t round = 0;

    // if ((DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round1_PIN) != 0)) {
    //     round = 1;
    // }
    // if ((DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round2_PIN) != 0)) {
    //     round = 2;
    // }
    // if ((DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round3_PIN) != 0)) {
    //     round = 3;
    // }

    // if ((DL_GPIO_readPins(TASK_SELECT_PORT, TASK_SELECT_task1_PIN) != 0)) {
    //     task = 1;
    //     Task_SetConfig(round,task,0,0,0,0,0,0);
    // }
    // if ((DL_GPIO_readPins(TASK_SELECT_PORT, TASK_SELECT_task2_PIN) != 0)) {
    //     task = 2;
    //     Task_SetConfig(round,task,-30,25,-30,25,-30,25);
    // }

    // 1. 读取圈数（使用 else if 严防覆盖！）
    // if (DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round3_PIN) != 0) {
    //     round = 3;
    // }
    // else if (DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round2_PIN) != 0) {
    //     round = 2;
    // }
    // else if (DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round1_PIN) != 0) {
    //     round = 1;
    // }

    // 默认保底值
    uint8_t task = 1; 
    uint8_t round = 1; 

    // 1. 二进制编码读取圈数
    uint8_t round_val = 0;
    // 读取低位 (Bit 0)
    if (DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round0_PIN) == 0) {
        round_val |= 0x01; 
    }
    // 读取高位 (Bit 1)s
    if (DL_GPIO_readPins(ROUND_SELLECT_PORT, ROUND_SELLECT_round1_PIN) == 0) {
        round_val |= 0x02; 
    }

    // 根据读取的二进制值设定圈数（如果是 00，保底给 1 圈）
    round = (round_val == 0) ? 1 : round_val;


    // 2. 读取任务类型并下发配置 (加入任务3的支持)
    if (DL_GPIO_readPins(TASK_SELECT_PORT, TASK_SELECT_task3_PIN) != 0) {
        task = 3;
        // 任务3（极速直线任务），角度配置一般和任务1类似，都是走直线
        Task_SetConfig(round, task, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    else if (DL_GPIO_readPins(TASK_SELECT_PORT, TASK_SELECT_task2_PIN) != 0) {
        task = 2;
        // 任务2（对角线任务），角度有明显的偏航
        Task_SetConfig(round, task, -30.0f, 20.0f, -30.0f, 20.0f, -35.0f, 15.0f);
    }
    else {
        // 保底逻辑：如果 task2 和 task3 都没被拉高，默认执行任务 1（普通直线）
        task = 1;
        Task_SetConfig(round, task, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    
    //----------------------- 关于定时器 ---------------------
    // 开启定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    // 开启定时器
    DL_TimerA_startCounter(TIMER_0_INST);
    // #define TIMER_0_INST                     (TIMG0)        // 定时器0本身
    // #define TIMER_0_INST_IRQHandler          TIMG0_IRQHandler // 中断服务函数名
    // #define TIMER_0_INST_INT_IRQN            (TIMG0_INT_IRQn) // 中断号
    // #define TIMER_0_INST_LOAD_VALUE          (4999U)         // 计数初值
    // -------------------------------------------------------------------
    
    while (1) {
        // 电机
        // set_omiga(2*PI, 2*PI);
        // SendFloat(measure_motor0.omiga,VOFA_UART_INST);
        // SendFloat(measure_motor1.omiga,VOFA_UART_INST);
        // SendFloat(set_motor0.omiga,VOFA_UART_INST);
        // SendFloat(set_motor1.omiga,VOFA_UART_INST);

        // 状态机
        SendFloat(carTask.state,VOFA_UART_INST);
        SendFloat(task,VOFA_UART_INST);
        SendFloat(round,VOFA_UART_INST);
        SendFloat(round_val,VOFA_UART_INST);


        // 角度
        SendFloat(angleData[2],VOFA_UART_INST);


        // 输出pwm占空比
        // SendFloat(set_motor0.output,VOFA_UART_INST);
        // SendFloat(set_motor1.output,VOFA_UART_INST);
        VOFASend(VOFA_UART_INST);
    }
}
// 外部中断的使用：
/*    
    uint32_t DL_GPIO_getEnabledInterruptStatus(GPIO_PORT,GPIO_PIN);  // 获取某个引脚的中断状态，返回一个32为数字，对应引脚为1，GPIO_PIN可以通过按位或的方式
                                                                     // GPIO_PIN可以通过按位或的方式例如：GPIO_PIN1|GPIO_PIN2来同时读取两个脚触发中断的状态
    例如：uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIO_PORT,GPIO_PIN1|GPIO_PIN2);
          if((gpioA & GPIO_PIN1) == GPIO_PIN1){
            // 中断回调1...
          }
          else if((gpioA & GPIO_PIN2) == GPIO_PIN2){
            // 中断回调2...
          }

    // 使能外部中断
    NVIC_EnableIRQ(Motor1_Encoder_INT_IRQN);   参数：中断向量号
    DL_GPIO_clearInterruptStatus(GPIO_PORT,GPIO_PIN)  //  清除中断标志位
    void GROUP1_IRQHandler(void){}      // 外部中断服务函数
*/

// 串口通讯例程
/*
    NVIC_ClearPendingIRQ(IMU_UART_INST_INT_IRQN) // 清除标志位
    NVIC_EnableIRQ(IMU_UART_INST_INT_IRQN); // 使能串口中断

    例程：
    void UART_0_INST_IRQHandler(void) // 串口中断服务函数，.h文件里面找名字
    {
        switch (DL_UART_Main_getPendingInterrupt(VOFA_UART_INST)) // 获取uart_0中的所有中断，.h文件里面找名字
        {
            case DL_UART_MAIN_IIDX_RX:  // 接收中断

                break;
            default:
            
                break;
        }
    }
*/
