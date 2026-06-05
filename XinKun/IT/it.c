#include "it.h"
#include <stdint.h>
#include "task.h"


// 20ms定时器中断
void TIMER_0_INST_IRQHandler(void){
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:{
            // 进行轮询控制
            Task_Run();
            // Track_Run();
            loopControl();
            run_beep_led();
            break;
        }
        default:
            // 其他中断不处理
        break;
    }
}

// 软件中断编码器
void GROUP1_IRQHandler(void){
    
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(Motor0_Encoder_PORT,Motor0_Encoder_A1_PIN);
 
    // 电机0
    if ((gpioA & Motor0_Encoder_A1_PIN) == Motor0_Encoder_A1_PIN)
    {
        uint8_t A1 = (DL_GPIO_readPins(Motor0_Encoder_PORT, Motor0_Encoder_A1_PIN) != 0);
        uint8_t B1 = (DL_GPIO_readPins(Motor0_Encoder_PORT, Motor0_Encoder_B1_PIN) != 0);
        // 异或判断方向
        if ((A1 ^ B1) == 1){
            // 正转
            encoder0++;
        }
        else{
            // 反转
            encoder0--;
        }
        // 清理标志位
        DL_GPIO_clearInterruptStatus(Motor0_Encoder_PORT, Motor0_Encoder_A1_PIN);
    }
    
    // 电机1
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(Motor1_Encoder_A2_PORT,Motor1_Encoder_A2_PIN);
    if ((gpioB & Motor1_Encoder_A2_PIN) == Motor1_Encoder_A2_PIN)
    {
        uint8_t A2 = (DL_GPIO_readPins(Motor1_Encoder_A2_PORT, Motor1_Encoder_A2_PIN) != 0);
        uint8_t B2 = (DL_GPIO_readPins(Motor1_Encoder_B2_PORT, Motor1_Encoder_B2_PIN) != 0);
        // 异或判断方向
        if ((A2 ^ B2) == 1){
            // 正转
            encoder1--;
        }
        else{
            // 反转
            encoder1++;
        }
        // 清理标志位
        DL_GPIO_clearInterruptStatus(Motor1_Encoder_A2_PORT, Motor1_Encoder_A2_PIN);
    }

    // 按键控制任务开始关闭
    uint32_t gpioButton = DL_GPIO_getEnabledInterruptStatus(BOTTON_PORT,BOTTON_P1_PIN|BOTTON_P2_PIN);
    // 按键1开任务
    if((gpioButton & BOTTON_P1_PIN) == BOTTON_P1_PIN){
        Task_Control(1);
        DL_GPIO_clearInterruptStatus(BOTTON_PORT, BOTTON_P1_PIN);
    }
    else if ((gpioButton & BOTTON_P2_PIN) == BOTTON_P2_PIN) {
        Task_Control(0);
        DL_GPIO_clearInterruptStatus(BOTTON_PORT, BOTTON_P2_PIN);
    }
}

// vofa调试串口中断
void VOFA_UART_INST_IRQHandler(void) // 串口中断服务函数，.h文件里面找名字
{
    switch (DL_UART_Main_getPendingInterrupt(VOFA_UART_INST)) // 获取uart_0中的所有中断，.h文件里面找名字
    {
        case DL_UART_MAIN_IIDX_RX:

            break;
        default:
            break;
    }
}



//-------------------------------------全局变量定义-----------------------------------
float aData[3] = {0};            // 虽不再使用，但保留防其他文件报错
float omigaData[3] = {0};        // 虽不再使用，但保留防其他文件报错
float angleData[3] = {0};        // angleData[2] 存放减去偏移量后的 Yaw 偏航角
unsigned char rx_buffer[32] = {0}; 

// 校准与偏移专用变量
uint8_t imu_ready = 0;           // 主程序等待5s后将其置1，表示准备好记录零点
uint8_t offset_recorded = 0;     // 是否已经抓取了初始偏航角 (0-未抓取，1-已抓取)
float yaw_offset = 0.0f;         // 存下来的偏航角初始偏移量

#define RAD_TO_DEG 57.29577951f  // 弧度转角度系数

//-------------------------------------新增：发送校准指令-------------------------------
// 上电时调用此函数发送校准指令
void IMU_SendCalibrationCmd(void) 
{
    uint8_t cmd[7] = {0x7E, 0x23, 0x07, 0x70, 0x01, 0x5F, 0x78};
    for(int i = 0; i < 7; i++) {
        DL_UART_Main_transmitData(IMU_UART_INST, cmd[i]);
        // 阻塞等待当前字节发送完毕，防止数据覆盖
        while(DL_UART_Main_isBusy(IMU_UART_INST)); 
    }
}

// 串口数据处理函数（解析欧拉角并减去偏移量）
void calculateData(unsigned char *buffer){
    
    // 如果功能字是返回欧拉角 (0x26)
    if(buffer[3] == 0x26) {
        union {
            uint32_t i;
            float f;
        } temp;

        // 1. 解析 ROLL -> angleData[0] (如果不需要可注释)
        temp.i = ((uint32_t)buffer[7] << 24) | ((uint32_t)buffer[6] << 16) | 
                 ((uint32_t)buffer[5] << 8)  | ((uint32_t)buffer[4]);
        angleData[0] = temp.f * RAD_TO_DEG;

        // 2. 解析 PITCH -> angleData[1] (如果不需要可注释)
        temp.i = ((uint32_t)buffer[11] << 24) | ((uint32_t)buffer[10] << 16) | 
                 ((uint32_t)buffer[9] << 8)  | ((uint32_t)buffer[8]);
        angleData[1] = temp.f * RAD_TO_DEG;

        // 3. 解析 YAW 硬件原始角度
        temp.i = ((uint32_t)buffer[15] << 24) | ((uint32_t)buffer[14] << 16) | 
                 ((uint32_t)buffer[13] << 8) | ((uint32_t)buffer[12]);
        float raw_yaw = temp.f * RAD_TO_DEG;

        // 4. 偏移量扣除逻辑
        if (imu_ready == 1) {
            // 第一次进来，记录当前的偏航角作为基准零度
            if (offset_recorded == 0) {
                yaw_offset = raw_yaw;
                offset_recorded = 1; 
            }
            
            // 减去初始偏移量，强制将当前朝向设为 0 度
            float final_yaw = raw_yaw - yaw_offset;
            
            // 按照需求将角度约束，大于90减180，小于-90加180
            if (final_yaw > 90.0f)  final_yaw -= 180.0f;
            if (final_yaw < -90.0f) final_yaw += 180.0f;
            
            // 存入最终使用的数组
            angleData[2] = final_yaw; 
        }
    }
}

// 串口中断处理函数 (保持不变)
void IMU_UART_INST_IRQHandler(void) 
{
    static uint8_t rx_index = 0;
    static uint8_t frame_len = 0; 

    switch (DL_UART_Main_getPendingInterrupt(IMU_UART_INST)) 
    {
        case DL_UART_MAIN_IIDX_RX:{
            uint8_t rx_data = DL_UART_Main_receiveData(IMU_UART_INST);
            
            if (rx_index == 0) {
                if (rx_data == 0x7E) rx_buffer[rx_index++] = rx_data;
            } else if (rx_index == 1) {
                if (rx_data == 0x23) {
                    rx_buffer[rx_index++] = rx_data;
                } else {
                    rx_index = (rx_data == 0x7E) ? 1 : 0; 
                    if(rx_index == 1) rx_buffer[0] = 0x7E;
                }
            } else if (rx_index == 2) {
                frame_len = rx_data;
                if (frame_len > 32 || frame_len < 4) {
                    rx_index = 0; 
                } else {
                    rx_buffer[rx_index++] = rx_data;
                }
            } else {
                rx_buffer[rx_index++] = rx_data;
                if (rx_index >= frame_len) {
                    uint8_t sum = 0;
                    for (int i = 0; i < frame_len - 1; i++) sum += rx_buffer[i];
                    
                    if (sum == rx_buffer[frame_len - 1]) {  
                        calculateData(rx_buffer);
                    }
                    rx_index = 0; 
                }
            }
            break;
        }
        default: break;
    }
}