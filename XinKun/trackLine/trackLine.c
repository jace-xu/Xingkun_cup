#include "trackLine.h"
#include "motorControl.h"

// 循迹参数配置
#define TRACK_SPEED                3.14f    // 基准速度
#define PID_KP                     1.0f     // 循迹PID P参数
#define PID_KI                     0.0f     // I
#define PID_KD                     0.0f     // D
#define TACK_PID_LIMIT             6.28f    // PID输出限幅
#define SPEED_CALC_INTERVAL_MS     0.02f    // 控制周期 20ms
#define LOST_LINE_THRESHOLD        10       // 丢线判断阈值（连续10次）

// 全局变量
PID track_pid;
float track_speed = TRACK_SPEED;

// 8路灰度传感器加权系数 (对应物理上的 S1, S2, S3, S4, S5, S6, S7, S8)
const int8_t weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

// 丢线判断状态变量
static uint8_t last_valid_sensor = 0x18;   // 8路中间两路是 bit3(S4) 和 bit4(S5)，掩码 0001 1000 (0x18)
static uint8_t lost_line_count = 0;        // 连续丢线计数
static float last_valid_left_speed = 0;    // 最后一次有效左速度
static float last_valid_right_speed = 0;   // 最后一次有效右速度

static uint8_t Track_ReadGPIO(void);
static float Track_GetError(void);

// 设置循迹基准速度
void Track_SetSpeed(float speed)
{
    track_speed = speed;
}

// 循迹初始化
void Track_Init(void)
{
    PID_Init(&track_pid,
             PID_KP,
             PID_KI,
             PID_KD,
             0.0f,
             TACK_PID_LIMIT);

    PID_SetTarget(&track_pid, 0.0f);
    Track_SetSpeed(TRACK_SPEED);

    // 初始化丢线状态 (默认中间两路压线，即 S4 和 S5)
    last_valid_sensor = 0x18; 
    lost_line_count = 0;
    last_valid_left_speed = TRACK_SPEED;
    last_valid_right_speed = TRACK_SPEED;
}

// 循迹主函数
TRACK_LINE_ENUM Track_Run(void)
{
    // 1. 获取黑线位置偏差，同时在内部更新丢线计数
    float error = Track_GetError();

    // 2. 根据丢线情况做出逻辑判断
    if (lost_line_count > 0)
    {
        // 发生丢线：判断上次有效记录是不是【只有边缘通道(S1 或 S8)】检测到了黑线
        // 映射关系：最左侧 S1 对应 bit0 (0x01), 最右侧 S8 对应 bit7 (0x80)
        if (last_valid_sensor == 0x01 || last_valid_sensor == 0x80)
        {
            // 在弯道转弯过急等导致的跑飞异常丢线
            return LOST;
        }
        else
        {
            // 上次记录不是边缘通道，可能是路口或断头路
            if (lost_line_count >= LOST_LINE_THRESHOLD)
            {
                // 连续多次没检测到，判定为到达任务点/十字路口盲区结束
                return NEXT;
            }
            else
            {
                // 还没达到连续丢线阈值，保持上一次的速度强行冲过盲区
                set_omiga(last_valid_left_speed, last_valid_right_speed);
                return TRACK; 
            }
        }
    }
    else
    {
        // 正常循迹
        float steer = PID_compute(&track_pid, error, SPEED_CALC_INTERVAL_MS);
        float left_speed  = track_speed - steer;
        float right_speed = track_speed + steer;

        // 保存当前有效速度
        last_valid_left_speed = left_speed;
        last_valid_right_speed = right_speed;

        // 输出到电机
        set_omiga(left_speed, right_speed);
        
        return TRACK;
    }
}

// 读取 8路 灰度传感器电平 (S1 到 S8)
static uint8_t Track_ReadGPIO(void)
{
    uint8_t data = 0;

    // 重新映射：将物理上的 S1~S8 依次放入 data 的 bit0~bit7
    data |= (DL_GPIO_readPins(Gray_Sensor_S1_PORT, Gray_Sensor_S1_PIN) == 0) << 0; // S1 -> bit0
    data |= (DL_GPIO_readPins(Gray_Sensor_S2_PORT, Gray_Sensor_S2_PIN) == 0) << 1; // S2 -> bit1
    data |= (DL_GPIO_readPins(Gray_Sensor_S3_PORT, Gray_Sensor_S3_PIN) == 0) << 2; // S3 -> bit2
    data |= (DL_GPIO_readPins(Gray_Sensor_S4_PORT, Gray_Sensor_S4_PIN) == 0) << 3; // S4 -> bit3
    data |= (DL_GPIO_readPins(Gray_Sensor_S5_PORT, Gray_Sensor_S5_PIN) == 0) << 4; // S5 -> bit4
    data |= (DL_GPIO_readPins(Gray_Sensor_S6_PORT, Gray_Sensor_S6_PIN) == 0) << 5; // S6 -> bit5
    data |= (DL_GPIO_readPins(Gray_Sensor_S7_PORT, Gray_Sensor_S7_PIN) == 0) << 6; // S7 -> bit6
    data |= (DL_GPIO_readPins(Gray_Sensor_S8_PORT, Gray_Sensor_S8_PIN) == 0) << 7; // S8 -> bit7

    return data;
}

// 计算黑线位置偏差（带丢线判断）
static float Track_GetError(void)
{
    uint8_t sensor = Track_ReadGPIO();
    int32_t sum = 0;
    uint8_t count = 0;

    // 遍历这 8 个传感器
    for (int i = 0; i < 8; i++)
    {
        if (sensor & (1 << i))
        {
            sum += weight[i];
            count++;
        }
    }

    // 检测到黑线
    if (count > 0)
    {
        last_valid_sensor = sensor; // 记录包含所有被触发通道的状态
        lost_line_count = 0;        // 清零丢线计数
        return (float)sum / count;
    }
    // 未检测到黑线
    else
    {
        lost_line_count++;
        return 0; // 丢线时不依据此返回值运算控制
    }
}

// 给外部任务状态机使用的接口：只要读出来的不是0，说明至少有一个传感器压线了
uint8_t Track_IsLineDetected(void)
{
    return (Track_ReadGPIO() != 0);
}