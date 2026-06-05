# XinKun — 智能小车竞速系统

基于 TI MSPM0G3507 的自主循迹竞速小车，支持多任务模式与 PID 闭环控制。

## 硬件平台

- **主控**: TI MSPM0G3507 (ARM Cortex-M0+, LQFP-64)
- **开发板**: 立创·天猛星
- **电机**: 轮趣 MG513X (编码器 13 线, 减速比 28:1)
- **开发环境**: Code Composer Studio (CCS) + SysConfig
- **烧录工具**: J-Link

## 硬件资源

| 外设 | 功能 |
|------|------|
| TIMA1 (PWM) | 双路电机 PWM 驱动 (PB4, PB1) |
| TIMG0 (Timer) | 20ms 定时器中断，主控制循环 |
| UART1 | VOFA+ 调试输出 (115200, PB6/PB7) |
| UART3 | IMU 陀螺仪通讯 (115200, PB2/PB3) |
| GPIO | 8 路灰度传感器、双编码器、任务/圈数选择、蜂鸣器/LED |

## 软件架构

```
XinKun/
├── main.c                  # 主程序入口，初始化与主循环（调试输出）
├── motorControl/           # 电机控制模块（PID + 里程计）
├── PID/                    # 通用 PID 控制器
├── task/                   # 任务状态机（多任务调度）
├── trackLine/              # 8 路灰度巡线
├── IT/                     # 所有中断服务函数（定时器/编码器/按键/串口）
├── filter/                 # 一阶低通滤波器
├── VOFA/                   # VOFA+ JustFloat 协议调试输出
├── Debug/                  # TI 驱动库自动生成的配置
└── XinKun.syscfg           # SysConfig 引脚/外设配置文件
```

## 程序执行流程

整个程序的逻辑控制全部在 **20ms 定时器中断** 中执行，不在 main() 的 while(1) 中跑控制逻辑。所有中断服务函数统一放在 `IT/` 目录下。

### 中断概览

| 中断 | 文件 | 职责 |
|------|------|------|
| TIMG0 (20ms) | `IT/it.c` | 主控制循环：调用 `Task_Run()` → `loopControl()` → `run_beep_led()` |
| GPIO GROUP1 | `IT/it.c` | 编码器 A 相脉冲计数（2 倍频）+ 按键启停任务 |
| IMU UART | `IT/it.c` | IMU 陀螺仪串口数据接收与解析（欧拉角） |
| VOFA UART | `IT/it.c` | VOFA+ 调试串口接收（预留） |

### main() 职责

main() 仅负责：初始化外设 → 等待陀螺仪开机并校准 → 读取任务/圈数拨码开关 → 启动定时器中断 → 主循环仅发送 VOFA 调试数据。

## 任务模式

小车支持 **3 种任务类型** × **多圈数** 的组合，通过硬件拨码开关选择：

| 任务 | 描述 |
|------|------|
| Task 1 | 普通直线任务 — 直行后巡线 |
| Task 2 | 对角线任务 — 带偏航角度（±30°）的对角行走 |
| Task 3 | 极速直线任务 — 高速/低速两档可调的快速一圈 |

圈数通过 2 位二进制拨码开关选择（1~3 圈，00 默认 1 圈）。

## 状态机流程

```
IDLE → TURN_TO_ANGLE → STRAIGHT_UNTIL_LINE → LINE_TRACKING → STOP
```

1. **IDLE**: 等待按键触发启动
2. **TURN_TO_ANGLE**: 原地旋转至目标偏航角
3. **STRAIGHT_UNTIL_LINE**: 直线行驶，直到 8 路灰度检测到黑线
4. **LINE_TRACKING**: 巡线行驶，完成指定圈数
5. **STOP**: 电机刹车，蜂鸣器提示

## 核心模块说明

### 电机控制 (motorControl)

电机控制模块定义了两组结构体，分别对应 **设定值** 和 **测量值**：

- `set_motor0` / `set_motor1` — PID 目标设定值（期望角速度）
- `measure_motor0` / `measure_motor1` — 编码器实际测量值（实际角速度）

编码器使用 **2 倍频**（仅 A 相脉冲触发中断，B 相判断方向），参数：
- 编码器线数: 13 PPR
- 减速比: 28:1
- 轮径: 65mm

**使用方法**：在定时器中断中调用一次 `loopControl()`，即可自动完成速度计算 → PID 运算 → PWM 输出。需要改变目标速度时，调用 `set_omiga(左轮角速度, 右轮角速度)` 即可，单位为 rad/s。

```c
// 定时器中断中
void TIMER_0_INST_IRQHandler(void) {
    loopControl();  // 每 20ms 执行一次速度闭环
}

// 任意位置设定目标速度
set_omiga(6.28f, 6.28f);  // 两轮均以 1 rev/s 转动
```

### PID 控制器

- 标准位置式 PID，带积分限幅和输出限幅
- `PID_SetTarget()` 在线修改目标值
- `setPID_parameter()` 运行时在线调参
- 默认参数：Kp = 60, Ki = 100, Kd = 0

### 巡线模块 (trackLine)

- 8 路灰度传感器，二分法计算中线偏差
- 状态检测：`TRACK`（正常巡线）/ `LOST`（丢线）/ `NEXT`（到达终点）
- PID 纠偏控制

### 调试输出 (VOFA)

- 基于 VOFA+ JustFloat 协议，4 字节 float 帧 + 帧尾
- 主循环中实时输出：状态机状态、任务类型、圈数、偏航角等

## 编译与烧录

1. 使用 Code Composer Studio 导入项目
2. 通过 SysConfig 确认外设配置与硬件一致
3. 编译后通过 **J-Link** 烧录至 MSPM0G3507

## 最近更新

- 新增任务 3（高速/低速极速直线）
- 新增 8 路灰度传感器支持
- 圈数选择改为 2 位二进制编码（支持 1~3 圈）
- 修复任务 2 角度参数
