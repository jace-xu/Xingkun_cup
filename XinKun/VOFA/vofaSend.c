#include "vofaSend.h"

/**
  * @brief  发送单个float数据
  */
void SendFloat(float data, UART_Regs *uart)
{
    if (uart == NULL)
        return;

    uint8_t buffer[4];
    // MSPM0和STM32一样是小端模式，直接memcpy即可
    memcpy(buffer, &data, sizeof(float));

    // MSPM0 DriverLib阻塞式发送
    for (int i = 0; i < 4; i++)
    {
        DL_UART_transmitDataBlocking(uart, buffer[i]);
    }
}

/**
  * @brief  发送VOFA+帧尾
  */
void VOFASend(UART_Regs *uart)
{
    if (uart == NULL)
        return;

    // VOFA+ JustFloat协议帧尾：+Infinity
    const uint8_t END_buf[4] = {0x00, 0x00, 0x80, 0x7F};

    for (int i = 0; i < 4; i++)
    {
        DL_UART_transmitDataBlocking(uart, END_buf[i]);
    }
}
