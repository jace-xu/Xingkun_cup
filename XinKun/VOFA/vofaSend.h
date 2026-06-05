#ifndef VOFASEND_H_
#define VOFASEND_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ti_msp_dl_config.h"
#include <string.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  通过UART发送一个浮点数的二进制数据 (VOFA+ JustFloat协议)
  * @param  data: 要发送的float类型数据
  * @param  uart: MSPM0 UART实例 (例如 UART0, UART1)
  * @retval None
  */
void SendFloat(float data, UART_Regs *uart);

/**
  * @brief  发送VOFA+ JustFloat协议帧尾 (+Infinity: 0x00 0x00 0x80 0x7F)
  * @param  uart: MSPM0 UART实例
  * @retval None
  */
void VOFASend(UART_Regs *uart);

#ifdef __cplusplus
}
#endif

#endif /* VOFASEND_H_ */
