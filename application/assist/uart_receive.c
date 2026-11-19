/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       uart_receive.c/h
  * @brief      uart 信号接收部分，只处理4pin口接收数据（3pin口数据由裁判系统部分进行处理）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-17-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "uart_receive.h"
#include "usart.h"
#include "robot_param.h"
#include "vt13_remote.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
#if VT13_ENABLE
        Vt13UartRxEvent();
#endif
    }
}
#if VT13_ENABLE
/* 连续数据达到半缓冲时及时处理，避免只依赖 UART 空闲中断。 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) Vt13UartRxEvent();
}
#endif
