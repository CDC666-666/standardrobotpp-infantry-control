/**
 * @brief uart板级支持包
 * @author Penguin
 */
#include "bsp_uart.h"
#include "robot_param.h"

/**
 * @brief 使用uart发送数据
 * @param huart 
 * @param pData 
 * @param Size 
 * @param Timeout 
 * @return 
 */
UartSendState_e UartSendTxMessage(
    UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout)
{
#if VOFA_ENABLE
    if (huart->Instance == USART1) return UART_SEND_FAIL;
#endif
    uint8_t cnt = 5;  //最大重发次数
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    while (cnt-- && status != HAL_OK) {
        status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    }

    if (status == HAL_OK) {
        return UART_SEND_OK;
    }
    return UART_SEND_FAIL;
}
