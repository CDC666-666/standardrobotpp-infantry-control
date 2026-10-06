#include "vofa.h"
#include "vofa_port.h"
#include "usart.h"

#if VOFA_ENABLE
uint32_t VofaPortNow(void) { return HAL_GetTick(); }
uint32_t VofaPortLock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return key;
}
void VofaPortUnlock(uint32_t key) { __DMB(); __set_PRIMASK(key); }

int VofaPortInit(const void *buffer, uint16_t capacity)
{
    uintptr_t address = (uintptr_t)buffer;
    // STM32F407 SRAM1+SRAM2. Fail closed if the linker put this in CCM.
    if (address < 0x20000000UL || address > 0x20020000UL - capacity || (address & 3U)) return 0;
    CLEAR_BIT(huart1.Instance->CR1, USART_CR1_IDLEIE);
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    return huart1.Init.Mode == UART_MODE_TX && huart1.Init.BaudRate == VOFA_BAUDRATE &&
           huart1.gState == HAL_UART_STATE_READY && huart1.hdmatx &&
           huart1.hdmatx->State == HAL_DMA_STATE_READY;
}

int VofaPortReady(void)
{
    return huart1.gState == HAL_UART_STATE_READY && huart1.hdmatx &&
           huart1.hdmatx->State == HAL_DMA_STATE_READY &&
           !(huart1.hdmatx->Instance->CR & DMA_SxCR_EN);
}

VofaPortResult VofaPortStart(uint8_t *buffer, uint16_t length)
{
    HAL_StatusTypeDef result;
    // This HAL version does not propagate HAL_DMA_Start_IT failures; check both
    // handles before entering it. No other USART1 sender is allowed in this mode.
    if (!VofaPortReady()) return VOFA_PORT_BUSY;
    __DMB();
    result = HAL_UART_Transmit_DMA(&huart1, buffer, length);
    if (result == HAL_BUSY) return VOFA_PORT_BUSY;
    if (result != HAL_OK || huart1.hdmatx->ErrorCode != HAL_DMA_ERROR_NONE) return VOFA_PORT_ERROR;
    return VOFA_PORT_OK;
}

int VofaPortRecover(void)
{
    HAL_StatusTypeDef result;
    DMA_HandleTypeDef *dma = huart1.hdmatx;
    // Mask only this UART's IRQs; HAL timeout clock and control IRQs stay live.
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
    result = HAL_UART_AbortTransmit(&huart1);
    if (dma->State == HAL_DMA_STATE_BUSY) {
        if (HAL_DMA_Abort(dma) != HAL_OK) result = HAL_ERROR;
    }
    // Stop all UART requests even if abort failed. Never reuse live DMA memory.
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE);
    CLEAR_BIT(huart1.Instance->CR1, USART_CR1_TCIE | USART_CR1_TXEIE | USART_CR1_IDLEIE);
    __HAL_UART_DISABLE(&huart1);
    if (result != HAL_OK || (dma->Instance->CR & DMA_SxCR_EN)) return 0;

    // Flush any partial UART word, then reinitialize this peripheral only.
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();
    if (HAL_DMA_Init(dma) != HAL_OK || HAL_UART_Init(&huart1) != HAL_OK) return 0;
    __HAL_DMA_CLEAR_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma) |
        __HAL_DMA_GET_HT_FLAG_INDEX(dma) | __HAL_DMA_GET_TE_FLAG_INDEX(dma) |
        __HAL_DMA_GET_DME_FLAG_INDEX(dma) | __HAL_DMA_GET_FE_FLAG_INDEX(dma));
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) VofaOnTxComplete();
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) VofaOnError(uart->ErrorCode);
}
#endif
