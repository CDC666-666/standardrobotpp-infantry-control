#include "vt13_remote.h"
#include "robot_param.h"
#include "usart.h"

#include <string.h>

volatile Vt13UartStats vt13_uart_stats;

#if VT13_ENABLE
#define VT13_DMA_CAPACITY 256U
#define VT13_RETRY_MS 100U

/* vofa.sct 将运行数据放在 DMA 可访问 SRAM，禁止放入 CCM。 */
static uint8_t vt13_dma_buffer[VT13_DMA_CAPACITY]
    __attribute__((section("VT13_DMA"), aligned(4)));
static Vt13Parser vt13_parser;
static uint16_t vt13_dma_read;
static volatile bool vt13_recover_pending;
static uint32_t vt13_last_attempt;

static bool Vt13DmaActive(void)
{
    return (huart1.Instance->CR3 & USART_CR3_DMAR) != 0U &&
           huart1.hdmarx != NULL &&
           (huart1.hdmarx->Instance->CR & DMA_SxCR_EN) != 0U;
}

static bool Vt13UartStart(void)
{
    uintptr_t address = (uintptr_t)vt13_dma_buffer;
    if (address < 0x20000000UL || address > 0x20020000UL - VT13_DMA_CAPACITY ||
        huart1.Init.BaudRate != VT13_BAUDRATE || huart1.Init.Mode != UART_MODE_RX ||
        huart1.hdmarx == NULL || huart1.hdmarx->Init.Mode != DMA_CIRCULAR) return false;
    vt13_dma_read = 0;
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    if (HAL_UART_Receive_DMA(&huart1, vt13_dma_buffer, VT13_DMA_CAPACITY) != HAL_OK)
        return false;
    /* 本 HAL 未传播 DMA 启动失败，检查硬件请求与通道都已打开。 */
    if (huart1.hdmarx->State != HAL_DMA_STATE_BUSY || !Vt13DmaActive()) return false;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    return true;
}

void Vt13UartInit(void)
{
    Vt13ParserInit(&vt13_parser);
    memset((void *)&vt13_uart_stats, 0, sizeof(vt13_uart_stats));
    vt13_last_attempt = HAL_GetTick();
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
    vt13_uart_stats.running = Vt13UartStart();
    vt13_recover_pending = !vt13_uart_stats.running;
    if (vt13_recover_pending) ++vt13_uart_stats.start_failed;
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USART1 和 DMA2_Stream5 均为抢占优先级 5，不互相抢占。
 * 半满/全满中断及时排空连续流；空闲中断处理不足半缓冲的遥控帧。
 * DMA 持续运行，空闲时不重启，避免帧头在重启窗口丢失。
 */
void Vt13UartRxEvent(void)
{
    if (!vt13_uart_stats.running || vt13_recover_pending) return;
    uint16_t write = (uint16_t)(VT13_DMA_CAPACITY - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (write > VT13_DMA_CAPACITY) return;
#if VT13_SWAP_LEFT_STICK_AXES
    uint32_t previous_sequence = vt13_parser.input.sequence;
#endif
    __DMB();
    if (write < vt13_dma_read) {
        Vt13ParserFeed(&vt13_parser, vt13_dma_buffer + vt13_dma_read,
                       (uint16_t)(VT13_DMA_CAPACITY - vt13_dma_read), HAL_GetTick());
        vt13_dma_read = 0;
    }
    Vt13ParserFeed(&vt13_parser, vt13_dma_buffer + vt13_dma_read,
                   (uint16_t)(write - vt13_dma_read), HAL_GetTick());
    vt13_dma_read = write;
#if VT13_SWAP_LEFT_STICK_AXES
    /* 仅交换本轮新发布的快照，空闲、坏帧和重复回调不重复交换。 */
    if (vt13_parser.input.sequence != previous_sequence) {
        uint16_t left_axis = vt13_parser.input.channel[2];
        float left_axis_normalized = vt13_parser.input.normalized[2];
        vt13_parser.input.channel[2] = vt13_parser.input.channel[3];
        vt13_parser.input.channel[3] = left_axis;
        vt13_parser.input.normalized[2] = vt13_parser.input.normalized[3];
        vt13_parser.input.normalized[3] = left_axis_normalized;
    }
#endif
}

void Vt13UartIrq(void)
{
    uint32_t status = huart1.Instance->SR;
    if (!(status & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) &&
        (status & USART_SR_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        Vt13UartRxEvent();
    }
    HAL_UART_IRQHandler(&huart1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart1) return;
    ++vt13_uart_stats.uart_errors;
    vt13_uart_stats.last_error = uart->ErrorCode;
    vt13_uart_stats.running = false;
    vt13_recover_pending = true;
    Vt13ParserInvalidate(&vt13_parser);
}

void Vt13UartService(void)
{
    uint32_t now = HAL_GetTick();
    if (vt13_uart_stats.running && !Vt13DmaActive()) {
        uint32_t key = __get_PRIMASK();
        __disable_irq();
        /* 若错误回调已经标记停机，沿用它的计数和恢复请求。 */
        if (vt13_uart_stats.running) {
            ++vt13_uart_stats.dma_stalls;
            vt13_uart_stats.running = false;
            vt13_recover_pending = true;
            Vt13ParserInvalidate(&vt13_parser);
        }
        __set_PRIMASK(key);
    }
    if (!vt13_recover_pending || now - vt13_last_attempt < VT13_RETRY_MS) return;
    vt13_last_attempt = now;
    /* 仅屏蔽本串口中断；HAL 超时所需的系统时钟保持运行。 */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    Vt13ParserInvalidate(&vt13_parser);
    bool stopped = HAL_UART_AbortReceive(&huart1) == HAL_OK;
    /* DMAR 已清零时，HAL_UART_AbortReceive 不会复位 DMA 通道和句柄。 */
    bool dma_ready = stopped && huart1.hdmarx != NULL && HAL_DMA_Init(huart1.hdmarx) == HAL_OK;
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
    bool started = dma_ready && Vt13UartStart();
    vt13_uart_stats.running = started;
    vt13_recover_pending = !started;
    if (started) ++vt13_uart_stats.recoveries;
    else ++vt13_uart_stats.start_failed;
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Vt13ReadInput(Vt13Input *out)
{
    if (out == NULL) return;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    Vt13ParserRead(&vt13_parser, out, HAL_GetTick());
    __set_PRIMASK(key);
}

void Vt13ReadStats(Vt13ParserStats *out)
{
    if (out == NULL) return;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    *out = vt13_parser.stats;
    __set_PRIMASK(key);
}
#else
/* 未选择 VT13 时提供无效快照，不占用串口或触及现有控制输入。 */
void Vt13UartInit(void) { }
void Vt13UartService(void) { }
void Vt13UartIrq(void) { }
void Vt13UartRxEvent(void) { }
void Vt13ReadInput(Vt13Input *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
}
void Vt13ReadStats(Vt13ParserStats *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
}
#endif
