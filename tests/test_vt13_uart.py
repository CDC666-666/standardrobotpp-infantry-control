"""执行实际原 UART 驱动与解析器，验证接收事件、帧校验和错误恢复。"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_vt13_remote import frame

ROOT = Path(__file__).resolve().parents[1]

HARDWARE = r'''
#ifndef UART_TEST_HW_H
#define UART_TEST_HW_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define VT13_ENABLE 1
#define VT13_BAUDRATE 921600UL
#define VT13_SWAP_LEFT_STICK_AXES 1
#define USART_CR3_DMAR 1U
#define DMA_SxCR_EN 1U
#define DMA_CIRCULAR 1U
#define UART_MODE_RX 1U
#define HAL_DMA_STATE_BUSY 1U
#define HAL_OK 0
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_IDLE 16U
#define UART_IT_IDLE USART_SR_IDLE
#define USART1_IRQn 1
#define DMA2_Stream5_IRQn 2
typedef struct {uint32_t CR,NDTR;} TestDma;
typedef struct {TestDma *Instance;struct {unsigned Mode;} Init;unsigned State;} DMA_HandleTypeDef;
typedef struct {uint32_t CR1,CR3,SR;} TestUart;
typedef struct {TestUart *Instance;DMA_HandleTypeDef *hdmarx;struct {unsigned BaudRate,Mode;} Init;unsigned ErrorCode;} UART_HandleTypeDef;
static TestUart uart_regs;
static TestDma dma_regs;
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef huart1;
static uint32_t now_ms,mask;
static unsigned clear_reads;
static uint32_t HAL_GetTick(void){return now_ms;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t v){mask=v;}
#define __DMB() ((void)0)
#define __HAL_DMA_GET_COUNTER(d) ((d)->Instance->NDTR)
/* 记录读 DR 清标志操作，软件补中断没有 IDLE 时不能执行该操作。 */
#define __HAL_UART_CLEAR_IDLEFLAG(u) do {assert((u)->Instance->SR&USART_SR_IDLE);++clear_reads;(u)->Instance->SR&=~USART_SR_IDLE;} while(0)
#define __HAL_UART_ENABLE_IT(u,b) ((u)->Instance->CR1|=(b))
#define __HAL_UART_DISABLE_IT(u,b) ((u)->Instance->CR1&=~(b))
static void HAL_NVIC_DisableIRQ(int irq){(void)irq;}
static void HAL_NVIC_EnableIRQ(int irq){(void)irq;}
static void HAL_NVIC_ClearPendingIRQ(int irq){(void)irq;}
static int HAL_UART_Receive_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){
    (void)p;u->Instance->CR3|=USART_CR3_DMAR;u->hdmarx->Instance->CR=DMA_SxCR_EN;
    u->hdmarx->Instance->NDTR=n;u->hdmarx->State=HAL_DMA_STATE_BUSY;return HAL_OK;
}
static int HAL_UART_AbortReceive(UART_HandleTypeDef *u){u->Instance->CR3&=~USART_CR3_DMAR;u->hdmarx->Instance->CR=0;return HAL_OK;}
static int HAL_DMA_Init(DMA_HandleTypeDef *d){d->State=0;d->Instance->CR=0;return HAL_OK;}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u);
/* 只模拟 HAL 的错误停接收与回调，其余判断仍执行实际驱动。 */
static void HAL_UART_IRQHandler(UART_HandleTypeDef *u){
    uint32_t error=u->Instance->SR&(USART_SR_PE|USART_SR_FE|USART_SR_NE|USART_SR_ORE);
    if(error){u->ErrorCode=error;u->Instance->SR&=~error;HAL_UART_AbortReceive(u);HAL_UART_ErrorCallback(u);}
}
#endif
'''

CASES = r'''
static unsigned producer;
/* 数据缓冲和生产位置沿用实际驱动，仅替换外设寄存器。 */
static void reset_uart(void){
    memset(&uart_regs,0,sizeof(uart_regs));memset(&dma_regs,0,sizeof(dma_regs));
    dma.Instance=&dma_regs;dma.Init.Mode=DMA_CIRCULAR;dma.State=0;
    huart1.Instance=&uart_regs;huart1.hdmarx=&dma;huart1.Init.BaudRate=VT13_BAUDRATE;
    huart1.Init.Mode=UART_MODE_RX;huart1.ErrorCode=0;mask=0;clear_reads=0;
    producer=0;now_ms=100;uart_regs.SR=USART_SR_IDLE;Vt13UartInit();clear_reads=0;
    assert(vt13_uart_stats.running);
}
static void produce(const uint8_t *p,unsigned n){
    for(unsigned i=0;i<n;++i){vt13_dma_buffer[producer]=p[i];producer=(producer+1)%VT13_DMA_CAPACITY;}
    dma_regs.NDTR=VT13_DMA_CAPACITY-producer;
}
int main(void){
    Vt13Input input;Vt13ParserStats stats;
    reset_uart();produce(wire,21);uart_regs.SR=USART_SR_IDLE;Vt13UartIrq();
    assert(vt13_parser.input.sequence==1&&clear_reads==1);
    /* 没有硬件接收事件时，任务不得消费DMA数据或更新旧输入时间。 */
    now_ms+=14;produce(wire,21);mask=1;Vt13UartService();assert(mask==1);
    mask=0;now_ms+=100;Vt13UartService();Vt13ReadInput(&input);
    assert(!input.valid&&vt13_parser.input.sequence==1&&vt13_parser.stats.bytes==21);
    uart_regs.SR=USART_SR_IDLE;Vt13UartIrq();Vt13ReadInput(&input);assert(input.valid);
    /* 原IDLE和DMA回调处理回卷/拆帧，重复事件不能重复发布。 */
    reset_uart();
    for(unsigned i=1;i<=40;++i){
        now_ms+=14;produce(wire,7);Vt13UartRxEvent();assert(vt13_parser.input.sequence==i-1);
        produce(wire+7,14);Vt13UartRxEvent();Vt13UartRxEvent();assert(vt13_parser.input.sequence==i);
    }
    Vt13ReadStats(&stats);assert(stats.bytes==840&&stats.crc_errors==0);
    reset_uart();produce(wire_optional,21);uart_regs.SR=USART_SR_IDLE;Vt13UartIrq();
    Vt13ReadInput(&input);Vt13ReadStats(&stats);
    assert(input.valid&&input.keyboard==0&&input.mouse_x==0&&input.mouse_left==0&&input.mouse_right==0);
    assert(stats.keymouse_invalid_frames==1&&stats.invalid_frames==0);
    now_ms+=100;Vt13ReadInput(&input);assert(!input.valid);
    /* 原中断路径仍拒绝坏CRC，错误恢复后必须重新收到新有效帧。 */
    reset_uart();uint8_t bad[21];memcpy(bad,wire,21);bad[20]^=1;
    produce(bad,21);uart_regs.SR=USART_SR_IDLE;Vt13UartIrq();Vt13ReadStats(&stats);
    assert(stats.crc_errors==1&&stats.valid_frames==0);
    produce(wire,21);uart_regs.SR=USART_SR_ORE;Vt13UartIrq();
    assert(vt13_uart_stats.uart_errors==1&&!vt13_uart_stats.running);
    Vt13ReadInput(&input);assert(!input.valid);
    now_ms+=100;uart_regs.SR=USART_SR_IDLE;Vt13UartService();
    assert(vt13_uart_stats.recoveries==1&&vt13_uart_stats.running);
    Vt13ReadInput(&input);assert(!input.valid);
    producer=0;produce(wire,21);uart_regs.SR=USART_SR_IDLE;Vt13UartIrq();
    Vt13ReadInput(&input);assert(input.valid);
    dma_regs.CR=0;now_ms+=1;Vt13UartService();
    assert(vt13_uart_stats.dma_stalls==1&&!vt13_uart_stats.running);
    Vt13ReadInput(&input);assert(!input.valid);
    puts("actual original UART: events, wrap, split, CRC, optional data and recovery passed");
    return 0;
}
'''



class Vt13UartTest(unittest.TestCase):
    def test_actual_original_driver(self):
        compiler = shutil.which("gcc")
        self.assertIsNotNone(compiler)
        source = (ROOT / "application/remote_control/vt13/vt13_uart.c").read_text(encoding="utf-8")
        # 只替换主机不具备的 SRAM 硬件地址输入，其余完整驱动和真实解析器保持原样。
        address = "uintptr_t address = (uintptr_t)vt13_dma_buffer;"
        self.assertEqual(source.count(address), 1)
        source = source.replace(address, "uintptr_t address = 0x20000000UL;")
        with tempfile.TemporaryDirectory(prefix="vt13-uart-") as directory:
            work = Path(directory)
            for name in ("robot_param.h", "usart.h"):
                (work / name).write_text('#include "test_hw.h"\n', encoding="utf-8")
            (work / "test_hw.h").write_text(HARDWARE, encoding="utf-8")
            (work / "uart_under_test.c").write_text(source, encoding="utf-8")
            wire = ",".join(str(x) for x in frame(mode=1))
            optional = ",".join(str(x) for x in frame(mode=1, buttons=(3, 3, 3), keyboard=0xffff))
            (work / "main.c").write_text('#include "uart_under_test.c"\n'
                                        + f"static const uint8_t wire[21]={{{wire}}};\n"
                                        + f"static const uint8_t wire_optional[21]={{{optional}}};\n" + CASES,
                                        encoding="utf-8")
            executable = work / "uart.exe"
            command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror=implicit-function-declaration",
                       "-I", str(work), "-I",
                       str(ROOT / "application/remote_control/vt13"), str(work / "main.c"),
                       str(ROOT / "application/remote_control/vt13/vt13_remote.c"), "-o", str(executable)]
            result = subprocess.run(command, capture_output=True, text=True, errors="replace")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True,
                                    errors="replace", timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main(verbosity=2)
