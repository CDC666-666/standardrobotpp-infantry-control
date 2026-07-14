/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"


#include "CAN_receive.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include <semphr.h>
#include "detect_task.h"
#include "fifo.h"
#include "jlui.h"
#include "main.h"
#include "protocol.h"
#include "referee.h"
#include "remote_control.h"

/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

    while (1) {
        referee_unpack_fifo_data();
        osDelay(10);
    }
}

/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t * p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo)) {
        byte = fifo_s_get(&referee_fifo);
        switch (p_obj->unpack_step) {
            case STEP_HEADER_SOF: {
                if (byte == sof) {
                    p_obj->unpack_step = STEP_LENGTH_LOW;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else {
                    p_obj->index = 0;
                }
            } break;

            case STEP_LENGTH_LOW: {
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_LENGTH_HIGH;
            } break;

            case STEP_LENGTH_HIGH: {
                p_obj->data_len |= (byte << 8);
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
                    p_obj->unpack_step = STEP_FRAME_SEQ;
                } else {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            } break;
            case STEP_FRAME_SEQ: {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_HEADER_CRC8;
            } break;

            case STEP_HEADER_CRC8: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                        p_obj->unpack_step = STEP_DATA_CRC16;
                    } else {
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            } break;

            case STEP_DATA_CRC16: {
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;

                    if (verify_CRC16_check_sum(
                            p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                        referee_data_solve(p_obj->protocol_packet);
                    }
                }
            } break;

            default: {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            } break;
        }
    }
}

void USART6_IRQHandler(void)
{
    // 判断是否是空闲中断，用于接收数据
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) {
        // 清除空闲中断标志位，否则会不断进入中断
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);

        // 停止当前的DMA接收
        HAL_UART_DMAStop(&huart6);

        // 计算本次接收到的数据长度
        uint16_t this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);

        // 将接收到的数据放入FIFO缓冲区
        if (this_time_rx_len > 0) {
            fifo_s_puts(&referee_fifo, (char *)usart6_buf[0], this_time_rx_len);
        }

        // 调用离线检测钩子函数
        detect_hook(REFEREE_TOE);

        // 重新启动DMA接收，准备下一次接收
        HAL_UART_Receive_DMA(&huart6, usart6_buf[0], USART_RX_BUF_LENGHT);
    }
    // 如果不是空闲中断，则调用HAL库的通用中断处理函数
    // 这会处理例如发送完成(TC)等其他中断，并清除相应标志位，避免卡死
    else {
        HAL_UART_IRQHandler(&huart6);
    }
}

uint8_t UI_RES = 1;
uint8_t UI_time = 100;
Uiid Super_Line;               // 超级电容能量条UI对象
Uiid CapStatus;                // 超级电容开启状态UI对象
Uiid BufferLabel;              // 缓冲能量标签UI对象
Uiid BufferEnergy;             // 缓冲能量显示UI对象
SupCap_s cap;                  // 超级电容数据
fp32 buffer_energy;            // 缓冲能量数据
uint8_t last_cap_state = 255;  // 上一次的电容状态（初始值255表示未设置）
void jlui_ui_task(void const * argument)
{
    JLUI_SetSenderReceiverId(0x01, 0x101);
    Uiid HP_Dec;
    HP_Dec = JLUI_CreateString(4, UiBlack, 0, 750, 700, 40, "Please Spin");
    osDelay(3);
    for (;;) {
        if (UI_RES == 1) {
            JLUI_DeleteAll();
            osDelay(35);  // 等待删除指令发送
            JLUI_10HzTick();
            // 重新创建所有UI元素
            HP_Dec = JLUI_CreateString(4, UiBlack, 0, 750, 700, 40, "Please Spin");
            osDelay(3);
            Uiid Super_Rec = JLUI_CreateRect(5, UiCyan, 0, 800, 140, 1200, 80);
            Super_Line = JLUI_CreateLine(40, UiYellow, 0, 800, 110, 1200, 110);

            // 添加超电开启状态显示
            CapStatus = JLUI_CreateString(4, UiGreen, 0, 60, 700, 30, "CAP ON");
            osDelay(3);

            // 添加缓冲能量标签
            BufferLabel = JLUI_CreateString(4, UiWhite, 0, 60, 600, 20, "Buffer");
            osDelay(3);

            // 添加缓冲能量显示
            BufferEnergy = JLUI_CreateInt(4, UiYellow, 0, 60, 550, 30, 0);
            osDelay(3);

            // 添加车道线
            Uiid CarBody_Left = JLUI_CreateLine(3, UiCyan, 0, 580, 80, 680, 350);
            osDelay(3);
            Uiid CarBody_Right = JLUI_CreateLine(3, UiCyan, 0, 1080, 350, 1180, 80);
            osDelay(3);

            UI_RES++;
            UI_time = 100;
        }

        osDelay(UI_time);
        if (UI_time > 35) UI_time -= 2;
        JLUI_10HzTick();

        // 按B键刷新UI
        if (GetDt7Keyboard(KEY_B)) {
            UI_RES = 1;
        }
        JLUI_SetVisible(HP_Dec, 1);

        // 获取超级电容数据，动态更新能量条
        // GetSupCapMeasure(&cap);
        // JLUI_MoveP2To(Super_Line, 800 + cap.fdb.per_energy * 4, 110);

        // // 获取并显示缓冲能量
        // fp32 temp_power = 0.0f;
        // get_chassis_power_and_buffer(&temp_power, &buffer_energy);
        // JLUI_SetInt(BufferEnergy, (int)buffer_energy);

        // 更新超电开启状态显示 - 仅在状态改变时更新，避免频繁调用UART函数
        if (g_chassis_power_control.cap_enable_state != last_cap_state) {
            last_cap_state = g_chassis_power_control.cap_enable_state;

            if (g_chassis_power_control.cap_enable_state == 1) {
                JLUI_SetColor(CapStatus, UiGreen);  // 绿色表示开启
                JLUI_SetString(CapStatus, "CAP ON");
            } else {
                JLUI_SetColor(CapStatus, UiOrange);  // 橙色表示关闭
                JLUI_SetString(CapStatus, "CAP OFF");
            }
        }


        osDelay(5);
    }
}

/**
  * @brief          JLUI库接口函数实现 - 用户必须实现这些函数
  */

/**
  * @brief          JLUI互斥锁上锁
  * @param[in]      mutex: 互斥锁指针
  * @retval         1: 成功获取锁, 0: 获取失败或超时
  */
int JLUI_MutexLock(void * mutex)
{
    // 根据 jlui_port_freertos.c 的推荐：
    // 容易在各种电控的车上出现问题，直接禁用互斥锁
    return 1;
}

/**
  * @brief          JLUI互斥锁解锁
  * @param[in]      mutex: 互斥锁指针
  * @retval         none
  */
void JLUI_MutexUnlock(void * mutex)
{
    // 根据 jlui_port_freertos.c 的推荐：
    // 禁用互斥锁时无需解锁
}

/**
  * @brief          JLUI发送数据函数 - 向裁判系统发送UI数据
  * @param[in]      data: 数据缓冲区指针
  * @param[in]      len: 数据长度
  * @retval         none
  * @note           根据README：此函数应将数据发送到连接裁判系统的串口
  *                 JLUI假定此函数会阻塞执行直到传输完成
  *                 函数返回后，data指针指向的内存应视为无效
  */
void JLUI_SendData(const uint8_t * data, size_t len)
{
    // 通过UART6的DMA方式向裁判系统发送UI数据包
    // huart6是全局的UART句柄，已在extern中声明
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)data, len);
}

/**
  * @brief          CRC8校验和计算并添加（JLUI库需要此函数）
  * @param[in]      data: 数据缓冲区
  * @param[in]      length: 数据长度（包含校验码）
  * @retval         none
  */
void Append_CRC8_Check_Sum(unsigned char * data, unsigned int length)
{
    // 调用已有的CRC8函数
    append_CRC8_check_sum(data, length);
}

/**
  * @brief          CRC16校验和计算并添加（JLUI库需要此函数）
  * @param[in]      data: 数据缓冲区
  * @param[in]      length: 数据长度（包含校验码）
  * @retval         none
  */
void Append_CRC16_Check_Sum(uint8_t * data, uint32_t length)
{
    // 调用已有的CRC16函数
    append_CRC16_check_sum(data, length);
}
