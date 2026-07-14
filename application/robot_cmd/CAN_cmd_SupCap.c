/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_cmd_SupCap.c/h
  * @brief      CAN发送函数，通过CAN信号控制超级电容
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-29-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_cmd_SupCap.h"

CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 5,
};

/**
 * @brief          通过CAN发送超电板控制指令
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      power 超电控制功率
 * @return         none
 */
void CanCmdSupCap(uint8_t can, uint16_t power_buffer, uint16_t power_limit, uint8_t automode_stage)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;
                                                                       
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = 426;
     
    // 第0，1字节：power_buffer 缓冲能量发送值 (小端序)
    CAN_CTRL_DATA.tx_data[0] = (power_buffer & 0xFF);
    CAN_CTRL_DATA.tx_data[1] = (power_buffer >> 8) & 0xFF;
         
    // 第2，3字节：power_limit 功率限制值 (小端序)
    CAN_CTRL_DATA.tx_data[2] = (power_limit & 0xFF);
    CAN_CTRL_DATA.tx_data[3] = (power_limit >> 8) & 0xFF;

    // 第4字节：automode_stage 是否开启电容
    CAN_CTRL_DATA.tx_data[4] = automode_stage;
    
    CAN_SendTxMessage(&CAN_CTRL_DATA);

}

/************************ END OF FILE ************************/
