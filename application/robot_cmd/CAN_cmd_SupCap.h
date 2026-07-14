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
#ifndef CAN_CMD_SUPCAP_H
#define CAN_CMD_SUPCAP_H

#include "bsp_can.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

extern void CanCmdSupCap(uint8_t can, uint16_t power_buffer, uint16_t power_limit, uint8_t automode_stage);

#endif
/************************ END OF FILE ************************/
