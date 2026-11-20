/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "gimbal.h"

#include "attribute_typedef.h"

/* 其他云台实现需显式提供工作模式许可，默认禁止发射。 */
__weak bool GimbalAllowsShoot(void)
{
    return false;
}

#if GIMBAL_TYPE != GIMBAL_NONE

#include <stdlib.h>

// GimbalApi_t gimbal = {
//     .SetCali = SetCali,
//     .CmdCali = CmdCali,
//     .GetStatus = GetStatus,
//     .GetDuration = GetDuration,
//     .GetYawMid = GetYawMid,
// };
// GimbalApi_t gimbal = {
//     .SetCali = NULL,
//     .CmdCali = NULL,
//     .GetStatus = NULL,
//     .GetDuration = NULL,
//     .GetYawMid = NULL,
// };
#endif  // GIMBAL_TYPE
/*------------------------------ End of File ------------------------------*/
