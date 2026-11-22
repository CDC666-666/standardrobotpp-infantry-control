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
#include <string.h>

/* 其他云台实现需显式提供工作模式许可，默认禁止发射。 */
__weak bool GimbalAllowsShoot(void)
{
    return false;
}

/* 未提供状态发布器的云台实现一律禁止 VT13 车辆工作。 */
/**
 * @brief 为未实现状态发布的云台提供默认无效快照。
 * @note 弱实现清零所有许可，具体云台必须显式覆盖才能放行 VT13 底盘和发射。
 */
__weak void GimbalReadControlStatus(GimbalControlStatus *out)
{
    /* 存在输出空间时才清零默认快照，所有工作许可保持关闭。 */
    if (out != NULL) memset(out, 0, sizeof(*out));
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
