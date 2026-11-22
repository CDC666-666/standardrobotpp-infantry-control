/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       usb.c/h
  * @brief      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-25-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef USB_H
#define USB_H

#include "stdbool.h"
#include "struct_typedef.h"

//API

/* 一次读取同一条已校验视觉命令，避免控制任务混用不同帧的字段。 */
typedef struct {
    bool valid, fire;
    float yaw, pitch, yaw_vel, pitch_vel, yaw_acc, pitch_acc;
    uint32_t update_time_ms;
} VisionControlInput;

/* 读取同帧视觉参考、开火许可和接收时间，并复核在线状态。 */
void VisionReadControl(VisionControlInput *out);

uint8_t GetUsbStatus(void);
extern inline float GetScCmdChassisSpeed(uint8_t axis);
extern inline float GetScCmdChassisVelocity(uint8_t axis);
extern inline float GetScCmdChassisAngle(uint8_t axis);
extern inline float GetScCmdChassisHeight(void);
extern inline float GetScCmdGimbalAngle(uint8_t axis);
extern inline float GetScCmdGimbalVelocity(uint8_t axis);
extern inline float GetScCmdGimbalAcceleration(uint8_t axis);
extern inline bool GetScCmdFire(void);
extern inline bool GetScCmdFricOn(void);
extern inline float GetVirtualRcCh(uint8_t channel);
extern inline char GetVirtualRcSw(uint8_t channel);

#endif  // USB_H
/*------------------------------ End of File ------------------------------*/
