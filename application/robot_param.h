/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H


#include "robot_typedef.h"
#include "struct_typedef.h"

// C 板丝印 UART2 对应 USART1；修改用途后重新编译烧录，不支持运行中热切换。
#define UART2_ROLE_VOFA 0
#define UART2_ROLE_VT13 1
#define UART2_ROLE_LEGACY 2
#ifndef UART2_ROLE
#define UART2_ROLE UART2_ROLE_VT13  // 当前用 VT13 控制整车。
#endif
#if UART2_ROLE != UART2_ROLE_VOFA && UART2_ROLE != UART2_ROLE_VT13 && UART2_ROLE != UART2_ROLE_LEGACY
#error "Unsupported UART2_ROLE"
#endif
// 单一用途宏派生开关，禁止独立配置造成串口多方占用。
#ifndef VOFA_ENABLE
#define VOFA_ENABLE (UART2_ROLE == UART2_ROLE_VOFA)
#elif VOFA_ENABLE != (UART2_ROLE == UART2_ROLE_VOFA)
#error "VOFA_ENABLE must match UART2_ROLE"
#endif
#define VT13_ENABLE (UART2_ROLE == UART2_ROLE_VT13)
#define VT13_BAUDRATE 921600UL
// 车辆控制源与底层 DT7 接收类型分开配置，禁止伪造双拨杆。
#define ROBOT_REMOTE_LEGACY 0
#define ROBOT_REMOTE_VT13 1
/* 未显式指定车辆控制源时，根据串口用途选择 VT13 或旧遥控。 */
#ifndef ROBOT_REMOTE_SOURCE
#if VT13_ENABLE
#define ROBOT_REMOTE_SOURCE ROBOT_REMOTE_VT13
#else
#define ROBOT_REMOTE_SOURCE ROBOT_REMOTE_LEGACY
#endif
#endif
/* 拒绝未支持的控制源，避免无意进入未定义操控路径。 */
#if ROBOT_REMOTE_SOURCE != ROBOT_REMOTE_LEGACY && ROBOT_REMOTE_SOURCE != ROBOT_REMOTE_VT13
#error "Unsupported ROBOT_REMOTE_SOURCE"
#endif
/* 选择 VT13 控车时必须同时启用其接收链路，不能只开控制逻辑。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13 && !VT13_ENABLE
#error "VT13 vehicle control requires UART2_ROLE_VT13"
#endif
// 本车实测左摇杆两轴与手册相反。设为 0 可查看帧内原始顺序。
#ifndef VT13_SWAP_LEFT_STICK_AXES
#define VT13_SWAP_LEFT_STICK_AXES 1
#endif
#ifndef VOFA_PERIOD_MS
#define VOFA_PERIOD_MS 10U
#endif
#define VOFA_BAUDRATE 115200UL

//导入具体的机器人参数配置文件
#include "robot_param_omni_infantry.h"
//#include "robot_param_mecannum.h"

// 选择机器人的各种类型
#define __RC_TYPE RC_DT7         // 遥控器类型
#define __DEVELOP 1                // 开发模式
#define __DEBUG 0                  // 调试模式
#define __TUNING 0                 // 调参模式
#define __MUSIC_ON 0               // 开启音乐
#define __TUNING_MODE TUNING_NONE  // 调参模式
#define __HEAT_IMU 1  // 加热IMU(防止Debug时因断点导致pid失效产生过热，烧坏IMU)
#define __IMU_CONTROL_TEMPERATURE 35 // (度)IMU目标控制温度

#define __BOARD_INSTALL_SPIN_MATRIX    \
{0.0f, -1.0f, 0.0f},                    \
{1.0f,  0.0f, 0.0f},                    \
{0.0f,  0.0f, 1.0f} \

// USB通信的部分选项
#define __USB_SEND_DEBUG 1  // 发送DEBUG数据

#ifndef __VIRTUAL_GIMBAL_FROM
#define __VIRTUAL_GIMBAL_FROM VG_FROM_NONE // 虚拟云台数据来源（用于云台底盘分离控制）
#endif

// 本板id
#ifndef __SELF_BOARD_ID
#define __SELF_BOARD_ID C_BOARD_DEFAULT
#endif

// 控制链路选择
#ifndef __CONTROL_LINK_RC
#define __CONTROL_LINK_RC CL_RC_DIRECT
#endif

#ifndef __CONTROL_LINK_KM
#define __CONTROL_LINK_KM CL_KM_RC
#endif

#ifndef __CONTROL_LINK_PS2
#define __CONTROL_LINK_PS2 CL_PS2_NONE
#endif

// 模块检查
#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE CHASSIS_NONE
#endif

#ifndef GIMBAL_TYPE
#define GIMBAL_TYPE GIMBAL_NONE
#endif

#ifndef SHOOT_TYPE
#define SHOOT_TYPE SHOOT_NONE
#endif

#ifndef MECHANICAL_ARM_TYPE
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE
#endif

#ifndef CUSTOM_CONTROLLER_TYPE
#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_NONE
#endif

#if UART2_ROLE != UART2_ROLE_LEGACY
#if CUSTOM_CONTROLLER_TYPE != CUSTOM_CONTROLLER_NONE
#error "Selected UART2 role conflicts with custom-controller PC output"
#endif
#if __CONTROL_LINK_RC == CL_RC_UART2 || __CONTROL_LINK_PS2 == CL_PS2_UART2 || __VIRTUAL_GIMBAL_FROM == VG_FROM_UART2
#error "Selected UART2 role cannot receive legacy board control data"
#endif
#endif
// __RC_TYPE 只选择旧 USART3 接收器，车辆输入源由 ROBOT_REMOTE_SOURCE 决定。
#if __RC_TYPE == VT13
#error "Select VT13 using ROBOT_REMOTE_SOURCE, not __RC_TYPE"
#endif

#if (VOFA_ENABLE != 0) && (VOFA_ENABLE != 1)
#error "VOFA_ENABLE must be 0 or 1"
#endif
#if VOFA_ENABLE
#if VOFA_PERIOD_MS < 1 || VOFA_PERIOD_MS > 1000
#error "VOFA_PERIOD_MS must be in [1, 1000]"
#endif
#endif

#endif /* ROBOT_PARAM_H */
