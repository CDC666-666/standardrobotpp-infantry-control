/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_omni.c/h
  * @brief      全向轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.03.03      Harry_Wong        1. 重新构建全向轮底盘，完成单底盘控制
  *  V1.1.0   2025.11.05       CJH              1. 完成底盘与云台协同控制
  *                                             2. 完成底盘键鼠控制
  *                                             3. 修正全向轮解算的错误
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#ifndef CHASSIS_OMNI_H
#define CHASSIS_OMNI_H
#include "motor.h"
#include "SupCap.h"//超级电容相关
#include "IMU_task.h"
#include "chassis.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include  "user_lib.h"
#include "CAN_cmd_dji.h"



/*-------------------- Structural definition --------------------*/
typedef enum {
    CHASSIS_LOCK,      //底盘锁定，所有轮子速度设定为0
    CHASSIS_SINGLE,    //只有底盘的模式
    CHASSIS_FOLLOW,    //云台跟随模式  
    CHASSIS_SPIN,    //底盘旋转模式
} ChassisMode_e;

/**
 * @brief  底盘跟随云台的偏航控制 PID
 */
 typedef struct
{

    pid_type_def follow; //云台跟随PID
    
} PID_t;   

/**
 * @brief  底盘坐标系下的目标速度
 */
typedef struct
{
    float vx;  // x 轴目标线速度，m/s
    float vy;  // y 轴目标线速度，m/s
    float wz;  // 绕 z 轴的目标角速度，rad/s
} Reference_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    const Imu_t * imu;     // imu数据
    ChassisMode_e mode;    // 底盘模式

    /*-------------------- Motors --------------------*/
    Motor_s wheel[4];  //底盘电机

    /*-------------------- Super Capacitor --------------------*/
    SupCap_s super_cap;  // 超级电容

    /*-------------------- Values --------------------*/
    Reference_t reference;  // 坐标变换后的底盘目标速度
    struct {
        float vx;  // 遥控器前后平移指令，坐标变换前，m/s
        float vy;  // 遥控器左右平移指令，坐标变换前，m/s
    } reference_rc;

    pid_type_def vel_x_pid;  // x 轴线速度外环 PID，输出目标牵引力
    pid_type_def vel_y_pid;  // y 轴线速度外环 PID，输出目标牵引力
    pid_type_def omega_pid;  // 绕 z 轴的角速度外环 PID，输出目标转矩

    fp32 feedback[4];     // 四个轮电机的反馈角速度，rad/s
    fp32 set[4];          // 运动学逆解得到的轮电机目标角速度，rad/s
    fp32 wheel_force[4];  // 各轮期望切向力，N；模型指令，不是测量值
    
    fp32 P_vx;  // 由轮速解算的底盘 x 轴线速度，m/s
    fp32 P_vy;  // 由轮速解算的底盘 y 轴线速度，m/s
    fp32 P_wz;  // 由轮速解算的底盘绕 z 轴角速度，rad/s

    fp32 force_x;    // x 轴期望牵引力，N；不是实际力测量值
    fp32 force_y;    // y 轴期望牵引力，N；不是实际力测量值
    fp32 torque_wz;  // 期望转矩，N*m；符号沿用 reference.wz 的现有约定


    fp32 yaw_delta;                  // 云台相对底盘的偏航差，rad
    ChassisMode_e spin_mode_last;    // 上一控制周期的底盘模式，用于检测模式切换
    fp32 wz_set_smoothed;            // 小陀螺模式平滑后的目标角速度，rad/s
    fp32 spin_acceleration_limit;    // 每次更新允许的角速度正向增量，rad/s
    
    fp32 sin_yaw;         // 偏航差的正弦值，用于平移指令坐标变换
    fp32 cos_yaw;         // 偏航差的余弦值，用于平移指令坐标变换
    fp32 spin_target_wz;  // 根据平移指令及机器人等级计算的小陀螺目标角速度，rad/s
    fp32 wz_error;        // 小陀螺目标角速度与平滑设定值之差，rad/s


} Chassis_s;

extern Chassis_s chassis;

extern void ChassisInit(void);

extern void ChassisHandleException(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif 
#endif


