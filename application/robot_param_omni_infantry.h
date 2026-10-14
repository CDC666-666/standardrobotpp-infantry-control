/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是全向轮步兵机器人参数配置文件，包括物理参数、PID参数等
  * @history
  *  版本       日期            作者            修改内容
  *  V1.1.0     2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *  V1.1.1     2025-1-15     CJH               1. 完成射击基本功能
  *  V1.1.2     2025-11-05    CJH               1. 完成底盘基本功能
  *                                             2. 完善参数   
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL         // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT         // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型

#define __GYRO_BIAS_YAW  0.001946855f           // 陀螺仪零飘，单位rad/s

/*-------------------- 底盘 --------------------*/
//物理参数 ---------------------
#define WHEEL_RADIUS (0.08f)            //(m)轮子半径
#define WHEEL_CENTER_DISTANCE (0.295f)  //(m)轮子到车的距离（0.22 + 0.21）          

//电机参数 ---------------------
//底盘电流发送参数
#define CHASSIS_CAN (1)
#define CHASSIS_STDID (0x200)

//电机ID ---------------------
#define WHEEL_1_ID (1)
#define WHEEL_2_ID (2)
#define WHEEL_3_ID (3)
#define WHEEL_4_ID (4)

//电机CAN ---------------------
#define WHEEL_1_CAN (1)
#define WHEEL_2_CAN (1)
#define WHEEL_3_CAN (1)
#define WHEEL_4_CAN (1)

//电机种类
#define WHEEL_1_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_2_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_3_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_4_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机方向
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (-1)
#define WHEEL_4_DIRECTION (-1)

//电机减速比
#define WHEEL_1_RATIO (19)
#define WHEEL_2_RATIO (19)
#define WHEEL_3_RATIO (19)
#define WHEEL_4_RATIO (19)

//电机模式
#define WHEEL_1_MODE (0)
#define WHEEL_2_MODE (0)
#define WHEEL_3_MODE (0)
#define WHEEL_4_MODE (0)

//PID参数 ---------------------
//驱动轮速度环PID参数
#define KP_OMNI_VEL (15.0f)
#define KI_OMNI_VEL (0.00f)
#define KD_OMNI_VEL (55.0f)
#define MAX_IOUT_OMNI_VEL (1000.0f)
#define MAX_OUT_OMNI_VEL (16000.0f)

// M3508 + C620：采用标称参数换算，并非实测轮端力标定。
// 0.3 N*m/A 对应原装 P19 减速箱输出轴；需先将其折算到转子侧。
#define CHASSIS_M3508_KT_ROTOR (0.3f / (3591.0f / 187.0f))
#define CHASSIS_WHEEL_KT (CHASSIS_M3508_KT_ROTOR * REDUCTION_RATIO)
#define CHASSIS_C620_CMD_PER_AMP (16384.0f / 20.0f)
// 单位为 A/(电机角速度 rad/s)：保留原速度误差系数所对应的 1 个 CAN 原始指令单位。
#define CHASSIS_SPEED_CURRENT_K_A (20.0f / 16384.0f)

// 单位换算：原增益和限幅值 * 0.0240510129490393。
// 当 n=268/17、r=0.08 m 时，该换算系数为 80 * 轮端标称 Kt / 819.2。
// 固定换算后的数值：后续修改 Kt 时，不得自动重新缩放 PID 参数。
// 保留原参数记录：Kp=100、Ki=0、Kd=0、积分限幅=1000、输出限幅=16000。
// 平面运动控制输出为期望力（N），Kp 的单位为 N/(m/s)。积分项和微分项采用 PID_calc 的离散计算约定。
// #define KP_OMNI_PLAIN_VEL (100.0f)
#define KP_OMNI_PLAIN_VEL (12.4051013f)
#define KI_OMNI_PLAIN_VEL (0.0f)
#define KD_OMNI_PLAIN_VEL (0.9f)
#define MAX_IOUT_OMNI_PLAIN_VEL (24.051013f)
#define MAX_OUT_OMNI_PLAIN_VEL (384.8162f)

// 独立偏航环：输出为期望底盘力矩（N*m），Kp 的单位为 (N*m)/(rad/s)。
// 这些初始值保留当前除以 (4*L) 的力分配方式，不沿用旧版乘以 L 的错误。
#define KP_OMNI_OMEGA (6.4051013f)
#define KI_OMNI_OMEGA (0.0f)
#define KD_OMNI_OMEGA (0.0f)
#define MAX_IOUT_OMNI_OMEGA (24.051013f)
#define MAX_OUT_OMNI_OMEGA (384.8162f)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (13.5f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.0f)     
#define KD_CHASSIS_FOLLOW_GIMBAL (5.05f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0.0f) 
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (15.0f) 
//跟随停止与角速度阻尼参数
#define CHASSIS_FOLLOW_WZ_DAMPING (0.95f)    //底盘角速度阻尼系数
// 小陀螺平移前瞻时间，单位秒；按实车偏移调节，设为 0 关闭补偿。
#define CHASSIS_SPIN_LEAD_TIME (0.10f)

//遥控器参数 ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (4.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式

/*-------------------- 云台 --------------------*/
//云台电流发送参数
#define GIMBAL_CAN (2)
#define GIMBAL_STDID (0x1FF)

//云台初始化-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)3000

//鼠标灵敏度 ---------------------
#define MOUSE_SENSITIVITY (40000.0f)
//遥控器灵敏度 ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//电机参数 ---------------------
//电机编号
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)3)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)3)

//电机CAN接口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DM_4310)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

//物理参数 ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.65f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.3f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (1.21874785f)  //云台初始化正对齐的时候使用的俯仰轴正中心量
#define GIMBAL_DIRECT_YAW_MID   (-0.734150f)    //云台初始化正对齐的时候使用的偏航轴正中心量

//PID参数 ---------------------
//偏航角度
#define KP_GIMBAL_YAW_ANGLE      (10.0f)
#define KI_GIMBAL_YAW_ANGLE      (0.0f)
#define KD_GIMBAL_YAW_ANGLE      (0.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_YAW_ANGLE (60.0f)
#define GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD (2.0f)
#define GIMBAL_DIRECT_YAW_GYRO_DAMPING (0.25f)
#define GIMBAL_DIRECT_YAW_PLAN_VEL_FF_GAIN (1.0f)
#define MAX_GIMBAL_DIRECT_YAW_PLAN_VEL_FF (1.5f)  // 速度前馈分量限幅 ±1.5 rad/s
#define GIMBAL_DIRECT_YAW_PLAN_ACC_FF_GAIN (0.004f) // 试验值：N*m/(rad/s^2)，等效惯量待辨识
#define MAX_GIMBAL_DIRECT_YAW_PLAN_ACC_FF (0.40f)  // 加速度前馈限幅，N*m
#define MAX_GIMBAL_DIRECT_YAW_PLAN_ACC (50.0f)     // 与上位机 MPC yaw 加速度约束同量级，rad/s^2
#define GIMBAL_DIRECT_YAW_PLAN_ACC_FILTER_ALPHA (0.25f) // 1 kHz 控制周期一阶滤波

#define GIMBAL_DIRECT_PITCH_PLAN_VEL_FF_GAIN (2.0f)
#define MAX_GIMBAL_DIRECT_PITCH_PLAN_VEL_FF (2.0f) //  pitch 速度前馈限幅，rad/s
#define GIMBAL_DIRECT_PITCH_PLAN_ACC_FF_GAIN (6.0f) // 试验初值：GM6020 CAN 电流指令单位/(rad/s^2)
#define MAX_GIMBAL_DIRECT_PITCH_PLAN_ACC_FF (6000.0f) // 加速度前馈电流指令限幅，raw
#define MAX_GIMBAL_DIRECT_PITCH_PLAN_ACC (100.0f) // 与上位机 MPC pitch 加速度约束同量级，rad/s^2
#define GIMBAL_DIRECT_PITCH_PLAN_ACC_FILTER_ALPHA (0.25f) // 1 kHz 控制周期一阶滤波
//角速度
#define KP_GIMBAL_YAW_VELOCITY   (1.25f)
#define KI_GIMBAL_YAW_VELOCITY   (0.0f)
#define KD_GIMBAL_YAW_VELOCITY   (0.00f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (1.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (7.0f)   // yaw 速度环输出力矩上限，N*m；首次力矩测试限幅

//俯仰角度
#define KP_GIMBAL_PITCH_ANGLE (70.0f)  //4.5
#define KI_GIMBAL_PITCH_ANGLE (0.0f)
#define KD_GIMBAL_PITCH_ANGLE (2.0f)  //3.0
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (1.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (15.0f)
//角速度
#define KP_GIMBAL_PITCH_VELOCITY (2800.0f)  //1200
#define KI_GIMBAL_PITCH_VELOCITY (0.0f)
#define KD_GIMBAL_PITCH_VELOCITY (30.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (30000.0f)

/*-------------------- 发射机构 --------------------*/
//物理参数 ---------------------
#define FRIC_RADIUS 0.03f              // (m)摩擦轮半径
#define BULLET_NUM 7.5                   // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1                      // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 1.0f   // 定义英雄电机到拨弹盘的齿轮减速比

/*电机参数 --------------------*/

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 6
#define FRIC_MOTOR_R_ID 1
#define FRIC_MOTOR_L_ID 2

//电机CAN接口
#define TRIGGER_MOTOR_CAN 1
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2

//电机标准帧标识符
#define STD_ID 0x200
//单环拨弹速度
#define TRIGGER_SPEED               (696.0f)
//摩擦轮速度
#define FRIC_R_SPEED                  (-613.0f)
#define FRIC_L_SPEED                  (613.0f)
#define FRIC_SPEED_LIMIT            (550.0f) 

/*编码器参数------------*/
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191

//电机转速（转/分）换算成旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
#define REDUCTION_RATIO             15.7647058824f   //齿轮减速比

/*堵转与反转参数------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED         5.0f
#define BLOCK_TIME                  1000
#define REVERSE_TIME                1250
#define REVERSE_SPEED               (-40.0f) 

/*MIT模式参数 ---------------------*/

// #define TRIGGER_SPEED_MIT_KD (0.0f)  //  （无）
#define TRIGGER_SPEED_MIT_KD (2.0f)

/*PID参数 ---------------------*/

//拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (200.0f)
#define TRIGGER_SPEED_PID_KI (0.0f)
#define TRIGGER_SPEED_PID_KD (1.2f)

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (600.0f)

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (120.0f)
#define TRIGGER_ANGEL_PID_KI (0.0f)
#define TRIGGER_ANGEL_PID_KD (0.95f)

// #define TRIGGER_ANGEL_PID_MAX_OUT (800.0f)
// #define TRIGGER_ANGEL_PID_MAX_OUT (10.0f)
#define TRIGGER_ANGEL_PID_MAX_OUT (40.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (300.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (70.0f)
#define FIRC_SPEED_PID_KI (0.00f)
#define FRIC_SPEED_PID_KD (4.3f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1100.0f)

#define SHOOT_HEAT_REMAIN_VALUE     70

#endif /* INCLUDED_ROBOT_PARAM_H */
