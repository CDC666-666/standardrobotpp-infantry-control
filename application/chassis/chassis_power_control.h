
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "SupCap.h"
#include "chassis.h"
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "referee.h"
#include "remote_control.h"
#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#include "chassis_mecanum.h"
#elif (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "chassis_omni.h"
#elif (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "chassis_balance.h"
#elif (CHASSIS_TYPE == CHASSIS_STEERING_WHEEL)
#include "chassis_steering.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------- 功率控制参数定义 --------------------*/
#define POWER_CONTROL_TORQUE_COEFF 2.43243860e-6f  // 扭矩系数(20/16384)*(0.3)*(17/268)/9.55  Kt
#define POWER_CONTROL_K1_COEFF 1.23e-07f           // k1系数
#define POWER_CONTROL_K2_COEFF 1.453e-07f         // k2系数
#define POWER_CONTROL_CONSTANT 4.081f              // 常数项
#define POWER_CONTROL_CAP_LOW_THRESHOLD 5.0f       // 电容低电量阈值
#define POWER_CONTROL_CAP_BOOST_POWER 50.0f       // 电容放电时额外功率

/*-------------------- 功率控制数据结构 --------------------*/
typedef struct
{
    // PID控制器
    pid_type_def buffer_pid;  // 功率缓冲区PID控制器

    // 电机相关
    Motor_s * motor_chassis[4];    // 底盘电机指针数组
    float * motor_current_cmd[4];  // 最终待发送电流指令指针数组
    float chassis_power;                // 当前底盘功率
    float chassis_power_buffer;         // 当前功率缓冲能量
    uint16_t max_power_limit;           // 最大功率限制

    // 超级电容相关
    SupCap_s * super_cap;      // 超级电容指针
    uint8_t cap_enable_state;  // 电容使能状态 (0=关闭, 1=开启)

    // 遥控器相关
    const RC_ctrl_t * rc_ctrl;  // 遥控器数据指针

} chassis_power_control_t;

/*-------------------- 外部变量声明 --------------------*/
extern chassis_power_control_t g_chassis_power_control;
extern volatile float g_initial_total_power_debug;

/*-------------------- 函数声明 --------------------*/

/**
 * @brief          初始化底盘功率控制
 * @param[in]      chassis: 底盘结构体指针
 * @retval         none
 * @note           在ChassisInit()中调用
 */
void chassis_power_control_init(Chassis_s * chassis);

/**
 * @brief          底盘功率控制主函数
 * @param[in]      none  
 * @retval         none
 * @note           在chassis_task中ChassisConsole()之后调用
 *                 对最终电流指令进行功率限制
 */
void chassis_power_control(void);

// /**
//  * @brief          获取底盘功率和缓冲能量
//  * @param[out]     power: 底盘功率指针
//  * @param[out]     buffer: 缓冲能量指针
//  * @retval         none
//  * @note           实现在 referee.c 中，这里显式声明给底盘功率控制模块使用
//  */
// void get_chassis_power_and_buffer(fp32 * power, fp32 * buffer);

/**
 * @brief          获取底盘功率和缓冲能量
 * @param[out]     chassis_power: 底盘功率指针
 * @param[out]     chassis_power_buffer: 缓冲能量指针  
 * @retval         none
 * @note           从裁判系统获取实时功率数据
 */
void get_chassis_power_and_buffer_adapter(fp32 * chassis_power, fp32 * chassis_power_buffer);

/**
 * @brief          获取底盘最大功率限制
 * @param[out]     max_power_limit: 最大功率限制指针
 * @retval         none
 * @note           从裁判系统获取功率上限
 */
void get_chassis_max_power_adapter(uint16_t * max_power_limit);

/**
 * @brief          超级电容控制命令
 * @param[in]      input_power: 输入功率
 * @retval         none  
 * @note           向超级电容发送功率控制命令
 */
void CAN_CMD_CAP(float input_power);

#ifdef __cplusplus
}
#endif

#endif
