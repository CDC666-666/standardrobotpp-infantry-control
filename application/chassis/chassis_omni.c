/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_omni.c/h
  * @brief      全向轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.03.03       Harry_Wong        1.重新构建全向轮底盘，完成单底盘控制
  *  V1.1.0   2025.11.05       CJH               1. 完成底盘与云台协同控制
  *                                              2. 完成底盘键鼠控制
  *                                              3. 修正全向轮解算的错误
  * 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/


#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "chassis_omni.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h" 
#include "detect_task.h"
#include "gimbal.h"
#include "math.h"
#include "usb_debug.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "IMU.h"

Chassis_s chassis;
PID_t chassis_pid;

inline int GetChassisSpinState(void)
{
    return chassis.spin_flag;
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    //step1 获取所有所需变量指针
    chassis.rc = get_remote_control_point();

    //step2 PID数据清零，设置PID参数
    const static fp32 wheel_vel[3]={KP_OMNI_VEL,KI_OMNI_VEL,KD_OMNI_VEL};
    for (int i=0;i<4;++i)
    {
        PID_init(&chassis_pid.wheel_velocity[i],PID_POSITION,wheel_vel,MAX_OUT_OMNI_VEL,MAX_IOUT_OMNI_VEL);
    }
    
    const static fp32 gimbal_follow[3]={KP_CHASSIS_FOLLOW_GIMBAL,KI_CHASSIS_FOLLOW_GIMBAL,KD_CHASSIS_FOLLOW_GIMBAL};
    PID_init(&chassis_pid.follow,PID_POSITION,gimbal_follow,MAX_OUT_CHASSIS_FOLLOW_GIMBAL,MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);

    MotorInit(&chassis.wheel[0], WHEEL_1_ID, WHEEL_1_CAN, WHEEL_1_MOTOR_TYPE, WHEEL_1_DIRECTION, WHEEL_1_RATIO, WHEEL_1_MODE);
    MotorInit(&chassis.wheel[1], WHEEL_2_ID, WHEEL_2_CAN, WHEEL_2_MOTOR_TYPE, WHEEL_2_DIRECTION, WHEEL_2_RATIO, WHEEL_2_MODE);
    MotorInit(&chassis.wheel[2], WHEEL_3_ID, WHEEL_3_CAN, WHEEL_3_MOTOR_TYPE, WHEEL_3_DIRECTION, WHEEL_3_RATIO, WHEEL_3_MODE);
    MotorInit(&chassis.wheel[3], WHEEL_4_ID, WHEEL_4_CAN, WHEEL_4_MOTOR_TYPE, WHEEL_4_DIRECTION, WHEEL_4_RATIO, WHEEL_4_MODE);

    //step4 初始模式设置
    chassis.mode = CHASSIS_LOCK;
    chassis.spin_flag = 0;
    chassis.sc_flag = 0;
    chassis.spin_mode_last = CHASSIS_LOCK;
    chassis.wz_set_smoothed = 0.0f;
    chassis.spin_acceleration_limit = 10.0f;
}


/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
    if ((toe_is_error(DBUS_TOE)) || switch_is_up(chassis.rc->rc.s[1]))
    {
        chassis.mode = CHASSIS_LOCK;
    }
    else if (switch_is_mid(chassis.rc->rc.s[1]))
    {
        chassis.mode = CHASSIS_FOLLOW;
    }
    else if (switch_is_down(chassis.rc->rc.s[1]))
    {
        chassis.mode = CHASSIS_SPIN;
    }
}


/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) 
{
    for (int i=0;i<4;++i)
    {
        GetMotorMeasure(&chassis.wheel[i]);
        chassis.feedback[i] = chassis.wheel[i].fdb.vel;
    }

    // 使用云台相对底盘的偏航差，供跟随和小陀螺模式使用。
    chassis.yaw_delta = CmdGimbalJointState(AX_YAW);
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    switch (chassis.mode){

        case CHASSIS_LOCK:{
            chassis.reference.vx=0;
            chassis.reference.vy=0;
            chassis.reference.wz=0;

            break;
        }

        case CHASSIS_SINGLE:{
            chassis.reference.vx = fp32_deadline( chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference.vy = fp32_deadline(-chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference.wz = fp32_deadline(-chassis.rc->rc.ch[0], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
        
            break;
        }

        case CHASSIS_FOLLOW:{
            chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference_rc.vy = fp32_deadline(chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;

            // 将云台坐标系下的平移指令转到底盘坐标系下。
            chassis.sin_yaw = sinf(chassis.yaw_delta);
            chassis.cos_yaw = cosf(chassis.yaw_delta);
            chassis.reference.vx = chassis.reference_rc.vx * chassis.cos_yaw - chassis.reference_rc.vy * chassis.sin_yaw;
            chassis.reference.vy = chassis.reference_rc.vx * chassis.sin_yaw + chassis.reference_rc.vy * chassis.cos_yaw;

            // 中心附近给一个小死区，避免底盘和云台在零点附近来回摆动。
            if (fabsf(chassis.yaw_delta) < 0.10f)
            {
                PID_clear(&chassis_pid.follow);
                chassis.reference.wz = 0.0f;
            }
            else
            {
                chassis.reference.wz = -PID_calc(&chassis_pid.follow, -chassis.yaw_delta, 0.0f);
            }


            break;
        }

        case CHASSIS_SPIN:{
            chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference_rc.vy = fp32_deadline(chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;

            // 小陀螺时保持平移方向相对云台一致。
            chassis.sin_yaw = sinf(chassis.yaw_delta);
            chassis.cos_yaw = cosf(chassis.yaw_delta);
            chassis.reference.vx = -chassis.reference_rc.vx * chassis.cos_yaw - chassis.reference_rc.vy * chassis.sin_yaw;
            chassis.reference.vy = -chassis.reference_rc.vx * chassis.sin_yaw + chassis.reference_rc.vy * chassis.cos_yaw;

            if (chassis.spin_mode_last != CHASSIS_SPIN) 
            {
                chassis.wz_set_smoothed = 0.0f;
                chassis.spin_acceleration_limit = 10.0f;
            }

            if (fabsf(chassis.reference_rc.vx) > 0.4f || fabsf(chassis.reference_rc.vy) > 0.4f)
            {
                chassis.spin_target_wz = 6.6f + robot_status.robot_level * 0.5f;
            }
            else
            {
                chassis.spin_target_wz = 7.6f + robot_status.robot_level * 0.6f;
            }

            chassis.wz_error = chassis.spin_target_wz - chassis.wz_set_smoothed;

            if (chassis.wz_error > chassis.spin_acceleration_limit)
            {
                chassis.wz_set_smoothed += chassis.spin_acceleration_limit;
            }
            else
            {
                chassis.wz_set_smoothed = chassis.spin_target_wz;
            }

            chassis.reference.wz = chassis.wz_set_smoothed;
            if (chassis.spin_acceleration_limit < 30.0f)
            {
                chassis.spin_acceleration_limit += 0.5f;
            }
             
            break;
        }
    }
    
    chassis.spin_mode_last = chassis.mode;
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{

    chassis.set[0] = ((-chassis.reference.vy - chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[0].reduction_ratio;
    chassis.set[1] = ((-chassis.reference.vy + chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[1].reduction_ratio;
    chassis.set[2] = (( chassis.reference.vy + chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[2].reduction_ratio;
    chassis.set[3] = (( chassis.reference.vy - chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * chassis.wheel[3].reduction_ratio;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel[i].set.curr = PID_calc(&chassis_pid.wheel_velocity[i], chassis.feedback[i], chassis.set[i]);
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ChassisSendCmd(void)
{
    CanCmdDjiMotor(CHASSIS_CAN, CHASSIS_STDID, chassis.wheel[3].set.curr, chassis.wheel[0].set.curr, chassis.wheel[1].set.curr, chassis.wheel[2].set.curr);
}


#endif
