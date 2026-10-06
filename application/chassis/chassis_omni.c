/**
  ****************************(C) 版权所有 2024 Polarbear****************************
  * @file       chassis_omni.c/h
  * @brief      全向轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  版本       日期            作者            修改内容
  *  V1.0.0   2025.03.03       Harry_Wong        1.重新构建全向轮底盘，完成单底盘控制
  *  V1.1.0   2025.11.05       CJH               1. 完成底盘与云台协同控制
  *                                              2. 完成底盘键鼠控制
  *                                              3. 修正全向轮解算的错误
  * 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) 版权所有 2024 Polarbear****************************
*/


#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "chassis_omni.h"
#include "vofa_observe.h"
#include "vofa_port.h"
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
#include "IMU.h"  // 添加IMU头文件，用于获取稳定的陀螺仪数据
#include "CAN_cmd_SupCap.h"  // 添加超级电容控制头文件，解决CanCmdSupCap函数声明问题
#include "SupCap.h"//超级电容

Chassis_s chassis;
PID_t chassis_pid;

/*-------------------- 初始化 --------------------*/

/**
 * @brief          初始化
 * @param[in]      无
 * @retval         无
 */
void ChassisInit(void)
{
    //步骤1 获取遥控器指针
    chassis.rc = get_remote_control_point();

    //步骤2 PID数据清零，设置PID参数
    //力控专用外环 PID 参数
    const static fp32 plain_vel[3]={KP_OMNI_PLAIN_VEL,KI_OMNI_PLAIN_VEL,KD_OMNI_PLAIN_VEL};
    const static fp32 plain_vel_z[3]={KP_OMNI_OMEGA,KI_OMNI_OMEGA,KD_OMNI_OMEGA};
    const static fp32 gimbal_follow[3]={KP_CHASSIS_FOLLOW_GIMBAL,KI_CHASSIS_FOLLOW_GIMBAL,KD_CHASSIS_FOLLOW_GIMBAL};

    //底盘平动/旋转外环 PID 初始化
    PID_init(&chassis.vel_x_pid, PID_POSITION, plain_vel, MAX_OUT_OMNI_PLAIN_VEL, MAX_IOUT_OMNI_PLAIN_VEL);
    PID_init(&chassis.vel_y_pid, PID_POSITION, plain_vel, MAX_OUT_OMNI_PLAIN_VEL, MAX_IOUT_OMNI_PLAIN_VEL);
    PID_init(&chassis.omega_pid, PID_POSITION, plain_vel_z, MAX_OUT_OMNI_OMEGA, MAX_IOUT_OMNI_OMEGA);

    // 初始化云台跟随PID
    PID_init(&chassis_pid.follow,PID_POSITION,gimbal_follow,MAX_OUT_CHASSIS_FOLLOW_GIMBAL,MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);

    //步骤3 初始化电机
    MotorInit(&chassis.wheel[0], WHEEL_1_ID, WHEEL_1_CAN, WHEEL_1_MOTOR_TYPE, WHEEL_1_DIRECTION, REDUCTION_RATIO, WHEEL_1_MODE);
    MotorInit(&chassis.wheel[1], WHEEL_2_ID, WHEEL_2_CAN, WHEEL_2_MOTOR_TYPE, WHEEL_2_DIRECTION, REDUCTION_RATIO, WHEEL_2_MODE);
    MotorInit(&chassis.wheel[2], WHEEL_3_ID, WHEEL_3_CAN, WHEEL_3_MOTOR_TYPE, WHEEL_3_DIRECTION, REDUCTION_RATIO, WHEEL_3_MODE);
    MotorInit(&chassis.wheel[3], WHEEL_4_ID, WHEEL_4_CAN, WHEEL_4_MOTOR_TYPE, WHEEL_4_DIRECTION, REDUCTION_RATIO, WHEEL_4_MODE);

    /*-------------------- 初始化超级电容 --------------------*/
    //SupCapInit(&chassis.super_cap, 1);  // 初始化超级电容，使用CAN1

    /*-------------------- 初始化功率控制 --------------------*/
    //chassis_power_control_init(&chassis);  // 初始化功率控制系统

    // 发送初始化超级电容控制命令
    // 初始化时从裁判系统获取缓冲能量数据
    // fp32 init_power, init_buffer;
    // get_chassis_power_and_buffer(&init_power, &init_buffer);
    // CanCmdSupCap(1, (uint16_t)init_buffer, 50, 1);  // 使用裁判系统的实际缓冲能量

    //步骤4 初始模式设置
    chassis.mode = CHASSIS_LOCK;
    chassis.spin_mode_last = CHASSIS_LOCK;
}


/*-------------------- 设置模式 --------------------*/

/**
 * @brief          设置模式
 * @param[in]      无
 * @retval         无
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


/*-------------------- 更新状态量 --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      无
 * @retval         无
 */
void ChassisObserver(void) 
{
    //更新电机反馈数据
    for (int i=0;i<4;++i)
    {
        GetMotorMeasure(&chassis.wheel[i]);
        chassis.feedback[i] = chassis.wheel[i].fdb.vel;
    }

    //整车运动状态量
    chassis.P_vx = WHEEL_RADIUS / REDUCTION_RATIO *
            (-chassis.wheel[0].fdb.vel - chassis.wheel[1].fdb.vel + chassis.wheel[2].fdb.vel + chassis.wheel[3].fdb.vel) /
            (2.0f * sqrtf(2.0f));
    chassis.P_vy = WHEEL_RADIUS / REDUCTION_RATIO *
            ( chassis.wheel[0].fdb.vel - chassis.wheel[1].fdb.vel - chassis.wheel[2].fdb.vel + chassis.wheel[3].fdb.vel) /
            (2.0f * sqrtf(2.0f));
    chassis.P_wz = -WHEEL_RADIUS / REDUCTION_RATIO *
            ( chassis.wheel[0].fdb.vel + chassis.wheel[1].fdb.vel + chassis.wheel[2].fdb.vel + chassis.wheel[3].fdb.vel) /
            (4.0f * WHEEL_CENTER_DISTANCE);

    // 更新超级电容反馈数据
    //GetSupCapMeasure(&chassis.super_cap);

    // 使用云台相对底盘的偏航差，供跟随和小陀螺模式使用。
    chassis.yaw_delta = GetGimbalDeltaYawMid();
}

/*-------------------- 更新目标量 --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      无
 * @retval         无
 */
void ChassisReference(void)
{
    switch (chassis.mode){
        case CHASSIS_LOCK:{
            chassis.reference.vx=0;
            chassis.reference.vy=0;
            chassis.reference.wz=0;

            PID_clear(&chassis.vel_x_pid);
            PID_clear(&chassis.vel_y_pid);
            PID_clear(&chassis.omega_pid);
            PID_clear(&chassis_pid.follow);
            chassis.force_x = chassis.force_y = chassis.torque_wz = 0.0f;
            for (int i = 0; i < 4; ++i)
            {
                chassis.set[i] = chassis.wheel_force[i] = 0.0f;
                chassis.wheel[i].set.curr = 0.0f;
            }
            break;
        }

        case CHASSIS_SINGLE:{
            chassis.reference.vx = fp32_deadline( chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference.vy = fp32_deadline(-chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference.wz = fp32_deadline(-chassis.rc->rc.ch[0], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
        
            break;
        }

        case CHASSIS_FOLLOW:{
            if (chassis.spin_mode_last != CHASSIS_FOLLOW) 
            {
                PID_clear(&chassis_pid.follow);
            }

            chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference_rc.vy = -fp32_deadline(chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;

            // 将云台坐标系下的平移指令转到底盘坐标系下。
            chassis.sin_yaw = sinf(chassis.yaw_delta);
            chassis.cos_yaw = cosf(chassis.yaw_delta);
            chassis.reference.vx = chassis.reference_rc.vx * chassis.cos_yaw - chassis.reference_rc.vy * chassis.sin_yaw;
            chassis.reference.vy = chassis.reference_rc.vx * chassis.sin_yaw + chassis.reference_rc.vy * chassis.cos_yaw;

            // 中心附近给一个小死区，避免底盘和云台在零点附近来回摆动。
            if (fabsf(chassis.yaw_delta) < 0.03f &&
                fabsf(chassis.P_wz) < 0.05f)
            {
                chassis.reference.wz = 0.0f;
            }
            else
            {
                chassis.reference.wz = fp32_constrain(
                    PID_calc(&chassis_pid.follow, chassis.yaw_delta, 0.0f) -
                        CHASSIS_FOLLOW_WZ_DAMPING * chassis.P_wz,
                    -MAX_OUT_CHASSIS_FOLLOW_GIMBAL,
                    MAX_OUT_CHASSIS_FOLLOW_GIMBAL);
            }
            break;
        }

        case CHASSIS_SPIN:{
            chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
            chassis.reference_rc.vy = fp32_deadline(chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;

            // 按反馈角速度前瞻，补偿平移响应滞后；加号沿用当前底盘与云台的方向约定。
            const fp32 yaw_lead = chassis.yaw_delta + CHASSIS_SPIN_LEAD_TIME * chassis.P_wz;
            chassis.sin_yaw = sinf(yaw_lead);
            chassis.cos_yaw = cosf(yaw_lead);
            chassis.reference.vx = chassis.reference_rc.vx * chassis.cos_yaw + chassis.reference_rc.vy * chassis.sin_yaw;
            chassis.reference.vy = chassis.reference_rc.vx * chassis.sin_yaw - chassis.reference_rc.vy * chassis.cos_yaw;

            if (chassis.spin_mode_last != CHASSIS_SPIN) 
            {
                chassis.wz_set_smoothed = 0.0f;
                chassis.spin_acceleration_limit = 10.0f;
            }

            if (fabsf(chassis.reference_rc.vx) > 0.4f || fabsf(chassis.reference_rc.vy) > 0.4f)
            {
                chassis.spin_target_wz = 5.0f + robot_status.robot_level * 0.5f;
            }
            else
            {
                chassis.spin_target_wz = 6.0f + robot_status.robot_level * 0.6f;
            }

            chassis.wz_error = chassis.spin_target_wz - chassis.wz_set_smoothed;

            if (chassis.wz_error > chassis.spin_acceleration_limit)
            {
                chassis.wz_set_smoothed += chassis.spin_acceleration_limit;
            }
            else
            {
                chassis.wz_set_smoothed = chassis.spin_target_wz ;
            }

            chassis.reference.wz = chassis.wz_set_smoothed * 1.0f ;
            if (chassis.spin_acceleration_limit < 30.0f)
            {
                chassis.spin_acceleration_limit += 0.5f;
            }         
            break;
        }
    }
    
    chassis.spin_mode_last = chassis.mode;
}

/*-------------------- 计算控制量 --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      无
 * @retval         无
 */
void ChassisConsole(void)
{
    if (chassis.mode == CHASSIS_LOCK) return;

    chassis.set[0] = (( chassis.reference.vy - chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;
    chassis.set[1] = ((-chassis.reference.vy - chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;
    chassis.set[2] = ((-chassis.reference.vy + chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;
    chassis.set[3] = (( chassis.reference.vy + chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;

    //动力学逆解算
    //底盘目标牵引力
    chassis.force_x = PID_calc(&chassis.vel_x_pid, chassis.P_vx, chassis.reference.vx);
    chassis.force_y = PID_calc(&chassis.vel_y_pid, chassis.P_vy, chassis.reference.vy);
    //底盘目标转矩
    chassis.torque_wz = PID_calc(&chassis.omega_pid, chassis.P_wz, chassis.reference.wz);
    
    //计算轮向电机扭矩
    for (int i = 0; i < 4; ++i)
    {
        switch (i)
        {
            case 0:
                // 0号轮： (Fy - Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = ( chassis.force_y - chassis.force_x) / (2.0f * sqrtf(2.0f)) - chassis.torque_wz / (4.0f * WHEEL_CENTER_DISTANCE);
                break;
            case 1:
                // 1号轮： (-Fy - Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = (-chassis.force_y - chassis.force_x) / (2.0f * sqrtf(2.0f)) - chassis.torque_wz / (4.0f * WHEEL_CENTER_DISTANCE);
                break;
            case 2:
                // 2号轮： (-Fy + Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = (-chassis.force_y + chassis.force_x) / (2.0f * sqrtf(2.0f)) - chassis.torque_wz / (4.0f * WHEEL_CENTER_DISTANCE);
                break;
            default:
                // 3号轮： (Fy + Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = ( chassis.force_y + chassis.force_x) / (2.0f * sqrtf(2.0f)) - chassis.torque_wz / (4.0f * WHEEL_CENTER_DISTANCE);
                break;
        }
    
        // 电流指令由两部分构成：
        // 1) 主力矩项：wheel_force * WHEEL_RADIUS
        // 2) 速度差补偿项：目标轮速 - 反馈轮速
        // 保留原有经验映射以供对比；外环 PID 的尺度也已完成换算。
        // chassis.wheel[i].set.curr = 80.0f * (chassis.wheel_force[i] * 0.08f) + (chassis.set[i] - chassis.feedback[i]);
        const fp32 wheel_torque_nm = chassis.wheel_force[i] * WHEEL_RADIUS;
        const fp32 current_force_a = wheel_torque_nm / CHASSIS_WHEEL_KT;
        const fp32 current_speed_a = CHASSIS_SPEED_CURRENT_K_A *
                                    (chassis.set[i] - chassis.feedback[i]);
        // 后续如需添加摩擦前馈，应在此处以安培（A）为单位加入，随后再进行换算和最终保护。
        const fp32 current_a = current_force_a + current_speed_a;
        // Motor_s.set.curr 仍使用 C620 原始指令值，以兼容现有发送和观测接口。
        chassis.wheel[i].set.curr = current_a * CHASSIS_C620_CMD_PER_AMP;
    }
}

float ChassisGetSpeedWz(void)
{
    return chassis.P_wz;
}
   

/*-------------------- 发送控制指令 --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      无
 * @retval         无
 */

#if VOFA_ENABLE
static ChassisObservation chassis_observation;
static uint32_t chassis_observation_time;
static uint8_t chassis_observation_ready;

static void ChassisPublishObservation(void)
{
    ChassisObservation next;
    uint32_t key;
    unsigned i;
    for (i = 0; i < 4; ++i) {
        next.wheel[i].speed_ref = chassis.set[i];
        next.wheel[i].speed_fdb = chassis.wheel[i].offline ? VofaUnavailable() : chassis.feedback[i];
        next.wheel_current_fdb[i] = chassis.wheel[i].offline ? VofaUnavailable() : chassis.wheel[i].fdb.curr;
        next.wheel[i].output_command = chassis.wheel[i].set.curr; // 经过输出保护后的 CAN 原始指令值
        next.wheel[i].online = !chassis.wheel[i].offline;
    }
    next.vx_ref = chassis.reference.vx;
    next.vx_fdb = chassis.P_vx;
    next.vy_ref = chassis.reference.vy;
    next.vy_fdb = chassis.P_vy;
    next.vx_iout = chassis.vel_x_pid.Iout;
    next.vy_iout = chassis.vel_y_pid.Iout;
    next.wz_ref = chassis.reference.wz;
    next.wz_fdb = chassis.P_wz;
    next.mode = (float)chassis.mode;
    key = VofaPortLock();
    chassis_observation = next;
    chassis_observation_time = HAL_GetTick();
    chassis_observation_ready = 1;
    VofaPortUnlock(key);
}

void ChassisReadObservation(ChassisObservation *out)
{
    uint32_t key = VofaPortLock();
    if (chassis_observation_ready && HAL_GetTick() - chassis_observation_time <= VOFA_OBSERVATION_MAX_AGE_MS)
        *out = chassis_observation;
    else
        memset(out, 0xff, sizeof(*out));
    VofaPortUnlock(key);
}
#endif

void ChassisSendCmd(void)
{
// #if 1 // TEMP lifted-wheel current test; change to 0 to restore normal chassis output.
//     static volatile uint8_t test_wheel_id = 2; // CAN ID: 1~4
//     // static volatile int16_t test_current_raw = 200; // Previous manual-current test.
//     static volatile int16_t test_current_raw = 0; // C620 command units; 819.2 = 1 A
//     static volatile uint8_t test_stopped = 0; // 0: ramping, 1: motion detected, 2: limit reached
//     static uint8_t test_last_wheel_id = 0;
//     static uint32_t test_step_time = 0;
//     const uint32_t test_now = HAL_GetTick();
//     const uint8_t test_valid = test_wheel_id >= 1 && test_wheel_id <= 4;
//     if (!test_valid || chassis.mode == CHASSIS_LOCK ||
//         (test_valid && chassis.wheel[test_wheel_id - 1].offline) ||
//         test_wheel_id != test_last_wheel_id) {
//         test_current_raw = 0;
//         test_stopped = 0;
//         test_step_time = test_now;
//         test_last_wheel_id = test_wheel_id;
//     } else if (!test_stopped) {
//         // Check motion BEFORE increasing current; latch even if the wheel later stops.
//         // Any nonzero motor ROTOR speed triggers the latch, including feedback noise.
//         if (fabsf(chassis.feedback[test_wheel_id - 1]) > 0.5f) {
//             test_stopped = 1;
//         } else if (test_now - test_step_time >= 10U) {
//             test_step_time = test_now;
//             test_current_raw = (int16_t)fp32_constrain(test_current_raw + 1, 0.0f, 2000.0f);
//             if (test_current_raw >= 2000) test_stopped = 2;
//         }
//     }
//     // Hold the detected current; enter CHASSIS_LOCK to zero and restart the test.
//     const int16_t test_cmd = (int16_t)fp32_constrain(test_current_raw, -16384.0f, 16384.0f);
//     for (int i = 0; i < 4; ++i) {
//         chassis.wheel[i].set.curr =
//             (chassis.mode != CHASSIS_LOCK && test_wheel_id == i + 1 && !chassis.wheel[i].offline)
//                 ? test_cmd : 0;
//     }
// #endif

    // 发送顺序必须与电机ID(1~4)一致，全向轮底盘参数中轮子索引与ID存在映射关系:
    // CanCmdDjiMotor(
    //     CHASSIS_CAN, CHASSIS_STDID, *g_chassis_power_control.motor_current_cmd[0],
    //     *g_chassis_power_control.motor_current_cmd[1],
    //     *g_chassis_power_control.motor_current_cmd[2],
    //     *g_chassis_power_control.motor_current_cmd[3]);
    CanCmdDjiMotor(CHASSIS_CAN, CHASSIS_STDID, chassis.wheel[0].set.curr, chassis.wheel[1].set.curr, chassis.wheel[2].set.curr, chassis.wheel[3].set.curr);

    // 发送超级电容控制命令（测试用）
    //    fp32 current_power, current_buffer;
    //    get_chassis_power_and_buffer(&current_power, &current_buffer);  // 从裁判系统获取实时数据

    //   uint16_t power_buffer = (uint16_t)current_buffer;  // 从裁判系统获取的当前缓冲能量
    //   uint16_t power_limit = 40;                         // 功率限制值，可根据比赛规则调整
    //   uint8_t automode_stage = 1;                        // 启用超级电容

    //   CanCmdSupCap(1, power_buffer, power_limit, automode_stage);
#if VOFA_ENABLE
    ChassisPublishObservation(); 
#endif
}

#endif
