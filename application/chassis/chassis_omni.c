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
#include "vofa_chassis.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h" 
#include "detect_task.h"
#include "gimbal.h"
#include "math.h"
#include <string.h>
#include "usb_debug.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "IMU.h"  // 添加IMU头文件，用于获取稳定的陀螺仪数据
#include "CAN_cmd_SupCap.h"  // 添加超级电容控制头文件，解决CanCmdSupCap函数声明问题
#include "SupCap.h"//超级电容

Chassis_s chassis;
PID_t chassis_pid;
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
static Vt13ControlInput chassis_input;
static GimbalControlStatus chassis_gimbal;
static Vt13SpinControl chassis_spin_control;
static Vt13KeyboardMotion chassis_keyboard_motion;

/**
 * @brief 更新小陀螺及松手回正状态并映射到底盘模式。
 * @note 纯状态机决定优先级，此函数只转换为底盘已有或新增的模式枚举。
 */
static void ChassisUpdateSpinReturn(void)
{
    Vt13ChassisMode_e mode = Vt13UpdateSpin(&chassis_spin_control, &chassis_input, &chassis_gimbal, chassis.P_wz, HAL_GetTick());
    /* 将统一状态机的结果转换为底盘模式，未知结果默认锁定。 */
    switch (mode) {
        case VT13_CHASSIS_FOLLOW: /* 常规跟随云台。 */ chassis.mode = CHASSIS_FOLLOW; break;
        case VT13_CHASSIS_BODY: /* 有目标时采用底盘坐标且不跟随。 */ chassis.mode = CHASSIS_BODY; break;
        case VT13_CHASSIS_SPIN: /* 按住右上扳机键时旋转。 */ chassis.mode = CHASSIS_SPIN; break;
        case VT13_CHASSIS_RETURN: /* 按当前档位与目标状态选择原固定回正或限速跟随。 */ chassis.mode = CHASSIS_SPIN_RETURN; break;
        default: /* 失能或未知状态均锁定。 */ chassis.mode = CHASSIS_LOCK; break;
    }
}

/**
 * @brief 根据当前模式生成底盘坐标系下的平移参考。
 * @note 底盘坐标及回正模式直接使用摇杆；跟随和旋转模式把云台方向指令转到底盘坐标。
 */
static void ChassisBuildTranslationReference(void)
{
    float vx = fp32_deadline(chassis_input.move_x * CHASSIS_RC_MAX_RANGE, -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
    float vy = fp32_deadline(chassis_input.move_y * CHASSIS_RC_MAX_RANGE, -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED;
    Vt13UpdateKeyboardMotion(&chassis_keyboard_motion, &chassis_input, HAL_GetTick(), CHASSIS_RC_MAX_SPEED);
    /* 方向键直接使用最高速度参考，松键同拍交回摇杆，不延续旧键盘速度。 */
    if (chassis_keyboard_motion.active) {
        vx = chassis_keyboard_motion.vx;
        vy = chassis_keyboard_motion.vy;
    }
    chassis.reference_rc.vx = vx;
    chassis.reference_rc.vy = vy;
    /* 有目标不跟随或松手回正时，摇杆始终按底盘自身方向平移。 */
    if (chassis.mode == CHASSIS_BODY || chassis.mode == CHASSIS_SPIN_RETURN) {
        chassis.reference.vx = vx;
        chassis.reference.vy = vy;
    } else { /* 跟随和旋转的平移输入按云台朝向解释，需要转换到底盘坐标。 */
        float angle = chassis.yaw_delta;
        /* 旋转时按实际角速度增加前瞻角，沿用原有平移滞后补偿。 */
        if (chassis.mode == CHASSIS_SPIN) angle += CHASSIS_SPIN_LEAD_TIME * chassis.P_wz;
        chassis.reference.vx = vx * cosf(angle) - vy * sinf(angle);
        chassis.reference.vy = vx * sinf(angle) + vy * cosf(angle);
    }
}

/**
 * @brief 强制底盘零输出并清除控制器和回正残留。
 * @note 同时清除参考、轮端电流、PID 与旋转状态，不能仅把目标速度设为零。
 */
static void ChassisForceDisable(void)
{
    chassis.mode = CHASSIS_LOCK;
    chassis.reference.vx = chassis.reference.vy = chassis.reference.wz = 0.0f;
    chassis.reference_rc.vx = chassis.reference_rc.vy = 0.0f;
    chassis.force_x = chassis.force_y = chassis.torque_wz = 0.0f;
    chassis.wz_set_smoothed = 0.0f;
    memset(&chassis_spin_control, 0, sizeof(chassis_spin_control));
    memset(&chassis_keyboard_motion, 0, sizeof(chassis_keyboard_motion));
    chassis_keyboard_motion.update_time_ms = HAL_GetTick();
    PID_clear(&chassis.vel_x_pid);
    PID_clear(&chassis.vel_y_pid);
    PID_clear(&chassis.omega_pid);
    PID_clear(&chassis_pid.follow);
    for (int i = 0; i < 4; ++i) {
        chassis.set[i] = chassis.wheel_force[i] = 0.0f;
        chassis.wheel[i].set.curr = 0.0f;
    }
}
#endif

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
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
    Vt13ReadControl(&chassis_input);
    GimbalReadControlStatus(&chassis_gimbal);
    chassis.yaw_delta = chassis_gimbal.yaw_joint;
    ChassisUpdateSpinReturn();
    for (int i = 0; i < 4; ++i) {
        /* 任意轮电机离线都禁止继续给整套底盘分配驱动力。 */
        if (chassis.wheel[i].offline) chassis.mode = CHASSIS_LOCK;
    }
    /* 选择锁定后立即清除参考、电流和旋转状态，避免残留输出。 */
    if (chassis.mode == CHASSIS_LOCK) ChassisForceDisable();
#else
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
#endif
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
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
    /* 失能分支清零后直接退出，不再生成本拍运动参考。 */
    if (chassis.mode == CHASSIS_LOCK) {
        ChassisForceDisable();
        chassis.spin_mode_last = chassis.mode;
        return;
    }
    /* 限速回正转正常跟随时目标来源不变，保留控制器连续性；其他切换清历史误差。 */
    if (chassis.mode != chassis.spin_mode_last &&
        !(chassis.spin_mode_last == CHASSIS_SPIN_RETURN && chassis.mode == CHASSIS_FOLLOW))
        PID_clear(&chassis_pid.follow);
    ChassisBuildTranslationReference();
    /* 小陀螺模式独立生成旋转速度，覆盖跟随和限速回正参考。 */
    if (chassis.mode == CHASSIS_SPIN) {
        /* 保留原有旋转速度、等级和加速参数。 */
        /* 刚进入旋转时重新建立速度平滑状态，不沿用退出前的旋转参考。 */
        if (chassis.spin_mode_last != CHASSIS_SPIN) {
            chassis.wz_set_smoothed = 0.0f;
            chassis.spin_acceleration_limit = 10.0f;
        }
        /* 平移指令较大时选择较低旋转速度，其余情况采用静止旋转速度，均叠加既有等级项。 */
        chassis.spin_target_wz = (fabsf(chassis.reference_rc.vx) > 0.4f || fabsf(chassis.reference_rc.vy) > 0.4f) ? 5.0f + robot_status.robot_level * 0.5f : 6.0f + robot_status.robot_level * 0.6f;
        chassis.wz_error = chassis.spin_target_wz - chassis.wz_set_smoothed;
        /* 目标增量超过本拍允许值时按原有增量限制逼近目标。 */
        if (chassis.wz_error > chassis.spin_acceleration_limit)
            chassis.wz_set_smoothed += chassis.spin_acceleration_limit;
        /* 剩余差值在允许范围内时直接到达旋转目标，避免越过目标。 */
        else chassis.wz_set_smoothed = chassis.spin_target_wz;
        chassis.reference.wz = chassis.wz_set_smoothed;
        /* 加速阶段逐步放宽每拍增量，达到既有上限后保持。 */
        if (chassis.spin_acceleration_limit < 30.0f) chassis.spin_acceleration_limit += 0.5f;
    } else /* 有目标的底盘坐标模式取消云台跟随，角速度目标置零。 */ if (chassis.mode == CHASSIS_BODY) {
        chassis.reference.wz = 0.0f;
    } else { /* 无有效自瞄时统一跟随；N 右键/S 档有效自瞄保留松手固定朝向回正。 */
        bool fixed_heading = chassis.mode == CHASSIS_SPIN_RETURN && Vt13AutoAimRequested(&chassis_input) && chassis_gimbal.auto_active;
        float error = fixed_heading ? Vt13WrapAngle(chassis_spin_control.return_heading - Vt13ChassisHeading(&chassis_gimbal)) : Vt13WrapAngle(chassis.yaw_delta);
        float max_wz = chassis.mode == CHASSIS_SPIN_RETURN ? CHASSIS_SPIN_RETURN_MAX_WZ : MAX_OUT_CHASSIS_FOLLOW_GIMBAL;
        float wz_damping = CHASSIS_FOLLOW_WZ_DAMPING;
        /* 仅 N/S 无目标回正时，越接近中点越增强制动；远处及其他模式保持原阻尼。 */
        if (chassis.mode == CHASSIS_SPIN_RETURN && !fixed_heading) {
            float blend = fp32_constrain(1.0f - fabsf(error) / CHASSIS_SPIN_RETURN_BRAKE_ANGLE, 0.0f, 1.0f);
            wz_damping += blend * (CHASSIS_SPIN_RETURN_NEAR_WZ_DAMPING - CHASSIS_FOLLOW_WZ_DAMPING);
        }
        /* 已对齐且基本停止旋转时不再施加跟随修正，减小中位摆动。 */
        if (fabsf(error) < VT13_CHASSIS_ALIGN_ANGLE_RAD &&
            fabsf(chassis.P_wz) < VT13_CHASSIS_ALIGN_WZ_RAD_S) {
            /* 新跟随回正路径清除停止前差分误差；S 档有目标保留原控制器行为。 */
            if (!fixed_heading) PID_clear(&chassis_pid.follow);
            chassis.reference.wz = 0.0f;
        }
        /* 仅退出小陀螺的回正阶段低速限幅；回正结束后恢复正常跟随的原速度上限。 */
        else chassis.reference.wz = fp32_constrain(PID_calc(&chassis_pid.follow, error, 0.0f) - wz_damping * chassis.P_wz, -max_wz, max_wz);
    }
    chassis.spin_mode_last = chassis.mode;
#else
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

            /* 目标增量超过本拍允许值时按原有增量限制逼近目标。 */
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
#endif
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

void ChassisSendCmd(void)
{
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
    Vt13ControlInput latest;
    GimbalControlStatus status;
    Vt13ReadControl(&latest);
    GimbalReadControlStatus(&status);
    /* 发送前复核遥控、初始化/反馈时效及档位一致性，失效即清零本拍电流。 */
    if (!Vt13ControlEnabled(&latest) || !Vt13StatusReady(&status, HAL_GetTick()) ||
        latest.mode != chassis_input.mode) ChassisForceDisable();
    for (int i = 0; i < 4; ++i) {
        /* 最终发送前任一轮离线都清零全部轮电流，不发送之前计算的残留值。 */
        if (chassis.wheel[i].offline) ChassisForceDisable();
    }
#endif

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
    vofa_chassis_update(&chassis);
}

#endif
