/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *   V1.1.0    2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *   V1.1.1    2024-11-11    Harry_Wong        1.为云台随动添加了yaw轴偏转角度的API
  *   V1.1.2    2024-11-25    Harry_Wong        1.云台模式设置逻辑重构，准备函数给底盘表明是否处于初始化模式
  *   V1.1.3    2024-12-16    Harry_Wong        1.云台的imu获取方式被更改为函数传递，防止Subcribe（）函数停用造成影响 
                                                2.云台在遥控器断联情况下直接发送0电流，防止赛场上出现意外情况影响稳定性
  *   V1.2.0    2025-2-25     Harry_Wong        1.向上位机发送数据进行了优化，采用了电机角度差值控制：（电机实际角度 - 电机中值角度）* 电机旋转方向 （距离中间的偏移角度）
                                                2.添加了Gimbal_direct_ecd_to_imu（）函数 接受上位机返回的目标角度，并且跟发送角度做差得到角度期望偏移值并映射成为云台imu的期望角度值
                                                3. Reference（） AUTO_AIM 模式下的reference计算方法进行了更改：将返回值进行Gimbal_direct_ecd_to_imu（）运算得到结果作为云台的目标角度
                                                4. 删除了 imu_base 变量 ，现在云台已经不需要在第一次校准的时候记录imu初始值了
                                                5.Console() INIT 模式下的PID计算优化：所有云台的旋转计算参考值已经更改为imu相关数据，尽可能的减轻调教负担
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/

#include "gimbal_yaw_pitch_direct.h"
#include "chassis.h"
#include "yaw_auto_lqr_eso_controller.h"
#include <float.h>
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
Gimbal_s gimbal_direct;
PID_t gimbal_direct_pid;

static YawAutoLqrEso_t yaw_auto_ctrl;                 // 开源模块的 ESO、积分和上次力矩等内部状态。
static YawAutoLqrEsoOutput_t yaw_auto_output;        // 开源模块的计算结果；本车取其中的 tau_cmd_nm 作为 yaw 基础力矩。
// 本车 yaw 控制器配置；J、B 和反馈增益是接入初值，需依据实车辨识和整定。
static const YawAutoLqrEsoConfig_t yaw_auto_cfg = {
    .j_kg_m2 = GIMBAL_AUTO_YAW_J_KG_M2,             // 转动惯量 J，kg·m²；决定加速度前馈 J*alpha。
    .b_nms_rad = GIMBAL_AUTO_YAW_B_NMS_RAD,         // 粘性阻尼 B，N·m·s/rad；决定速度前馈 B*omega。
    .k_theta = GIMBAL_AUTO_YAW_K_THETA,             // 位置误差反馈增益，输出贡献为 -Ktheta*e_theta。
    .k_omega = GIMBAL_AUTO_YAW_K_OMEGA,             // 速度误差反馈增益，输出贡献为 -Komega*e_omega。
    .k_i = 0.0f,                                    // 位置误差积分增益；当前为 0，不施加积分力矩。
    .theta_integral_limit_rad_s = 1.0f,             // 位置误差积分状态限幅；当前积分力矩关闭。
    .tau_coulomb_nm = 0.0f,                         // 库仑摩擦补偿幅值，N·m；当前关闭。
    .coulomb_smooth_rad_s = 0.0f,                   // 库仑摩擦平滑速度尺度，rad/s；当前未使用。
    .eso_bandwidth_rad_s = GIMBAL_AUTO_YAW_ESO_BANDWIDTH_RAD_S, // ESO 观测带宽，rad/s。
    .eso_comp_gain = 1.0f,                          // ESO 扰动补偿比例；仅在补偿开关开启时生效。
    .eso_comp_limit_nm = 0.5f,                      // ESO 扰动补偿力矩限幅，N·m。
    .eso_omega_gate_rad_s = 0.0f,                   // 补偿的反馈速度门限；仅在补偿开启时使用。
    .eso_alpha_gate_rad_s2 = 0.0f,                  // 补偿的参考加速度门限；仅在补偿开启时使用。
    .tau_bias_ki = 0.0f,                            // 力矩偏置自适应增益；当前关闭。
    .tau_bias_limit_nm = 0.0f,                      // 自适应力矩偏置限幅，N·m。
    .tau_meas_lpf_alpha = 0.1f,                     // 实测力矩的一阶滤波系数。
    .theta_deadband_rad = 0.0f,                     // 位置误差死区，rad；当前无死区。
    .torque_soft_limit_nm = GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM, // 模块内部对称软力矩限幅，N·m。
    .torque_min_nm = -GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM,       // 模块内部力矩下限，N·m。
    .torque_max_nm = GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM,        // 模块内部力矩上限，N·m。
    .torque_slew_rate_nm_s = GIMBAL_AUTO_YAW_TORQUE_SLEW_NM_S, // 每秒最大力矩变化，N·m/s。
    .eso_enable = 1U,                               // 启用 ESO 状态观测；不等同于启用扰动补偿。
    .eso_comp_enable = GIMBAL_AUTO_YAW_ESO_COMP_ENABLE, // 是否把 ESO 估计扰动加到力矩输出。
    .torque_slew_enable = 1U,                       // 启用模块内部力矩斜率限制。
};
static bool yaw_auto_active = false;                // 本车连续角跟踪是否已建立。
static bool yaw_auto_feedback_valid = false;       // 当前周期 yaw 反馈及参考是否通过有效性检查。
static fp32 yaw_auto_theta_unwrapped = 0.0f;       // 解包后的连续 IMU yaw 角，rad，供 ESO 使用。
static fp32 yaw_auto_theta_last = 0.0f;            // 上一周期的折返 IMU yaw 角，rad。
static TickType_t yaw_auto_last_tick = 0;          // 上次有效计算的 RTOS tick，用于估计控制周期。

/**
 * @brief 检查 yaw 控制输入或输出是否为有限浮点数。
 * @param value 待检查的数值。
 * @return true 为有限值；false 为 NaN 或正负无穷。
 */
static bool GimbalAutoYawFinite(fp32 value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

/**
 * @brief 复位自瞄 yaw 控制器及下发设定，用于模式切换或异常输入。
 * @note 开源 Reset 清除 ESO、积分和上次力矩状态；本函数还清除本车有效标志及输出。
 */
static void GimbalAutoYawReset(void)
{
    YawAutoLqrEso_Reset(&yaw_auto_ctrl, 0.0f, 0.0f);
    yaw_auto_active = false;
    yaw_auto_feedback_valid = false;
    gimbal_direct.yaw.set.vel = 0.0f;
    gimbal_direct.yaw.set.tor = 0.0f;
}

/**
 * @brief 计算自瞄 yaw 的力矩命令，并写入 gimbal_direct.yaw.set.tor。
 * @note 将本车 IMU/电机反馈与上位机规划参考送入 LQR-ESO，再叠加底盘补偿并限幅。
 *       首个有效反馈周期只初始化开源控制器，实际输出零力矩。
 */
static void GimbalAutoYawConsole(void)
{
    YawAutoLqrEsoFeedback_t feedback; // 开源模块反馈输入：连续角、角速度、实测力矩及有效标志。
    YawAutoLqrEsoReference_t ref;    // 开源模块参考输入：目标角、规划速度和规划加速度。
    TickType_t now = xTaskGetTickCount(); // 当前 RTOS tick。
    TickType_t elapsed = now - yaw_auto_last_tick; // 距上次有效计算经过的 tick。
    fp32 theta = gimbal_direct.feedback_pos.yaw; // IMU 世界 yaw 角，rad，原始值会在 ±pi 折返。
    fp32 omega = gimbal_direct.feedback_vel.yaw; // IMU 世界 yaw 角速度，rad/s。
    fp32 tau_meas = gimbal_direct.yaw.direction * gimbal_direct.yaw.fdb.tor; // 电机实测力矩映射到控制器正方向，N·m。
    fp32 chassis_wz = ChassisGetSpeedWz(); // 底盘角速度，供本车额外力矩补偿使用。
    fp32 plan_vel = GetScCmdGimbalVelocity(AX_YAW); // 上位机规划 yaw 速度，rad/s。
    fp32 plan_acc = GetScCmdGimbalAcceleration(AX_YAW); // 上位机规划 yaw 加速度，rad/s²。
    fp32 dt_s;                       // 送入控制器的计算周期，s。
    fp32 tau_previous;               // 上一周期实际下发的控制器正方向力矩，N·m。
    fp32 tau_world;                  // 叠加底盘补偿并限幅后的控制器正方向力矩，N·m。
    bool controller_was_ready;       // 调用前是否已有有效反馈，决定首周期是否输出力矩。

    // 电机离线、非有限值或明显异常的规划速度/加速度：清状态并停止本拍输出。
    if (gimbal_direct.yaw.offline ||
        !GimbalAutoYawFinite(theta) || !GimbalAutoYawFinite(omega) ||
        !GimbalAutoYawFinite(tau_meas) ||
        !GimbalAutoYawFinite(chassis_wz) ||
        !GimbalAutoYawFinite(gimbal_direct.reference.yaw) ||
        !GimbalAutoYawFinite(plan_vel) || !GimbalAutoYawFinite(plan_acc) ||
        plan_vel > GIMBAL_AUTO_YAW_REF_VEL_VALID_MAX_RAD_S ||
        plan_vel < -GIMBAL_AUTO_YAW_REF_VEL_VALID_MAX_RAD_S ||
        plan_acc > GIMBAL_AUTO_YAW_REF_ACC_VALID_MAX_RAD_S2 ||
        plan_acc < -GIMBAL_AUTO_YAW_REF_ACC_VALID_MAX_RAD_S2) {
        GimbalAutoYawReset();
        return;
    }

    // IMU yaw 在 +/-pi 折返；ESO 必须接收连续角度，参考仍取最近等效角。
    if (!yaw_auto_active || elapsed == 0U || elapsed > 10U) {
        yaw_auto_theta_unwrapped = theta;
        YawAutoLqrEso_Reset(&yaw_auto_ctrl, theta, omega);
    } else {
        yaw_auto_theta_unwrapped +=
            loop_fp32_constrain(theta - yaw_auto_theta_last, -M_PI, M_PI);
        // 长时间连续旋转时重置等效 2pi 基点，保持单精度误差分辨率。
        if (yaw_auto_theta_unwrapped > 2.0f * M_PI) {
            yaw_auto_theta_unwrapped -= 2.0f * M_PI;
            yaw_auto_ctrl.eso.z1 -= 2.0f * M_PI;
        } else if (yaw_auto_theta_unwrapped < -2.0f * M_PI) {
            yaw_auto_theta_unwrapped += 2.0f * M_PI;
            yaw_auto_ctrl.eso.z1 += 2.0f * M_PI;
        }
    }
    // 保存本拍连续角与时间，供下一拍解包和周期计算。
    yaw_auto_active = true;
    yaw_auto_feedback_valid = true;
    yaw_auto_theta_last = theta;
    yaw_auto_last_tick = now;

    // 目标角映射到当前连续角附近；速度和加速度直接采用上位机规划值。
    ref.theta_rad = yaw_auto_theta_unwrapped +
        loop_fp32_constrain(gimbal_direct.reference.yaw - theta, -M_PI, M_PI);
    ref.omega_rad_s = plan_vel;
    ref.alpha_rad_s2 = plan_acc;
    // 开源模块使用 IMU 角度/角速度和映射方向后的 DM 实测力矩作为反馈。
    feedback.theta_rad = yaw_auto_theta_unwrapped;
    feedback.omega_rad_s = omega;
    feedback.tau_meas_nm = tau_meas;
    feedback.feedback_ok = 1U;
    // 正常按 tick 换算周期；首次或间隔异常时使用 1 ms 作为模块输入。
    dt_s = (elapsed >= 1U && elapsed <= 10U)
        ? (fp32)elapsed / (fp32)configTICK_RATE_HZ : 0.001f;
    tau_previous = yaw_auto_ctrl.tau_cmd_last_nm;
    controller_was_ready = yaw_auto_ctrl.feedback_ready != 0U;
    // 开源链路计算参考前馈、误差反馈和 ESO 状态，输出限幅后的 yaw 力矩。
    YawAutoLqrEso_Calc(&yaw_auto_ctrl, &yaw_auto_cfg, &feedback, &ref,
                       dt_s, &yaw_auto_output);
    // set.vel 仅保留原始规划速度参考供现有 VOFA 通道观察；MIT 发送只使用 set.tor。
    gimbal_direct.yaw.set.vel = ref.omega_rad_s;
    // 底盘角速度补偿有独立增益；最终力矩另做总限幅与斜率限制。
    tau_world = controller_was_ready
        ? yaw_auto_output.tau_cmd_nm +
              GIMBAL_AUTO_YAW_CHASSIS_WZ_TORQUE_FF * chassis_wz
        : 0.0f;
    // 底盘补偿叠加后，再按本车总力矩上限及上次实际输出做最终约束。
    tau_world = fp32_constrain(tau_world,
                              -GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM,
                              GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM);
    tau_world = fp32_constrain(tau_world,
                              tau_previous - yaw_auto_cfg.torque_slew_rate_nm_s * dt_s,
                              tau_previous + yaw_auto_cfg.torque_slew_rate_nm_s * dt_s);
    // 控制器正方向力矩映射回电机方向，作为 MIT 命令的力矩分量。
    gimbal_direct.yaw.set.tor = fp32_constrain(
        gimbal_direct.yaw.direction * tau_world,
        -GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM, GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM);
    if (!GimbalAutoYawFinite(gimbal_direct.yaw.set.tor)) {
        GimbalAutoYawReset();
    } else {
        // ESO 下一拍应使用实际下发值，而非叠加底盘前馈前的模块输出。
        yaw_auto_ctrl.tau_cmd_last_nm =
            gimbal_direct.yaw.direction * gimbal_direct.yaw.set.tor;
    }
}

#ifndef GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD
#define GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD (0.0f)
#endif


/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/*----------------angle_solution--------------------*/

/**
 * @brief          解算imu和电机角度的差值（用于更新云台的限位范围）,确认新的电机中值对应的imu值
 * @param[in]      none
 * @retval         none
 */

void Angle_solution(void)
{
  float motor_feedback=gimbal_direct.pitch.fdb.pos,
        imu_feedback=gimbal_direct.feedback_pos.pitch,motor_mid=GIMBAL_DIRECT_PITCH_MID,
        imu_mid=0.0;
  float motor_delta=GIMBAL_DIRECT_PITCH_DIRECTION*(motor_feedback-motor_mid),
        imu_delta=imu_feedback-imu_mid;
  gimbal_direct.angle_zero_for_imu=imu_delta-motor_delta;
}

/*----------------Gimbal_direct_init_judge--------------------*/
/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */

bool Gimbal_direct_init_judge (void)
{
  fp32 delta_yaw = loop_fp32_constrain(gimbal_direct.yaw.direction * (GIMBAL_DIRECT_YAW_MID - gimbal_direct.yaw.fdb.pos), -M_PI, M_PI);
  //fp32 delta_pitch = gimbal_direct.pitch.direction *(GIMBAL_DIRECT_PITCH_MID - gimbal_direct.pitch.fdb.pos);
  fp32 delta_pitch = gimbal_direct.reference.pitch - gimbal_direct.feedback_pos.pitch;

  if (((delta_yaw < 0.03f && (-0.03f) < delta_yaw) &&
       (delta_pitch < 0.03f && (-0.03f) < delta_pitch))
       || gimbal_direct.init_timer >= GIMBAL_INIT_TIME)
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*----------------Gimbal_direct_ecd_to_imu--------------------*/
/**
 * @brief          ecd角度值转换成imu角度值
 * @param[in]      axis 用于知道读 取哪一个轴的角度转换
 * @param[in]      value 用于准换的值
 * @retval         float imu映射角度值
 */

 float Gimbal_direct_ecd_to_imu(uint8_t axis,float value)
 {
  if (axis == AX_PITCH)
  {
    return value - CmdGimbalJointState(AX_PITCH) + gimbal_direct.feedback_pos.pitch;
  }

  // else if (axis == AX_YAW)
  // { 
  //   return value - CmdGimbalJointState(AX_YAW) + gimbal_direct.feedback_pos.yaw;
  // }
  else if (axis == AX_YAW)
  {
      fp32 cur_joint_yaw = CmdGimbalJointState(AX_YAW);
      fp32 cmd_joint_yaw = value;

      // 上位机目标关节角 - 当前关节角，只取最近方向的角度差
      fp32 delta_joint_yaw =
          loop_fp32_constrain(cmd_joint_yaw - cur_joint_yaw, -M_PI, M_PI);

      // 把“关节角差值”映射到当前 IMU yaw 上
      // 注意：这里不要再强行 wrap 到 [-pi, pi]
      return gimbal_direct.feedback_pos.yaw + delta_joint_yaw;
  }

  else 
  {
    return 0.0f;
  }
 }


/*-------------------------The end of internal functions--------------------------------------*/
/* ---------------- GetGimbalDeltaYawMid -------------------- */
/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
  return loop_fp32_constrain(
      gimbal_direct.yaw.direction * (gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID),
      -M_PI,
      M_PI);
}

/* ---------------- GetGimbalInitJudgeReturn -------------------- */
/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
  return gimbal_direct.init_continue;
}

/* --------------------- CmdGimbalJointState ------------------- */
/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 （float)
 */
inline float CmdGimbalJointState(uint8_t axis)
{
  if ( axis == AX_PITCH )
  {
    return loop_fp32_constrain(gimbal_direct.pitch.direction * (gimbal_direct.pitch.fdb.pos - GIMBAL_DIRECT_PITCH_MID),-M_PI,M_PI);
  }
  else if ( axis == AX_YAW )
  {
    return loop_fp32_constrain(gimbal_direct.yaw.direction * (gimbal_direct.yaw.fdb.pos - GIMBAL_DIRECT_YAW_MID),-M_PI,M_PI); 
  }
  else 
  {
    return 0.0;
  }
}


/*-------------------- Init --------------------*/
/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void) 
{
   //step0 初始化控制器
   YawAutoLqrEso_Init(&yaw_auto_ctrl);
   GimbalAutoYawReset();
   //step1 获取所有所需变量指针
   gimbal_direct.rc = get_remote_control_point(); 
   //step2 置零所有值
   gimbal_direct.reference.pitch=0;
   gimbal_direct.reference.yaw=0;

   gimbal_direct.feedback_pos.pitch=0;
   gimbal_direct.feedback_pos.yaw=0;

   gimbal_direct.feedback_vel.pitch=0;
   gimbal_direct.feedback_vel.yaw=0;

   gimbal_direct.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;

   gimbal_direct.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;
   //step3 PID数据清零，设置PID参数
   const static fp32 gimbal_yaw_angle[3]={KP_GIMBAL_YAW_ANGLE,KI_GIMBAL_YAW_ANGLE,KD_GIMBAL_YAW_ANGLE};
   const static fp32 gimbal_yaw_velocity[3]={KP_GIMBAL_YAW_VELOCITY,KI_GIMBAL_YAW_VELOCITY,KD_GIMBAL_YAW_VELOCITY};
	 const static fp32 gimbal_pitch_angle[3]={KP_GIMBAL_PITCH_ANGLE,KI_GIMBAL_PITCH_ANGLE,KD_GIMBAL_PITCH_ANGLE};
   const static fp32 gimbal_pitch_velocity[3]={KP_GIMBAL_PITCH_VELOCITY,KI_GIMBAL_PITCH_VELOCITY,KD_GIMBAL_PITCH_VELOCITY};

   PID_init(&gimbal_direct_pid.yaw_angle,PID_POSITION,gimbal_yaw_angle,MAX_OUT_GIMBAL_YAW_ANGLE,MAX_IOUT_GIMBAL_YAW_ANGLE);
   PID_init(&gimbal_direct_pid.yaw_velocity,PID_POSITION,gimbal_yaw_velocity,MAX_OUT_GIMBAL_YAW_VELOCITY,MAX_IOUT_GIMBAL_YAW_VELOCITY);

   PID_init(&gimbal_direct_pid.pitch_angle,PID_POSITION,gimbal_pitch_angle,MAX_OUT_GIMBAL_PITCH_ANGLE,MAX_IOUT_GIMBAL_PITCH_ANGLE);
   PID_init(&gimbal_direct_pid.pitch_velocity,PID_POSITION,gimbal_pitch_velocity,MAX_OUT_GIMBAL_PITCH_VELOCITY,MAX_IOUT_GIMBAL_PITCH_VELOCITY);

   //step4 初始化电机
   MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,GIMBAL_DIRECT_YAW_REDUCTION_RATIO,GIMBAL_DIRECT_YAW_MODE);
   //DM4310
   //MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);
   //DJI_M6020
   MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);

   //step5 初始化云台初始化校准相关变量
   gimbal_direct.init_start_time=0;
   gimbal_direct.init_timer=0;
   gimbal_direct.init_continue=false;

   //step6 模式设置初始化
   gimbal_direct.mode=GIMBAL_ZERO_FORCE;
   gimbal_direct.last_mode = GIMBAL_ZERO_FORCE;
   gimbal_direct.mode_before_rc_err = GIMBAL_ZERO_FORCE;
}


/*-------------------- Set mode --------------------*/
/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
  if ( toe_is_error(DBUS_TOE) ){
    gimbal_direct.mode=GIMBAL_DBUS_ERR;
  }
  else if (gimbal_direct.last_mode == GIMBAL_DBUS_ERR){
    gimbal_direct.mode = gimbal_direct.mode_before_rc_err;
  }
  else if ((switch_is_up(gimbal_direct.rc->rc.s[0]))){//上档无力
    gimbal_direct.mode=GIMBAL_ZERO_FORCE;
    gimbal_direct.init_continue=false;
  }
  //初始校准模式
  else if (gimbal_direct.mode==GIMBAL_ZERO_FORCE || gimbal_direct.mode==GIMBAL_INIT){
    gimbal_direct.mode = GIMBAL_INIT;//从 switch_is_up 切到 switch_is_mid 的第一拍，mode 会先稳定进入 GIMBAL_INIT,不会立刻被旧 reference 的 Gimbal_direct_init_judge() 
    //判成完成并跳到 GIMBAL_GAP。第二拍开始才判断初始化是否完成。
    if (gimbal_direct.last_mode == GIMBAL_INIT){
      gimbal_direct.init_continue = Gimbal_direct_init_judge();
      if (gimbal_direct.init_continue==true){//判断是否需要跳出循环
        gimbal_direct.mode=GIMBAL_GAP;
      }
    }
  }
  //下，中档陀螺仪和自瞄控制
  else if (switch_is_down(gimbal_direct.rc->rc.s[0]) || switch_is_mid(gimbal_direct.rc->rc.s[0])){{
      gimbal_direct.mode=GIMBAL_IMU;
    }
    if (!GetUsbStatus() && GetScCmdFricOn()){
          gimbal_direct.mode = GIMBAL_AUTO_AIM;
   }    
  }
}

/*-------------------- Observe --------------------*/
/** 
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) 
{
  //电机相关数据更新
  GetMotorMeasure(&gimbal_direct.yaw);
  GetMotorMeasure(&gimbal_direct.pitch);

  //IMU相关数据更新
  gimbal_direct.feedback_pos.pitch=GetImuAngle(AX_PITCH);
  gimbal_direct.feedback_pos.yaw=GetImuAngle(AX_YAW);

  gimbal_direct.feedback_vel.pitch=GetImuVelocity(AX_PITCH);
  gimbal_direct.feedback_vel.yaw=GetImuVelocity(AX_YAW);

  Angle_solution();

  //初始化校准模式时钟更新
  if (gimbal_direct.mode == GIMBAL_INIT){
    if (gimbal_direct.last_mode != GIMBAL_INIT){
      gimbal_direct.init_start_time=xTaskGetTickCount();
    }
    gimbal_direct.init_timer=xTaskGetTickCount()-gimbal_direct.init_start_time;
  }
  else{
      gimbal_direct.init_timer=0;
  }

  if (gimbal_direct.mode == GIMBAL_DBUS_ERR && gimbal_direct.last_mode != GIMBAL_DBUS_ERR ){
    gimbal_direct.mode_before_rc_err=gimbal_direct.last_mode;
  }

  gimbal_direct.last_mode=gimbal_direct.mode; //上一运行模式更新
}


/*-------------------- Reference --------------------*/
/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{
  switch (gimbal_direct.mode){
      case GIMBAL_INIT:{
        gimbal_direct.reference.pitch = gimbal_direct.pitch.direction *(GIMBAL_DIRECT_PITCH_MID - gimbal_direct.pitch.fdb.pos) + gimbal_direct.feedback_pos.pitch;
        gimbal_direct.reference.yaw=loop_fp32_constrain(gimbal_direct.yaw.direction   * (GIMBAL_DIRECT_YAW_MID   - gimbal_direct.yaw.fdb.pos  ) + gimbal_direct.feedback_pos.yaw , -M_PI , M_PI);  
        break;
      }

      case GIMBAL_GAP:{
        gimbal_direct.reference.pitch=gimbal_direct.feedback_pos.pitch;
        gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;
        break;
      }

      case GIMBAL_IMU:{
        if (gimbal_direct.last_mode != GIMBAL_IMU){
          gimbal_direct.reference.pitch=gimbal_direct.feedback_pos.pitch;
          gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;
        }
        else {
          //摇杆控制
					gimbal_direct.reference.pitch= fp32_constrain(gimbal_direct.reference.pitch-fp32_deadline(gimbal_direct.rc->rc.ch[1], REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY,GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu,GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu);
          gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.reference.yaw-fp32_deadline(gimbal_direct.rc->rc.ch[0],REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY,-M_PI,M_PI);
        }
        break;
      }

      case GIMBAL_AUTO_AIM:{
        fp32 yaw_cmd_abs = GetScCmdGimbalAngle(AX_YAW);
        fp32 yaw_now = gimbal_direct.feedback_pos.yaw;
        gimbal_direct.reference.yaw = yaw_now + loop_fp32_constrain(yaw_cmd_abs - yaw_now, -M_PI, M_PI);
        gimbal_direct.reference.pitch = fp32_constrain( -GetScCmdGimbalAngle(AX_PITCH),GIMBAL_LOWER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu,GIMBAL_UPPER_LIMIT_PITCH + gimbal_direct.angle_zero_for_imu);
        break;


      }
  }
}


/*-------------------- Console --------------------*/
/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) 
{
  if (gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == GIMBAL_DBUS_ERR || gimbal_direct.yaw.offline || gimbal_direct.pitch.offline)
  {
    gimbal_direct.pitch.set.curr=0; 
    gimbal_direct.yaw.set.tor = 0;
  } 
  else if(gimbal_direct.mode == GIMBAL_IMU || gimbal_direct.mode== GIMBAL_GAP || gimbal_direct.mode == GIMBAL_AUTO_AIM || gimbal_direct.mode == GIMBAL_INIT){
    static fp32 plan_pitch_acc_filtered = 0.0f;
    fp32 plan_pitch_vel_ff = 0.0f;
    fp32 pitch_acc_current_ff = 0.0f;
    fp32 pitch_vel_cmd;
    fp32 pitch_velocity_current;

    if (gimbal_direct.mode == GIMBAL_AUTO_AIM) {
        //pitch自瞄相关
        if (!gimbal_direct.pitch.offline) {
            fp32 plan_pitch_acc = fp32_constrain(GetScCmdGimbalAcceleration(AX_PITCH), -MAX_GIMBAL_DIRECT_PITCH_PLAN_ACC, MAX_GIMBAL_DIRECT_PITCH_PLAN_ACC);
            plan_pitch_vel_ff = fp32_constrain(GetScCmdGimbalVelocity(AX_PITCH), -MAX_GIMBAL_DIRECT_PITCH_PLAN_VEL_FF, MAX_GIMBAL_DIRECT_PITCH_PLAN_VEL_FF);
            if (plan_pitch_acc != plan_pitch_acc) {
                plan_pitch_acc = 0.0f;
                plan_pitch_acc_filtered = 0.0f;
            }
            plan_pitch_acc_filtered += GIMBAL_DIRECT_PITCH_PLAN_ACC_FILTER_ALPHA * (-plan_pitch_acc - plan_pitch_acc_filtered);
            pitch_acc_current_ff = fp32_constrain(GIMBAL_DIRECT_PITCH_PLAN_ACC_FF_GAIN * plan_pitch_acc_filtered, -MAX_GIMBAL_DIRECT_PITCH_PLAN_ACC_FF, MAX_GIMBAL_DIRECT_PITCH_PLAN_ACC_FF);
        } else {
            plan_pitch_acc_filtered = 0.0f;
        }
        //yaw自瞄相关，纯力矩控制
        if (gimbal_direct.last_mode != GIMBAL_AUTO_AIM) {
            GimbalAutoYawReset();
        }
        GimbalAutoYawConsole();
    } else {
        plan_pitch_acc_filtered = 0.0f;
        GimbalAutoYawReset();
    }

    // 位置环根据 pitch 目标角与反馈角计算速度指令。
    pitch_vel_cmd = PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch);
    // pitch规划速度前馈。
    pitch_vel_cmd -= GIMBAL_DIRECT_PITCH_PLAN_VEL_FF_GAIN * plan_pitch_vel_ff;
    // 限制速度设定值。
    gimbal_direct.pitch.set.vel = fp32_constrain(pitch_vel_cmd, -MAX_OUT_GIMBAL_PITCH_ANGLE, MAX_OUT_GIMBAL_PITCH_ANGLE);
    // 计算电流控制量。
    pitch_velocity_current = PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.feedback_vel.pitch, gimbal_direct.pitch.set.vel);
    // 叠加规划加速度对应的电流前馈，经总限幅和电机方向映射后输出。
    gimbal_direct.pitch.set.curr = gimbal_direct.pitch.direction * fp32_constrain(pitch_velocity_current + pitch_acc_current_ff,-MAX_OUT_GIMBAL_PITCH_VELOCITY,MAX_OUT_GIMBAL_PITCH_VELOCITY);

    if (gimbal_direct.mode != GIMBAL_AUTO_AIM){
      // INIT / IMU / GAP 保留原有回中与手动控制。
      fp32 delta_yaw = loop_fp32_constrain(gimbal_direct.reference.yaw - gimbal_direct.feedback_pos.yaw, -M_PI, M_PI);
      fp32 yaw_vel_cmd = PID_calc(&gimbal_direct_pid.yaw_angle, 0, delta_yaw);
      yaw_vel_cmd -= GIMBAL_DIRECT_YAW_GYRO_DAMPING * gimbal_direct.feedback_vel.yaw;//后期删去看有无影响
      //WZ反馈
      yaw_vel_cmd += GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD * ChassisGetSpeedWz();
      //回中速度限幅
      gimbal_direct.yaw.set.vel = gimbal_direct.yaw.direction * fp32_constrain(yaw_vel_cmd, -(gimbal_direct.mode == GIMBAL_INIT ? 2.4 : 20), (gimbal_direct.mode == GIMBAL_INIT ? 2.4 : 20));
      //输出限幅
      gimbal_direct.yaw.set.tor = fp32_constrain(PID_calc(&gimbal_direct_pid.yaw_velocity, gimbal_direct.yaw.fdb.vel, gimbal_direct.yaw.set.vel), -MAX_OUT_GIMBAL_YAW_VELOCITY,MAX_OUT_GIMBAL_YAW_VELOCITY);
    }
  }
}
  

/*-------------------- Cmd --------------------*/
/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void) 
{
    // if (gimbal_direct.pitch.fdb.state == DM_STATE_DISABLE) {
    //     DmEnable(&gimbal_direct.pitch);
    // }
    if (gimbal_direct.yaw.fdb.state == DM_STATE_DISABLE) {
        DmEnable(&gimbal_direct.yaw);
    }

    if (gimbal_direct.mode == GIMBAL_ZERO_FORCE || gimbal_direct.mode == GIMBAL_DBUS_ERR ||
        gimbal_direct.yaw.offline ||
        (gimbal_direct.mode == GIMBAL_AUTO_AIM && !yaw_auto_feedback_valid)) {
        DmMitStop(&gimbal_direct.yaw);
        //DmMitStop(&gimbal_direct.pitch);
    } else {
        DmMitCtrlTorque(&gimbal_direct.yaw);
    }

  CanCmdDjiMotor(GIMBAL_CAN,GIMBAL_STDID,0,0,gimbal_direct.pitch.set.curr,0);
}


#endif  // GIMBAL_YAW_PITCH
