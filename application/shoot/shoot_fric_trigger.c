/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  版本       日期            作者            修改说明
  *  V1.0.0     2024-04-01      Penguin         1. 完成初版
  *  V1.0.1     2024-04-16      Penguin         1. 完成基本框架
  *  V1.1.0     2025-1-15       CJH             1. 实现基本功能
  *  V2.0.0     2025-3-3        CJH             1. 兼容了达妙4310拨弹盘和大疆2006拨弹盘
  *                                             2. 完善了单发功能，上位机火控功能
  *                                             3. 增加了热量限制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "shoot_fric_trigger.h"


#include "vofa_shoot.h"
#include "gimbal.h"
#include <string.h>

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

static Shoot_s SHOOT = {
  .mode = LOAD_STOP,
  .state = FRIC_NOT_READY,
  .fric_flag = 0,
  .move_flag = 0,
  .ecd_count = 0,
  .shoot_flag = 0,
  .heat = 0,
  .heat_limit = 0,
};

#define SINGLE_SHOT_INTERVAL_MS 500U
static uint32_t last_single_shot_time;
static uint8_t single_shot_started;
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
static Vt13ControlInput shoot_input;
static GimbalControlStatus shoot_gimbal;
static Vt13ShootControl shoot_control;
static bool shoot_enabled;

/**
 * @brief 强制摩擦轮和拨弹零输出并清除所有发射残留。
 * @note 清除 Fn 锁存、堵转计时、单发状态和 PID；恢复后需要重新按 Fn 开轮。
 */
static void ShootForceDisable(void)
{
    memset(&shoot_control, 0, sizeof(shoot_control));
    shoot_enabled = false;
    SHOOT.mode = LOAD_STOP;
    SHOOT.state = FRIC_NOT_READY;
    SHOOT.fric_flag = SHOOT.move_flag = 0;
    SHOOT.block_time = SHOOT.reverse_time = 0;
    last_single_shot_time = 0;
    single_shot_started = 0;
    SHOOT.REF.fric_speed_ref_R = SHOOT.REF.fric_speed_ref_L = 0.0f;
    SHOOT.REF.trigger_speed_ref = 0.0f;
    SHOOT.REF.trigger_angel_ref = SHOOT.FDB.trigger_angel_fdb;
    SHOOT.fric_motor[0].set.curr = SHOOT.fric_motor[1].set.curr = 0.0f;
    SHOOT.trigger_motor.set.curr = 0.0f;
    PID_clear(&SHOOT.fric_pid[0]);
    PID_clear(&SHOOT.fric_pid[1]);
    PID_clear(&SHOOT.trigger_angel_pid);
    PID_clear(&SHOOT.trigger_speed_pid);
}

/**
 * @brief 更新本发射任务独占的 Fn 摩擦轮开关状态。
 * @note 只处理有效按下沿，不因任务重复读取同一遥控帧而反复开关。
 */
static void ShootUpdateFrictionSwitch(const Vt13ControlInput *input, bool enabled)
{
    Vt13UpdateFriction(&shoot_control, input, enabled);
}

/**
 * @brief 更新并读取拨轮负端的手动连发请求。
 * @note 请求采用迟滞判定；实际拨弹仍由到速、电机健康和整车许可共同限制。
 */
static bool ShootManualBurstRequested(const Vt13ControlInput *input)
{
    return Vt13ManualBurst(&shoot_control, input);
}

/**
 * @brief 判断当前发射机构是否允许驱动拨弹。
 * @note 必须已使能、摩擦轮已开启、三个电机在线且两摩擦轮有限转速均达到阈值。
 */
static bool ShootCanFeed(void)
{
    return shoot_enabled && SHOOT.fric_flag &&
           !SHOOT.fric_motor[0].offline && !SHOOT.fric_motor[1].offline &&
           !SHOOT.trigger_motor.offline &&
           isfinite(SHOOT.FDB.fric_speed_fdb_R) && isfinite(SHOOT.FDB.fric_speed_fdb_L) &&
           fabsf(SHOOT.FDB.fric_speed_fdb_R) >= FRIC_SPEED_LIMIT &&
           fabsf(SHOOT.FDB.fric_speed_fdb_L) >= FRIC_SPEED_LIMIT;
}
#else
static ControlInput_t shoot_input;
#endif


uint8_t fric_ui;
fp32 delta;

/*-------------------- 初始化 --------------------*/

/**
 * @brief          初始化
 * @param[in]      无
 * @retval         无
 */
void ShootInit(void) 
{ 
    //获取遥控器指针
    SHOOT.rc = get_remote_control_point(); 

    //摩擦轮相关
    MotorInit(&SHOOT.fric_motor[0],FRIC_MOTOR_R_ID, FRIC_MOTOR_R_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化R摩擦轮电机结构体
    MotorInit(&SHOOT.fric_motor[1],FRIC_MOTOR_L_ID, FRIC_MOTOR_L_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化L摩擦轮电机结构体

    const fp32 pid_fric[3] = {FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD};//摩擦轮速度环

    PID_init(&SHOOT.fric_pid[0], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&SHOOT.fric_pid[1], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);//摩擦轮初始化pid

    //拨弹盘相关
    MotorInit(&SHOOT.trigger_motor,TRIGGER_MOTOR_ID, TRIGGER_MOTOR_CAN, TRIGGER_MOTOR_TYPE, 1, 1.0f, 0);//初始化拨弹盘电机结构体
    if (TRIGGER_MOTOR_TYPE == DJI_M2006)
    {
        const fp32 pid_angel_trigger[3] = {TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};//拨弹盘角度环
        const fp32 pid_speed_trigger[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};//拨弹盘速度环

        PID_init(&SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT, TRIGGER_ANGEL_PID_MAX_IOUT);
        PID_init(&SHOOT.trigger_speed_pid, PID_POSITION, pid_speed_trigger, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);  //拨弹盘初始化pid
    }
    else if (TRIGGER_MOTOR_TYPE == DM_4310)
    {
        const fp32 pid_angel_trigger[3] = {TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};//拨弹盘角度环

        PID_init(&SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT, TRIGGER_ANGEL_PID_MAX_IOUT); //拨弹盘初始化pid
    }
 
}

/*-------------------- 设置模式 --------------------*/

/**
 * @brief          设置模式
 * @param[in]      无
 * @retval         无
 */
void ShootSetMode(void)
{
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
    bool manual_request, auto_request, auto_target;
    Vt13ReadControl(&shoot_input);
    GimbalReadControlStatus(&shoot_gimbal);
    /* 只有有效 N/S 输入与未超时的已就绪云台状态才能使能发射任务。 */
    shoot_enabled = Vt13ControlEnabled(&shoot_input) &&
                    Vt13StatusReady(&shoot_gimbal, HAL_GetTick());
    /* 整车或云台不允许工作时清除开轮锁存、发射状态和电流，并停止本拍计算。 */
    if (!shoot_enabled) {
        ShootForceDisable();
        return;
    }
    ShootUpdateFrictionSwitch(&shoot_input, shoot_enabled);
    auto_target = Vt13AutoAimRequested(&shoot_input) && shoot_gimbal.auto_active;
    /* N 右键/S 档有效自瞄沿用自动开轮；左键仅许可拨弹，不取消预热。 */
    SHOOT.fric_flag = auto_target || shoot_control.friction_on;
    SHOOT.state = SHOOT.fric_flag ? FRIC_READY : FRIC_NOT_READY;
    manual_request = ShootManualBurstRequested(&shoot_input);
    auto_request = Vt13AutoFireRequested(&shoot_input, &shoot_gimbal) && GetScCmdFire();
    /* 有效自瞄时拨弹需要视觉 fire 与左键，任一撤销后拨轮都不能续发。 */
    if (auto_target) manual_request = false;
    SHOOT.move_flag = 0;
    /* 全部机构许可通过，且手动或视觉至少一路请求开火时才进入连发。 */
    if (ShootCanFeed() && (manual_request || auto_request)) SHOOT.mode = LOAD_BURSTFIRE;
    /* 未通过机构许可或没有开火请求时，本拍停止拨弹并清除堵转计时。 */
    else {
        SHOOT.mode = LOAD_STOP;
        SHOOT.block_time = SHOOT.reverse_time = 0;
    }
#else
    bool fric_allowed, armed, vision_auto, ready;
    ControlInputRead(&shoot_input);
    /* 右拨杆中/下档均自动开摩擦轮，左拨杆各档位均允许开轮。 */
    fric_allowed = shoot_input.valid && GimbalAllowsShoot() &&
                   (switch_is_mid(shoot_input.sw[SHOOT_MODE_CHANNEL]) ||
                    switch_is_down(shoot_input.sw[SHOOT_MODE_CHANNEL]));
    SHOOT.fric_flag = fric_allowed;
    SHOOT.state = fric_allowed ? FRIC_READY : FRIC_NOT_READY;
    /* 左上档禁止拨弹；只有右下档允许视觉控制拨弹盘。 */
    armed = fric_allowed && !switch_is_up(shoot_input.sw[1]);
    vision_auto = armed && switch_is_down(shoot_input.sw[SHOOT_MODE_CHANNEL]);
    ready = armed && SHOOT.state == FRIC_READY &&
            !SHOOT.fric_motor[0].offline && !SHOOT.fric_motor[1].offline &&
            !SHOOT.trigger_motor.offline &&
            fabsf(SHOOT.FDB.fric_speed_fdb_R) >= FRIC_SPEED_LIMIT &&
            fabsf(SHOOT.FDB.fric_speed_fdb_L) >= FRIC_SPEED_LIMIT;

    /* 退出下档或通信许可失效时立即取消拨弹，不能延续旧请求。 */
    SHOOT.move_flag = 0;
    if (!ready || !vision_auto || GetUsbStatus() || !GetScCmdFricOn()) {
        SHOOT.mode = LOAD_STOP;
        SHOOT.block_time = SHOOT.reverse_time = 0;
    } else {
        /* 沿用原下档视觉火控判断，外围联锁保持。 */
        if (GetScCmdFire())
        {
            SHOOT.mode = LOAD_BURSTFIRE;
        }
        else
        {
            SHOOT.mode = LOAD_STOP;
        }
    }

#endif

    /* 保留原有堵转反转处理。 */
    if (SHOOT.mode == LOAD_BURSTFIRE || (SHOOT.mode == LAOD_BULLET && SHOOT.move_flag))
    {
        if(SHOOT.block_time >= BLOCK_TIME)
        {
            SHOOT.mode = LOAD_BLOCK;
            SHOOT.move_flag = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        }
  
        if(fabs(SHOOT.last_trigger_vel)<BLOCK_TRIGGER_SPEED&&SHOOT.block_time<BLOCK_TIME)
        {
            SHOOT.block_time++;
            SHOOT.reverse_time = 0;
        }
        else if(SHOOT.block_time== BLOCK_TIME&& SHOOT.reverse_time< REVERSE_TIME)
        {
            SHOOT.reverse_time++;  
        }
        else
        {
            SHOOT.block_time = 0;
        }  
    }    
    else
    {
        SHOOT.block_time = 0;
        SHOOT.reverse_time = 0;
    }
}

/*-------------------- 更新反馈 --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      无
 * @retval         无
 */
void ShootObserver(void) 
{
    GetMotorMeasure(&SHOOT.trigger_motor);
    GetMotorMeasure(&SHOOT.fric_motor[0]);
    GetMotorMeasure(&SHOOT.fric_motor[1]);

    SHOOT.FDB.fric_speed_fdb_R = SHOOT.fric_motor[0].fdb.vel;
    SHOOT.FDB.fric_speed_fdb_L = SHOOT.fric_motor[1].fdb.vel;

    SHOOT.FDB.trigger_speed_fdb = SHOOT.trigger_motor.fdb.vel;

    if (TRIGGER_MOTOR_TYPE == DJI_M2006)
    {
        if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
        {
            SHOOT.ecd_count--;
        }
        else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
        {
            SHOOT.ecd_count++;
        }

        if (SHOOT.ecd_count == FULL_COUNT)
        {
            SHOOT.ecd_count = -(FULL_COUNT - 1);
        }
        else if (SHOOT.ecd_count == -FULL_COUNT)
        {
            SHOOT.ecd_count = FULL_COUNT-1;
        }
        //计算输出轴角度
        SHOOT.FDB.trigger_angel_fdb = (SHOOT.ecd_count * ECD_RANGE + SHOOT.trigger_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

        //记录上一个ecd值
        SHOOT.last_ecd = SHOOT.trigger_motor.fdb.ecd;
    }
  //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
  //if(FULL_COUNT%2 == 0)
  //{
    // if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
    // {
    //     SHOOT.ecd_count--;
    // }
    // else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
    // {
        
    //     SHOOT.ecd_count++;
    // }

    // if (SHOOT.ecd_count == FULL_COUNT)
    // {
    //     SHOOT.ecd_count = -(FULL_COUNT - 1);
    // }
    // else if (SHOOT.ecd_count == -FULL_COUNT)
    // {
    //     SHOOT.ecd_count = FULL_COUNT-1;
    // }
  //}
  // //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 51圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
  // else
  // {
  //   if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
  //   {
  //       SHOOT.ecd_count--;
  //   }
  //   else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
  //   {
        
  //       SHOOT.ecd_count++;
  //   }

  //   if (SHOOT.ecd_count == FULL_COUNT)
  //   {
  //       SHOOT.ecd_count = -FULL_COUNT;
  //   }
  //   else if (SHOOT.ecd_count == -FULL_COUNT)
  //   {
  //       SHOOT.ecd_count = FULL_COUNT;
  //   }
  //}

  
    // else if (TRIGGER_MOTOR_TYPE == DM_4310)
    // {
    //     SHOOT.FDB.trigger_angel_fdb = theta_format(SHOOT.trigger_motor.fdb.pos);
    // }
  
    //记录上一个拨弹盘vel,用于堵转模式判断
    SHOOT.last_trigger_vel = SHOOT.trigger_motor.fdb.vel;

    //记录上一个摩擦轮vel,用于过热保护
    SHOOT.last_fric_vel = SHOOT.fric_motor[0].fdb.vel;
    
}

/*-------------------- 更新目标量 --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      无
 * @retval         无
 */
void ShootReference(void) 
{
  switch (SHOOT.state)
  {
  case FRIC_NOT_READY:
  SHOOT.REF.fric_speed_ref_R=0.0f;
  SHOOT.REF.fric_speed_ref_L=0.0f;
  break;

  case FRIC_READY:
  SHOOT.REF.fric_speed_ref_R=FRIC_R_SPEED;
  SHOOT.REF.fric_speed_ref_L=FRIC_L_SPEED;
  break;
  
  default:
  break;
  }

  switch (SHOOT.mode)
  {
  case LOAD_STOP:
  SHOOT.REF.trigger_speed_ref=0.0f;
  SHOOT.move_flag = 0;
  break;
  
  case LAOD_BULLET:
  if (TRIGGER_MOTOR_TYPE == DJI_M2006)
  {
    if (!SHOOT.move_flag &&
        (!single_shot_started || HAL_GetTick() - last_single_shot_time >= SINGLE_SHOT_INTERVAL_MS))
    {
      SHOOT.REF.trigger_angel_ref = theta_format(SHOOT.FDB.trigger_angel_fdb + 2*PI/BULLET_NUM/TRIGGER_REDUCTION_RATIO);
      last_single_shot_time = HAL_GetTick();
      single_shot_started = 1;
      SHOOT.move_flag = 1;
    }

    if (SHOOT.move_flag &&
        fabsf(theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb)) <= 0.01f)
    {
      SHOOT.move_flag = 0;
    }
  }
  else if (TRIGGER_MOTOR_TYPE == DM_4310)
  {
    if (SHOOT.move_flag == 0)
    {
      SHOOT.REF.trigger_angel_ref = theta_format(SHOOT.FDB.trigger_angel_fdb - 2*PI/BULLET_NUM/TRIGGER_REDUCTION_RATIO);
    }

    if (theta_format(SHOOT.FDB.trigger_angel_fdb - SHOOT.REF.trigger_angel_ref) > 0.01f)
    {
      SHOOT.move_flag = 1;
    }
    else
    {
      SHOOT.move_flag = 0;
    }
  }
  break;

  case LOAD_BURSTFIRE:
  SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED;
  break;

  case LOAD_BLOCK:
  SHOOT.REF.trigger_speed_ref = REVERSE_SPEED;
  break;

  default:
    break;
  }
  

  
}

/*-------------------- 计算控制量 --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      无
 * @retval         无
 */
void ShootConsole(void) 
{
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
  /* 整车或云台不允许工作时清除开轮锁存、发射状态和电流，并停止本拍计算。 */
  if (!shoot_enabled) {
      ShootForceDisable();
      return;
  }
#endif

  SHOOT.fric_motor[0].set.curr= PID_calc(&SHOOT.fric_pid[0], SHOOT.FDB.fric_speed_fdb_R,SHOOT.REF.fric_speed_ref_R);
  SHOOT.fric_motor[1].set.curr= PID_calc(&SHOOT.fric_pid[1], SHOOT.FDB.fric_speed_fdb_L,SHOOT.REF.fric_speed_ref_L);

  if (TRIGGER_MOTOR_TYPE == DJI_M2006)
  {
    if (SHOOT.mode == LOAD_STOP)
    {
        SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
    }
    else if (SHOOT.mode == LOAD_BURSTFIRE)
    {
        SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
    }
    else if (SHOOT.mode == LAOD_BULLET)
    {
        delta = theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb);

        SHOOT.REF.trigger_speed_ref = PID_calc(&SHOOT.trigger_angel_pid,0,delta);
        SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid,SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
    }
    else if (SHOOT.mode == LOAD_BLOCK) 
    {
        SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
    }
  }
  // else if (TRIGGER_MOTOR_TYPE == DM_4310)
  // {
  //   if (SHOOT.mode == LOAD_STOP)
  //   {
  //       SHOOT.trigger_motor.set.vel = SHOOT.REF.trigger_speed_ref;
  //   }
  //   else if (SHOOT.mode == LOAD_BURSTFIRE)
  //   {
  //       SHOOT.trigger_motor.set.vel = SHOOT.REF.trigger_speed_ref;
  //   }
  //   else if (SHOOT.mode == LAOD_BULLET)
  //   {
  //       delta = theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb);
  //       SHOOT.trigger_motor.set.vel = PID_calc(&SHOOT.trigger_angel_pid,0,delta);
  //   }
  //   else if (SHOOT.mode == LOAD_BLOCK) 
  //   {
  //       SHOOT.trigger_motor.set.vel = SHOOT.REF.trigger_speed_ref;
  //   }
  // }
  
}

/*-------------------- 发送控制量 --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      无
 * @retval         无
 */
void ShootSendCmd(void) 
{
/* VT13 控制源使用新映射与安全门控；其他控制源保留旧路径。 */
#if ROBOT_REMOTE_SOURCE == ROBOT_REMOTE_VT13
  Vt13ControlInput latest;
  GimbalControlStatus status;
  VisionControlInput vision;
  Vt13ReadControl(&latest);
  GimbalReadControlStatus(&status);
  /* 发送前再次检查 C 档、失联和云台时效，不依赖之前计算的使能状态。 */
  if (!Vt13ControlEnabled(&latest) || !Vt13StatusReady(&status, HAL_GetTick())) {
      ShootForceDisable();
  } else {
      /* 手动请求不依赖 USB；自动请求还需最新视觉快照允许。 */
      bool auto_target = Vt13AutoAimRequested(&latest) && status.auto_active;
      bool manual_request = ShootManualBurstRequested(&latest);
      /* 有效自瞄只接受视觉与左键共同许可，拨轮和手动左键均不能覆盖视觉停火。 */
      if (auto_target) manual_request = false;
      /* 普通关轮只将速度目标归零，保留速度环电流用于制动和零速控制。 */
      if (!auto_target && !shoot_control.friction_on) {
          bool target_changed = SHOOT.REF.fric_speed_ref_R != 0.0f || SHOOT.REF.fric_speed_ref_L != 0.0f;
          SHOOT.fric_flag = 0;
          SHOOT.state = FRIC_NOT_READY;
          SHOOT.REF.fric_speed_ref_R = SHOOT.REF.fric_speed_ref_L = 0.0f;
          /* 发送前才退出自瞄时按零速重算，避免继续发送原开轮目标的驱动电流。 */
          if (target_changed) {
              SHOOT.fric_motor[0].set.curr = PID_calc(&SHOOT.fric_pid[0], SHOOT.FDB.fric_speed_fdb_R, 0.0f);
              SHOOT.fric_motor[1].set.curr = PID_calc(&SHOOT.fric_pid[1], SHOOT.FDB.fric_speed_fdb_L, 0.0f);
          }
      }
      VisionReadControl(&vision);
      /* 电机/到速许可失效或两路开火请求都撤销时，立即清零拨弹输出并取消堵转残留。 */
      if (!ShootCanFeed() || !(manual_request ||
          (Vt13AutoFireRequested(&latest, &status) && vision.valid && vision.fire))) {
          SHOOT.mode = LOAD_STOP;
          SHOOT.move_flag = 0;
          SHOOT.block_time = SHOOT.reverse_time = 0;
          SHOOT.REF.trigger_speed_ref = 0.0f;
          SHOOT.trigger_motor.set.curr = 0.0f;
          PID_clear(&SHOOT.trigger_speed_pid);
      }
  }
#endif

  if (TRIGGER_MOTOR_TYPE == DJI_M2006)
  {

    CanCmdDjiMotor(FRIC_MOTOR_R_CAN, STD_ID , SHOOT.fric_motor[0].set.curr,SHOOT.fric_motor[1].set.curr,0, 0);
    CanCmdDjiMotor(TRIGGER_MOTOR_CAN, 0x1FF , 0, SHOOT.trigger_motor.set.curr, 0, 0);
  }
  // else if (TRIGGER_MOTOR_TYPE == DM_4310)
  // {
  //   if (SHOOT.trigger_motor.fdb.state == DM_STATE_DISABLE)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
  //   {
  //     DmEnable(&SHOOT.trigger_motor);
  //   }
  //   DmMitCtrlVelocity(&SHOOT.trigger_motor, TRIGGER_SPEED_MIT_KD);
  
  //   CanCmdDjiMotor(FRIC_MOTOR_R_CAN, STD_ID ,0 ,SHOOT.fric_motor[1].set.curr,SHOOT.fric_motor[0].set.curr, 0);
  // }
  vofa_shoot_update(&SHOOT);
}

#endif  // 发射机构类型为摩擦轮加拨弹盘
