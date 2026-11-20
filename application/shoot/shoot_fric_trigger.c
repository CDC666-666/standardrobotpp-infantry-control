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
static ControlInput_t shoot_input;


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
