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
#include "IMU.h"  // 添加IMU头文件，用于获取稳定的陀螺仪数据
#include "CAN_cmd_SupCap.h"  // 添加超级电容控制头文件，解决CanCmdSupCap函数声明问题
#include "SupCap.h"//超级电容

Chassis_s chassis;
PID_t chassis_pid;

// 跟随模式下实际输出给 reference.wz 的平滑值，用于做斜坡限幅。
//static fp32 chassis_follow_wz_smoothed = 0.0f;


/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    //step1 获取遥控器指针
    chassis.rc = get_remote_control_point();

    //step2 PID数据清零，设置PID参数
    const static fp32 wheel_vel[3]={KP_OMNI_VEL,KI_OMNI_VEL,KD_OMNI_VEL};

    const static fp32 plain_vel[3]={KP_OMNI_PLAIN_VEL,KI_OMNI_PLAIN_VEL,KD_OMNI_PLAIN_VEL};
    const static fp32 plain_vel_z[3]={KP_OMNI_PLAIN_VEL,KI_OMNI_PLAIN_VEL, 2.0f};

    const static fp32 gimbal_follow[3]={KP_CHASSIS_FOLLOW_GIMBAL,KI_CHASSIS_FOLLOW_GIMBAL,KD_CHASSIS_FOLLOW_GIMBAL};

    // 初始化底盘平动速度环PID
    PID_init(&chassis.vel_x_pid, PID_POSITION, plain_vel, MAX_OUT_OMNI_PLAIN_VEL, MAX_IOUT_OMNI_PLAIN_VEL);
    PID_init(&chassis.vel_y_pid, PID_POSITION, plain_vel, MAX_OUT_OMNI_PLAIN_VEL, MAX_IOUT_OMNI_PLAIN_VEL);
    PID_init(&chassis.omega_pid, PID_POSITION, plain_vel_z, MAX_OUT_OMNI_PLAIN_VEL, MAX_IOUT_OMNI_PLAIN_VEL);

    // 初始化全向轮速度环PID
    for (int i=0;i<4;++i)
    {
        PID_init(&chassis.wheel_vel[i],PID_POSITION,wheel_vel,MAX_OUT_OMNI_VEL,MAX_IOUT_OMNI_VEL);
    }

    // 初始化云台跟随PID
    PID_init(&chassis_pid.follow,PID_POSITION,gimbal_follow,MAX_OUT_CHASSIS_FOLLOW_GIMBAL,MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);

    //step3 初始化电机
    MotorInit(&chassis.wheel[0], WHEEL_1_ID, WHEEL_1_CAN, WHEEL_1_MOTOR_TYPE, WHEEL_1_DIRECTION, WHEEL_1_RATIO, WHEEL_1_MODE);
    MotorInit(&chassis.wheel[1], WHEEL_2_ID, WHEEL_2_CAN, WHEEL_2_MOTOR_TYPE, WHEEL_2_DIRECTION, WHEEL_2_RATIO, WHEEL_2_MODE);
    MotorInit(&chassis.wheel[2], WHEEL_3_ID, WHEEL_3_CAN, WHEEL_3_MOTOR_TYPE, WHEEL_3_DIRECTION, WHEEL_3_RATIO, WHEEL_3_MODE);
    MotorInit(&chassis.wheel[3], WHEEL_4_ID, WHEEL_4_CAN, WHEEL_4_MOTOR_TYPE, WHEEL_4_DIRECTION, WHEEL_4_RATIO, WHEEL_4_MODE);

    /*-------------------- 初始化超级电容 --------------------*/
    SupCapInit(&chassis.super_cap, 1);  // 初始化超级电容，使用CAN1

    /*-------------------- 初始化功率控制 --------------------*/
    chassis_power_control_init(&chassis);  // 初始化功率控制系统

    // 发送初始化超级电容控制命令
    // 初始化时从裁判系统获取缓冲能量数据
    // fp32 init_power, init_buffer;
    // get_chassis_power_and_buffer(&init_power, &init_buffer);
    // CanCmdSupCap(1, (uint16_t)init_buffer, 50, 1);  // 使用裁判系统的实际缓冲能量

    //step4 初始模式设置
    chassis.mode = CHASSIS_LOCK;
    chassis.spin_mode_last = CHASSIS_LOCK;
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
    //更新电机反馈数据
    for (int i=0;i<4;++i)
    {
        GetMotorMeasure(&chassis.wheel[i]);
        chassis.feedback[i] = chassis.wheel[i].fdb.vel;
    }

    // fdb.vel是电机侧角速度，换算到底盘运动时需要除以减速比
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
    GetSupCapMeasure(&chassis.super_cap);

    // 使用云台相对底盘的偏航差，供跟随和小陀螺模式使用。
    chassis.yaw_delta = GetGimbalDeltaYawMid();
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
			chassis.P_wz = 0 ;
				for(int i=0;i<4;++i)
				{
					chassis.feedback[i] = 0.0f;
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
                // 刚进入跟随模式时清掉上一模式留下的平滑角速度，防止切模式瞬间给出突变 wz。
                //chassis_follow_wz_smoothed = 0.0f;
            }

            chassis.reference_rc.vx = fp32_deadline(chassis.rc->rc.ch[3], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED * 2.0f;
            chassis.reference_rc.vy = -fp32_deadline(chassis.rc->rc.ch[2], -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE) / CHASSIS_RC_MAX_RANGE * CHASSIS_RC_MAX_SPEED * 2.0f;

            // 将云台坐标系下的平移指令转到底盘坐标系下。
            chassis.sin_yaw = sinf(chassis.yaw_delta);
            chassis.cos_yaw = cosf(chassis.yaw_delta);
            chassis.reference.vx = chassis.reference_rc.vx * chassis.cos_yaw - chassis.reference_rc.vy * chassis.sin_yaw;
            chassis.reference.vy = chassis.reference_rc.vx * chassis.sin_yaw + chassis.reference_rc.vy * chassis.cos_yaw;

            // 中心附近给一个小死区，避免底盘和云台在零点附近来回摆动。
            // 跟随停止条件同时看角度和角速度：
            // 1) yaw_delta 小，说明底盘方向已经接近云台中位；
            // 2) P_wz 小，说明底盘本身也快停住了。
            // 只看角度会导致底盘高速冲进死区后被强行置零，随后靠惯性越过中位并反向摆动。
            if (fabsf(chassis.yaw_delta) < 0.03f &&
                fabsf(chassis.P_wz) < CHASSIS_FOLLOW_DEAD_WZ)
            {
                PID_clear(&chassis_pid.follow);
                // 静止条件满足后同步清平滑量，下一次偏离时从 0 平滑起步。
                //chassis_follow_wz_smoothed = 0.0f;
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

            // 小陀螺时保持平移方向相对云台一致。
            chassis.sin_yaw = sinf(chassis.yaw_delta);
            chassis.cos_yaw = cosf(chassis.yaw_delta);
            chassis.reference.vx = 2.0f * (chassis.reference_rc.vx * chassis.cos_yaw + chassis.reference_rc.vy * chassis.sin_yaw);
            chassis.reference.vy = 2.0f * (chassis.reference_rc.vx * chassis.sin_yaw - chassis.reference_rc.vy * chassis.cos_yaw);

            if (chassis.spin_mode_last != CHASSIS_SPIN) 
            {
                chassis.wz_set_smoothed = 0.0f;
                chassis.spin_acceleration_limit = 10.0f;
            }

            if (fabsf(chassis.reference_rc.vx) > 0.4f || fabsf(chassis.reference_rc.vy) > 0.4f)
            {
                chassis.spin_target_wz = 4.5f + robot_status.robot_level * 0.5f;
            }
            else
            {
                chassis.spin_target_wz = 6.5f + robot_status.robot_level * 0.6f;
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

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
     // 力控链路到DJI电流指令的标定系数（先给可运行初值，后续按实车再微调）
    //const fp32 FORCE_TO_CURRENT_K = 80.0f;
    //const fp32 SPEED_TO_CURRENT_K = 10.0f;

    chassis.set[0] = (( chassis.reference.vy - chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;
    chassis.set[1] = ((-chassis.reference.vy - chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;
    chassis.set[2] = ((-chassis.reference.vy + chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;
    chassis.set[3] = (( chassis.reference.vy + chassis.reference.vx) / sqrt(2) - WHEEL_CENTER_DISTANCE * chassis.reference.wz) / WHEEL_RADIUS * REDUCTION_RATIO;

    //chassis.set[0] = ((chassis.reference.vx - chassis.reference.wz*(WHEEL_CENTER_DISTANCE/sqrt(2))) * cos(135.0f*M_PI/180.0f) + (chassis.reference.vy - chassis.reference.wz*(WHEEL_CENTER_DISTANCE/sqrt(2))) * sin(135.0f*M_PI/180.0f))/WHEEL_RADIUS * 15.76f;

   
    //动力学逆解算
    //底盘目标牵引力
    chassis.force_x = PID_calc(&chassis.vel_x_pid, chassis.P_vx, chassis.reference.vx);
    chassis.force_y = PID_calc(&chassis.vel_y_pid, chassis.P_vy, chassis.reference.vy);
    //底盘目标转矩
    chassis.torque_wz = PID_calc(&chassis.omega_pid, chassis.P_wz, chassis.reference.wz);

    //计算轮向电机扭矩  
    // 将底盘期望合力(force_x, force_y)和期望自旋力矩(torque_wz)分配到4个全向轮
    // wheel_force[i] 表示第 i 个轮子切向方向上的等效驱动力（后续再映射为电流指令）
    for (int i = 0; i < 4; ++i)
    {
        // 4个轮子几何方向不同，所以同一底盘合力要按各轮方向投影
        // 这里使用 45°全向轮分解关系（1/sqrt(2)）+ 自旋项（与轮到中心距离成反比）
        switch (i)
        {
            case 0:
                // 0号轮： (Fy - Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = ( chassis.force_y - chassis.force_x) / sqrtf(2.0f) - chassis.torque_wz * WHEEL_CENTER_DISTANCE;
                break;
            case 1:
                // 1号轮： (-Fy - Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = (-chassis.force_y - chassis.force_x) / sqrtf(2.0f) - chassis.torque_wz * WHEEL_CENTER_DISTANCE;
                break;
            case 2:
                // 2号轮： (-Fy + Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = (-chassis.force_y + chassis.force_x) / sqrtf(2.0f) - chassis.torque_wz * WHEEL_CENTER_DISTANCE;
                break;
            default:
                // 3号轮： (Fy + Fx)/sqrt(2) + 自旋需求
                chassis.wheel_force[i] = ( chassis.force_y + chassis.force_x) / sqrtf(2.0f) - chassis.torque_wz * WHEEL_CENTER_DISTANCE;
                break;
        }

        // 电流指令由两部分构成：
        // 1) 主力矩项：wheel_force * WHEEL_RADIUS
        // 2) 速度差补偿项：target_wheel_speed - feedback_wheel_speed
         chassis.wheel[i].set.curr = 80.0f * (chassis.wheel_force[i] * 0.08f) + (chassis.set[i] - chassis.feedback[i]);

    //chassis.wheel[i].set.curr = PID_calc(&chassis.wheel_vel[i], chassis.feedback[i], chassis.set[i]);
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
    // 发送顺序必须与电机ID(1~4)一致，omni参数中轮子索引与ID存在映射关系:
    CanCmdDjiMotor(
        CHASSIS_CAN, CHASSIS_STDID, *g_chassis_power_control.motor_current_cmd[0],
        *g_chassis_power_control.motor_current_cmd[1],
        *g_chassis_power_control.motor_current_cmd[2],
        *g_chassis_power_control.motor_current_cmd[3]);
    //CanCmdDjiMotor(CHASSIS_CAN, CHASSIS_STDID, chassis.wheel[0].set.curr, chassis.wheel[1].set.curr, chassis.wheel[2].set.curr, chassis.wheel[3].set.curr);

        // 发送超级电容控制命令（测试用）
    //    fp32 current_power, current_buffer;
    //    get_chassis_power_and_buffer(&current_power, &current_buffer);  // 从裁判系统获取实时数据

    //   uint16_t power_buffer = (uint16_t)current_buffer;  // 从裁判系统获取的当前缓冲能量
    //   uint16_t power_limit = 40;                         // 功率限制值，可根据比赛规则调整
    //   uint8_t automode_stage = 1;                        // 启用超级电容

    //   CanCmdSupCap(1, power_buffer, power_limit, automode_stage);
}

#endif
