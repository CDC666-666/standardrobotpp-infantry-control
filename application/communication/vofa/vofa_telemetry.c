#include "vofa_telemetry.h"

#if VOFA_ENABLE
#include "gimbal_yaw_pitch_direct.h"
#include "vofa.h"
#include "vofa_observe.h"

// 设为 1 时发送链路检查数据，默认发送 yaw 观测数据。
#ifndef VOFA_YAW_LINK_TEST
#define VOFA_YAW_LINK_TEST 0
#endif

static ShootObservation shoot_obs;
static ChassisObservation chassis_obs;
#if CHASSIS_TYPE == CHASSIS_OMNI_WHEEL
static GimbalYawObservation yaw_obs;
#endif

void vofa_telemetry_update(uint32_t now)
{
    (void)now;
    VofaService();
    ShootReadObservation(&shoot_obs);
    ChassisReadObservation(&chassis_obs);

#if CHASSIS_TYPE == CHASSIS_OMNI_WHEEL
    GimbalReadYawObservation(&yaw_obs);
#if VOFA_YAW_LINK_TEST && GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT
    // I0..I3：毫秒时间、云台模式、yaw 电机离线标志、已发送帧数。
    VOFA_SEND((float)now, (float)gimbal_direct.mode,
        (float)gimbal_direct.yaw.offline, (float)vofa_stats.completed);
#else
    // 取同一云台控制周期的观测值与参数中的机械中值。
    VOFA_SEND(yaw_obs.time_ms,       // I0: 控制周期时间戳，ms
        yaw_obs.mode,               // I1: 当前模式（1=IMU，5=AUTO_AIM）
        yaw_obs.theta_ref_rad,      // I2: yaw 位置参考，rad
        yaw_obs.theta_rad,          // I3: 控制器使用的 IMU yaw 角，rad
        yaw_obs.omega_imu_rad_s,    // I4: IMU yaw 角速度，rad/s
        yaw_obs.omega_motor_rad_s,  // I5: DM 编码器角速度，rad/s
        yaw_obs.tau_cmd_nm,         // I6: 最终力矩指令，N*m
        yaw_obs.tau_meas_nm,        // I7: DM 力矩反馈，N*m
        GIMBAL_DIRECT_YAW_MID,      // I8: 配置的 yaw 电机中值，rad
        yaw_obs.theta_motor_rad    // I9: 方向映射后的 DM 编码器角度，rad
    );
#endif
#endif
}
#endif
