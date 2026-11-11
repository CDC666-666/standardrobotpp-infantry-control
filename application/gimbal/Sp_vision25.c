/**
 * @file Sp_vision25.c
 * @brief AIM-only cascade PID plus trajectory feedforward from d49991a.
 * See Sp_vision25.h for parameters, timing and pending integration details.
 */
#include "Sp_vision25.h"

#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#include "chassis.h"

static PID_t sp_vision25_pid;
static fp32 sp_vision25_yaw_acc_filtered = 0.0f;
static fp32 sp_vision25_pitch_acc_filtered = 0.0f;
static bool sp_vision25_initialized = false;
static bool sp_vision25_active = false;

static void SpVision25_ResetYaw(void)
{
    PID_clear(&sp_vision25_pid.yaw_angle);
    PID_clear(&sp_vision25_pid.yaw_velocity);
    sp_vision25_yaw_acc_filtered = 0.0f;
}

static void SpVision25_ResetPitch(void)
{
    PID_clear(&sp_vision25_pid.pitch_angle);
    PID_clear(&sp_vision25_pid.pitch_velocity);
    sp_vision25_pitch_acc_filtered = 0.0f;
}

void SpVision25_Reset(void)
{
    SpVision25_ResetYaw();
    SpVision25_ResetPitch();
    sp_vision25_active = false;
}

void SpVision25_Init(void)
{
    static const fp32 yaw_angle[3] = {
        SP_VISION25_KP_YAW_ANGLE, SP_VISION25_KI_YAW_ANGLE, SP_VISION25_KD_YAW_ANGLE};
    static const fp32 yaw_velocity[3] = {
        SP_VISION25_KP_YAW_VELOCITY, SP_VISION25_KI_YAW_VELOCITY,
        SP_VISION25_KD_YAW_VELOCITY};
    static const fp32 pitch_angle[3] = {
        SP_VISION25_KP_PITCH_ANGLE, SP_VISION25_KI_PITCH_ANGLE,
        SP_VISION25_KD_PITCH_ANGLE};
    static const fp32 pitch_velocity[3] = {
        SP_VISION25_KP_PITCH_VELOCITY, SP_VISION25_KI_PITCH_VELOCITY,
        SP_VISION25_KD_PITCH_VELOCITY};

    PID_init(&sp_vision25_pid.yaw_angle, PID_POSITION, yaw_angle,
             SP_VISION25_MAX_OUT_YAW_ANGLE, SP_VISION25_MAX_IOUT_YAW_ANGLE);
    PID_init(&sp_vision25_pid.yaw_velocity, PID_POSITION, yaw_velocity,
             SP_VISION25_MAX_OUT_YAW_VELOCITY, SP_VISION25_MAX_IOUT_YAW_VELOCITY);
    PID_init(&sp_vision25_pid.pitch_angle, PID_POSITION, pitch_angle,
             SP_VISION25_MAX_OUT_PITCH_ANGLE, SP_VISION25_MAX_IOUT_PITCH_ANGLE);
    PID_init(&sp_vision25_pid.pitch_velocity, PID_POSITION, pitch_velocity,
             SP_VISION25_MAX_OUT_PITCH_VELOCITY, SP_VISION25_MAX_IOUT_PITCH_VELOCITY);
    sp_vision25_initialized = true;
    SpVision25_Reset();
}

bool SpVision25_Reference(Gimbal_s * gimbal)
{
    if (gimbal == NULL || gimbal->mode != GIMBAL_AUTO_AIM) {
        SpVision25_Reset();
        return false;
    }
    if (!sp_vision25_initialized) {
        SpVision25_Init();
    }

    if (!sp_vision25_active || gimbal->last_mode != GIMBAL_AUTO_AIM) {
        gimbal->reference.yaw = gimbal->feedback_pos.yaw;
        gimbal->reference.pitch = fp32_constrain(
            gimbal->feedback_pos.pitch,
            GIMBAL_LOWER_LIMIT_PITCH + gimbal->angle_zero_for_imu,
            GIMBAL_UPPER_LIMIT_PITCH + gimbal->angle_zero_for_imu);
    } else {
        /* World yaw: use the nearest equivalent angle around current IMU yaw. */
        gimbal->reference.yaw = gimbal->feedback_pos.yaw + loop_fp32_constrain(
            GetScCmdGimbalAngle(AX_YAW) - gimbal->feedback_pos.yaw, -M_PI, M_PI);
        gimbal->reference.pitch = fp32_constrain(
            -GetScCmdGimbalAngle(AX_PITCH),
            GIMBAL_LOWER_LIMIT_PITCH + gimbal->angle_zero_for_imu,
            GIMBAL_UPPER_LIMIT_PITCH + gimbal->angle_zero_for_imu);
    }
    return true;
}

bool SpVision25_Console(Gimbal_s * gimbal)
{
    bool allow_plan_feedforward;
    if (gimbal == NULL || gimbal->mode != GIMBAL_AUTO_AIM) {
        SpVision25_Reset();
        return false;
    }
    if (!sp_vision25_initialized) {
        SpVision25_Init();
    }

    allow_plan_feedforward = sp_vision25_active && gimbal->last_mode == GIMBAL_AUTO_AIM;
    if (!allow_plan_feedforward) {
        SpVision25_Reset();
    }

    if (gimbal->pitch.offline) {
        SpVision25_ResetPitch();
        gimbal->pitch.set.vel = 0.0f;
        gimbal->pitch.set.curr = 0.0f;
    } else {
        fp32 pitch_acc_current_ff = 0.0f;
        fp32 pitch_vel_cmd;
        fp32 pitch_velocity_current;

        if (allow_plan_feedforward) {
            fp32 plan_pitch_acc = fp32_constrain(
                GetScCmdGimbalAcceleration(AX_PITCH),
                -SP_VISION25_MAX_PITCH_PLAN_ACC, SP_VISION25_MAX_PITCH_PLAN_ACC);
            /* Preserve the original acceleration NaN handling and negative sign. */
            if (plan_pitch_acc != plan_pitch_acc) {
                plan_pitch_acc = 0.0f;
                sp_vision25_pitch_acc_filtered = 0.0f;
            }
            sp_vision25_pitch_acc_filtered += SP_VISION25_PITCH_PLAN_ACC_FILTER_ALPHA *
                (-plan_pitch_acc - sp_vision25_pitch_acc_filtered);
            pitch_acc_current_ff = fp32_constrain(
                SP_VISION25_PITCH_PLAN_ACC_FF_GAIN * sp_vision25_pitch_acc_filtered,
                -SP_VISION25_MAX_PITCH_PLAN_ACC_FF, SP_VISION25_MAX_PITCH_PLAN_ACC_FF);
        }

        pitch_vel_cmd = PID_calc(&sp_vision25_pid.pitch_angle,
                                gimbal->feedback_pos.pitch, gimbal->reference.pitch);
        if (allow_plan_feedforward) {
            pitch_vel_cmd -= SP_VISION25_PITCH_PLAN_VEL_FF_GAIN * fp32_constrain(
                GetScCmdGimbalVelocity(AX_PITCH),
                -SP_VISION25_MAX_PITCH_PLAN_VEL_FF, SP_VISION25_MAX_PITCH_PLAN_VEL_FF);
        }
        gimbal->pitch.set.vel = fp32_constrain(
            pitch_vel_cmd, -SP_VISION25_MAX_OUT_PITCH_ANGLE, SP_VISION25_MAX_OUT_PITCH_ANGLE);
        pitch_velocity_current = PID_calc(&sp_vision25_pid.pitch_velocity,
                                         gimbal->feedback_vel.pitch, gimbal->pitch.set.vel);
        gimbal->pitch.set.curr = gimbal->pitch.direction * fp32_constrain(
            pitch_velocity_current + pitch_acc_current_ff,
            -SP_VISION25_MAX_OUT_PITCH_VELOCITY, SP_VISION25_MAX_OUT_PITCH_VELOCITY);
    }

    if (gimbal->yaw.offline) {
        SpVision25_ResetYaw();
        gimbal->yaw.set.vel = 0.0f;
        gimbal->yaw.set.tor = 0.0f;
    } else {
        fp32 delta_yaw = loop_fp32_constrain(
            gimbal->reference.yaw - gimbal->feedback_pos.yaw, -M_PI, M_PI);
        fp32 yaw_vel_cmd = PID_calc(&sp_vision25_pid.yaw_angle, 0.0f, delta_yaw);
        fp32 yaw_acc_torque_ff = 0.0f;
        fp32 yaw_velocity_torque;

        yaw_vel_cmd -= SP_VISION25_YAW_GYRO_DAMPING * gimbal->feedback_vel.yaw;
        yaw_vel_cmd += SP_VISION25_YAW_WZ_FEEDFORWARD * ChassisGetSpeedWz();
        if (allow_plan_feedforward) {
            fp32 plan_yaw_acc = fp32_constrain(
                GetScCmdGimbalAcceleration(AX_YAW),
                -SP_VISION25_MAX_YAW_PLAN_ACC, SP_VISION25_MAX_YAW_PLAN_ACC);
            yaw_vel_cmd += SP_VISION25_YAW_PLAN_VEL_FF_GAIN * fp32_constrain(
                GetScCmdGimbalVelocity(AX_YAW),
                -SP_VISION25_MAX_YAW_PLAN_VEL_FF, SP_VISION25_MAX_YAW_PLAN_VEL_FF);
            if (plan_yaw_acc != plan_yaw_acc) {
                plan_yaw_acc = 0.0f;
                sp_vision25_yaw_acc_filtered = 0.0f;
            }
            sp_vision25_yaw_acc_filtered += SP_VISION25_YAW_PLAN_ACC_FILTER_ALPHA *
                (plan_yaw_acc - sp_vision25_yaw_acc_filtered);
            yaw_acc_torque_ff = fp32_constrain(
                SP_VISION25_YAW_PLAN_ACC_FF_GAIN * sp_vision25_yaw_acc_filtered,
                -SP_VISION25_MAX_YAW_PLAN_ACC_FF, SP_VISION25_MAX_YAW_PLAN_ACC_FF);
        }

        /* Direction mapping precedes the DM encoder velocity loop, as in d49991a. */
        gimbal->yaw.set.vel = gimbal->yaw.direction * fp32_constrain(
            yaw_vel_cmd, -SP_VISION25_MAX_OUT_YAW_ANGLE, SP_VISION25_MAX_OUT_YAW_ANGLE);
        yaw_velocity_torque = PID_calc(&sp_vision25_pid.yaw_velocity,
                                      gimbal->yaw.fdb.vel, gimbal->yaw.set.vel);
        gimbal->yaw.set.tor = fp32_constrain(
            yaw_velocity_torque + yaw_acc_torque_ff,
            -SP_VISION25_MAX_OUT_YAW_VELOCITY, SP_VISION25_MAX_OUT_YAW_VELOCITY);
    }

    sp_vision25_active = true;
    return true;
}

#endif /* GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT */
