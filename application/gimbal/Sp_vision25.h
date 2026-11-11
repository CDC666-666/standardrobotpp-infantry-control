/**
 * @file Sp_vision25.h
 * @brief Standalone AIM controller for DM4310 yaw / GM6020 pitch.
 *
 * Extracted from StandardRobotpp d49991a ("前馈微调"). Control parameters
 * retain that revision's values except the final yaw velocity limit, which
 * uses the current +/-20 rad/s instead of the historical +/-60 rad/s.
 * Hardware direction, pitch mechanical limits and IMU offset remain external.
 * PID derivatives and acceleration filters assume one calculation per 1 ms.
 *
 * Pending integration (this module is not registered in the build):
 *   - Call SpVision25_Init() once after the existing gimbal initialization.
 *   - Keep observation, mode selection and last_mode bookkeeping outside.
 *   - Each cycle call SpVision25_Reference(gimbal), then
 *     SpVision25_Console(gimbal), including cycles outside AUTO_AIM.
 *   - A true return replaces the corresponding existing AIM branch; false
 *     leaves reference/output ownership with the existing non-AIM controller.
 *   - Send the calculated commands through the existing CAN sending layer.
 *     Its current LQR-ESO-specific validity gate must be adapted at integration.
 * Do not run both AIM controllers on the same cycle. This is a single-instance
 * module: the supplied Gimbal_s must be the same instance on every call.
 */
#ifndef SP_VISION25_H
#define SP_VISION25_H

#include "robot_param.h"

#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#include "gimbal_yaw_pitch_direct.h"

/* Independent AIM PID parameters; no aliases to the existing controller. */
#define SP_VISION25_KP_YAW_ANGLE (10.0f)
#define SP_VISION25_KI_YAW_ANGLE (0.0f)
#define SP_VISION25_KD_YAW_ANGLE (0.0f)
#define SP_VISION25_MAX_IOUT_YAW_ANGLE (0.0f)
#define SP_VISION25_MAX_OUT_YAW_ANGLE (20.0f) /* Final speed limit, rad/s. */

#define SP_VISION25_KP_YAW_VELOCITY (1.25f)
#define SP_VISION25_KI_YAW_VELOCITY (0.0f)
#define SP_VISION25_KD_YAW_VELOCITY (0.0f)
#define SP_VISION25_MAX_IOUT_YAW_VELOCITY (1.0f)
#define SP_VISION25_MAX_OUT_YAW_VELOCITY (7.0f) /* Total torque, N*m. */

#define SP_VISION25_KP_PITCH_ANGLE (70.0f)
#define SP_VISION25_KI_PITCH_ANGLE (0.0f)
#define SP_VISION25_KD_PITCH_ANGLE (2.0f)
#define SP_VISION25_MAX_IOUT_PITCH_ANGLE (1.0f)
#define SP_VISION25_MAX_OUT_PITCH_ANGLE (15.0f) /* Final speed limit, rad/s. */

#define SP_VISION25_KP_PITCH_VELOCITY (2800.0f)
#define SP_VISION25_KI_PITCH_VELOCITY (0.0f)
#define SP_VISION25_KD_PITCH_VELOCITY (30.0f)
#define SP_VISION25_MAX_IOUT_PITCH_VELOCITY (10000.0f)
#define SP_VISION25_MAX_OUT_PITCH_VELOCITY (30000.0f) /* Total CAN current, raw. */

#define SP_VISION25_YAW_GYRO_DAMPING (0.25f)
#define SP_VISION25_YAW_WZ_FEEDFORWARD (2.0f)
#define SP_VISION25_YAW_PLAN_VEL_FF_GAIN (1.0f)
#define SP_VISION25_MAX_YAW_PLAN_VEL_FF (1.5f) /* Input speed, rad/s. */
#define SP_VISION25_YAW_PLAN_ACC_FF_GAIN (0.004f) /* Trial N*m/(rad/s^2). */
#define SP_VISION25_MAX_YAW_PLAN_ACC_FF (0.40f) /* Torque component, N*m. */
#define SP_VISION25_MAX_YAW_PLAN_ACC (50.0f) /* Input acceleration, rad/s^2. */
#define SP_VISION25_YAW_PLAN_ACC_FILTER_ALPHA (0.25f)

#define SP_VISION25_PITCH_PLAN_VEL_FF_GAIN (2.0f)
#define SP_VISION25_MAX_PITCH_PLAN_VEL_FF (2.0f) /* Input speed, rad/s. */
#define SP_VISION25_PITCH_PLAN_ACC_FF_GAIN (6.0f) /* Trial raw/(rad/s^2). */
#define SP_VISION25_MAX_PITCH_PLAN_ACC_FF (6000.0f) /* Current component, raw. */
#define SP_VISION25_MAX_PITCH_PLAN_ACC (100.0f) /* Input acceleration, rad/s^2. */
#define SP_VISION25_PITCH_PLAN_ACC_FILTER_ALPHA (0.25f)

/** Initialize the four private PIDs and reset the module's mode/filter state. */
void SpVision25_Init(void);

/** Clear PID history and acceleration filters without changing parameters or
 *  any Gimbal_s reference/output. The next AIM cycle is an entry cycle. */
void SpVision25_Reset(void);

/** Update only AIM references. First entry holds measured position (pitch is
 *  constrained to the current mechanical range). NULL/non-AIM resets private
 *  state and returns false without touching references or motor commands. */
bool SpVision25_Reference(Gimbal_s * gimbal);

/** Calculate only AIM outputs after Reference(). Entry suppresses trajectory
 *  feedforward; an offline axis clears its own PID/filter and output commands.
 *  NULL/non-AIM resets private state and returns false without touching motor
 *  commands. No motor initialization, mode selection or CAN sending occurs.
 *  As in d49991a, only acceleration NaNs are sanitized here; finite position,
 *  speed and feedback values must be guaranteed by the caller/receive layer. */
bool SpVision25_Console(Gimbal_s * gimbal);

#endif /* GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT */
#endif /* SP_VISION25_H */
