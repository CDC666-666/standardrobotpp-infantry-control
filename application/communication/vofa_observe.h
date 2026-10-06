#ifndef VOFA_OBSERVE_H
#define VOFA_OBSERVE_H

// No controller headers here: gimbal/chassis have conflicting private PID_t types.
#include "robot_param.h"
#include <stdint.h>
#include <string.h>

typedef struct {
    float speed_ref;       // motor-shaft rad/s, same units as the speed PID
    float speed_fdb;       // motor-shaft rad/s; NaN when offline
    float output_command;  // DJI protocol command units, NOT amperes
    float online;          // 0/1
} WheelObservation;

typedef struct {
    float position_ref, position_fdb; // M2006 output-shaft rad (existing conversion)
    float speed_ref, speed_fdb;       // motor-shaft rad/s
    float current_command;           // M2006 raw current command
    float velocity_command;          // NaN: DM command generation is commented
    float online;
} TriggerObservation;

typedef struct {
    WheelObservation fric_left, fric_right;
    TriggerObservation trigger;
    float mode, fric_state, output_enabled, block_count;
} ShootObservation;

typedef struct {
    WheelObservation wheel[4]; // array index, not CAN ID; see robot_param mapping
    float wheel_current_fdb[4]; // signed C620 CAN feedback raw values; NaN when offline
    float vx_ref, vy_ref, wz_ref, mode; // m/s, m/s, rad/s, enum
    float vx_fdb; // chassis.P_vx, m/s
    float vy_fdb; // chassis.P_vy, m/s
    float vx_iout, vy_iout; // planar PID integral force contributions, N
    float wz_fdb; // chassis.P_wz, rad/s, wheel-derived estimate
} ChassisObservation;

static inline float VofaUnavailable(void)
{
    const uint32_t bits = 0x7fc00000UL;
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

// All observation fields are binary32 floats. 0xffffffff is a quiet NaN.
// Whole structs are invalid before first publication / when stale.
#define VOFA_OBSERVATION_MAX_AGE_MS ((VOFA_PERIOD_MS * 2U > 20U) ? VOFA_PERIOD_MS * 2U : 20U)

#if VOFA_ENABLE && SHOOT_TYPE == SHOOT_FRIC_TRIGGER
void ShootReadObservation(ShootObservation *out);
#else
static inline void ShootReadObservation(ShootObservation *out) { memset(out, 0xff, sizeof(*out)); }
#endif
#if VOFA_ENABLE && CHASSIS_TYPE == CHASSIS_OMNI_WHEEL
void ChassisReadObservation(ChassisObservation *out);
#else
static inline void ChassisReadObservation(ChassisObservation *out) { memset(out, 0xff, sizeof(*out)); }
#endif

#endif
