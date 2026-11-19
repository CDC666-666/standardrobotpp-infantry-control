#include "vofa_chassis.h"

#if VOFA_ENABLE && CHASSIS_TYPE == CHASSIS_OMNI_WHEEL
#include "vofa_observe.h"
#include "vofa_port.h"

static ChassisObservation chassis_observation;
static uint32_t chassis_observation_time;
static uint8_t chassis_observation_ready;

void vofa_chassis_update(const Chassis_s *state)
{
    ChassisObservation next;
    uint32_t key;
    unsigned i;
    for (i = 0; i < 4; ++i) {
        next.wheel[i].speed_ref = state->set[i];
        next.wheel[i].speed_fdb = state->wheel[i].offline ? VofaUnavailable() : state->feedback[i];
        next.wheel_current_fdb[i] = state->wheel[i].offline ? VofaUnavailable() : state->wheel[i].fdb.curr;
        next.wheel[i].output_command = state->wheel[i].set.curr; // 经过输出保护后的 CAN 原始指令值
        next.wheel[i].online = !state->wheel[i].offline;
    }
    next.vx_ref = state->reference.vx;
    next.vx_fdb = state->P_vx;
    next.vy_ref = state->reference.vy;
    next.vy_fdb = state->P_vy;
    next.vx_iout = state->vel_x_pid.Iout;
    next.vy_iout = state->vel_y_pid.Iout;
    next.wz_ref = state->reference.wz;
    next.wz_fdb = state->P_wz;
    next.mode = (float)state->mode;
    key = VofaPortLock();
    chassis_observation = next;
    chassis_observation_time = VofaPortNow();
    chassis_observation_ready = 1;
    VofaPortUnlock(key);
}

void ChassisReadObservation(ChassisObservation *out)
{
    uint32_t key = VofaPortLock();
    if (chassis_observation_ready && VofaPortNow() - chassis_observation_time <= VOFA_OBSERVATION_MAX_AGE_MS)
        *out = chassis_observation;
    else
        memset(out, 0xff, sizeof(*out));
    VofaPortUnlock(key);
}

#endif
