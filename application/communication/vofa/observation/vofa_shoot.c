#include "vofa_shoot.h"

#if VOFA_ENABLE && SHOOT_TYPE == SHOOT_FRIC_TRIGGER
#include "vofa_observe.h"
#include "vofa_port.h"

static ShootObservation shoot_observation;
static uint32_t shoot_observation_time;
static uint8_t shoot_observation_ready;

void vofa_shoot_update(const Shoot_s *state)
{
    ShootObservation next;
    uint32_t key;
    memset(&next, 0xff, sizeof(next));
    next.fric_left.speed_ref = state->REF.fric_speed_ref_L;
    next.fric_right.speed_ref = state->REF.fric_speed_ref_R;
    next.fric_left.online = !state->fric_motor[1].offline;
    next.fric_right.online = !state->fric_motor[0].offline;
    next.fric_left.speed_fdb = state->fric_motor[1].offline ? VofaUnavailable() : state->FDB.fric_speed_fdb_L;
    next.fric_right.speed_fdb = state->fric_motor[0].offline ? VofaUnavailable() : state->FDB.fric_speed_fdb_R;
    next.fric_left.output_command = state->fric_motor[1].set.curr;
    next.fric_right.output_command = state->fric_motor[0].set.curr;
    next.trigger.online = !state->trigger_motor.offline;
    next.trigger.speed_ref = state->REF.trigger_speed_ref;
    next.trigger.speed_fdb = state->trigger_motor.offline ? VofaUnavailable() : state->FDB.trigger_speed_fdb;
    if (TRIGGER_MOTOR_TYPE == DJI_M2006) {
        next.trigger.position_ref = state->REF.trigger_angel_ref;
        next.trigger.position_fdb = state->trigger_motor.offline ? VofaUnavailable() : state->FDB.trigger_angel_fdb;
        next.trigger.current_command = state->trigger_motor.set.curr;
    }
    // 仅表示编译启用了发送分支，不代表电机已收到控制命令。
    next.output_enabled = (TRIGGER_MOTOR_TYPE == DJI_M2006);
    next.mode = (float)state->mode;
    next.fric_state = (float)state->state;
    next.block_count = (float)state->block_time;
    key = VofaPortLock();
    shoot_observation = next;
    shoot_observation_time = VofaPortNow();
    shoot_observation_ready = 1;
    VofaPortUnlock(key);
}

void ShootReadObservation(ShootObservation *out)
{
    uint32_t key = VofaPortLock();
    if (shoot_observation_ready && VofaPortNow() - shoot_observation_time <= VOFA_OBSERVATION_MAX_AGE_MS)
        *out = shoot_observation;
    else
        memset(out, 0xff, sizeof(*out));
    VofaPortUnlock(key);
}

#endif
