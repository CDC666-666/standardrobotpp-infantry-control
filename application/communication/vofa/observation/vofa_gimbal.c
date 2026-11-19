#include "vofa_gimbal.h"

#if VOFA_ENABLE && GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT
#include "vofa_observe.h"
#include "vofa_port.h"

static GimbalYawObservation yaw_observation;
static uint32_t yaw_observation_time;
static uint8_t yaw_observation_ready;

void vofa_gimbal_update(const Gimbal_s *state, float sent_torque,
                        bool auto_feedback_valid, float auto_theta_ref_rad,
                        float auto_theta_rad, float usb_offline, float vision_request)
{
    GimbalYawObservation next;
    uint32_t now_ms = VofaPortNow();
    uint32_t key;
    next.time_ms = (float)now_ms;
    next.mode = (float)state->mode;
    next.theta_ref_rad = state->reference.yaw;
    next.theta_rad = state->feedback_pos.yaw;
    if (state->mode == GIMBAL_AUTO_AIM) {
        next.theta_ref_rad = auto_feedback_valid ? auto_theta_ref_rad : VofaUnavailable();
        if (auto_feedback_valid) next.theta_rad = auto_theta_rad;
    }
    next.omega_imu_rad_s = state->feedback_vel.yaw;
    next.omega_motor_rad_s = state->yaw.offline ? VofaUnavailable() :
        state->yaw.direction * state->yaw.fdb.vel;
    next.tau_cmd_nm = state->yaw.direction * sent_torque;
    next.tau_meas_nm = state->yaw.offline ? VofaUnavailable() :
        state->yaw.direction * state->yaw.fdb.tor;
    next.theta_motor_rad = state->yaw.offline ? VofaUnavailable() :
        state->yaw.direction * state->yaw.fdb.pos;
    next.pitch_imu_rad = state->feedback_pos.pitch;
    next.usb_offline = usb_offline;
    next.vision_request = vision_request;
    key = VofaPortLock();
    yaw_observation = next;
    yaw_observation_time = now_ms;
    yaw_observation_ready = 1;
    VofaPortUnlock(key);
}

void GimbalReadYawObservation(GimbalYawObservation *out)
{
    uint32_t key = VofaPortLock();
    if (yaw_observation_ready && VofaPortNow() - yaw_observation_time <= VOFA_OBSERVATION_MAX_AGE_MS)
        *out = yaw_observation;
    else
        memset(out, 0xff, sizeof(*out));
    VofaPortUnlock(key);
}

#endif
