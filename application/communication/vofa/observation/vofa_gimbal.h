#ifndef VOFA_GIMBAL_H
#define VOFA_GIMBAL_H

#include "gimbal_yaw_pitch_direct.h"

#if VOFA_ENABLE && GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT
void vofa_gimbal_update(const Gimbal_s *state, float sent_torque,
                        bool auto_feedback_valid, float auto_theta_ref_rad,
                        float auto_theta_rad, float usb_offline, float vision_request);

#endif

#endif
