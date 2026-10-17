/**
 ******************************************************************************
 * @file    yaw_auto_lqr_eso_controller.h
 * @school  南京理工大学（NJUST）
 * @author  戴子轩
 * @email   daizeming24@gmail.com
 * @date    2026/5/16
 * @brief   Standalone yaw auto-aim LQR-ESO controller matching the math summary.
 *
 * This pair is a documentation companion for yaw-auto-lqreso-math-summary.docx.
 * It keeps only the continuous-time reference tracking, feedforward, LQR,
 * third-order ESO, disturbance compensation, torque limit, and slew limit chain.
 ******************************************************************************
 */

#ifndef YAW_AUTO_LQR_ESO_CONTROLLER_H
#define YAW_AUTO_LQR_ESO_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float j_kg_m2;
    float b_nms_rad;
    float k_theta;
    float k_omega;
    float k_i;
    float theta_integral_limit_rad_s;
    float tau_coulomb_nm;
    float coulomb_smooth_rad_s;
    float eso_bandwidth_rad_s;
    float eso_comp_gain;
    float eso_comp_limit_nm;
    float eso_omega_gate_rad_s;
    float eso_alpha_gate_rad_s2;
    float tau_bias_ki;
    float tau_bias_limit_nm;
    float tau_meas_lpf_alpha;
    float theta_deadband_rad;
    float torque_soft_limit_nm;
    float torque_min_nm;
    float torque_max_nm;
    float torque_slew_rate_nm_s;
    uint8_t eso_enable;
    uint8_t eso_comp_enable;
    uint8_t torque_slew_enable;
} YawAutoLqrEsoConfig_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
} YawAutoLqrEsoState_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float alpha_rad_s2;
} YawAutoLqrEsoReference_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float tau_meas_nm;
    uint8_t feedback_ok;
} YawAutoLqrEsoFeedback_t;

typedef struct
{
    float z1;
    float z2;
    float z3;
    float b0;
    float w0;
    float beta1;
    float beta2;
    float beta3;
} YawAutoLqrEsoObserver_t;

typedef struct
{
    YawAutoLqrEsoObserver_t eso;
    float theta_integral_rad_s;
    float tau_bias_nm;
    float tau_meas_filt_nm;
    float tau_cmd_last_nm;
    uint8_t feedback_ready;
} YawAutoLqrEso_t;

typedef struct
{
    float a[2][2];
    float b[2][2];   /* input vector is [tau, d] */
    float c[2];
} YawAutoLqrEsoStateSpace_t;

typedef struct
{
    float theta_ref_rad;          /* 本周期参考角位置，rad */
    float omega_ref_rad_s;        /* 本周期参考角速度，rad/s */
    float alpha_ref_rad_s2;       /* 本周期参考角加速度，rad/s^2 */
    float e_theta_rad;            /* 角位置误差：实际角度 - 参考角度，rad */
    float e_omega_rad_s;          /* 角速度误差：实际角速度 - 参考角速度，rad/s */
    float tau_ff_alpha_nm;       /* 角加速度前馈力矩，N·m */
    float tau_ff_viscous_nm;     /* 粘性阻尼前馈力矩，N·m */
    float tau_ff_coulomb_nm;     /* 库仑摩擦前馈力矩，N·m */
    float tau_ff_nm;             /* 上述三项之和：总前馈力矩，N·m */
    float tau_i_nm;              /* 角位置误差积分产生的力矩，N·m */
    float tau_lqr_nm;            /* 前馈、LQR 状态反馈及积分项之和，N·m */
    float tau_eso_raw_nm;        /* ESO 估计出的原始扰动补偿力矩，N·m */
    float tau_eso_active_nm;     /* 实际接入模块控制的 ESO 补偿力矩，N·m */
    float tau_bias_nm;           /* 偏置补偿力矩，N·m */
    float tau_pre_limit_nm;      /* 软限幅、硬限幅和斜率限制前的模块力矩，N·m */
    float tau_cmd_before_slew_nm; /* 软/硬限幅后、斜率限制前的模块力矩，N·m */
    float tau_cmd_nm;            /* 模块最终命令力矩，N·m；下位机发送前还会叠加底盘补偿 */
    uint8_t eso_comp_active;     /* ESO 补偿是否实际接入模块控制：1 是，0 否 */
    uint8_t soft_limit_active;   /* 模块软限幅是否触发：1 是，0 否 */
    uint8_t hard_limit_active;   /* 模块硬限幅是否触发：1 是，0 否 */
    uint8_t slew_limit_active;   /* 模块力矩斜率限制是否触发：1 是，0 否 */
} YawAutoLqrEsoOutput_t;

void YawAutoLqrEso_Init(YawAutoLqrEso_t *ctrl);
void YawAutoLqrEso_Reset(YawAutoLqrEso_t *ctrl, float theta_rad, float omega_rad_s);
uint8_t YawAutoLqrEso_GetStateSpace(const YawAutoLqrEsoConfig_t *cfg,
                                    YawAutoLqrEsoStateSpace_t *model);
void YawAutoLqrEso_Calc(YawAutoLqrEso_t *ctrl,
                        const YawAutoLqrEsoConfig_t *cfg,
                        const YawAutoLqrEsoFeedback_t *feedback,
                        const YawAutoLqrEsoReference_t *ref,
                        float dt_s,
                        YawAutoLqrEsoOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif
