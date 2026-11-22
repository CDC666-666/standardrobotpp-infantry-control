#ifndef VT13_CONTROL_H
#define VT13_CONTROL_H

#include "vt13_remote.h"

#define VT13_CONTROL_STATUS_TIMEOUT_MS 10U
#define VT13_RETURN_SETTLE_MS 100U

typedef enum { VT13_MODE_C, VT13_MODE_N, VT13_MODE_S } Vt13Mode_e;

typedef struct {
    bool valid;
    Vt13Mode_e mode;
    float move_x, move_y, yaw_input, pitch_input, wheel;
    bool fn_down;   // 左 Fn 功能键，按下沿切换摩擦轮。
    bool spin_down; // 右上扳机键，按住旋转、松开回正。
    uint32_t sequence, update_time_ms;
} Vt13ControlInput;

typedef enum {
    VT13_INIT_DISABLED, // C 档或遥控无效，初始化结果已清除，下次使能重新回中。
    VT13_INIT_RUNNING,  // 云台正在回中初始化，底盘与发射保持失能。
    VT13_INIT_READY,    // 云台已实际回中，后续仍须通过反馈和状态时效等工作许可检查。
    VT13_INIT_FAILED    // 初始化超时或工作中反馈失效；经 C 档或失联复位后才能重试。
} Vt13InitState_e;

typedef struct {
    Vt13InitState_e state;
    uint32_t start_ms;
} Vt13InitControl;

/* 云台单写者发布；其他任务只能读取自己的完整副本。 */
typedef struct {
    Vt13InitState_e init_state;
    bool ready, auto_active, vision_fire, feedback_valid;
    float yaw_world, yaw_joint;
    uint32_t update_time_ms;
} GimbalControlStatus;

typedef enum {
    VT13_CHASSIS_DISABLED, VT13_CHASSIS_FOLLOW, VT13_CHASSIS_BODY,
    VT13_CHASSIS_SPIN, VT13_CHASSIS_RETURN
} Vt13ChassisMode_e;

typedef struct {
    bool spin_last, returning, settling;
    float return_heading;
    uint32_t settle_start_ms;
} Vt13SpinControl;

typedef struct {
    bool friction_on, fn_last, fn_released, manual_burst;
} Vt13ShootControl;

/* 映射函数不维护按键边沿，重复读取同一帧没有副作用。 */
/* 转换原始快照为整车输入；不在映射层维护按键边沿。 */
void Vt13MapControl(const Vt13Input *raw, Vt13ControlInput *out);
/* 读取最新接收快照并转换；输出为调用方独占的副本。 */
void Vt13ReadControl(Vt13ControlInput *out);
bool Vt13ControlEnabled(const Vt13ControlInput *input);
bool Vt13StatusReady(const GimbalControlStatus *status, uint32_t now_ms);
float Vt13WrapAngle(float angle);
float Vt13ChassisHeading(const GimbalControlStatus *status);

/* 以下纯状态机由各控制任务独占，时间单位均为毫秒。 */
/* 推进云台回中初始化；超时锁定失败，失能后才能重试。 */
void Vt13UpdateInit(Vt13InitControl *state, bool enabled, bool feedback_valid,
                    float yaw_error, float pitch_error, uint32_t now_ms, uint32_t timeout_ms);
Vt13ChassisMode_e Vt13UpdateSpin(Vt13SpinControl *state, const Vt13ControlInput *input,
                                const GimbalControlStatus *gimbal, float chassis_wz,
                                uint32_t now_ms);
/* 按 Fn 有效按下沿切换开轮状态；失能会清除锁存。 */
void Vt13UpdateFriction(Vt13ShootControl *state, const Vt13ControlInput *input, bool enabled);
bool Vt13ManualBurst(Vt13ShootControl *state, const Vt13ControlInput *input);
bool Vt13AutoFireRequested(const Vt13ControlInput *input, const GimbalControlStatus *gimbal);

#endif
