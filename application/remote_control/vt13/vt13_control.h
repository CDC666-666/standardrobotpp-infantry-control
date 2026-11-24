#ifndef VT13_CONTROL_H
#define VT13_CONTROL_H

#include "vt13_remote.h"

#define VT13_CONTROL_STATUS_TIMEOUT_MS 10U
#define VT13_RETURN_SETTLE_MS 100U
/* N 档及 S 档无目标时，跟随停止区与回正完成判据共用。 */
#define VT13_CHASSIS_ALIGN_ANGLE_RAD 0.015f
#define VT13_CHASSIS_ALIGN_WZ_RAD_S 0.03f

typedef enum { VT13_MODE_C, VT13_MODE_N, VT13_MODE_S } Vt13Mode_e;

typedef struct {
    bool valid;
    Vt13Mode_e mode;
    float move_x, move_y, yaw_input, pitch_input, wheel;
    bool fn_down;   // 左 Fn 功能键，按下沿切换摩擦轮。
    bool spin_down; // 右上扳机或 N 档 Shift 任一按住旋转，全部松开回正。
    uint32_t sequence, update_time_ms;
    int16_t mouse_x, mouse_y; // N 档鼠标相对位移，只在新有效帧消费一次。
    bool keyboard_move; // N 档存在 WASD 输入时优先于摇杆，反向同时按下也占用优先权。
    bool mouse_left, mouse_right; // 左键允许手动/自瞄发射；N 档右键按住请求自瞄。
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
    float return_heading; // N 右键/S 档有效自瞄时采用松手固定朝向回正。
    uint32_t settle_start_ms;
} Vt13SpinControl;

typedef struct {
    bool friction_on, fn_last, fn_released, manual_burst;
} Vt13ShootControl;

/* 各控制任务独占键盘速度和鼠标消费状态，不向接收器写回数据。 */
typedef struct {
    float vx, vy;
    uint32_t update_time_ms;
    bool active;
} Vt13KeyboardMotion;

typedef struct {
    uint32_t sequence;
    bool seen;
} Vt13MouseControl;

/* 映射函数不维护按键边沿，重复读取同一帧没有副作用。 */
/* 转换原始快照为整车输入；不在映射层维护按键边沿。 */
void Vt13MapControl(const Vt13Input *raw, Vt13ControlInput *out);
/* 读取最新接收快照并转换；输出为调用方独占的副本。 */
void Vt13ReadControl(Vt13ControlInput *out);
bool Vt13ControlEnabled(const Vt13ControlInput *input);
/* N 档按住右键或 S 档请求自瞄，是否有目标仍由视觉及云台状态判断。 */
bool Vt13AutoAimRequested(const Vt13ControlInput *input);
/* 按新接收帧消费鼠标位移；首次进入 N 档或失能恢复不补算旧位移。 */
bool Vt13TakeMouseDelta(Vt13MouseControl *state, const Vt13ControlInput *input, int16_t *dx, int16_t *dy);
/* WASD 优先期间及松键交接阶段限制平移速度变化，S 档和失能会复位。 */
void Vt13UpdateKeyboardMotion(Vt13KeyboardMotion *state, const Vt13ControlInput *input,
                              uint32_t now_ms, float max_speed, float accel, float decel);
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
