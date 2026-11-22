#include "vt13_control.h"

#include <math.h>
#include <string.h>

#define VT13_PI 3.14159265358979323846f

/**
 * @brief 将校验后的原始遥控快照转换为整车统一输入。
 * @note 输出使用调用方副本；C 档或无效帧不发布操控量，不在此处维护按键边沿。
 */
void Vt13MapControl(const Vt13Input *raw, Vt13ControlInput *out)
{
    /* 调用方未提供输出空间时直接退出，避免解引用空指针。 */
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    /* 没有原始快照时保留清零后的无效输入。 */
    if (raw == NULL) return;
    out->sequence = raw->sequence;
    out->update_time_ms = raw->update_time_ms;
    /* 接收无效或档位超出 C/N/S 范围时，不发布整车控制许可。 */
    if (!raw->valid || raw->mode_switch > VT13_MODE_S) return;
    out->valid = true;
    out->mode = (Vt13Mode_e)raw->mode_switch;
    /* C 档只保留档位与元数据，摇杆、拨轮和按键均保持清零。 */
    if (out->mode == VT13_MODE_C) return;
    /* 按实车方向反馈交换平移轴；保留接收层原始快照与左移为正约定。 */
    out->move_x = raw->normalized[3];
    out->move_y = -raw->normalized[2];
    out->yaw_input = raw->normalized[0];
    out->pitch_input = raw->normalized[1];
    out->wheel = ((float)raw->wheel - 1024.0f) / 660.0f;
    out->fn_down = raw->function_left;
    /* 小陀螺使用右上扳机；右上功能键不参与该映射。 */
    out->spin_down = raw->trigger;
}

/**
 * @brief 读取最新 VT13 快照并完成整车操控映射。
 * @note 由接收层处理有效帧超时和左摇杆轴交换；本函数可被各任务重复调用。
 */
void Vt13ReadControl(Vt13ControlInput *out)
{
    Vt13Input raw;
    Vt13ReadInput(&raw);
    Vt13MapControl(&raw, out);
}

/**
 * @brief 判断遥控输入是否允许整车使能。
 * @note 只有有效输入的 N、S 档返回 true，C 档和空指针均返回 false。
 */
bool Vt13ControlEnabled(const Vt13ControlInput *input)
{
    return input != NULL && input->valid &&
           (input->mode == VT13_MODE_N || input->mode == VT13_MODE_S);
}

/**
 * @brief 判断云台是否完成初始化且发布状态仍然有效。
 * @note 同时要求工作许可、反馈有效和状态未超时；时间差使用无符号减法兼容回卷。
 */
bool Vt13StatusReady(const GimbalControlStatus *status, uint32_t now_ms)
{
    return status != NULL && status->ready && status->feedback_valid &&
           status->init_state == VT13_INIT_READY &&
           now_ms - status->update_time_ms < VT13_CONTROL_STATUS_TIMEOUT_MS;
}

/**
 * @brief 将有限角度折返到 [-pi, pi) 范围。
 * @note 使用取余避免大角度反复加减；非有限值返回零，调用方仍需独立检查反馈有效性。
 */
float Vt13WrapAngle(float angle)
{
    /* 非有限角度不能进行正常取余，返回安全占位值。 */
    if (!isfinite(angle)) return 0.0f;
    angle = fmodf(angle + VT13_PI, 2.0f * VT13_PI);
    /* C 语言取余可为负，补一周后统一到目标角度区间。 */
    if (angle < 0.0f) angle += 2.0f * VT13_PI;
    return angle - VT13_PI;
}

/**
 * @brief 由云台世界朝向和相对关节角推算底盘朝向。
 * @note 调用方须先检查完整云台快照有效，结果只表示朝向，不表示地面位置。
 */
float Vt13ChassisHeading(const GimbalControlStatus *status)
{
    return Vt13WrapAngle(status->yaw_world - status->yaw_joint);
}

/**
 * @brief 推进云台初始化状态，区分回中完成与超时失败。
 * @note 状态由云台任务独占；首次使能先进入初始化，失败状态需失能后才能重试。
 */
void Vt13UpdateInit(Vt13InitControl *state, bool enabled, bool feedback_valid,
                    float yaw_error, float pitch_error, uint32_t now_ms, uint32_t timeout_ms)
{
    /* C 档或失联撤销初始化结果，下一次使能必须重新回中。 */
    if (!enabled) {
        state->state = VT13_INIT_DISABLED;
        state->start_ms = now_ms;
    } else /* 从失能恢复时只启动计时，本拍不直接判定回中完成。 */ if (state->state == VT13_INIT_DISABLED) {
        state->state = VT13_INIT_RUNNING;
        state->start_ms = now_ms;
    } else /* 初始化运行期间按超时和实际回中误差分别判定失败或完成。 */ if (state->state == VT13_INIT_RUNNING) {
        /* 达到截止时间即锁定失败，不能因超时自动放行车辆。 */
        if (now_ms - state->start_ms >= timeout_ms) state->state = VT13_INIT_FAILED;
        /* 截止前仅在反馈有效且 yaw/pitch 都实际回中时放行。 */
        else if (feedback_valid && isfinite(yaw_error) && isfinite(pitch_error) &&
                 fabsf(yaw_error) < 0.03f && fabsf(pitch_error) < 0.03f)
            state->state = VT13_INIT_READY;
    } else /* 工作中反馈失效时锁定故障，避免反馈恢复后直接续用旧初始化状态。 */ if (state->state == VT13_INIT_READY && !feedback_valid) {
        /* 工作中失去反馈也要求重新回到 C 档，不能静默恢复旧状态。 */
        state->state = VT13_INIT_FAILED;
    }
}

/**
 * @brief 按失能、旋转、松手回正和目标状态选择底盘模式。
 * @note 底盘任务独占该状态；只在松手沿保存一次云台朝向，回正需连续满足稳定条件。
 */
Vt13ChassisMode_e Vt13UpdateSpin(Vt13SpinControl *state, const Vt13ControlInput *input,
                                const GimbalControlStatus *gimbal, float chassis_wz,
                                uint32_t now_ms)
{
    /* 遥控、云台时效或数值任一不满足时，取消旋转和回正并选择失能。 */
    if (!Vt13ControlEnabled(input) || !Vt13StatusReady(gimbal, now_ms) ||
        !isfinite(gimbal->yaw_world) || !isfinite(gimbal->yaw_joint) || !isfinite(chassis_wz)) {
        memset(state, 0, sizeof(*state));
        return VT13_CHASSIS_DISABLED;
    }
    /* 右上扳机按住时旋转优先，同时取消之前未完成的回正。 */
    if (input->spin_down) {
        state->spin_last = true;
        state->returning = state->settling = false;
        return VT13_CHASSIS_SPIN;
    }
    /* 上拍按住、本拍松开构成松手沿，只在这一拍保存固定回正方向。 */
    if (state->spin_last) {
        state->return_heading = gimbal->yaw_world;
        state->returning = true;
        state->settling = false;
    }
    state->spin_last = false;
    /* 回正尚未完成时，优先处理固定朝向，不立即切换跟随或底盘坐标模式。 */
    if (state->returning) {
        float error = Vt13WrapAngle(state->return_heading - Vt13ChassisHeading(gimbal));
        /* 角度误差与实际角速度同时足够小时，才开始累计稳定时间。 */
        if (fabsf(error) < 0.03f && fabsf(chassis_wz) < 0.05f) {
            /* 首次进入稳定区时记录起点，连续满足时不能反复重置计时。 */
            if (!state->settling) state->settle_start_ms = now_ms;
            state->settling = true;
            /* 稳定持续达到 100 ms 后，撤销回正状态并恢复正常模式选择。 */
            if (now_ms - state->settle_start_ms >= VT13_RETURN_SETTLE_MS)
                state->returning = state->settling = false;
        } else /* 任一误差重新超限就中断稳定累计，下次进入稳定区重新计时。 */ state->settling = false;
        /* 未连续稳定完成时，本拍继续保持回正模式。 */
        if (state->returning) return VT13_CHASSIS_RETURN;
    }
    /* 回正结束后，只有 S 档实际自瞄时采用底盘坐标，其余情况跟随云台。 */
    return input->mode == VT13_MODE_S && gimbal->auto_active
           ? VT13_CHASSIS_BODY : VT13_CHASSIS_FOLLOW;
}

/**
 * @brief 根据左 Fn 的有效按下沿切换摩擦轮开关。
 * @note 失能清除锁存；恢复后先确认松开，长按或重复读取同一帧不会重复切换。
 */
void Vt13UpdateFriction(Vt13ShootControl *state, const Vt13ControlInput *input, bool enabled)
{
    /* 整车或云台许可失效时清除开轮锁存，恢复后不能自动重新开轮。 */
    if (!enabled || !Vt13ControlEnabled(input)) {
        memset(state, 0, sizeof(*state));
        return;
    }
    /* 上电、初始化及失能恢复后，必须先看到松开再允许下一次按下。 */
    /* 先观察到 Fn 松开，才允许之后的新按下沿切换开轮状态。 */
    if (!input->fn_down) state->fn_released = true;
    /* 仅处理已重新解锁的按下沿，长按及重复读取不构成新事件。 */
    if (state->fn_released && input->fn_down && !state->fn_last)
        state->friction_on = !state->friction_on;
    state->fn_last = input->fn_down;
}

/**
 * @brief 按拨轮负端阈值和迟滞更新手动连发请求。
 * @note 该函数只生成请求，不检查摩擦轮到速或电机状态，也不依赖视觉在线。
 */
bool Vt13ManualBurst(Vt13ShootControl *state, const Vt13ControlInput *input)
{
    /* 失能立即撤销手动连发请求，防止跨档位或失联保留旧请求。 */
    if (!Vt13ControlEnabled(input)) state->manual_burst = false;
    /* 拨轮接近负端点时进入连发，不要求浮点值恰好等于 -1。 */
    else if (input->wheel <= -0.95f) state->manual_burst = true;
    /* 拨轮退回释放阈值后退出连发，中间区间保持原请求以避免抖动。 */
    else if (input->wheel >= -0.90f) state->manual_burst = false;
    return state->manual_burst;
}

/**
 * @brief 判断 S 档是否具备视觉自动连发请求。
 * @note 还须由发射任务检查云台快照时效、Fn 开轮、轮速和电机健康状态。
 */
bool Vt13AutoFireRequested(const Vt13ControlInput *input, const GimbalControlStatus *gimbal)
{
    return Vt13ControlEnabled(input) && input->mode == VT13_MODE_S &&
           gimbal->auto_active && gimbal->vision_fire;
}
