#include "chassis_power_control.h"

#include "CAN_cmd_SupCap.h"
#include "arm_math.h"
#include "detect_task.h"
#include "pid.h"
#include "referee.h"
#include "remote_control.h"
#include "voltage_task.h"

// 全局功率控制结构体
chassis_power_control_t g_chassis_power_control;
volatile float g_initial_total_power_debug = 0.0f;

/**
 * @brief          初始化底盘功率控制
 * @param[in]      chassis: 底盘结构体指针
 * @retval         none
 */
void chassis_power_control_init(Chassis_s * chassis)
{
    // 初始化功率缓冲区PID控制器
    float buffer_pid[3] = {3.0f, 0.00f, 0.0f};
    PID_init(
        &g_chassis_power_control.buffer_pid, PID_POSITION, buffer_pid, 80.0f,
        10.0f);  // 最大输出80W，积分限幅10W

    // 关联底盘电机和最终电流指令
    for (uint8_t i = 0; i < 4; i++) {
        g_chassis_power_control.motor_chassis[i] = &chassis->wheel[i];
        g_chassis_power_control.motor_current_cmd[i] = &chassis->wheel[i].set.curr;
    }

    // 关联超级电容
    g_chassis_power_control.super_cap = &chassis->super_cap;

    // 关联遥控器
    g_chassis_power_control.rc_ctrl = get_remote_control_point();

    // 初始化状态
    g_chassis_power_control.cap_enable_state = 1;
}

void chassis_power_control(void)
{
    uint16_t max_power_limit = 40;
    fp32 chassis_max_power = 0;
    float input_power = 0;        // 电池输入功率（来自裁判系统）
    float initial_give_power[4];  // 电流指令对应的初始功率
    // float initial_total_power = 0;
    fp32 scaled_give_power[4];

    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;

    fp32 toque_coefficient = POWER_CONTROL_TORQUE_COEFF;  // 扭矩系数
    fp32 a = POWER_CONTROL_K1_COEFF;                      // k1系数
    fp32 k2 = POWER_CONTROL_K2_COEFF;                     // k2系数
    fp32 constant = POWER_CONTROL_CONSTANT;

    g_initial_total_power_debug = 0.0f;

    //这里封装了一层，尽量跟西交利物浦的原函数保持一致
    get_chassis_power_and_buffer_adapter(&chassis_power, &chassis_power_buffer);
    PID_calc(&g_chassis_power_control.buffer_pid, chassis_power_buffer, 30.0f);
    get_chassis_max_power_adapter(&max_power_limit);
    input_power =max_power_limit - g_chassis_power_control.buffer_pid.out;  // 输入功率浮动在最大功率附近（为了维持缓冲能量在30J）
    
    // 电容开关控制
    // if (g_chassis_power_control.rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) {
         g_chassis_power_control.cap_enable_state = 0;  // 按E键关闭电容
    // }
    //if (g_chassis_power_control.rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q) {
    //  g_chassis_power_control.cap_enable_state = 1;  // 按Q键打开电容
    //}
    

    CAN_CMD_CAP(input_power);        // 设置电容控制器的输入功率


    // 根据电容电量和状态调整最大功率
    if (g_chassis_power_control.super_cap->fdb.voltage_cap > POWER_CONTROL_CAP_LOW_THRESHOLD) {
        if (g_chassis_power_control.cap_enable_state == 0) 
        {
            chassis_max_power = input_power + 5;  // 略大于最大功率，避免电容一直满载，提高能量利用率
        } 
        else 
        {
            chassis_max_power = input_power + POWER_CONTROL_CAP_BOOST_POWER;  //包括裁判系统和超级电容带来的额外的功率
        }
    } 
    else 
    {
        chassis_max_power = input_power;
    }
    //计算初始电机功率

    for (uint8_t i = 0; i < 4; i++)  // 首先获取所有初始电机功率和总电机功率
    {
        const fp32 motor_current = *g_chassis_power_control.motor_current_cmd[i];

        initial_give_power[i] = motor_current * toque_coefficient *
                                     g_chassis_power_control.motor_chassis[i]->fdb.vel * (60.0f /(2.0f * M_PI)) +  // 转换为RPM
                                k2 * g_chassis_power_control.motor_chassis[i]->fdb.vel * (60.0f /(2.0f * M_PI)) *
                                     g_chassis_power_control.motor_chassis[i]->fdb.vel * (60.0f /(2.0f * M_PI)) +
                                a * motor_current *
                                    motor_current +
                                constant;

        if (initial_give_power[i] < 0)  // 负功率不包括在内（瞬态）
            continue;
        // initial_total_power += initial_give_power[i];  //缩放前的总功率
        g_initial_total_power_debug += initial_give_power[i];  //缩放前的总功率
    }
    //反向计算新的电流输出
    // if (initial_total_power >
    if (g_initial_total_power_debug >
        chassis_max_power)  // 判断是否大于最大功率（包括裁判系统和超级电容带来的额外的功率）
    {
        fp32 power_scale = chassis_max_power / g_initial_total_power_debug;  //得到功率缩放系数
        for (uint8_t i = 0; i < 4; i++) {
            scaled_give_power[i] = initial_give_power[i] * power_scale;  // 获取缩放后的功率
            if (scaled_give_power[i] < 0) {
                continue;
            }

            //拟合电机功率的模型
            fp32 b = toque_coefficient * 
                     g_chassis_power_control.motor_chassis[i]->fdb.vel * (60.0f / (2.0f * M_PI));
            fp32 c = a * g_chassis_power_control.motor_chassis[i]->fdb.vel * (60.0f /(2.0f * M_PI)) *   
                         g_chassis_power_control.motor_chassis[i]->fdb.vel * (60.0f /(2.0f * M_PI)) -
                         scaled_give_power[i] + constant;
            fp32 inside = b * b - 4 * a * c;
            //无解，保持原来的状态
            if (inside < 0) 
            {
                continue;   
            }

            else if (*g_chassis_power_control.motor_current_cmd[i] > 0)  // 根据原始力矩的方向选择计算公式
            {
                fp32 temp = (-b + sqrt(inside)) / (2 * a);
                if (temp > 16000)  //如果超过最大电流，以最大电流位准
                {
                    *g_chassis_power_control.motor_current_cmd[i] = 16000;
                } 
                else
                    *g_chassis_power_control.motor_current_cmd[i] = temp;
            } 
            else 
            {
                fp32 temp = (-b - sqrt(inside)) / (2 * a);
                if (temp < -16000) {
                    *g_chassis_power_control.motor_current_cmd[i] = -16000;
                } else
                    *g_chassis_power_control.motor_current_cmd[i] = temp;
            }
        }
    }
}

/*---------------------------为了和西交利物浦保持一致，这里封装了一层-------------------------*/
/**
 * @brief          获取底盘功率和缓冲能量
 * @param[out]     chassis_power: 底盘功率指针  
 * @param[out]     chassis_power_buffer: 缓冲能量指针
 * @retval         none
 */
void get_chassis_power_and_buffer_adapter(fp32 * chassis_power, fp32 * chassis_power_buffer)
{
    // 调用框架中已存在的裁判系统函数
    get_chassis_power_and_buffer(chassis_power, chassis_power_buffer);
}

/**
 * @brief          获取底盘最大功率限制
 * @param[out]     max_power_limit: 最大功率限制指针
 * @retval         none  
 */
void get_chassis_max_power_adapter(uint16_t * max_power_limit)
{
    // 从裁判系统机器人状态获取功率限制
    extern robot_status_t robot_status;
    *max_power_limit = robot_status.chassis_power_limit;
}

/**
 * @brief          超级电容控制命令
 * @param[in]      input_power: 输入功率
 * @retval         none
 */
void CAN_CMD_CAP(float input_power)
{
    // 将功率转换为合适的参数发送给超级电容
    uint16_t power_buffer = (uint16_t)g_chassis_power_control.buffer_pid.out;
    uint16_t power_limit = (uint16_t)input_power;
    uint8_t automode_stage = g_chassis_power_control.cap_enable_state;
    // 调用框架已有的超级电容控制函数
    CanCmdSupCap(1, power_buffer, power_limit, automode_stage);
}
