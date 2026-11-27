"""用硬件替身执行实际云台、底盘和发射 C 文件，检查最终发送门控。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

# 仅替换硬件访问；参数、控制文件和 VT13 状态机均使用仓库实际文件。
HARDWARE = r'''
#ifndef TEST_HW_H
#define TEST_HW_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define STRUCT_TYPEDEF_H
typedef float fp32;
typedef double fp64;
typedef unsigned char bool_t;
#undef M_PI
#define M_PI 3.14159265358979323846f
#define PI M_PI
#include "robot_typedef.h"
#include "macro_typedef.h"
#include "vt13_control.h"
#include "supervisory_computer_cmd.h"
#ifdef TEST_SHOOT
#include "pid.h"
#else
#define PID_H
enum { PID_POSITION, PID_DELTA };
typedef struct { float Kp, out, Iout, max_out; } pid_type_def;
/* 初始化 PID 替身参数，保留门控测试所需的比例和限幅。 */
static void PID_init(pid_type_def *p, uint8_t mode, const float gains[3], float out, float iout) {
    (void)mode; (void)iout; memset(p,0,sizeof(*p)); p->Kp=gains[0]; p->max_out=out;
}
static float PID_calc(pid_type_def *p, float feedback, float reference) {
    p->out=p->Kp*(reference-feedback);
    /* 正向输出超过上限时饱和，避免无界输出掩盖门控结果。 */
    if(p->out > p->max_out) p->out=p->max_out;
    /* 负向输出超过下限时对称限幅。 */
    if(p->out < -p->max_out) p->out=-p->max_out;
    return p->out;
}
/* 清除替身输出与积分，用于核验失能复位路径。 */
static void PID_clear(pid_type_def *p) { p->out=p->Iout=0; }
#endif
typedef struct { float pos,vel,tor,curr; int state,ecd; } TestMotorValue;
typedef struct { TestMotorValue set,fdb; bool offline; int direction; } Motor_s;
/* 初始化电机替身并保存方向，不访问实际 CAN 电机。 */
static void MotorInit(Motor_s *m,int id,int can,int type,int direction,float ratio,int mode) {
    (void)id;(void)can;(void)type;(void)ratio;(void)mode; memset(m,0,sizeof(*m));m->direction=direction;
}
/* 反馈由用例直接设置，此处不读取硬件或改写测试反馈。 */
static void GetMotorMeasure(Motor_s *m) { (void)m; }
typedef struct { struct { int16_t ch[5]; uint8_t s[2]; } rc; } RC_ctrl_t;
static RC_ctrl_t dummy_rc;
static const RC_ctrl_t *get_remote_control_point(void) { return &dummy_rc; }
typedef struct { float dummy; } Imu_t;
typedef struct { struct { float voltage_cap; } fdb; } SupCap_s;
/* 超电只替换硬件反馈和发送，功率限制使用实际源码。 */
static void SupCapInit(SupCap_s *cap, uint8_t can) { (void)can; memset(cap,0,sizeof(*cap)); }
static void GetSupCapMeasure(SupCap_s *cap) { (void)cap; }
typedef struct { int robot_level; uint16_t chassis_power_limit; } robot_status_t;
static robot_status_t robot_status;
/* 保持缓冲在目标值，单独检验四轮电流的限制与发送链路。 */
static void get_chassis_power_and_buffer(float *power, float *buffer) { *power=0; *buffer=30; }
static void CanCmdSupCap(uint8_t can, uint16_t buffer, uint16_t limit, uint8_t mode) {
    (void)can; (void)buffer; (void)limit; (void)mode;
}
/* 既有独立诊断只检查各自控制链；功率接线回归显式启用真实模块。 */
#ifndef TEST_REAL_POWER
#define CHASSIS_POWER_CONTROL_H
static void chassis_power_control_init(void *chassis) { (void)chassis; }
#endif
static uint32_t now_ms=100, irq_mask;
typedef uint32_t TickType_t;
#define configTICK_RATE_HZ 1000
static uint32_t HAL_GetTick(void) { return now_ms; }
static TickType_t xTaskGetTickCount(void) { return now_ms; }
static uint32_t __get_PRIMASK(void) { return irq_mask; }
/* 记录替身中断屏蔽状态，用于执行实际快照代码。 */
static void __disable_irq(void) { irq_mask=1; }
/* 恢复替身屏蔽状态，不影响主机真实中断。 */
static void __set_PRIMASK(uint32_t v) { irq_mask=v; }
static bool sensor_error, gyro_error, accel_error;
#define DBUS_TOE 0
#define BOARD_GYRO_TOE 1
#define BOARD_ACCEL_TOE 2
static bool toe_is_error(uint8_t toe) {
    return sensor_error || (toe==BOARD_GYRO_TOE&&gyro_error) || (toe==BOARD_ACCEL_TOE&&accel_error);
}
static float imu_angle[3], imu_velocity[3];
static float GetImuAngle(uint8_t a) { return imu_angle[a]; }
static float GetImuVelocity(uint8_t a) { return imu_velocity[a]; }
static float fp32_constrain(float x,float lo,float hi) { return x<lo?lo:(x>hi?hi:x); }
static float fp32_deadline(float x,float lo,float hi) { return x>=lo&&x<=hi?0:x; }
static float loop_fp32_constrain(float x,float lo,float hi) {
    /* 保留原折返方式；非有限输入应由实际控制代码提前拦截。 */
    while(x<lo)x+=hi-lo; while(x>hi)x-=hi-lo; return x;
}
static float theta_format(float x) { return loop_fp32_constrain(x,-M_PI,M_PI); }
static float sent[3][2][4], last_torque;
static unsigned enables, stops;
/* 按总线和帧 ID 保存最终发送电流，供失能断言检查。 */
static void CanCmdDjiMotor(uint8_t can,uint16_t id,float a,float b,float cc,float d) {
    /* 0x200 与另一组电流帧分开记录，防止不同机构互相覆盖。 */
    unsigned slot=id==0x200?0:1; sent[can][slot][0]=a;sent[can][slot][1]=b;
    sent[can][slot][2]=cc;sent[can][slot][3]=d;
}
#define DM_STATE_DISABLE 0
/* 记录使能次数，检查失能路径没有重新开启电机。 */
static void DmEnable(Motor_s *m) { (void)m; ++enables; }
/* 记录达妙停机请求，并把最后发送力矩置零。 */
static void DmMitStop(Motor_s *m) { (void)m; ++stops; last_torque=0; }
/* 记录实际控制文件要求发送的 yaw 力矩。 */
static void DmMitCtrlTorque(Motor_s *m) { last_torque=m->set.tor; }
static Vt13Input raw;
static VisionControlInput vision;
/* 读取用例设置的原始遥控快照，替代真实串口接收。 */
void Vt13ReadInput(Vt13Input *out) { *out=raw; }
/* 读取用例设置的视觉命令，替代 USB 接收。 */
void VisionReadControl(VisionControlInput *out) { *out=vision; }
/* 沿用固件原始 fire 接口，视觉有效性由完整快照另外检查。 */
bool GetScCmdFire(void) { return vision.fire; }
/* 实际云台用自己的状态发布器，其余模块使用状态替身。 */
#ifndef TEST_GIMBAL
static GimbalControlStatus fake_status;
/* 向底盘和发射用例提供完整云台状态副本。 */
void GimbalReadControlStatus(GimbalControlStatus *out) { *out=fake_status; }
float GetGimbalDeltaYawMid(void) { return fake_status.yaw_joint; }
#endif
/* 实际底盘提供自己的角速度接口，其余用例设为静止。 */
#ifndef TEST_CHASSIS
float ChassisGetSpeedWz(void) { return 0; }
#endif
/* 建立有效 N 档输入，非云台用例还建立已就绪的云台状态。 */
static void enable_remote(void) {
    memset(&raw,0,sizeof(raw));raw.valid=true;raw.mode_switch=1;raw.wheel=1024;
#ifndef TEST_GIMBAL
    memset(&fake_status,0,sizeof(fake_status));fake_status.ready=true;
    fake_status.feedback_valid=true;fake_status.init_state=VT13_INIT_READY;
    fake_status.update_time_ms=now_ms;
#endif
}
#endif
'''

GIMBAL = r'''
/* 按真实云台任务顺序执行一拍，包含状态、参考、控制与发送。 */
static void step(void) {
    ++now_ms;GimbalObserver();GimbalSetMode();GimbalReference();GimbalConsole();GimbalSendCmd();
}

/* 验证实际云台对鼠标优先、重复帧和右键进出自瞄的处理。 */
static void check_keyboard_mouse_gimbal(void) {
    raw.mode_switch=0;step();enable_remote();sensor_error=false;
    gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;
    gimbal_direct.pitch.fdb.pos=GIMBAL_DIRECT_PITCH_MID;
    imu_angle[AX_YAW]=imu_angle[AX_PITCH]=0;
    step();step();assert(gimbal_direct.mode==GIMBAL_IMU);
    raw.normalized[0]=raw.normalized[1]=1;
    /* 从非零目标开始验证反向，避免整体取负在零点上偶然通过。 */
    gimbal_direct.reference.pitch=gimbal_direct.angle_zero_for_imu+0.1f;
    raw.mouse_x=400;raw.mouse_y=-200;++raw.sequence;
    float yaw_before=gimbal_direct.reference.yaw,pitch_before=gimbal_direct.reference.pitch;
    step();
    /* 保留当前灵敏度，只反转 pitch 鼠标增量，不能翻转原有参考角。 */
    /* 当前原分母10000，yaw单独提高至4倍：400 counts对应0.16 rad，pitch原比例保持。 */
    assert(fabsf(gimbal_direct.reference.yaw-(yaw_before-0.16f))<0.00001f);
    assert(fabsf(gimbal_direct.reference.pitch-(pitch_before-200.0f/MOUSE_SENSITIVITY))<0.00001f);
    yaw_before=gimbal_direct.reference.yaw;pitch_before=gimbal_direct.reference.pitch;
    for(int i=0;i<20;++i)step();
    assert(gimbal_direct.reference.yaw==yaw_before&&gimbal_direct.reference.pitch==pitch_before);
    /* 正反大位移必须在原来的不对称上下限内，限幅后的结果不能再整体取负。 */
    raw.mouse_x=0;raw.mouse_y=32767;++raw.sequence;step();
    assert(fabsf(gimbal_direct.reference.pitch-(GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu))<0.00001f);
    raw.mouse_y=-32768;++raw.sequence;step();
    assert(fabsf(gimbal_direct.reference.pitch-(GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu))<0.00001f);
    raw.mouse_x=raw.mouse_y=0;++raw.sequence;step();
    assert(fabsf(gimbal_direct.reference.yaw-(yaw_before-660.0f/REMOTE_CONTROLLER_SENSITIVITY))<0.00001f);
    raw.mouse_right=1;vision.valid=true;vision.fire=true;step();
    assert(gimbal_direct.mode==GIMBAL_AUTO_AIM);
    raw.mouse_right=0;step();assert(gimbal_direct.mode==GIMBAL_IMU);
    assert(gimbal_direct.reference.yaw==gimbal_direct.feedback_pos.yaw);
    raw.mouse_right=1;vision.valid=false;step();assert(gimbal_direct.mode==GIMBAL_IMU);
    raw.mode_switch=0;step();assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0);
}
/* 失联在两个撤销入口均立即停机，恢复有效N/S后沿原流程重新回中。 */
static void check_input_loss_recovery(void) {
    for(unsigned mode=1;mode<=2;++mode) {
        for(unsigned ready=0;ready<=1;++ready) {
            for(unsigned at_send=0;at_send<=1;++at_send) {
                GimbalInit();enable_remote();raw.mode_switch=mode;vision.valid=false;
                sensor_error=gyro_error=accel_error=false;
                gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;
                gimbal_direct.pitch.fdb.pos=GIMBAL_DIRECT_PITCH_MID;
                for(int i=0;i<3;++i)imu_angle[i]=imu_velocity[i]=0;
                step();if(ready)step();
                assert(gimbal_init_control.state==(ready?VT13_INIT_READY:VT13_INIT_RUNNING));
                raw.valid=false;unsigned before=enables;
                if(at_send)GimbalSendCmd();else step();
                assert(gimbal_init_control.state==VT13_INIT_DISABLED&&!gimbal_direct.init_continue);
                assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0&&enables==before);
                raw.valid=true;step();assert(gimbal_direct.mode==GIMBAL_INIT);
                step();assert(gimbal_init_control.state==VT13_INIT_READY);
            }
        }
    }
}
/* 实际云台不能因不可用键鼠段而失去基础初始化，也不能误消费默认键盘/鼠标量。 */
static void check_optional_keymouse_remote_only(void) {
    for(unsigned mode=1;mode<=2;++mode) {
        GimbalInit();enable_remote();raw.mode_switch=mode;raw.update_time_ms=now_ms;
        sensor_error=gyro_error=accel_error=false;vision.valid=false;
        gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;
        gimbal_direct.pitch.fdb.pos=GIMBAL_DIRECT_PITCH_MID;
        for(int i=0;i<3;++i)imu_angle[i]=imu_velocity[i]=0;
        raw.mouse_left=3;raw.mouse_right=raw.mouse_middle=1;
        raw.mouse_x=32767;raw.mouse_y=-32768;raw.keyboard=0xffff;
        step();assert(gimbal_direct.mode==GIMBAL_INIT);
        step();assert(gimbal_direct.mode==GIMBAL_IMU&&gimbal_direct.init_continue);
        for(unsigned i=0;i<200;++i){++raw.sequence;step();}
        assert(gimbal_init_control.state==VT13_INIT_READY);
        assert(!gimbal_input.keyboard_move&&!gimbal_input.spin_down);
        assert(!gimbal_input.mouse_left&&!gimbal_input.mouse_right);
        assert(gimbal_direct.reference.yaw==gimbal_direct.feedback_pos.yaw);
        assert(gimbal_direct.reference.pitch==gimbal_direct.feedback_pos.pitch);
        raw.mode_switch=0;step();assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0);
    }
}
int main(void) {
    GimbalControlStatus status;
    enable_remote();GimbalInit();
    gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;
    gimbal_direct.pitch.fdb.pos=GIMBAL_DIRECT_PITCH_MID;
    step();assert(gimbal_direct.mode==GIMBAL_INIT);
    GimbalReadControlStatus(&status);assert(!status.ready);
    step();assert(gimbal_direct.mode==GIMBAL_IMU);
    GimbalReadControlStatus(&status);assert(status.ready);
    raw.mode_switch=2;vision.valid=true;vision.fire=false;
    step();assert(gimbal_direct.mode==GIMBAL_AUTO_AIM);
    GimbalReadControlStatus(&status);assert(status.auto_active&&!status.vision_fire);
    vision.fire=true;step();GimbalReadControlStatus(&status);assert(status.vision_fire);
    raw.mode_switch=1;step();assert(gimbal_direct.mode==GIMBAL_IMU);
    raw.mode_switch=2;vision.valid=false;step();assert(gimbal_direct.mode==GIMBAL_IMU);
    gimbal_direct.pitch.set.curr=123;gimbal_direct.yaw.set.tor=4;
    raw.mode_switch=0;
    unsigned previous_enables=enables;
    GimbalSendCmd();assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0);
    assert(enables==previous_enables);
    GimbalReadControlStatus(&status);assert(!status.ready);
    raw.mode_switch=1;gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID+1;
    step();now_ms+=GIMBAL_INIT_TIME;step();
    assert(gimbal_init_control.state==VT13_INIT_FAILED);
    assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0);
    gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;step();
    assert(gimbal_init_control.state==VT13_INIT_FAILED);
    raw.mode_switch=0;step();raw.mode_switch=1;step();step();
    assert(gimbal_init_control.state==VT13_INIT_READY);
    sensor_error=true;step();assert(gimbal_init_control.state==VT13_INIT_FAILED);
    assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0);
    sensor_error=false;raw.mode_switch=0;step();raw.mode_switch=1;
    gimbal_direct.yaw.fdb.pos=INFINITY;imu_angle[AX_YAW]=INFINITY;
    step();assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0);
    assert(isfinite(gimbal_direct_pid.yaw_angle.out));
    gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;imu_angle[AX_YAW]=0;
    step();assert(gimbal_init_control.state==VT13_INIT_READY);
    check_keyboard_mouse_gimbal();
    check_input_loss_recovery();
    check_optional_keymouse_remote_only();
    puts("actual gimbal state, mouse and send gates: passed");return 0;
}
'''

CHASSIS = r'''
/* 验证实际功率模块原地修改最终指令，发送使用同一份电流且失能仍清零。 */
static void check_power_current_path(void) {
    ChassisInit();ChassisForceDisable();enable_remote();ChassisSetMode();
    robot_status.chassis_power_limit=40;
    assert(g_chassis_power_control.super_cap==&chassis.super_cap);
    for(int i=0;i<4;++i) {
        assert(g_chassis_power_control.motor_chassis[i]==&chassis.wheel[i]);
        assert(g_chassis_power_control.motor_current_cmd[i]==&chassis.wheel[i].set.curr);
        chassis.wheel[i].fdb.vel=0;
        chassis.wheel[i].set.curr=(i%2?-1:1)*1000;
    }
    chassis_power_control();
    for(int i=0;i<4;++i) assert(chassis.wheel[i].set.curr==(i%2?-1:1)*1000);
    for(int i=0;i<4;++i) chassis.wheel[i].set.curr=(i%2?-1:1)*12000;
    chassis_power_control();ChassisSendCmd();
    assert(g_initial_total_power_debug>40);
    for(int i=0;i<4;++i) {
        assert(fabsf(chassis.wheel[i].set.curr)<12000);
        assert(chassis.wheel[i].set.curr*(i%2?-1:1)>0);
        assert(sent[CHASSIS_CAN][0][i]==chassis.wheel[i].set.curr);
    }
    raw.valid=false;ChassisSendCmd();
    for(int i=0;i<4;++i) assert(sent[CHASSIS_CAN][0][i]==0);
    puts("actual power control: pointers, pass-through, limiting, send and disable passed");
}


/* 验证 WASD 覆盖摇杆、Shift 小陀螺及空闲交回摇杆的实际参考链路。 */
static void check_keyboard_chassis(void) {
    ChassisInit();ChassisForceDisable();enable_remote();chassis.P_wz=0;
    raw.normalized[3]=-1;raw.keyboard=1;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_FOLLOW);
    /* 第一拍即按最高速度生成平移参考，不需要再等一个调度周期。 */
    assert(chassis.reference.vx==CHASSIS_RC_MAX_SPEED&&chassis.reference.vy==0);
    now_ms+=20;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.reference.vx>0&&chassis.reference.vy==0);
    raw.keyboard=17;now_ms+=20;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_SPIN);
    raw.keyboard=1;now_ms+=20;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_SPIN_RETURN);
    now_ms+=100;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_FOLLOW);
    /* 收到松键帧且摇杆居中后，本拍平移参考就为零，不保留一秒速度拖尾。 */
    raw.keyboard=0;raw.normalized[3]=0;
    ChassisSetMode();ChassisReference();
    assert(chassis.reference.vx==0&&chassis.reference.vy==0);
    /* 零目标不等于零电流：有前向反馈时保留反向制动力，而不是继续前向推力。 */
    chassis.P_vx=1.0f;chassis.P_vy=0;ChassisConsole();
    assert(chassis.force_x<0);
    chassis.P_vx=0;
    /* 摇杆本身有输入时，松键后立即按摇杆移动，不能误当成必须停车。 */
    raw.normalized[3]=-1;ChassisSetMode();ChassisReference();
    assert(chassis.reference.vx==-CHASSIS_RC_MAX_SPEED);
    raw.mouse_right=1;fake_status.auto_active=true;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_BODY&&chassis.reference.wz==0);
    raw.mode_switch=0;ChassisSendCmd();
    for(int i=0;i<4;++i)assert(sent[CHASSIS_CAN][0][i]==0);
}
/* 验证跟随、旋转、松手回正、稳定后恢复跟随的实际模式链路及最终速度边界。 */
static void check_follow_return_wz_limit(void) {
    const float directions[2]={-1.0f,1.0f};
    raw.mode_switch=0;ChassisSetMode();
    enable_remote();
    /* 使用实际等级起点，令原小陀螺转速高于当前回正限速，检查两者独立。 */
    robot_status.robot_level=1;
    for(int i=0;i<2;++i) {
        float direction=directions[i];
        fake_status.yaw_joint=direction;
        chassis.P_wz=8.0f*direction;
        ChassisSetMode();ChassisReference();
        assert(chassis.mode==CHASSIS_FOLLOW);
        assert(fabsf(chassis.reference.wz+direction*MAX_OUT_CHASSIS_FOLLOW_GIMBAL)<0.0001f);
        raw.trigger=true;ChassisSetMode();ChassisReference();
        assert(chassis.mode==CHASSIS_SPIN);
        assert(chassis.reference.wz==chassis.spin_target_wz);
        assert(chassis.reference.wz>CHASSIS_SPIN_RETURN_MAX_WZ);
        raw.trigger=false;ChassisSetMode();ChassisReference();
        assert(chassis.mode==CHASSIS_SPIN_RETURN);
        assert(fabsf(chassis.reference.wz+direction*CHASSIS_SPIN_RETURN_MAX_WZ)<0.0001f);
        /* 角度误差很小时也要限制阻尼叠加后的最终参考，不能仅限制 PID 输出。 */
        fake_status.yaw_joint=0.01f*direction;
        chassis.P_wz=-8.0f*direction;
        ChassisSetMode();ChassisReference();
        assert(chassis.mode==CHASSIS_SPIN_RETURN);
        assert(fabsf(chassis.reference.wz-direction*CHASSIS_SPIN_RETURN_MAX_WZ)<0.0001f);
        fake_status.yaw_joint=0;chassis.P_wz=0;
        ChassisSetMode();ChassisReference();assert(chassis.reference.wz==0);
        now_ms+=VT13_RETURN_SETTLE_MS;fake_status.update_time_ms=now_ms;
        ChassisSetMode();ChassisReference();
        assert(chassis.mode==CHASSIS_FOLLOW&&chassis.reference.wz==0);
        /* 回正结束后再次给大误差，确认正常跟随恢复原来的快速响应。 */
        fake_status.yaw_joint=direction;chassis.P_wz=8.0f*direction;
        ChassisSetMode();ChassisReference();
        assert(chassis.mode==CHASSIS_FOLLOW);
        assert(fabsf(chassis.reference.wz+direction*MAX_OUT_CHASSIS_FOLLOW_GIMBAL)<0.0001f);
    }
}

int main(void) {
    enable_remote();ChassisInit();raw.normalized[3]=1;
    fake_status.yaw_joint=1;
    ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_FOLLOW);
    assert(fabsf(chassis.reference.vx-CHASSIS_RC_MAX_SPEED*cosf(1))<0.0001f);
    raw.mode_switch=2;fake_status.auto_active=true;
    ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_BODY&&chassis.reference.wz==0);
    assert(chassis.reference.vx==CHASSIS_RC_MAX_SPEED&&chassis.reference.vy==0);
    /* 右上功能键不能触发小陀螺，仍保持有目标时的底盘坐标模式。 */
    raw.function_right=true;ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_BODY);
    raw.function_right=false;
    /* 右上扳机才触发旋转，松开后进入原有回正流程。 */
    raw.trigger=true;ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN);
    raw.trigger=false;fake_status.yaw_world=1.2f;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_SPIN_RETURN);
    /* S 档有目标保留松手固定朝向；云台移动不改变已保存目标。 */
    assert(fabsf(chassis_spin_control.return_heading-1.2f)<0.0001f);
    fake_status.yaw_world=2;fake_status.yaw_joint=0.8f;
    ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN_RETURN);
    assert(fabsf(chassis_spin_control.return_heading-1.2f)<0.0001f);
    assert(chassis.reference.wz==0);
    for(int i=0;i<4;++i)chassis.wheel[i].set.curr=123;
    raw.valid=false;ChassisSendCmd();
    for(int i=0;i<4;++i)assert(sent[CHASSIS_CAN][0][i]==0);
    assert(!chassis_spin_control.returning);
    enable_remote();fake_status.ready=false;
    ChassisSetMode();ChassisReference();ChassisConsole();ChassisSendCmd();
    assert(chassis.mode==CHASSIS_LOCK);
    for(int i=0;i<4;++i)assert(sent[CHASSIS_CAN][0][i]==0);
    check_follow_return_wz_limit();
    check_keyboard_chassis();
    check_power_current_path();
    puts("actual chassis keyboard, coordinates, wz limits and send gates: passed");return 0;
}
'''

SHOOT = r'''
/* 按真实发射任务顺序执行一拍，检查模式到发送门控的完整路径。 */
static void step(void) { ShootSetMode();ShootReference();ShootConsole();ShootSendCmd(); }

/* 复核现场 N 档未按右键时，第二次 Fn 是否贯穿关轮、停拨弹及最终发送。 */
static void check_n_fn_second_press_stops(void) {
    for (int auto_active=0;auto_active<=1;++auto_active) {
        for (int left=0;left<=1;++left) {
            enable_remote();ShootInit();ShootForceDisable();
            fake_status.auto_active=auto_active;
            fake_status.vision_fire=true;vision.valid=vision.fire=true;
            raw.mouse_left=left;raw.mouse_right=0;raw.wheel=364;
            SHOOT.FDB.fric_speed_fdb_R=-600;SHOOT.FDB.fric_speed_fdb_L=600;
            step();
            raw.function_left=true;step();
            assert(shoot_control.friction_on&&SHOOT.fric_flag);
            assert(SHOOT.mode==LOAD_BURSTFIRE);
            raw.function_left=false;step();
            assert(shoot_control.friction_on&&SHOOT.fric_flag);
            raw.function_left=true;
            for (int repeat=0;repeat<10;++repeat) {
                step();
                assert(!shoot_control.friction_on&&!SHOOT.fric_flag);
                assert(SHOOT.state==FRIC_NOT_READY&&SHOOT.mode==LOAD_STOP);
                assert(SHOOT.REF.fric_speed_ref_R==0&&SHOOT.REF.fric_speed_ref_L==0);
                assert(sent[FRIC_MOTOR_R_CAN][0][0]>0&&sent[FRIC_MOTOR_R_CAN][0][1]<0);
                assert(sent[TRIGGER_MOTOR_CAN][1][1]==0);
            }
        }
    }
    puts("N Fn second press: zero speed targets, braking current retained and feeding stopped: passed");
}

/* 用真实PID验证N/S关轮、双向制动、稳定零速和失能仍为零电流。 */
static void check_zero_speed_braking(void) {
    for(int mode=1;mode<=2;++mode) {
        for(int direction=-1;direction<=1;direction+=2) {
            enable_remote();ShootInit();ShootForceDisable();raw.mode_switch=mode;
            SHOOT.FDB.fric_speed_fdb_R=100*direction;
            SHOOT.FDB.fric_speed_fdb_L=-100*direction;
            ShootSetMode();ShootReference();ShootConsole();
            float current_r=SHOOT.fric_motor[0].set.curr,current_l=SHOOT.fric_motor[1].set.curr;
            ShootSendCmd();
            assert(!SHOOT.fric_flag&&SHOOT.mode==LOAD_STOP);
            assert(sent[FRIC_MOTOR_R_CAN][0][0]==current_r&&sent[FRIC_MOTOR_R_CAN][0][1]==current_l);
            assert(sent[FRIC_MOTOR_R_CAN][0][0]*SHOOT.FDB.fric_speed_fdb_R<0);
            assert(sent[FRIC_MOTOR_R_CAN][0][1]*SHOOT.FDB.fric_speed_fdb_L<0);
            SHOOT.FDB.fric_speed_fdb_R=SHOOT.FDB.fric_speed_fdb_L=0;
            step();step();
            assert(sent[FRIC_MOTOR_R_CAN][0][0]==0&&sent[FRIC_MOTOR_R_CAN][0][1]==0);
            SHOOT.FDB.fric_speed_fdb_R=100*direction;
            SHOOT.FDB.fric_speed_fdb_L=-100*direction;
            step();raw.mode_switch=0;ShootSendCmd();
            assert(sent[FRIC_MOTOR_R_CAN][0][0]==0&&sent[FRIC_MOTOR_R_CAN][0][1]==0);
            assert(SHOOT.fric_pid[0].out==0&&SHOOT.fric_pid[1].out==0);
        }
    }
}

/* 发送前右键释放或S档目标失效时，不沿用按开轮速度计算的旧驱动电流。 */
static void check_auto_exit_before_send_brakes(void) {
    for(int mode=1;mode<=2;++mode) {
        enable_remote();ShootInit();ShootForceDisable();raw.mode_switch=mode;
        raw.mouse_right=1;fake_status.auto_active=true;
        SHOOT.FDB.fric_speed_fdb_R=-300;SHOOT.FDB.fric_speed_fdb_L=300;
        ShootSetMode();ShootReference();ShootConsole();
        assert(SHOOT.fric_motor[0].set.curr<0&&SHOOT.fric_motor[1].set.curr>0);
        if(mode==1) raw.mouse_right=0;
        else fake_status.auto_active=false;
        ShootSendCmd();
        assert(!SHOOT.fric_flag&&SHOOT.mode==LOAD_STOP);
        assert(SHOOT.REF.fric_speed_ref_R==0&&SHOOT.REF.fric_speed_ref_L==0);
        assert(sent[FRIC_MOTOR_R_CAN][0][0]>0&&sent[FRIC_MOTOR_R_CAN][0][1]<0);
        assert(sent[TRIGGER_MOTOR_CAN][1][1]==0);
    }
}

/* 验证 N 档左右键以及 S 档自瞄均不能绕过视觉或机构安全许可。 */
static void check_mouse_shoot_gates(void) {
    enable_remote();ShootInit();ShootForceDisable();
    SHOOT.FDB.fric_speed_fdb_R=-600;SHOOT.FDB.fric_speed_fdb_L=600;
    raw.mouse_left=1;step();assert(SHOOT.mode==LOAD_STOP&&!SHOOT.fric_flag);
    raw.function_left=true;step();assert(SHOOT.mode==LOAD_BURSTFIRE);
    raw.mouse_left=0;step();assert(SHOOT.mode==LOAD_STOP);
    raw.mouse_right=1;fake_status.auto_active=true;fake_status.vision_fire=true;
    vision.valid=vision.fire=true;shoot_control.friction_on=false;raw.function_left=false;
    step();assert(SHOOT.fric_flag&&SHOOT.state==FRIC_READY&&SHOOT.mode==LOAD_STOP);
    raw.mouse_left=1;step();assert(SHOOT.mode==LOAD_BURSTFIRE);
    raw.wheel=364;vision.fire=false;fake_status.vision_fire=false;
    step();assert(SHOOT.mode==LOAD_STOP);
    vision.fire=true;fake_status.vision_fire=true;step();assert(SHOOT.mode==LOAD_BURSTFIRE);
    raw.mouse_left=0;ShootSendCmd();assert(sent[TRIGGER_MOTOR_CAN][1][1]==0);
    raw.mouse_left=1;raw.wheel=1024;fake_status.auto_active=false;vision.valid=false;
    step();assert(SHOOT.mode==LOAD_STOP&&!SHOOT.fric_flag);
    raw.function_left=true;step();assert(SHOOT.mode==LOAD_BURSTFIRE);
    raw.mode_switch=0;ShootSendCmd();
    assert(sent[FRIC_MOTOR_R_CAN][0][0]==0&&sent[TRIGGER_MOTOR_CAN][1][1]==0);
}
int main(void) {
    enable_remote();ShootInit();
    SHOOT.FDB.fric_speed_fdb_R=-600;SHOOT.FDB.fric_speed_fdb_L=600;
    step();raw.function_left=true;step();assert(SHOOT.fric_flag);
    step();step();assert(SHOOT.fric_flag);
    raw.wheel=364;step();assert(SHOOT.mode==LOAD_BURSTFIRE);
    assert(sent[TRIGGER_MOTOR_CAN][1][1]!=0);
    raw.wheel=1024;fake_status.auto_active=true;fake_status.vision_fire=true;
    vision.valid=vision.fire=true;
    step();assert(SHOOT.mode==LOAD_STOP);
    raw.mode_switch=2;step();assert(SHOOT.mode==LOAD_STOP&&SHOOT.fric_flag);
    /* 视觉已允许仍不能直接发弹，必须按住左键确认。 */
    raw.mouse_left=1;step();assert(SHOOT.mode==LOAD_BURSTFIRE);
    /* S 档有效目标自动开轮；视觉停火不能被拨轮覆盖。 */
    shoot_control.friction_on=false;
    step();assert(SHOOT.fric_flag&&SHOOT.state==FRIC_READY);
    assert(SHOOT.mode==LOAD_BURSTFIRE&&!shoot_control.friction_on);
    raw.wheel=364;vision.fire=false;fake_status.vision_fire=false;
    step();assert(SHOOT.mode==LOAD_STOP&&SHOOT.state==FRIC_READY);
    vision.fire=true;fake_status.vision_fire=true;
    SHOOT.trigger_motor.offline=true;step();assert(SHOOT.mode==LOAD_STOP);
    SHOOT.trigger_motor.offline=false;
    SHOOT.FDB.fric_speed_fdb_L=100;step();assert(SHOOT.mode==LOAD_STOP);
    SHOOT.FDB.fric_speed_fdb_L=600;step();assert(SHOOT.mode==LOAD_BURSTFIRE);
    fake_status.vision_fire=true;ShootSetMode();ShootReference();ShootConsole();
    vision.valid=false;ShootSendCmd();assert(sent[TRIGGER_MOTOR_CAN][1][1]==0);
    raw.mode_switch=1;raw.wheel=364;fake_status.auto_active=false;step();
    assert(SHOOT.mode==LOAD_STOP&&!SHOOT.fric_flag);
    raw.function_left=false;step();raw.function_left=true;step();
    assert(SHOOT.mode==LOAD_BURSTFIRE);
    SHOOT.FDB.fric_speed_fdb_L=100;step();assert(sent[TRIGGER_MOTOR_CAN][1][1]==0);
    SHOOT.FDB.fric_speed_fdb_L=600;step();
    raw.mode_switch=0;ShootSendCmd();
    assert(sent[FRIC_MOTOR_R_CAN][0][0]==0&&sent[FRIC_MOTOR_R_CAN][0][1]==0);
    assert(sent[TRIGGER_MOTOR_CAN][1][1]==0&&!shoot_control.friction_on);
    raw.mode_switch=1;step();assert(!SHOOT.fric_flag);
    raw.function_left=false;step();raw.function_left=true;step();assert(SHOOT.fric_flag);
    fake_status.ready=false;ShootSendCmd();
    assert(sent[FRIC_MOTOR_R_CAN][0][0]==0&&sent[TRIGGER_MOTOR_CAN][1][1]==0);
    check_n_fn_second_press_stops();
    check_zero_speed_braking();
    check_auto_exit_before_send_brakes();
    check_mouse_shoot_gates();
    puts("actual shooting mouse interlocks and send gates: passed");return 0;
}
'''


class FirmwareTest(unittest.TestCase):
    def test_actual_control_modules(self):
        compiler = shutil.which("gcc")
        self.assertIsNotNone(compiler, "必须使用 GCC 执行实际控制文件")
        with tempfile.TemporaryDirectory(prefix="vt13-firmware-") as directory:
            work = Path(directory)
            hardware = HARDWARE.replace('#include "pid.h"',
                                        f'#include "{(ROOT / "components/controller/pid.h").as_posix()}"')
            (work / "test_hw.h").write_text(hardware, encoding="utf-8")
            stubs = ("pid.h motor.h remote_control.h IMU.h detect_task.h user_lib.h CAN_cmd_dji.h "
                     "CAN_cmd_damiao.h cmsis_os.h CAN_receive.h usb_debug.h main.h SupCap.h "
                     "IMU_task.h CAN_cmd_SupCap.h referee.h arm_math.h CAN_communication.h voltage_task.h")
            for name in stubs.split():
                (work / name).write_text('#include "test_hw.h"\n', encoding="utf-8")
            includes = [work, ROOT / "components/controller", ROOT / "application", ROOT / "application/typedef",
                        ROOT / "application/remote_control/vt13", ROOT / "application/gimbal",
                        ROOT / "application/chassis", ROOT / "application/shoot",
                        ROOT / "application/communication", ROOT / "application/communication/vofa/observation"]
            for component, suffix in (("gimbal", GIMBAL), ("chassis", CHASSIS), ("shoot", SHOOT)):
                with self.subTest(component=component):
                    filename = {"gimbal": "gimbal_yaw_pitch_direct.c", "chassis": "chassis_omni.c",
                                "shoot": "shoot_fric_trigger.c"}[component]
                    source = ROOT / "application" / component / filename
                    bridge = work / f"{component}.c"
                    # 底盘直接执行真实功率模块，超电通信由硬件替身承接。
                    power_source = (f'#include "{(ROOT / "application/chassis/chassis_power_control.c").as_posix()}"\n'
                                    if component == "chassis" else "")
                    power_define = '#define TEST_REAL_POWER\n' if component == "chassis" else ""
                    pid_source = (f'#include "{(ROOT / "components/controller/pid.c").as_posix()}"\n'
                                  if component == "shoot" else "")
                    bridge.write_text(power_define + f'#define TEST_{component.upper()}\n#include "test_hw.h"\n'
                                      f'#include "{source.as_posix()}"\n' + power_source + pid_source + suffix, encoding="utf-8")
                    # Windows 使用可执行文件后缀，其他主机保持无后缀。
                    executable = work / (component + (".exe" if os.name == "nt" else ""))
                    command = [compiler, "-std=c99", "-fgnu89-inline", "-Werror=implicit-function-declaration"]
                    for include in includes:
                        command += ["-I", str(include)]
                    command += [str(bridge), str(ROOT / "application/remote_control/vt13/vt13_control.c")]
                    # 云台实际代码需要链接原自瞄控制器，另两个模块不需要。
                    if component == "gimbal":
                        command += [str(ROOT / "application/gimbal/yaw_auto_lqr_eso_controller.c")]
                    command += ["-lm", "-o", str(executable)]
                    result = subprocess.run(command, capture_output=True, text=True, errors="replace")
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    result = subprocess.run([str(executable)], capture_output=True, text=True,
                                            errors="replace", timeout=10)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    print(result.stdout.strip())


# 直接运行时编译并执行三个实际控制模块的替身程序。
if __name__ == "__main__":
    unittest.main(verbosity=2)
