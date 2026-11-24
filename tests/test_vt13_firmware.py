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
typedef struct { float dummy; } SupCap_s;
static struct { int robot_level; } robot_status;
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
static bool sensor_error;
#define DBUS_TOE 0
#define BOARD_GYRO_TOE 1
#define BOARD_ACCEL_TOE 2
static bool toe_is_error(uint8_t toe) { (void)toe; return sensor_error; }
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
    raw.mouse_x=400;raw.mouse_y=-200;++raw.sequence;
    float yaw_before=gimbal_direct.reference.yaw,pitch_before=gimbal_direct.reference.pitch;
    step();
    assert(fabsf(gimbal_direct.reference.yaw-(yaw_before-400.0f/MOUSE_SENSITIVITY))<0.00001f);
    assert(fabsf(gimbal_direct.reference.pitch-(pitch_before+200.0f/MOUSE_SENSITIVITY))<0.00001f);
    yaw_before=gimbal_direct.reference.yaw;pitch_before=gimbal_direct.reference.pitch;
    for(int i=0;i<20;++i)step();
    assert(gimbal_direct.reference.yaw==yaw_before&&gimbal_direct.reference.pitch==pitch_before);
    raw.mouse_x=raw.mouse_y=0;++raw.sequence;step();
    assert(fabsf(gimbal_direct.reference.yaw-(yaw_before-660.0f/REMOTE_CONTROLLER_SENSITIVITY))<0.00001f);
    raw.mouse_right=1;vision.valid=true;vision.fire=true;step();
    assert(gimbal_direct.mode==GIMBAL_AUTO_AIM);
    raw.mouse_right=0;step();assert(gimbal_direct.mode==GIMBAL_IMU);
    assert(gimbal_direct.reference.yaw==gimbal_direct.feedback_pos.yaw);
    raw.mouse_right=1;vision.valid=false;step();assert(gimbal_direct.mode==GIMBAL_IMU);
    raw.mode_switch=0;step();assert(last_torque==0&&sent[GIMBAL_CAN][1][2]==0);
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
    puts("actual gimbal state, mouse and send gates: passed");return 0;
}
'''

CHASSIS = r'''

/* 验证 WASD 覆盖摇杆、Shift 小陀螺及空闲交回摇杆的实际参考链路。 */
static void check_keyboard_chassis(void) {
    ChassisInit();ChassisForceDisable();enable_remote();chassis.P_wz=0;
    raw.normalized[3]=-1;raw.keyboard=1;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_FOLLOW);
    now_ms+=20;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.reference.vx>0&&chassis.reference.vy==0);
    raw.keyboard=17;now_ms+=20;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_SPIN);
    raw.keyboard=1;now_ms+=20;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_SPIN_RETURN);
    now_ms+=100;fake_status.update_time_ms=now_ms;
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_FOLLOW);
    raw.keyboard=0;
    for(int i=0;i<500;++i) {
        now_ms+=20;fake_status.update_time_ms=now_ms;
        ChassisSetMode();ChassisReference();
    }
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
    puts("actual chassis keyboard, coordinates, wz limits and send gates: passed");return 0;
}
'''

SHOOT = r'''
/* 按真实发射任务顺序执行一拍，检查模式到发送门控的完整路径。 */
static void step(void) { ShootSetMode();ShootReference();ShootConsole();ShootSendCmd(); }

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
            (work / "test_hw.h").write_text(HARDWARE, encoding="utf-8")
            stubs = ("pid.h motor.h remote_control.h IMU.h detect_task.h user_lib.h CAN_cmd_dji.h "
                     "CAN_cmd_damiao.h cmsis_os.h CAN_receive.h usb_debug.h main.h SupCap.h "
                     "IMU_task.h CAN_cmd_SupCap.h referee.h arm_math.h CAN_communication.h")
            for name in stubs.split():
                (work / name).write_text('#include "test_hw.h"\n', encoding="utf-8")
            includes = [work, ROOT / "application", ROOT / "application/typedef",
                        ROOT / "application/remote_control/vt13", ROOT / "application/gimbal",
                        ROOT / "application/chassis", ROOT / "application/shoot",
                        ROOT / "application/communication", ROOT / "application/communication/vofa/observation"]
            for component, suffix in (("gimbal", GIMBAL), ("chassis", CHASSIS), ("shoot", SHOOT)):
                with self.subTest(component=component):
                    filename = {"gimbal": "gimbal_yaw_pitch_direct.c", "chassis": "chassis_omni.c",
                                "shoot": "shoot_fric_trigger.c"}[component]
                    source = ROOT / "application" / component / filename
                    bridge = work / f"{component}.c"
                    bridge.write_text(f'#define TEST_{component.upper()}\n#include "test_hw.h"\n'
                                      f'#include "{source.as_posix()}"\n' + suffix, encoding="utf-8")
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
