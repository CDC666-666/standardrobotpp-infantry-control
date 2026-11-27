"""用实际底盘/PID和发送函数复核链路；人工反馈及抢占注入不等于实车根因。"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

from test_vt13_firmware import HARDWARE, ROOT


CASES = r'''
/* 每组都建立新输入与就绪快照，清除上组的失能和键盘历史。 */
static void reset_case(void) {
    ChassisInit();ChassisForceDisable();enable_remote();
    chassis.P_vx=chassis.P_vy=chassis.P_wz=0;
}
/* 执行实际参考、控制和发送函数，硬件反馈由用例指定。 */
static void output_step(void) {
    ChassisSetMode();ChassisReference();ChassisConsole();ChassisSendCmd();
}
/* 最终四槽电流必须全部清零，避免只看参考而遗漏发送前联锁。 */
static void assert_disabled(void) {
    assert(chassis.mode==CHASSIS_LOCK);
    for(int i=0;i<4;++i) assert(sent[CHASSIS_CAN][0][i]==0);
}
int main(void) {
    for(int mode=1;mode<=2;++mode) {
        reset_case();raw.mode_switch=mode;raw.normalized[3]=0.5f;
        output_step();assert(chassis.mode==CHASSIS_FOLLOW);
        assert(chassis.reference.vx==2.0f);
        assert(sent[1][0][0]<0&&sent[1][0][1]<0&&sent[1][0][2]>0&&sent[1][0][3]>0);
        printf("mode=%d stick vx=%.1f raw=[%.3f,%.3f,%.3f,%.3f]\n",mode,
               chassis.reference.vx,sent[1][0][0],sent[1][0][1],sent[1][0][2],sent[1][0][3]);
        reset_case();raw.mode_switch=mode;raw.trigger=true;output_step();
        assert(chassis.mode==CHASSIS_SPIN);
        for(int i=0;i<4;++i) assert(sent[1][0][i]<0);
        printf("mode=%d trigger wz=%.1f all four current slots nonzero\n",mode,chassis.reference.wz);
        for(int wheel=0;wheel<4;++wheel) {
            reset_case();raw.mode_switch=mode;raw.trigger=true;
            chassis.wheel[wheel].offline=true;output_step();assert_disabled();
            reset_case();raw.mode_switch=mode;raw.normalized[3]=1;
            ChassisSetMode();ChassisReference();ChassisConsole();
            chassis.wheel[wheel].offline=true;ChassisSendCmd();assert_disabled();
        }
    }
    reset_case();raw.keyboard=1;output_step();now_ms+=20;fake_status.update_time_ms=now_ms;
    output_step();assert(chassis.reference.vx>0&&sent[1][0][2]>0);
    reset_case();raw.trigger=true;fake_status.ready=false;output_step();assert_disabled();
    reset_case();raw.trigger=true;fake_status.feedback_valid=false;output_step();assert_disabled();
    reset_case();raw.trigger=true;fake_status.init_state=VT13_INIT_RUNNING;output_step();assert_disabled();
    reset_case();raw.trigger=true;now_ms+=VT13_CONTROL_STATUS_TIMEOUT_MS-1;
    output_step();assert(chassis.mode==CHASSIS_SPIN);
    ++now_ms;output_step();assert_disabled();
    reset_case();raw.trigger=true;ChassisSetMode();ChassisReference();ChassisConsole();
    raw.mode_switch=VT13_MODE_S;ChassisSendCmd();assert_disabled();
    reset_case();raw.trigger=true;raw.valid=false;output_step();assert_disabled();
    reset_case();raw.trigger=true;fake_status.yaw_joint=NAN;output_step();assert_disabled();
    puts("actual chassis/PID: stick, WASD, trigger, each wheel offline and final gates passed");
    return 0;
}
'''

CAN_HARDWARE = r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
typedef struct { unsigned bus; } hcan_t;
typedef struct { uint32_t StdId,IDE,RTR,DLC; } CAN_TxHeaderTypeDef;
typedef struct { hcan_t *hcan; CAN_TxHeaderTypeDef tx_header; uint8_t tx_data[8]; } CanCtrlData_s;
static hcan_t hcan1={1},hcan2={2};
enum { CAN_ID_STD=0, CAN_RTR_DATA=0 };
static unsigned inject, count, recorded_bus[2], recorded_id[2];
static uint8_t recorded_data[2][8];
void CanCmdDjiMotor(uint8_t,uint16_t,int16_t,int16_t,int16_t,int16_t);
/* 在底盘调用发送层后模拟高优先级云台抢占，不模拟抢占发生的实车频率。 */
static uint32_t HAL_CAN_GetTxMailboxesFreeLevel(hcan_t *can) {
    (void)can;
    if(inject) { inject=0;CanCmdDjiMotor(2,0x1ff,0,0,321,0); }
    return 3;
}
/* 保存实际进入 HAL 的总线、标识符与字节；这里只证明提交请求，不能证明线端发送成功。 */
static int HAL_CAN_AddTxMessage(hcan_t *can,CAN_TxHeaderTypeDef *header,uint8_t *data,uint32_t *mailbox) {
    assert(count<2);recorded_bus[count]=can->bus;recorded_id[count]=header->StdId;
    memcpy(recorded_data[count],data,8);++count;*mailbox=0;return 0;
}
'''

CAN_CASES = r'''
int main(void) {
    CanCmdDjiMotor(1,0x200,101,-202,303,-404);
    assert(count==1&&recorded_bus[0]==1&&recorded_id[0]==0x200);
    assert(recorded_data[0][0]==0&&recorded_data[0][1]==101);
    assert(recorded_data[0][2]==0xff&&recorded_data[0][3]==0x36);
    count=0;inject=1;CanCmdDjiMotor(1,0x200,101,-202,303,-404);
    assert(count==2&&recorded_bus[0]==2&&recorded_id[0]==0x1ff);
    assert(recorded_bus[1]==1&&recorded_id[1]==0x200);
    assert(recorded_data[1][0]==0&&recorded_data[1][1]==101);
    assert(recorded_data[1][2]==0xff&&recorded_data[1][3]==0x36);
    puts("actual DJI/BSP functions: normal CAN1 0x200 packing passed");
    puts("injected preemption: chassis CAN1 0x200 and gimbal CAN2 0x1ff retain independent frames");
    return 0;
}
'''


def function_text(path, signature):
    """按完整函数体抽取磁盘实现；不用手写同义逻辑替代生产函数。"""
    source = path.read_text(encoding="utf-8")
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    for end in range(opening + 1, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[start:end + 1]
    raise RuntimeError(f"函数体未闭合：{signature}")


def run(command):
    result = subprocess.run(command, capture_output=True, text=True, errors="replace", timeout=30)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    if result.stdout:
        print(result.stdout, end="")


def main():
    compiler = shutil.which("gcc")
    if not compiler:
        raise RuntimeError("需要 GCC 执行实际 C 源码")
    with tempfile.TemporaryDirectory(prefix="chassis-link-") as directory:
        work = Path(directory)
        # 沿用已有反馈替身，控制器换成实际离散 PID，保留生产增益及电流换算。
        start = HARDWARE.index("#define PID_H")
        end = HARDWARE.index("typedef struct { float pos,vel,tor,curr;")
        hardware = HARDWARE[:start] + '#include "pid.h"\n' + HARDWARE[end:]
        (work / "test_hw.h").write_text(hardware, encoding="utf-8")
        stubs = ("motor.h remote_control.h IMU.h detect_task.h user_lib.h CAN_cmd_dji.h "
                 "CAN_cmd_damiao.h cmsis_os.h CAN_receive.h usb_debug.h main.h SupCap.h "
                 "IMU_task.h CAN_cmd_SupCap.h referee.h arm_math.h CAN_communication.h struct_typedef.h")
        for name in stubs.split():
            (work / name).write_text('#include "test_hw.h"\n', encoding="utf-8")
        includes = [work, ROOT / "components/controller"] + [ROOT / path for path in (
            "application", "application/typedef", "application/remote_control/vt13",
            "application/gimbal", "application/chassis", "application/communication",
            "application/communication/vofa/observation")]
        bridge = work / "chassis.c"
        bridge.write_text('#define TEST_CHASSIS\n#include "test_hw.h"\n'
                          + f'#include "{(ROOT / "components/controller/pid.c").as_posix()}"\n'
                          + f'#include "{(ROOT / "application/chassis/chassis_omni.c").as_posix()}"\n'
                          + CASES, encoding="utf-8")
        executable = work / ("chassis.exe" if os.name == "nt" else "chassis")
        command = [compiler, "-std=c99", "-fgnu89-inline", "-Werror=implicit-function-declaration"]
        for include in includes:
            command += ["-I", str(include)]
        run(command + [str(bridge), str(ROOT / "application/remote_control/vt13/vt13_control.c"),
                       "-lm", "-o", str(executable)])
        run([str(executable)])
        # 抽取当前两层发送实现，只替换HAL；确认抢占后调用独占报文保持完整。
        bridge = work / "can_send.c"
        bridge.write_text(CAN_HARDWARE + "\n"
                          + function_text(ROOT / "bsp/boards/bsp_can.c", "void CAN_SendTxMessage(")
                          + "\n" + function_text(ROOT / "application/robot_cmd/CAN_cmd_dji.c", "static void MultipleMotorControl(")
                          + "\n" + function_text(ROOT / "application/robot_cmd/CAN_cmd_dji.c", "void CanCmdDjiMotor(")
                          + CAN_CASES, encoding="utf-8")
        executable = work / ("can_send.exe" if os.name == "nt" else "can_send")
        run([compiler, "-std=c99", str(bridge), "-o", str(executable)])
        run([str(executable)])


if __name__ == "__main__":
    main()
