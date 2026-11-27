"""验证注释底盘力矩补偿后的纠偏，并以临时副本复现历史缺陷；不代表实车动力学。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

from test_vt13_firmware import HARDWARE, ROOT


CASES = r'''
/* 固定视觉世界角，人工设置反馈和底盘速度；不运行电机或机器人。 */
static float run_case(float wz, float theta, unsigned count, unsigned tick_ms) {
    test_chassis_wz=wz;
    gimbal_direct.feedback_pos.yaw=theta;
    gimbal_direct.feedback_vel.yaw=0;
    for(unsigned i=0;i<count;++i) {
        now_ms+=tick_ms;
        GimbalReference();
        assert(fabsf(loop_fp32_constrain(gimbal_direct.reference.yaw,-M_PI,M_PI))<0.00001f);
        GimbalAutoYawConsole();
        assert(yaw_auto_feedback_valid);
        assert(fabsf(gimbal_direct.yaw.set.tor)<=GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM);
    }
    return gimbal_direct.yaw.direction*gimbal_direct.yaw.set.tor;
}

/* 每组独立复位，排除上一组 ESO、斜率和模式历史的影响。 */
static void reset_case(void) {
    GimbalInit();
    gimbal_direct.mode=gimbal_direct.last_mode=GIMBAL_AUTO_AIM;
    gimbal_vision.valid=true;
    gimbal_vision.yaw=gimbal_vision.yaw_vel=gimbal_vision.yaw_acc=0;
    gimbal_direct.yaw.offline=gimbal_direct.pitch.offline=false;
    gimbal_direct.pitch.fdb.pos=GIMBAL_DIRECT_PITCH_MID;
    gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;
}

int main(void) {
    printf("Ktheta=%.9f Komega=%.9f ESO=%u configured_chassis_ff=%.3f torque_limit=%.1f\n",
           GIMBAL_AUTO_YAW_K_THETA,GIMBAL_AUTO_YAW_K_OMEGA,
           GIMBAL_AUTO_YAW_ESO_COMP_ENABLE,GIMBAL_AUTO_YAW_CHASSIS_WZ_TORQUE_FF,
           GIMBAL_AUTO_YAW_TORQUE_LIMIT_NM);
    /* 静止底盘对照：目标固定为零，正负角偏差应得到反向纠偏力矩。 */
    for(int sign=-1;sign<=1;sign+=2) {
        reset_case();
        float tau=run_case(0,sign*0.2f,30,1);
        assert(tau*sign<0);
        printf("stationary theta=%+.3f tau=%+.6f correcting=1\n",sign*0.2f,tau);
    }
    /* 取此前用户小陀螺截图附近的速度，仅复用量级，不冒充本轮测量。 */
    for(int sign=-1;sign<=1;sign+=2) {
        reset_case();
        float aligned=run_case(sign*6.15f,0,30,1);
        float opposed=run_case(sign*6.15f,sign*2.0f,100,1);
        float recovered=run_case(0,sign*2.0f,100,1);
        printf("spin wz=%+.2f aligned_tau=%+.6f theta=%+.3f opposed_tau=%+.6f stop_tau=%+.6f\n",
               sign*6.15f,aligned,sign*2.0f,opposed,recovered);
        /* 当前生产版本已注释前馈，必须恢复纠偏；历史副本保留饱和复现。 */
#ifdef TEST_NO_FF
        assert(fabsf(aligned)<0.00001f && opposed*sign<0);
#else
        assert(aligned*sign>6.99f && opposed*sign>6.99f);
#endif
        assert(recovered*sign<0);
    }
    /* 前馈小于总上限仍可能被内外两次斜率限制锁住；覆盖 1/2 ms。 */
    for(unsigned tick=1;tick<=2;++tick) for(int sign=-1;sign<=1;sign+=2) {
        reset_case();
        float aligned=run_case(sign*1.0f,0,30,tick);
        float opposed=run_case(sign*1.0f,sign*2.0f,100,tick);
        printf("slew tick_ms=%u wz=%+.2f aligned_tau=%+.6f opposed_tau=%+.6f\n",
               tick,sign*1.0f,aligned,opposed);
#if defined(TEST_NO_FF) || defined(TEST_NO_INNER_SLEW)
        assert(opposed*sign<0);
#else
        assert(aligned*sign>6.99f && opposed*sign>6.99f);
#endif
    }
    puts("actual C reproduction passed; prescribed feedback only, no plant or hardware validation");
    return 0;
}
'''


def main():
    compiler = shutil.which("gcc")
    if not compiler:
        raise RuntimeError("需要 GCC 执行实际控制源码")
    hardware = HARDWARE.replace(
        "float ChassisGetSpeedWz(void) { return 0; }",
        "static float test_chassis_wz;\nfloat ChassisGetSpeedWz(void) { return test_chassis_wz; }")
    with tempfile.TemporaryDirectory(prefix="auto-yaw-spin-") as directory:
        work = Path(directory)
        (work / "test_hw.h").write_text(hardware, encoding="utf-8")
        stubs = ("pid.h motor.h remote_control.h IMU.h detect_task.h user_lib.h CAN_cmd_dji.h "
                 "CAN_cmd_damiao.h cmsis_os.h CAN_receive.h usb_debug.h main.h SupCap.h "
                 "IMU_task.h CAN_cmd_SupCap.h referee.h arm_math.h CAN_communication.h")
        for name in stubs.split():
            (work / name).write_text('#include "test_hw.h"\n', encoding="utf-8")
        source = ROOT / "application/gimbal/gimbal_yaw_pitch_direct.c"
        command = [compiler, "-std=c99", "-fgnu89-inline", "-Werror=implicit-function-declaration"]
        includes = [work] + [ROOT / path for path in (
            "application", "application/typedef", "application/remote_control/vt13",
            "application/gimbal", "application/chassis", "application/communication",
            "application/communication/vofa/observation")]
        for include in includes:
            command += ["-I", str(include)]
        # 当前代码直接验证；恢复补偿的历史对照只修改系统临时源码副本。
        original = source.read_text(encoding="utf-8")
        disabled = "tau_world = controller_was_ready ? yaw_auto_output.tau_cmd_nm : 0.0f;"
        enabled = ("tau_world = controller_was_ready ? yaw_auto_output.tau_cmd_nm + "
                   "GIMBAL_AUTO_YAW_CHASSIS_WZ_TORQUE_FF * chassis_wz : 0.0f;")
        if original.count(disabled) != 1:
            raise RuntimeError("无法唯一定位已关闭底盘补偿的实际输出式")
        for variant in ("current", "historical-chassis-ff", "historical-no-inner-slew"):
            selected_source = source
            override = "#define TEST_NO_FF\n"
            if variant != "current":
                historical = original.replace(disabled, enabled)
                override = ""
            if variant == "historical-no-inner-slew":
                marker = ".torque_slew_enable = 1U,"
                if historical.count(marker) != 1:
                    raise RuntimeError("无法唯一定位内部斜率开关")
                historical = historical.replace(marker, ".torque_slew_enable = 0U,")
                override = "#define TEST_NO_INNER_SLEW\n"
            if variant != "current":
                selected_source = work / f"{variant}_gimbal.c"
                selected_source.write_text(historical, encoding="utf-8")
            bridge = work / f"{variant}.c"
            bridge.write_text('#define TEST_GIMBAL\n#include "test_hw.h"\n' + override
                              + f'#include "{selected_source.as_posix()}"\n' + CASES,
                              encoding="utf-8")
            executable = work / f"{variant}.exe"
            invocation = command + [str(bridge),
                str(ROOT / "application/remote_control/vt13/vt13_control.c"),
                str(ROOT / "application/gimbal/yaw_auto_lqr_eso_controller.c"),
                "-lm", "-o", str(executable)]
            result = subprocess.run(invocation, capture_output=True, text=True, errors="replace")
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True,
                                    errors="replace", timeout=10)
            print(f"variant={variant}", flush=True)
            print(result.stdout, end="")
            if result.returncode:
                raise RuntimeError(result.stderr or f"诊断退出码 {result.returncode}")


if __name__ == "__main__":
    main()
