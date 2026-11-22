"""用实际控制源码验证退出小陀螺持续跟随当前云台；反馈为人工输入，不模拟实车动力学。"""
from pathlib import Path
import shutil
import subprocess
import tempfile

from test_vt13_firmware import HARDWARE, ROOT


CHASSIS = r'''
/* 发布一致的世界角、关节角及静止轮速，保持底盘朝向等于指定值。 */
static void feedback(float world, float heading) {
    fake_status.yaw_world=world;
    fake_status.yaw_joint=Vt13WrapAngle(world-heading);
    fake_status.update_time_ms=now_ms;
    chassis.P_wz=0;
}
/* 对称构造松手偏角，检查云台恢复后不会停在旧方向或因模式切换再次跳目标。 */
static void check_offset(float offset, int mode) {
    ChassisInit();enable_remote();raw.mode_switch=mode;
    float initial_heading=-copysignf(1.0f,offset);
    raw.trigger=true;feedback(offset, initial_heading);
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_SPIN);
    raw.trigger=false;now_ms+=2;feedback(offset,initial_heading);
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_SPIN_RETURN);
    /* 假设云台已经恢复世界零角，底盘已到达松手时保存的偏角。 */
    now_ms+=2;feedback(0,offset);
    ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN_RETURN && chassis.reference.wz*offset>0);
    assert(!chassis_spin_control.settling);
    assert(fabsf(chassis.reference.wz)<=CHASSIS_SPIN_RETURN_MAX_WZ);
    float correction=chassis.reference.wz;
    /* 即使旧方向停够 100 ms，只要与当前云台未对齐，也不能结束回正。 */
    now_ms+=VT13_RETURN_SETTLE_MS;feedback(0,offset);
    ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN_RETURN && chassis.reference.wz*offset>0);
    /* 云台世界角连续跨越折返点，底盘与当前云台保持小角差。 */
    now_ms+=2;feedback(-M_PI+0.005f,M_PI-0.005f);
    ChassisSetMode();ChassisReference();assert(chassis.reference.wz==0);
    uint32_t start=now_ms;
    assert(chassis_pid.follow.out==0);
    now_ms=start+VT13_RETURN_SETTLE_MS-1;feedback(0,0);
    ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN_RETURN && chassis.reference.wz==0);
    now_ms=start+VT13_RETURN_SETTLE_MS;feedback(0,0);
    ChassisSetMode();ChassisReference();assert(chassis.mode==CHASSIS_FOLLOW);
    float first=chassis.reference.wz;
    assert(first==0);
    now_ms+=2;feedback(0,0);ChassisSetMode();ChassisReference();
    assert(chassis.reference.wz==0);
    /* 完成后正常跟随恢复原上限，不永久保留回正限速。 */
    now_ms+=2;feedback(0,offset>0?2.0f:-2.0f);ChassisSetMode();ChassisReference();
    assert(fabsf(chassis.reference.wz)==MAX_OUT_CHASSIS_FOLLOW_GIMBAL);
    printf("release_offset_deg=%.1f moving_target_correction=%.6f settle_ms=%u follow_first_wz=%.1f normal_max_wz=%.1f\n",
           offset*180/M_PI,correction,VT13_RETURN_SETTLE_MS,first,fabsf(chassis.reference.wz));
}
/* S 档有目标仍按旧保存方向回正，完成后只进入 BODY。 */
static void check_target_return(void) {
    ChassisInit();enable_remote();raw.mode_switch=2;fake_status.auto_active=true;
    raw.trigger=true;feedback(M_PI/12,-1);ChassisSetMode();ChassisReference();
    raw.trigger=false;now_ms+=2;feedback(M_PI/12,-1);ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN_RETURN);
    float saved=chassis_spin_control.return_heading;
    now_ms+=2;feedback(0,saved);ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN_RETURN && chassis.reference.wz==0);
    assert(chassis_spin_control.settling);
    assert(fabsf(chassis.yaw_delta)>0.2f);
    now_ms+=VT13_RETURN_SETTLE_MS;feedback(0,saved);ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_BODY && chassis.reference.wz==0);
    assert(chassis_spin_control.return_heading==saved);
    puts("S with target: fixed release heading -> BODY, no FOLLOW: passed");
}
/* 人为输入过中前后的角差/余速，检查刹车方向及模式，不模拟实际惯性或制动能力。 */
static void check_return_braking(int mode, float direction) {
    ChassisInit();enable_remote();raw.mode_switch=mode;
    raw.trigger=true;feedback(0,-0.5f*direction);
    ChassisSetMode();ChassisReference();
    raw.trigger=false;now_ms+=2;feedback(0,-0.02f*direction);chassis.P_wz=-direction;
    ChassisSetMode();ChassisReference();ChassisConsole();
    assert(chassis.mode==CHASSIS_SPIN_RETURN && !chassis_spin_control.settling);
    assert(chassis.reference.wz*chassis.P_wz<0 && chassis.torque_wz*chassis.P_wz<0);
    float before=chassis.reference.wz;
    /* 到中心但仍有转速时只能继续制动，不能开始到位计时或退出回正。 */
    now_ms+=2;feedback(0,0);chassis.P_wz=-0.5f*direction;
    ChassisSetMode();ChassisReference();ChassisConsole();
    assert(chassis.mode==CHASSIS_SPIN_RETURN && !chassis_spin_control.settling);
    assert(chassis.reference.wz*chassis.P_wz<0 && chassis.torque_wz*chassis.P_wz<0);
    float at_center=chassis.reference.wz;
    /* 若真实底盘依然越过，则角差变号后产生返向修正；这不是新模式或旧目标。 */
    now_ms+=2;feedback(0,0.03f*direction);
    ChassisSetMode();ChassisReference();
    assert(chassis.mode==CHASSIS_SPIN_RETURN && !chassis_spin_control.settling);
    assert(chassis.reference.wz*direction>0);
    printf("prescribed_return mode=%d sign=%.0f brake_before=%.6f brake_at_zero=%.6f after_cross=%.6f still_RETURN=1\n",
           mode,direction,before,at_center,chassis.reference.wz);
}
/* 按明确的角度/阻尼表验证近中点制动，并检查正常跟随与 S 有目标不受影响。 */
static void check_near_return_damping(void) {
    const float angles[]={0.0f,0.10f*CHASSIS_SPIN_RETURN_BRAKE_ANGLE,0.50f*CHASSIS_SPIN_RETURN_BRAKE_ANGLE,CHASSIS_SPIN_RETURN_BRAKE_ANGLE,1.50f*CHASSIS_SPIN_RETURN_BRAKE_ANGLE};
    const float blend_table[]={1.0f,0.9f,0.5f,0.0f,0.0f};
    unsigned checked=0;
    for(int mode=1;mode<=2;++mode) for(int returning=0;returning<=1;++returning)
    for(int fixed=0;fixed<=1;++fixed) {
        if(fixed && (mode!=2 || !returning)) continue;
        for(int a=0;a<5;++a) for(int sign=-1;sign<=1;sign+=2) for(int spin=-1;spin<=1;spin+=2) {
            ChassisInit();enable_remote();raw.mode_switch=mode;fake_status.auto_active=fixed;
            fake_status.yaw_world=0;fake_status.yaw_joint=sign*angles[a];
            chassis.P_wz=spin*0.5f;chassis_spin_control.returning=returning;
            ChassisSetMode();ChassisReference();
            assert(chassis.mode==(returning?CHASSIS_SPIN_RETURN:CHASSIS_FOLLOW));
            float damping=CHASSIS_FOLLOW_WZ_DAMPING+(returning&&!fixed?blend_table[a]*(CHASSIS_SPIN_RETURN_NEAR_WZ_DAMPING-CHASSIS_FOLLOW_WZ_DAMPING):0.0f);
            float limit=returning?CHASSIS_SPIN_RETURN_MAX_WZ:MAX_OUT_CHASSIS_FOLLOW_GIMBAL;
            float expected=fp32_constrain(chassis_pid.follow.out-damping*chassis.P_wz,-limit,limit);
            assert(fabsf(chassis.reference.wz-expected)<0.00002f);
            /* 大余速下叠加增强阻尼后，最终参考仍须服从原来的回正限幅。 */
            chassis.P_wz=spin*80.0f;ChassisSetMode();ChassisReference();
            assert(fabsf(chassis.reference.wz)<=limit);++checked;
        }
    }
    printf("near-return damping scope, both signs, boundary and final limits: %u cases passed\n",checked);
}
int main(void) {
    check_near_return_damping();
    for(int mode=1;mode<=2;++mode) {
        check_offset(M_PI/12,mode);check_offset(-M_PI/12,mode);
        check_return_braking(mode,1);check_return_braking(mode,-1);
    }
    check_target_return();
    puts("chassis continuous target return: passed (prescribed feedback, no plant simulation)");return 0;
}
'''

GIMBAL = r'''
/* 用人为指定的速度比例构造平衡点，只验证实际控制公式，不把假设当成实车标定。 */
static void check_spin_equilibrium(float speed_ratio, float wz) {
    assert(KI_GIMBAL_YAW_ANGLE==0 && KI_GIMBAL_YAW_VELOCITY==0);
    float theta=(GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD-speed_ratio)*wz/KP_GIMBAL_YAW_ANGLE;
    gimbal_direct.mode=gimbal_direct.last_mode=GIMBAL_IMU;
    gimbal_direct.reference.yaw=0;
    gimbal_direct.feedback_pos.yaw=theta;
    gimbal_direct.feedback_vel.yaw=0;
    gimbal_direct.yaw.fdb.vel=speed_ratio*wz;
    test_chassis_wz=wz;
    /* 按住旋转不会改变手动世界角目标，偏差来自控制平衡而非目标被加了偏移。 */
    gimbal_input.spin_down=true;
    GimbalReference();assert(gimbal_direct.reference.yaw==0);
    PID_clear(&gimbal_direct_pid.yaw_angle);PID_clear(&gimbal_direct_pid.yaw_velocity);
    GimbalConsole();assert(fabsf(gimbal_direct.yaw.set.tor)<0.00002f);
    printf("hypothetical_ratio=%.1f wz=%.6f equilibrium_yaw_deg=%.6f torque=%.6f\n",
           speed_ratio,wz,theta*180/M_PI,gimbal_direct.yaw.set.tor);
}
/* 用户 Watch 单帧：补全阻尼所需 IMU 值是反算量，不是该截图的实测量。 */
static void check_watch_sample(void) {
    const float wz=6.18286324f, motor=8.69597244f;
    const float reference=0.323672593f, theta=0.661943674f, observed_torque=0.0202047825f;
    gimbal_direct.reference.yaw=reference;
    gimbal_direct.feedback_pos.yaw=theta;
    gimbal_direct.feedback_vel.yaw=0;
    gimbal_direct.yaw.fdb.vel=motor;test_chassis_wz=wz;
    PID_clear(&gimbal_direct_pid.yaw_angle);PID_clear(&gimbal_direct_pid.yaw_velocity);
    GimbalConsole();
    float zero_imu_torque=gimbal_direct.yaw.set.tor;
    assert(fabsf(zero_imu_torque-0.35880404f)<0.00002f);
    float inferred_imu=(zero_imu_torque-observed_torque)/
                       (KP_GIMBAL_YAW_VELOCITY*GIMBAL_DIRECT_YAW_GYRO_DAMPING);
    gimbal_direct.feedback_vel.yaw=inferred_imu;
    PID_clear(&gimbal_direct_pid.yaw_angle);PID_clear(&gimbal_direct_pid.yaw_velocity);
    GimbalConsole();assert(fabsf(gimbal_direct.yaw.set.tor-observed_torque)<0.00002f);
    printf("watch_sample bias_deg=%.6f tau_if_imu_zero=%.6f inferred_not_measured_imu=%.6f tau_after_closure=%.6f\n",
           (theta-reference)*180/M_PI,zero_imu_torque,inferred_imu,gimbal_direct.yaw.set.tor);
}
/* 用户第二张图给出了实际 IMU 速度；验证本机预测值并保留 Watch 非同拍限制。 */
static void check_watch_with_imu(void) {
    const float wz=6.14818573f, motor=8.63736243f;
    const float reference=0.801405847f, theta=1.14804602f, imu=-0.0330231972f;
    gimbal_direct.reference.yaw=reference;gimbal_direct.feedback_pos.yaw=theta;
    gimbal_direct.feedback_vel.yaw=imu;gimbal_direct.yaw.fdb.vel=motor;test_chassis_wz=wz;
    PID_clear(&gimbal_direct_pid.yaw_angle);PID_clear(&gimbal_direct_pid.yaw_velocity);
    GimbalConsole();assert(fabsf(gimbal_direct.yaw.set.tor-0.25107887f)<0.00003f);
    float angle_equivalent_gain=GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD+
                                KP_GIMBAL_YAW_ANGLE*(reference-theta)/wz;
    assert(angle_equivalent_gain>1.43f && angle_equivalent_gain<1.44f);
    printf("watch_with_imu bias_deg=%.6f imu=%.6f predicted_tau=%.6f angle_equivalent_gain=%.6f\n",
           (theta-reference)*180/M_PI,imu,gimbal_direct.yaw.set.tor,angle_equivalent_gain);
}
/* 用户确认 1.45 的对照图；只在对应配置下核对，不把旧图套到新参数。 */
static void check_watch_tuned(void) {
    const float wz=6.15404081f, motor=8.41758347f;
    const float reference=0.00313734333f, theta=0.00809197966f, imu=0.0319579318f;
    gimbal_direct.reference.yaw=reference;gimbal_direct.feedback_pos.yaw=theta;
    gimbal_direct.feedback_vel.yaw=imu;gimbal_direct.yaw.fdb.vel=motor;test_chassis_wz=wz;
    PID_clear(&gimbal_direct_pid.yaw_angle);PID_clear(&gimbal_direct_pid.yaw_velocity);
    GimbalConsole();assert(fabsf(gimbal_direct.yaw.set.tor-0.56029982f)<0.00003f);
    assert(fabsf(theta-reference)<0.005f);
    printf("watch_tuned_ff145 bias_deg=%.6f predicted_tau=%.6f observed_tau=0.525876284\n",
           (theta-reference)*180/M_PI,gimbal_direct.yaw.set.tor);
}
int main(void) {
    printf("compiled_manual_yaw_ff=%.3f\n",GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD);
    enable_remote();GimbalInit();gimbal_direct.mode=GIMBAL_IMU;
    gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw=0;
    gimbal_direct.feedback_vel.yaw=0;gimbal_direct.yaw.fdb.vel=0;
    gimbal_direct.yaw.fdb.pos=GIMBAL_DIRECT_YAW_MID;
    gimbal_direct.pitch.fdb.pos=GIMBAL_DIRECT_PITCH_MID;
    for(int i=-1;i<=1;++i) {
        test_chassis_wz=2.0f*i;PID_clear(&gimbal_direct_pid.yaw_angle);PID_clear(&gimbal_direct_pid.yaw_velocity);
        GimbalConsole();
        assert(fabsf(gimbal_direct.yaw.set.vel-GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD*test_chassis_wz)<0.0001f);
        printf("manual chassis_wz=%.1f yaw_error=0 yaw_vel_cmd=%.1f torque_cmd=%.1f\n",
               test_chassis_wz,gimbal_direct.yaw.set.vel,gimbal_direct.yaw.set.tor);
    }
    /* 选定 15 度等效量仅为可复核构造，不是测得的底盘转速。 */
    float wz=KP_GIMBAL_YAW_ANGLE*M_PI/12;
    check_spin_equilibrium(1.0f,wz);
    check_spin_equilibrium(1.0f,-wz);
    check_spin_equilibrium(1.5f,wz);
    check_spin_equilibrium(2.0f,wz);
    check_spin_equilibrium(-1.0f,wz);
    /* 历史 2.0 图和用户 1.45 图分别按采样时配置复核。 */
    if (GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD==2.0f) {
        check_watch_sample();check_watch_with_imu();
    } else if (GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD==1.45f) check_watch_tuned();
    puts("manual yaw chassis feedforward and hypothetical equilibria: passed (no plant identification)");return 0;
}
'''


def main():
    compiler = shutil.which("gcc")
    if not compiler:
        raise RuntimeError("需要 GCC 执行实际控制源码")
    # 复用硬件替身，改为实际 PID，避免原门控测试的纯 P 替身掩盖切换微分项。
    start = HARDWARE.index("#define PID_H")
    end = HARDWARE.index("typedef struct { float pos,vel,tor,curr;")
    hardware = HARDWARE[:start] + '#include "pid.h"\n' + HARDWARE[end:]
    hardware = hardware.replace("float ChassisGetSpeedWz(void) { return 0; }",
                                "static float test_chassis_wz;\nfloat ChassisGetSpeedWz(void) { return test_chassis_wz; }")
    with tempfile.TemporaryDirectory(prefix="yaw-chassis-diagnosis-") as directory:
        work = Path(directory)
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
        # 当前参数与历史 2.0 分开编译；覆盖宏只作用于临时桥接文件，不修改生产头文件。
        for component, source, suffix, historical in (
            ("chassis", "application/chassis/chassis_omni.c", CHASSIS, False),
            ("gimbal", "application/gimbal/gimbal_yaw_pitch_direct.c", GIMBAL, False),
            ("gimbal", "application/gimbal/gimbal_yaw_pitch_direct.c", GIMBAL, True),
        ):
            variant = component + ("-historical-ff2" if historical else "-current")
            bridge = work / f"{variant}.c"
            override = ('#include "robot_param.h"\n#undef GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD\n'
                        '#define GIMBAL_DIRECT_YAW_WZ_FEEDFORWARD 2.0f\n') if historical else ""
            bridge.write_text(f'#define TEST_{component.upper()}\n#include "test_hw.h"\n' + override
                              + f'#include "{(ROOT / "components/controller/pid.c").as_posix()}"\n'
                              f'#include "{(ROOT / source).as_posix()}"\n' + suffix, encoding="utf-8")
            executable = work / f"{variant}.exe"
            command = [compiler, "-std=c99", "-fgnu89-inline", "-Werror=implicit-function-declaration"]
            for include in includes:
                command += ["-I", str(include)]
            command += [str(bridge), str(ROOT / "application/remote_control/vt13/vt13_control.c")]
            if component == "gimbal":
                command += [str(ROOT / "application/gimbal/yaw_auto_lqr_eso_controller.c")]
            command += ["-lm", "-o", str(executable)]
            subprocess.run(command, check=True, capture_output=True, text=True)
            result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
            print(result.stdout, end="")


if __name__ == "__main__":
    main()
