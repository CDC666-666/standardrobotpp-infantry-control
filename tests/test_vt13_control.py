"""编译实际 VT13 控制层，验证输入映射及初始化、回正、开轮和连发状态机。"""
import ctypes as c
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_vt13_remote import Input, frame

ROOT = Path(__file__).resolve().parents[1]


class Control(c.Structure):
    _fields_ = [("valid", c.c_bool), ("mode", c.c_int)] + [
        (name, c.c_float) for name in ("move_x", "move_y", "yaw_input", "pitch_input", "wheel")
    ] + [("fn_down", c.c_bool), ("spin_down", c.c_bool),
         ("sequence", c.c_uint32), ("update_time_ms", c.c_uint32),
         ("mouse_x", c.c_int16), ("mouse_y", c.c_int16), ("keyboard_move", c.c_bool),
         ("mouse_left", c.c_bool), ("mouse_right", c.c_bool)]


class KeyboardMotion(c.Structure):
    _fields_ = [("vx", c.c_float), ("vy", c.c_float),
                ("update_time_ms", c.c_uint32), ("active", c.c_bool)]


class MouseControl(c.Structure):
    _fields_ = [("sequence", c.c_uint32), ("seen", c.c_bool)]


class Init(c.Structure):
    _fields_ = [("state", c.c_int), ("start_ms", c.c_uint32)]


class Status(c.Structure):
    _fields_ = [("init_state", c.c_int)] + [
        (name, c.c_bool) for name in ("ready", "auto_active", "vision_fire", "feedback_valid")
    ] + [("yaw_world", c.c_float), ("yaw_joint", c.c_float), ("update_time_ms", c.c_uint32)]


class Spin(c.Structure):
    _fields_ = [("spin_last", c.c_bool), ("returning", c.c_bool), ("settling", c.c_bool),
                ("return_heading", c.c_float), ("settle_start_ms", c.c_uint32)]


class Shoot(c.Structure):
    _fields_ = [(name, c.c_bool) for name in
               ("friction_on", "fn_last", "fn_released", "manual_burst")]


class Vt13ControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        # 缺少 GCC 时明确失败，不能把未执行的控制回归当成通过。
        if not compiler:
            raise RuntimeError("需要 GCC 执行实际控制层回归，不能跳过验证")
        cls.directory = tempfile.TemporaryDirectory(prefix="vt13-control-")
        work = Path(cls.directory.name)
        bridge = work / "bridge.c"
        bridge.write_text('''
#include "vt13_control.h"
static Vt13Parser parser;
static uint32_t now_ms;
/* 重置实际协议解析器，避免用例之间共享接收状态。 */
void test_init(void) { Vt13ParserInit(&parser); }
/* 把测试字节流和接收时间送入实际协议解析器。 */
void test_feed(const uint8_t *data, uint16_t length, uint32_t now) {
    now_ms = now;
    Vt13ParserFeed(&parser, data, length, now);
}
/* 推进替身时钟，验证超时和计数回卷。 */
void test_time(uint32_t now) { now_ms = now; }
/* 以实际解析器读取代替 UART 入口，保留协议层超时行为。 */
void Vt13ReadInput(Vt13Input *out) { Vt13ParserRead(&parser, out, now_ms); }
unsigned test_size(unsigned type) {
    /* 按类型返回实际 C 结构大小，检查 ctypes 布局是否匹配。 */
    switch (type) {
        case 0: return sizeof(Vt13ControlInput);
        case 1: return sizeof(Vt13InitControl);
        case 2: return sizeof(GimbalControlStatus);
        case 3: return sizeof(Vt13SpinControl);
        case 4: return sizeof(Vt13ShootControl);
        case 5: return sizeof(Vt13KeyboardMotion);
        default: return sizeof(Vt13MouseControl);
    }
}
''', encoding="utf-8")
        # 按主机系统选择可由 ctypes 加载的动态库后缀。
        library = work / ("control.dll" if os.name == "nt" else "control.so")
        source = ROOT / "application/remote_control/vt13"
        subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared",
                        "-fPIC", "-I", str(source), str(source / "vt13_remote.c"),
                        str(source / "vt13_control.c"), str(bridge), "-lm", "-o", str(library)], check=True)
        cls.lib = c.CDLL(str(library))
        signatures = {
            "Vt13MapControl": ([c.POINTER(Input), c.POINTER(Control)], None),
            "Vt13ReadControl": ([c.POINTER(Control)], None),
            "Vt13ControlEnabled": ([c.POINTER(Control)], c.c_bool),
            "Vt13AutoAimRequested": ([c.POINTER(Control)], c.c_bool),
            "Vt13TakeMouseDelta": ([c.POINTER(MouseControl), c.POINTER(Control),
                                    c.POINTER(c.c_int16), c.POINTER(c.c_int16)], c.c_bool),
            "Vt13UpdateKeyboardMotion": ([c.POINTER(KeyboardMotion), c.POINTER(Control),
                                           c.c_uint32, c.c_float], None),
            "Vt13StatusReady": ([c.POINTER(Status), c.c_uint32], c.c_bool),
            "Vt13WrapAngle": ([c.c_float], c.c_float),
            "Vt13ChassisHeading": ([c.POINTER(Status)], c.c_float),
            "Vt13UpdateInit": ([c.POINTER(Init), c.c_bool, c.c_bool, c.c_float,
                                c.c_float, c.c_uint32, c.c_uint32], None),
            "Vt13UpdateSpin": ([c.POINTER(Spin), c.POINTER(Control), c.POINTER(Status),
                                c.c_float, c.c_uint32], c.c_int),
            "Vt13UpdateFriction": ([c.POINTER(Shoot), c.POINTER(Control), c.c_bool], None),
            "Vt13ManualBurst": ([c.POINTER(Shoot), c.POINTER(Control)], c.c_bool),
            "Vt13AutoFireRequested": ([c.POINTER(Control), c.POINTER(Status)], c.c_bool),
            "test_feed": ([c.c_void_p, c.c_uint16, c.c_uint32], None),
            "test_time": ([c.c_uint32], None),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(cls.lib, name)
            fn.argtypes, fn.restype = args, result
        for index, structure in enumerate((Control, Init, Status, Spin, Shoot, KeyboardMotion, MouseControl)):
            assert cls.lib.test_size(index) == c.sizeof(structure)

    @classmethod
    def tearDownClass(cls):
        # Windows 需先卸载已加载 DLL，临时目录才能正常删除。
        if os.name == "nt":
            import _ctypes
            _ctypes.FreeLibrary(cls.lib._handle)
        cls.directory.cleanup()

    def setUp(self):
        self.input = Control(valid=True, mode=1)
        self.status = Status(init_state=2, ready=True, feedback_valid=True)
        self.spin, self.shoot, self.init = Spin(), Shoot(), Init()

    def step_spin(self, now=100, wz=0.0):
        self.status.update_time_ms = now
        return self.lib.Vt13UpdateSpin(c.byref(self.spin), c.byref(self.input),
                                      c.byref(self.status), wz, now)

    def friction(self, down, enabled=True):
        self.input.fn_down = down
        self.lib.Vt13UpdateFriction(c.byref(self.shoot), c.byref(self.input), enabled)

    def test_wire_to_semantic_mapping_and_timeout(self):
        self.lib.test_init()
        wire = frame(channels=(1684, 364, 1684, 364), wheel=364, mode=2)
        self.lib.test_feed(wire, len(wire), 1000)
        value = Control()
        self.lib.Vt13ReadControl(c.byref(value))
        self.assertEqual((value.move_x, value.move_y, value.yaw_input,
                          value.pitch_input, value.wheel), (-1, -1, 1, -1, -1))
        self.assertTrue(value.valid and value.fn_down and value.spin_down)
        self.lib.test_time(1100)
        self.lib.Vt13ReadControl(c.byref(value))
        self.assertFalse(value.valid)
        self.assertEqual((value.mode, value.move_x, value.wheel), (0, 0, 0))
        self.assertFalse(value.fn_down or value.spin_down)

    def test_c_switch_masks_controls_and_invalid_modes_fail_closed(self):
        raw = Input(valid=True, mode_switch=0, function_left=True, function_right=True, trigger=True,
                    keyboard=31, mouse_left=1, mouse_right=1, mouse_x=32767, mouse_y=-32768)
        raw.normalized[:] = (1, 1, 1, 1)
        value = Control()
        for mode in (0, 3):
            raw.mode_switch = mode
            self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
            self.assertFalse(self.lib.Vt13ControlEnabled(c.byref(value)))
            self.assertEqual((value.move_x, value.move_y, value.wheel), (0, 0, 0))
            self.assertFalse(value.fn_down or value.spin_down)
            self.assertFalse(value.keyboard_move or value.mouse_left or value.mouse_right)
            self.assertEqual((value.mouse_x, value.mouse_y), (0, 0))

    def test_invalid_client_buttons_never_override_sticks_or_fire(self):
        """映射层也拒绝可选段操控量，防止未经过解析器的替身/调用方误用保留值。"""
        for mode in (0, 1, 2):
            raw = Input(valid=True, mode_switch=mode, wheel=1024, function_left=True,
                        keyboard=0xffff, mouse_left=3, mouse_right=1, mouse_middle=1,
                        mouse_x=32767, mouse_y=-32768)
            raw.normalized[:] = (0.2, 0.3, 0.4, 0.5)
            value = Control()
            self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
            self.assertTrue(value.valid)
            self.assertFalse(value.keyboard_move or value.mouse_left or value.mouse_right or value.spin_down)
            self.assertEqual((value.mouse_x, value.mouse_y), (0, 0))
            if mode:
                self.assertTrue(value.fn_down)
                self.assertAlmostEqual(value.move_x, 0.5)
                self.assertAlmostEqual(value.move_y, -0.4)
                raw.trigger = True
                self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
                self.assertTrue(value.spin_down)

    def test_wire_optional_absence_keeps_n_s_initialization_enabled(self):
        """实际CRC帧经解析和映射后，不可用键鼠不再使N/S初始化反复失能。"""
        for mode in (1, 2):
            self.lib.test_init()
            state = Init()
            for i in range(16):
                now = 1000 + 14 * i
                wire = frame(channels=(1024, 1024, 1684, 1024), mode=mode,
                             buttons=(3, 3, 3), mouse=(32767, -32768, 0), keyboard=0xffff)
                self.lib.test_feed(wire, len(wire), now)
                value = Control()
                self.lib.Vt13ReadControl(c.byref(value))
                enabled = self.lib.Vt13ControlEnabled(c.byref(value))
                self.assertTrue(enabled)
                self.assertFalse(value.keyboard_move or value.mouse_left or value.mouse_right)
                self.assertEqual((value.move_x, value.move_y, value.mouse_x, value.mouse_y), (0, -1, 0, 0))
                self.lib.Vt13UpdateInit(c.byref(state), enabled, True, 0, 0, now, 3000)
                self.assertEqual(state.state, 1 if i == 0 else 2)
            self.lib.test_time(now + 100)
            self.lib.Vt13ReadControl(c.byref(value))
            self.assertFalse(self.lib.Vt13ControlEnabled(c.byref(value)))

    def test_spin_uses_trigger_not_right_function_key(self):
        """经过实际协议解码，确认右功能键和右上扳机不会混用。"""
        for mode in (1, 2):
            for right_function in (False, True):
                for trigger in (False, True):
                    self.lib.test_init()
                    wire = frame(mode=mode, right=int(right_function), trigger=int(trigger), keyboard=0)
                    self.lib.test_feed(wire, len(wire), 1000)
                    value = Control()
                    self.lib.Vt13ReadControl(c.byref(value))
                    self.assertTrue(value.valid)
                    self.assertEqual(value.spin_down, trigger)

    def test_init_requires_a_cycle_and_actual_alignment(self):
        update = self.lib.Vt13UpdateInit
        update(c.byref(self.init), True, True, 0, 0, 100, 3000)
        self.assertEqual(self.init.state, 1)
        update(c.byref(self.init), True, True, 0.04, 0, 101, 3000)
        self.assertEqual(self.init.state, 1)
        update(c.byref(self.init), True, True, 0.01, -0.01, 102, 3000)
        self.assertEqual(self.init.state, 2)
        update(c.byref(self.init), True, False, 0, 0, 103, 3000)
        self.assertEqual(self.init.state, 3)
        update(c.byref(self.init), True, True, 0, 0, 104, 3000)
        self.assertEqual(self.init.state, 3)
        update(c.byref(self.init), False, True, 0, 0, 105, 3000)
        self.assertEqual(self.init.state, 0)

    def test_init_timeout_never_becomes_ready_and_tick_wrap(self):
        for start in (100, 0xfffffff0):
            state = Init()
            self.lib.Vt13UpdateInit(c.byref(state), True, True, 1, 1, start, 3000)
            self.lib.Vt13UpdateInit(c.byref(state), True, True, 0, 0,
                                    (start + 3000) & 0xffffffff, 3000)
            self.assertEqual(state.state, 3)

    def test_init_rejects_nan_and_offline_feedback(self):
        for valid, error in ((False, 0), (True, math.nan), (True, math.inf)):
            state = Init()
            self.lib.Vt13UpdateInit(c.byref(state), True, valid, error, 0, 10, 3000)
            self.lib.Vt13UpdateInit(c.byref(state), True, valid, error, 0, 11, 3000)
            self.assertEqual(state.state, 1)

    def test_mode_priority_and_disable_cancel_return(self):
        for mode in (1, 2):
            self.input.mode = mode
            self.status.auto_active = True
            self.assertEqual(self.step_spin(), 2 if mode == 2 else 1)
            self.input.spin_down = True
            self.assertEqual(self.step_spin(), 3)
            self.input.spin_down = False
            self.assertEqual(self.step_spin(), 4)
            self.input.valid = False
            self.assertEqual(self.step_spin(), 0)
            self.assertFalse(self.spin.returning)
            self.input.valid = True

    def test_return_tracks_current_joint_after_world_angle_recovers(self):
        self.input.spin_down = True
        self.status.yaw_world = 0.1
        self.step_spin()
        self.status.yaw_world, self.status.yaw_joint = 1.2, 0.8
        self.input.spin_down = False
        self.assertEqual(self.step_spin(102), 4)
        # 云台恢复原方向，而底盘停在旧松手方向时，必须继续回正，不能开始到位计时。
        self.status.yaw_world, self.status.yaw_joint = 0.0, -1.2
        self.assertEqual(self.step_spin(104), 4)
        self.assertFalse(self.spin.settling)
        self.assertEqual(self.step_spin(204), 4)
        self.status.yaw_joint = 0.0
        self.assertEqual(self.step_spin(206), 4)
        self.assertEqual(self.step_spin(306), 1)

    def test_return_settle_restarts_on_motion_and_exits_to_correct_mode(self):
        for target in (False, True):
            self.spin = Spin()
            self.input.mode, self.status.auto_active = 2, target
            self.input.spin_down = True
            self.step_spin(10)
            self.input.spin_down = False
            self.assertEqual(self.step_spin(12), 4)
            self.assertEqual(self.step_spin(110, wz=0.1), 4)
            self.assertEqual(self.step_spin(112), 4)
            self.assertEqual(self.step_spin(210), 4)
            self.assertEqual(self.step_spin(212), 2 if target else 1)

    def test_s_with_target_keeps_fixed_heading_and_original_settle_limits(self):
        self.input.mode, self.status.auto_active = 2, True
        self.input.spin_down = True
        self.status.yaw_world, self.status.yaw_joint = 0.1, 0.8
        self.step_spin(100)
        self.input.spin_down = False
        self.status.yaw_world = 1.2
        self.assertEqual(self.step_spin(102), 4)
        self.assertAlmostEqual(self.spin.return_heading, 1.2, places=6)
        # 固定方向误差为 0.02，角速度为 0.04，保留原有目标路径的完成阈值。
        self.status.yaw_world, self.status.yaw_joint = 2.0, 0.82
        self.assertEqual(self.step_spin(104, wz=0.04), 4)
        self.assertTrue(self.spin.settling)
        self.assertAlmostEqual(self.spin.return_heading, 1.2, places=6)
        self.assertEqual(self.step_spin(203, wz=0.04), 4)
        self.assertEqual(self.step_spin(204, wz=0.04), 2)

    def test_repress_cancels_return_and_restarts_settle_on_release(self):
        self.input.spin_down = True
        self.step_spin()
        self.input.spin_down = False
        self.step_spin(102)
        self.input.spin_down = True
        self.assertEqual(self.step_spin(104), 3)
        self.assertFalse(self.spin.returning)
        self.assertFalse(self.spin.settling)
        self.status.yaw_world = 1.0
        self.status.yaw_joint = 0.2
        self.input.spin_down = False
        self.assertEqual(self.step_spin(106), 4)
        self.assertFalse(self.spin.settling)
        self.status.yaw_joint = 0
        self.assertEqual(self.step_spin(108), 4)
        self.assertEqual(self.step_spin(208), 1)

    def test_return_uses_follow_stop_thresholds_in_both_directions(self):
        for direction in (-1, 1):
            self.spin = Spin()
            self.status.yaw_joint = 0
            self.input.spin_down = True
            self.step_spin(10)
            self.input.spin_down = False
            self.step_spin(12)
            # 旧完成阈值内但仍未进入跟随停止区，不能提前恢复快速跟随。
            self.status.yaw_joint = direction * 0.02
            self.assertEqual(self.step_spin(112), 4)
            self.assertFalse(self.spin.settling)
            self.status.yaw_joint = direction * 0.014
            self.assertEqual(self.step_spin(114, wz=direction * 0.04), 4)
            self.assertFalse(self.spin.settling)
            self.assertEqual(self.step_spin(116, wz=direction * 0.029), 4)
            self.assertTrue(self.spin.settling)
            # 云台移动打断稳定计时，即使旧的计时期限已到也不能退出回正。
            self.status.yaw_joint = direction * 0.02
            self.assertEqual(self.step_spin(216), 4)
            self.assertFalse(self.spin.settling)
            self.status.yaw_joint = 0
            self.assertEqual(self.step_spin(218), 4)
            self.assertEqual(self.step_spin(317), 4)
            self.assertEqual(self.step_spin(318), 1)

    def test_heading_crosses_pi_by_shortest_path(self):
        self.status.yaw_world, self.status.yaw_joint = -3.13, 0.02
        heading = self.lib.Vt13ChassisHeading(c.byref(self.status))
        self.assertAlmostEqual(heading, 2 * math.pi - 3.15, places=5)
        delta = self.lib.Vt13WrapAngle(-3.13 - heading)
        self.assertAlmostEqual(delta, 0.02, places=5)

    def test_status_timeout_and_clock_wrap(self):
        self.status.update_time_ms = 0xfffffffc
        self.assertTrue(self.lib.Vt13StatusReady(c.byref(self.status), 5))
        self.assertFalse(self.lib.Vt13StatusReady(c.byref(self.status), 6))
        self.status.init_state = 1
        self.assertFalse(self.lib.Vt13StatusReady(c.byref(self.status), 0xfffffffd))

    def test_stale_or_invalid_feedback_cancels_spin(self):
        self.input.spin_down = True
        self.step_spin()
        self.status.update_time_ms = 100
        result = self.lib.Vt13UpdateSpin(c.byref(self.spin), c.byref(self.input),
                                         c.byref(self.status), 0, 110)
        self.assertEqual(result, 0)
        self.status.yaw_world = math.nan
        self.assertEqual(self.step_spin(112), 0)
        self.assertFalse(self.spin.spin_last)

    def test_fn_edge_hold_and_rearming(self):
        self.friction(True)
        self.assertFalse(self.shoot.friction_on)
        self.friction(False)
        self.friction(True)
        for _ in range(100):
            self.friction(True)
            self.assertTrue(self.shoot.friction_on)
        self.friction(False)
        self.friction(True)
        self.assertFalse(self.shoot.friction_on)
        self.friction(True, enabled=False)
        self.friction(True)
        self.assertFalse(self.shoot.friction_on)
        self.friction(False)
        self.friction(True)
        self.assertTrue(self.shoot.friction_on)

    def test_wheel_hysteresis_in_n_and_s_without_vision(self):
        for mode in (1, 2):
            self.input.mode = mode
            self.input.wheel = -1
            self.assertTrue(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))
            self.input.wheel = -0.93
            self.assertTrue(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))
            self.input.wheel = -0.89
            self.assertFalse(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))
        self.input.mode, self.input.wheel = 0, -1
        self.assertFalse(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))

    def test_vision_fire_requires_mode_right_key_left_key_and_permission(self):
        for mode in (0, 1, 2):
            for valid in (False, True):
                for target in (False, True):
                    for fire in (False, True):
                        for left in (False, True):
                            for right in (False, True):
                                self.input.mode, self.input.valid = mode, valid
                                self.input.mouse_left, self.input.mouse_right = left, right
                                self.status.auto_active, self.status.vision_fire = target, fire
                                result = self.lib.Vt13AutoFireRequested(c.byref(self.input), c.byref(self.status))
                                self.assertEqual(result, valid and (mode == 2 or (mode == 1 and right)) and target and fire and left)

    def test_wasd_overrides_stick_and_opposite_keys_cancel(self):
        raw = Input(valid=True, mode_switch=1, wheel=1024)
        raw.normalized[:] = (0.3, -0.4, 0.6, -0.7)
        value = Control()
        for keys, expected in ((1, (1, 0)), (2, (-1, 0)), (4, (0, 1)), (8, (0, -1)), (15, (0, 0))):
            raw.keyboard = keys
            self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
            self.assertTrue(value.keyboard_move)
            self.assertEqual((value.move_x, value.move_y), expected)
        raw.keyboard = 0
        self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
        self.assertFalse(value.keyboard_move)
        self.assertAlmostEqual(value.move_x, -0.7, places=6)
        self.assertAlmostEqual(value.move_y, -0.6, places=6)

    def test_diagonal_keyboard_speed_is_normalized(self):
        raw = Input(valid=True, mode_switch=1, wheel=1024, keyboard=9)
        value = Control()
        self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
        self.assertGreater(value.move_x, 0)
        self.assertLess(value.move_y, 0)
        self.assertAlmostEqual(math.hypot(value.move_x, value.move_y), 1.0, places=6)

    def test_shift_only_applies_in_n_and_combines_with_trigger(self):
        for mode in (0, 1, 2):
            for shift in (False, True):
                for trigger in (False, True):
                    raw = Input(valid=True, mode_switch=mode, wheel=1024,
                                keyboard=16 if shift else 0, trigger=trigger)
                    value = Control()
                    self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
                    self.assertEqual(value.spin_down, mode != 0 and (trigger or (mode == 1 and shift)))

    def test_s_keeps_stick_inputs_and_ignores_keyboard_motion(self):
        raw = Input(valid=True, mode_switch=2, wheel=1024, keyboard=17,
                    mouse_x=400, mouse_y=200, mouse_left=1, mouse_right=1)
        raw.normalized[:] = (0.2, 0.3, 0.4, 0.5)
        value = Control()
        self.lib.Vt13MapControl(c.byref(raw), c.byref(value))
        self.assertFalse(value.keyboard_move or value.spin_down)
        self.assertAlmostEqual(value.move_x, 0.5)
        self.assertAlmostEqual(value.move_y, -0.4)
        self.assertEqual((value.mouse_x, value.mouse_y), (0, 0))
        self.assertTrue(value.mouse_left)

    def test_left_mouse_is_manual_burst_in_n_only(self):
        self.input.wheel = 0
        self.input.mouse_left = True
        self.input.mouse_right = True
        self.assertTrue(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))
        self.input.mouse_left = False
        self.assertFalse(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))
        self.input.mode, self.input.mouse_left = 2, True
        self.assertFalse(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))
        self.input.mode, self.input.valid = 1, False
        self.assertFalse(self.lib.Vt13ManualBurst(c.byref(self.shoot), c.byref(self.input)))

    def test_n_right_mouse_selects_target_body_and_fixed_return(self):
        self.input.mouse_right = True
        self.status.auto_active = True
        self.assertEqual(self.step_spin(), 2)
        self.input.spin_down = True
        self.assertEqual(self.step_spin(), 3)
        self.input.spin_down = False
        self.status.yaw_world, self.status.yaw_joint = 1.2, 0.8
        self.assertEqual(self.step_spin(102), 4)
        self.assertAlmostEqual(self.spin.return_heading, 1.2, places=6)

    def test_mouse_delta_is_consumed_once_per_frame_and_resets_on_disable(self):
        state = MouseControl()
        dx, dy = c.c_int16(), c.c_int16()
        def take():
            return self.lib.Vt13TakeMouseDelta(c.byref(state), c.byref(self.input), c.byref(dx), c.byref(dy))
        self.input.sequence, self.input.mouse_x, self.input.mouse_y = 10, 400, -200
        self.assertFalse(take())
        self.input.sequence = 11
        self.assertTrue(take())
        self.assertEqual((dx.value, dy.value), (400, -200))
        for _ in range(20):
            self.assertFalse(take())
            self.assertEqual((dx.value, dy.value), (0, 0))
        self.input.mode = 0
        self.assertFalse(take())
        self.input.mode, self.input.sequence = 1, 12
        self.assertFalse(take())
        state.sequence, self.input.sequence = 0xffffffff, 0
        self.assertTrue(take())

    def test_keyboard_press_and_release_are_immediate(self):
        state = KeyboardMotion()
        self.input.keyboard_move, self.input.move_x = True, 1
        def update(now):
            self.lib.Vt13UpdateKeyboardMotion(c.byref(state), c.byref(self.input), now, 4)
        # 首次检测到按键即达到最高参考速度，同一毫秒反向也不等待加速。
        update(100)
        self.assertEqual(state.vx, 4)
        self.input.move_x = -1
        update(100)
        self.assertEqual(state.vx, -4)
        # 松键的同一拍撤销旧速度；摇杆居中为零，有输入则直接交回摇杆。
        self.input.keyboard_move, self.input.move_x = False, 0
        update(100)
        self.assertEqual(state.vx, 0)
        self.assertFalse(state.active)
        self.input.move_x = 0.5
        update(100)
        self.assertEqual(state.vx, 2)
        # 斜向合速度保持上限，对向抵消立即为零，不残留上一方向速度。
        self.input.keyboard_move = True
        self.input.move_x = self.input.move_y = math.sqrt(0.5)
        update(7000)
        self.assertAlmostEqual(math.hypot(state.vx, state.vy), 4, places=5)
        self.input.move_x = self.input.move_y = 0
        update(7000)
        self.assertEqual((state.vx, state.vy), (0, 0))
        self.input.mode = 0
        update(7000)
        self.assertFalse(state.active)
        self.assertEqual((state.vx, state.vy), (0, 0))

    def test_keyboard_release_does_not_depend_on_elapsed_time(self):
        state = KeyboardMotion()
        self.input.keyboard_move, self.input.move_x = True, 1
        update = self.lib.Vt13UpdateKeyboardMotion
        # 输入切换在毫秒回卷或长调度间隔下仍直接生效，不再等待减速积分。
        update(c.byref(state), c.byref(self.input), 0xfffffff0, 4)
        self.assertEqual(state.vx, 4)
        self.input.keyboard_move, self.input.move_x = False, 0
        update(c.byref(state), c.byref(self.input), 4, 4)
        self.assertEqual(state.vx, 0)
        self.assertFalse(state.active)
        self.input.keyboard_move, self.input.move_x = True, -1
        update(c.byref(state), c.byref(self.input), 1004, 4)
        self.assertEqual(state.vx, -4)
        self.input.keyboard_move, self.input.move_x = False, 0
        update(c.byref(state), c.byref(self.input), 1004, 4)
        self.assertEqual(state.vx, 0)
        self.input.mode = 2
        update(c.byref(state), c.byref(self.input), 1005, 4)
        self.assertFalse(state.active)
        self.assertEqual((state.vx, state.vy), (0, 0))


# 直接运行脚本时启动完整控制层测试。
if __name__ == "__main__":
    unittest.main(verbosity=2)
