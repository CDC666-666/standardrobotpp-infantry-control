"""编译实际 VT13 协议模块，验证帧边界、字段、校验和失联处理。"""
import binascii
import ctypes
import os
from pathlib import Path
import random
import shutil
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Input(ctypes.Structure):
    _fields_ = [
        ("channel", ctypes.c_uint16 * 4), ("wheel", ctypes.c_uint16),
        ("mode_switch", ctypes.c_uint8),
        ("pause", ctypes.c_bool), ("function_left", ctypes.c_bool),
        ("function_right", ctypes.c_bool), ("trigger", ctypes.c_bool),
        ("mouse_x", ctypes.c_int16), ("mouse_y", ctypes.c_int16),
        ("mouse_z", ctypes.c_int16), ("mouse_left", ctypes.c_uint8),
        ("mouse_right", ctypes.c_uint8), ("mouse_middle", ctypes.c_uint8),
        ("keyboard", ctypes.c_uint16), ("sequence", ctypes.c_uint32),
        ("update_time_ms", ctypes.c_uint32), ("valid", ctypes.c_bool),
        ("normalized", ctypes.c_float * 4),
    ]


class Stats(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint32) for name in
                ("bytes", "valid_frames", "crc_errors", "invalid_frames", "keymouse_invalid_frames")]


def reference_crc(data):
    """用标准库非反射算法加位反转独立核算官方反射 CRC。"""
    reversed_bytes = bytes(int(f"{value:08b}"[::-1], 2) for value in data)
    crc = binascii.crc_hqx(reversed_bytes, 0xffff)
    return int(f"{crc:016b}"[::-1], 2)


def frame(channels=(364, 1024, 1684, 1200), wheel=1684, mode=2,
          pause=1, left=1, right=1, trigger=1,
          mouse=(-32768, 32767, -1234), buttons=(1, 0, 1), keyboard=0xa55a):
    """按线上的 64 位通道段打包，避免使用 C 位域 ABI。"""
    controls = sum(value << (11 * i) for i, value in enumerate(channels))
    controls |= mode << 44 | pause << 46 | left << 47 | right << 48
    controls |= wheel << 49 | trigger << 60
    mouse_buttons = buttons[0] | buttons[1] << 2 | buttons[2] << 4
    data = struct.pack("<BBQhhhBH", 0xa9, 0x53, controls, *mouse, mouse_buttons, keyboard)
    return data + struct.pack("<H", reference_crc(data))


class Vt13ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if not compiler:
            raise unittest.SkipTest("需要 GCC 编译实际 C 模块")
        cls.directory = tempfile.TemporaryDirectory(prefix="vt13-protocol-")
        work = Path(cls.directory.name)
        bridge = work / "bridge.c"
        bridge.write_text('''
#include "vt13_remote.h"
static Vt13Parser parser;
void test_init(void) { Vt13ParserInit(&parser); }
void test_feed(const uint8_t *data, uint16_t count, uint32_t now) {
    Vt13ParserFeed(&parser, data, count, now);
}
void test_read(Vt13Input *out, uint32_t now) { Vt13ParserRead(&parser, out, now); }
void test_stats(Vt13ParserStats *out) { *out = parser.stats; }
void test_invalidate(void) { Vt13ParserInvalidate(&parser); }
unsigned test_size(void) { return sizeof(Vt13Input); }
''', encoding="utf-8")
        library = work / ("vt13.dll" if os.name == "nt" else "vt13.so")
        subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared",
                        "-fPIC", "-I", str(ROOT / "application/remote_control/vt13"),
                        str(ROOT / "application/remote_control/vt13/vt13_remote.c"), str(bridge),
                        "-o", str(library)], check=True)
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.test_feed.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint32]
        cls.lib.test_read.argtypes = [ctypes.POINTER(Input), ctypes.c_uint32]
        cls.lib.test_stats.argtypes = [ctypes.POINTER(Stats)]
        cls.lib.Vt13Crc16.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
        cls.lib.Vt13Crc16.restype = ctypes.c_uint16
        assert cls.lib.test_size() == ctypes.sizeof(Input)

    @classmethod
    def tearDownClass(cls):
        # Windows 已加载的 DLL 不能立即删除；在子进程结束后由临时目录清理。
        if os.name == "nt":
            import _ctypes
            _ctypes.FreeLibrary(cls.lib._handle)
        cls.directory.cleanup()

    def setUp(self):
        self.lib.test_init()

    def feed(self, data, now=1000):
        self.lib.test_feed(bytes(data), len(data), now)

    def read(self, now=1000):
        value = Input()
        self.lib.test_read(ctypes.byref(value), now)
        return value

    def stats(self):
        value = Stats()
        self.lib.test_stats(ctypes.byref(value))
        return value

    def test_crc_known_vector_and_library_crosscheck(self):
        self.assertEqual(self.lib.Vt13Crc16(b"123456789", 9), 0x6f91)
        rng = random.Random(13)
        for count in range(256):
            data = bytes(rng.randrange(256) for _ in range(count))
            self.assertEqual(self.lib.Vt13Crc16(data, count), reference_crc(data))

    def test_all_fields_and_signed_limits(self):
        self.feed(frame())
        value = self.read()
        self.assertTrue(value.valid)
        self.assertEqual(list(value.channel), [364, 1024, 1684, 1200])
        self.assertEqual(list(value.normalized)[:3], [-1.0, 0.0, 1.0])
        self.assertAlmostEqual(value.normalized[3], 176 / 660, places=6)
        self.assertEqual(value.wheel, 1684)
        self.assertEqual(value.mode_switch, 2)
        self.assertTrue(value.pause and value.function_left and value.function_right and value.trigger)
        self.assertEqual((value.mouse_x, value.mouse_y, value.mouse_z), (-32768, 32767, -1234))
        self.assertEqual((value.mouse_left, value.mouse_right, value.mouse_middle), (1, 0, 1))
        self.assertEqual(value.keyboard, 0xa55a)
        self.assertEqual((value.sequence, value.update_time_ms), (1, 1000))

    def test_every_split_and_bytewise_feed(self):
        wire = frame()
        for split in range(22):
            self.lib.test_init()
            self.feed(wire[:split])
            self.feed(wire[split:])
            self.assertEqual(self.read().sequence, 1)
        self.lib.test_init()
        for byte in wire:
            self.feed(bytes([byte]))
        self.assertEqual(self.read().sequence, 1)

    def test_noise_overlapping_header_and_concatenation(self):
        self.feed(b"\0\xa5\xa9\xa9" + frame() * 40 + b"\xa9")
        self.assertEqual(self.read().sequence, 40)
        self.assertEqual(self.stats().valid_frames, 40)

    def test_each_bit_corruption_and_recovery(self):
        wire = frame()
        for bit in range(168):
            self.lib.test_init()
            bad = bytearray(wire)
            bad[bit // 8] ^= 1 << (bit % 8)
            self.feed(bad)
            self.assertFalse(self.read().valid)
            self.feed(wire)
            self.assertTrue(self.read().valid)
            self.assertEqual(self.read().sequence, 1)

    def test_crc_is_little_endian_and_not_ccitt_false(self):
        wire = frame()
        self.feed(wire[:-2] + wire[-2:][::-1])
        self.assertFalse(self.read().valid)
        ccitt = binascii.crc_hqx(wire[:-2], 0xffff)
        self.feed(wire[:-2] + struct.pack("<H", ccitt))
        self.assertFalse(self.read().valid)
        self.feed(wire)
        self.assertTrue(self.read().valid)

    def test_channel_wheel_switch_and_button_boundaries(self):
        for channel in range(4):
            for value in (363, 364, 1684, 1685):
                self.lib.test_init()
                channels = [1024] * 4
                channels[channel] = value
                self.feed(frame(channels=channels))
                self.assertEqual(self.read().valid, value in (364, 1684))
        for value in (363, 364, 1684, 1685):
            self.lib.test_init()
            self.feed(frame(wheel=value))
            self.assertEqual(self.read().valid, value in (364, 1684))
        for mode in range(4):
            self.lib.test_init()
            self.feed(frame(mode=mode))
            self.assertEqual(self.read().valid, mode < 3)
        for button in range(3):
            for value in range(4):
                self.lib.test_init()
                buttons = [0, 0, 0]
                buttons[button] = value
                self.feed(frame(buttons=buttons))
                self.assertTrue(self.read().valid)
                self.assertEqual(self.stats().keymouse_invalid_frames, int(value >= 2))

    def test_optional_keymouse_neutralized_without_losing_remote(self):
        """客户端可选段异常不能拒绝CRC正确、基础遥控合法的新帧。"""
        for mode in (0, 1, 2):
            for button in range(3):
                for invalid in (2, 3):
                    self.lib.test_init()
                    buttons = [1, 1, 1]
                    buttons[button] = invalid
                    self.feed(frame(mode=mode, buttons=buttons, keyboard=0xffff,
                                    mouse=(32767, -32768, 1234)), 1000)
                    value = self.read(1001)
                    self.assertTrue(value.valid)
                    self.assertEqual(list(value.channel), [364, 1024, 1684, 1200])
                    self.assertEqual((value.mode_switch, value.wheel, value.trigger), (mode, 1684, True))
                    self.assertEqual((value.mouse_x, value.mouse_y, value.mouse_z, value.keyboard), (0, 0, 0, 0))
                    self.assertEqual((value.mouse_left, value.mouse_right, value.mouse_middle), (0, 0, 0))
                    self.assertEqual((value.sequence, value.update_time_ms), (1, 1000))
                    self.assertEqual(self.stats().invalid_frames, 0)
                    self.assertEqual(self.stats().keymouse_invalid_frames, 1)

    def test_optional_gap_refreshes_base_and_recovers_new_client_data(self):
        """可选段降级期间只发布新基础帧，恢复后使用新键鼠，不保留旧按键。"""
        self.feed(frame(mouse=(400, -200, 0), keyboard=17), 1000)
        for i in range(1, 16):
            now = 1000 + 14 * i
            self.feed(frame(mode=1, buttons=(3, 3, 3), keyboard=0xffff), now)
            value = self.read(now)
            self.assertTrue(value.valid)
            self.assertEqual((value.mouse_x, value.keyboard, value.mouse_left, value.mouse_right), (0, 0, 0, 0))
            self.assertEqual((value.sequence, value.update_time_ms), (i + 1, now))
        self.feed(frame(mouse=(20, 30, 0), buttons=(0, 1, 0), keyboard=8), 1224)
        value = self.read(1224)
        self.assertEqual((value.mouse_x, value.mouse_y, value.keyboard, value.mouse_right), (20, 30, 8, 1))
        self.assertEqual(self.stats().keymouse_invalid_frames, 15)

    def test_optional_degradation_never_bypasses_crc_or_core_checks(self):
        """基础通道、拨轮、档位与CRC仍为整帧保护，不能伪造在线。"""
        cases = [frame(channels=(0, 1024, 1024, 1024), buttons=(3, 3, 3)),
                 frame(wheel=0, buttons=(3, 3, 3)), frame(mode=3, buttons=(3, 3, 3))]
        bad_crc = bytearray(frame(buttons=(3, 3, 3)))
        bad_crc[-1] ^= 1
        cases.append(bytes(bad_crc))
        for wire in cases:
            self.lib.test_init()
            self.feed(frame(mode=1), 1000)
            self.feed(wire, 1090)
            value = self.read(1099)
            self.assertEqual((value.sequence, value.update_time_ms), (1, 1000))
            self.assertFalse(self.read(1100).valid)
            self.assertEqual(self.stats().keymouse_invalid_frames, 0)

    def test_missing_optional_still_requires_fresh_frames(self):
        self.feed(frame(mode=1, buttons=(3, 3, 3), keyboard=0xffff), 1000)
        self.assertTrue(self.read(1099).valid)
        value = self.read(1100)
        self.assertFalse(value.valid)
        self.assertEqual((value.keyboard, value.mouse_left, value.trigger), (0, 0, False))

    def test_bad_frames_do_not_refresh_input(self):
        self.feed(frame())
        bad = bytearray(frame())
        bad[-1] ^= 1
        self.feed(bad, 1090)
        self.feed(frame(mode=3), 1091)
        self.assertEqual(self.read(1099).sequence, 1)
        self.assertEqual(self.read(1099).update_time_ms, 1000)
        self.assertTrue(self.read(1099).valid)
        self.assertFalse(self.read(1100).valid)
        self.assertGreater(self.stats().crc_errors, 0)
        self.assertEqual(self.stats().invalid_frames, 1)

    def test_timeout_clears_controls_and_preserves_metadata(self):
        self.assertFalse(self.read().valid)
        self.feed(frame())
        value = self.read(1100)
        self.assertFalse(value.valid)
        self.assertEqual(list(value.channel), [1024] * 4)
        self.assertEqual(list(value.normalized), [0.0] * 4)
        self.assertEqual(value.wheel, 1024)
        self.assertEqual(value.keyboard, 0)
        self.assertEqual((value.mouse_x, value.mouse_left, value.trigger, value.pause), (0, 0, False, False))
        self.assertEqual((value.sequence, value.update_time_ms), (1, 1000))

    def test_tick_wrap_and_discontinuous_partial_frame(self):
        self.feed(frame(), 0xffffffd0)
        self.assertTrue(self.read((0xffffffd0 + 99) & 0xffffffff).valid)
        self.assertFalse(self.read((0xffffffd0 + 100) & 0xffffffff).valid)
        self.lib.test_init()
        wire = frame()
        self.feed(wire[:10], 0)
        self.feed(wire[10:], 100)
        self.assertFalse(self.read(100).valid)
        self.feed(wire, 101)
        self.assertTrue(self.read(101).valid)

    def test_explicit_error_invalidation_and_reconnect(self):
        self.feed(frame())
        self.feed(frame()[:10])
        self.lib.test_invalidate()
        self.assertFalse(self.read().valid)
        self.feed(frame())
        self.assertEqual(self.read().sequence, 2)

    def test_random_chunks_and_mouse_values(self):
        rng = random.Random(1313)
        for sequence in range(1, 501):
            channels = [rng.randint(364, 1684) for _ in range(4)]
            mouse = tuple(rng.randint(-32768, 32767) for _ in range(3))
            wire = frame(channels=channels, mouse=mouse)
            while wire:
                count = rng.randint(1, 11)
                self.feed(wire[:count], sequence)
                wire = wire[count:]
            value = self.read(sequence)
            self.assertEqual(value.sequence, sequence)
            self.assertEqual(list(value.channel), channels)
            for normalized, raw in zip(value.normalized, channels):
                self.assertAlmostEqual(normalized, (raw - 1024) / 660, places=6)
            self.assertEqual((value.mouse_x, value.mouse_y, value.mouse_z), mouse)


if __name__ == "__main__":
    unittest.main(verbosity=2)
