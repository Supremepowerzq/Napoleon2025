"""RMD 协议有符号字段编码回归测试。"""

import unittest

from Interface.RmdInterfaceV2 import RmdMotor


class TestRmdSignedEncoding(unittest.TestCase):
    def setUp(self):
        # 编码函数不依赖串口，跳过会访问硬件的 RmdMotor.__init__。
        motor = object.__new__(RmdMotor)
        self.encode = motor._RmdMotor__encode_signed_hex

    def test_int16_boundaries(self):
        self.assertEqual(self.encode(-32768, 4), ["80", "00"])
        self.assertEqual(self.encode(-1, 4), ["FF", "FF"])
        self.assertEqual(self.encode(32767, 4), ["7F", "FF"])

    def test_a4_negative_multiturn_positions_use_int32(self):
        # A4 位置字段单位为 0.01°，因此 -360° 对应 -36000。
        self.assertEqual(self.encode(-32700, 8), ["FF", "FF", "80", "44"])
        self.assertEqual(self.encode(-32800, 8), ["FF", "FF", "7F", "E0"])
        self.assertEqual(self.encode(-36000, 8), ["FF", "FF", "73", "60"])
        self.assertEqual(self.encode(-53900, 8), ["FF", "FF", "2D", "74"])

    def test_width_overflow_is_rejected(self):
        with self.assertRaises(OverflowError):
            self.encode(-32769, 4)
        with self.assertRaises(OverflowError):
            self.encode(32768, 4)


if __name__ == "__main__":
    unittest.main()
