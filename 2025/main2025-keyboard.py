import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import time
import ctypes
import serial

from Interface.SerialInterface import find_rmd_motor_port
from Interface.RmdInterfaceV2 import RmdMotor

from ToolKits.ToolBox import get_time
from ToolKits.Timer import busy_maintain_target_frequency

from config import BAUDRATE, TIMEOUT


class MotorGroup2025:
    def __init__(self, ser: serial.Serial) -> None:
        self.ser = ser
        self.m0 = RmdMotor(0, ser)  # 前进/后退
        self.m1 = RmdMotor(1, ser)  # 左右
        self.m2 = RmdMotor(2, ser)  # 上下

    def move_forward(self, speed_forward: float) -> None:
        self.m0.set_speed(speed_forward)

    def map_horizontal(self, value: float) -> None:
        self.m1.set_speed(value)

    def map_vertical(self, value: float) -> None:
        self.m2.set_speed(value)

    def stop(self) -> None:
        self.m0.stop()
        self.m1.stop()
        self.m2.stop()


def is_key_pressed(vk_code: int) -> bool:
    return ctypes.windll.user32.GetAsyncKeyState(vk_code) & 0x8000 != 0


def main() -> None:
    print(f"{get_time()}-main2025-keyboard 启动：初始化电机与键盘控制...")

    port = find_rmd_motor_port(0)
    if port is None:
        raise RuntimeError("未找到RMD电机串口")
    ser = serial.Serial(port, baudrate=BAUDRATE, timeout=TIMEOUT)
    motors = MotorGroup2025(ser)

    # 速度参数（可按需调整）
    SPEED_HORIZONTAL = 50.0
    SPEED_VERTICAL = 50.0
    SPEED_FORWARD = 50.0

    print(
        f"{get_time()}-初始化完成：\n"
        f"  ESC 退出\n"
        f"  空格 停止全部电机\n"
        f"  方向键 控制左右/上下（电机1/2）\n"
        f"  数字键1/2 控制前进/后退（电机0）"
    )

    # 虚拟键码
    VK_LEFT, VK_UP, VK_RIGHT, VK_DOWN = 0x25, 0x26, 0x27, 0x28
    VK_ESC, VK_SPACE = 0x1B, 0x20
    VK_1, VK_2 = 0x31, 0x32

    try:
        while True:
            t0 = time.perf_counter()

            if is_key_pressed(VK_ESC):
                print(f"{get_time()}-接收到 ESC，准备退出...")
                break

            if is_key_pressed(VK_SPACE):
                motors.stop()

            # 左右（电机1）
            horiz = 0.0
            if is_key_pressed(VK_LEFT):
                horiz = -SPEED_HORIZONTAL
            elif is_key_pressed(VK_RIGHT):
                horiz = SPEED_HORIZONTAL
            motors.map_horizontal(horiz)

            # 上下（电机2）
            vert = 0.0
            if is_key_pressed(VK_UP):
                vert = SPEED_VERTICAL
            elif is_key_pressed(VK_DOWN):
                vert = -SPEED_VERTICAL
            motors.map_vertical(vert)

            # 前进/后退（电机0）
            forward = 0.0
            if is_key_pressed(VK_1):
                forward = SPEED_FORWARD
            elif is_key_pressed(VK_2):
                forward = -SPEED_FORWARD
            motors.move_forward(forward)

            busy_maintain_target_frequency(60, t0)

    except KeyboardInterrupt:
        print(f"{get_time()}-收到中断信号，退出中...")
    finally:
        try:
            motors.stop()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print(f"{get_time()}-main2025-keyboard 已退出")


if __name__ == '__main__':
    main()


