import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import time
import serial

from Interface.JoystickInterfaceV2 import XboxController
from Interface.SerialInterface import find_rmd_motor_port
from Interface.RmdInterfaceV2 import RmdMotor

from ToolKits.ToolBox import get_time
from ToolKits.Timer import busy_maintain_target_frequency

from config import BAUDRATE, TIMEOUT


class MotorGroup2025:
    def __init__(self, ser: serial.Serial) -> None:
        self.ser = ser
        # 仅使用 0/1/2 三个电机
        self.m0 = RmdMotor(0, ser)  # 前进/后退
        self.m1 = RmdMotor(1, ser)  # 左右映射
        self.m2 = RmdMotor(2, ser)  # 上下映射

    def move_forward(self, speed_forward: float) -> None:
        self.m0.set_speed(speed_forward)

    def map_horizontal(self, value: float) -> None:
        # 将水平分量映射到电机1
        self.m1.set_speed(value)

    def map_vertical(self, value: float) -> None:
        # 将垂直分量映射到电机2
        self.m2.set_speed(value)

    def angle_return(self) -> bool:
        finish = True
        finish &= self.m0.angle_return()
        finish &= self.m1.angle_return()
        finish &= self.m2.angle_return()
        return finish

    def set_current_position_as_zero_point(self) -> None:
        self.m0.set_current_position_as_zero_point()
        self.m1.set_current_position_as_zero_point()
        self.m2.set_current_position_as_zero_point()

    def stop(self) -> None:
        self.m0.stop()
        self.m1.stop()
        self.m2.stop()

    def shutdown_all(self) -> None:
        # 关闭输出，进入空闲可自由转动
        self.m0.shutdown()
        self.m1.shutdown()
        self.m2.shutdown()


def main() -> None:
    print(f"{get_time()}-main2025 启动：初始化手柄与电机...")

    # 初始化手柄（内部线程监听事件）
    xbox = XboxController()

    # 初始化串口与电机组（仅一个RMD串口）
    port = find_rmd_motor_port(0)
    if port is None:
        raise RuntimeError("未找到RMD电机串口")
    ser = serial.Serial(port, baudrate=BAUDRATE, timeout=TIMEOUT)
    motors = MotorGroup2025(ser)

    print(f"{get_time()}-初始化完成：\n"
          f"  START 退出\n"
          f"  B 切换 手动/空闲 模式（空闲=释放输出，可自由转动）\n"
          f"  A 将当前位置写入为零点（三电机，将重启更新）\n"
          f"  X 一键回零（三电机）\n"
          f"  右摇杆Y 控制电机0前后\n"
          f"  左摇杆X 控制电机1左右\n"
          f"  左摇杆Y 控制电机2上下")

    # 模式：True=手动控制；False=空闲（关闭输出可自由转动）
    manual_mode = True
    was_manual_mode = True

    try:
        while True:
            t0 = time.perf_counter()

            # 按键事件
            if xbox.is_button_pressed('START'):
                print(f"{get_time()}-接收到 START，准备退出...")
                break

            # 模式切换：B
            if xbox.is_button_pressed('B'):
                manual_mode = not manual_mode
                mode_str = "手动控制" if manual_mode else "空闲模式"
                print(f"{get_time()}-切换为 {mode_str}")

            # A：将当前位置写为零点（会重启电机）
            if xbox.is_button_pressed('A'):
                print(f"{get_time()}-设定当前位置为零点（写入ROM并重启）...")
                motors.set_current_position_as_zero_point()
                print(f"{get_time()}-零点设定完成")

            if xbox.is_button_pressed('X'):
                print(f"{get_time()}-触发角度归零...")
                # 循环调用直到完成
                while not motors.angle_return():
                    busy_maintain_target_frequency(60, time.perf_counter())
                print(f"{get_time()}-角度归零完成")

            # 进入/退出空闲模式时的瞬时处理
            if was_manual_mode and not manual_mode:
                # 切到空闲：关闭输出
                motors.shutdown_all()
            was_manual_mode = manual_mode

            if manual_mode:
                # 摇杆映射（仅在手动模式下生效）
                speed_forward, speed_turn = xbox.map_joystick_values_to_motion_values()

                # 电机0：前后
                motors.move_forward(speed_forward)

                # 电机1：左右（水平分量）
                motors.map_horizontal(speed_turn[0])

                # 电机2：上下（垂直分量）
                motors.map_vertical(speed_turn[1])

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
        print(f"{get_time()}-main2025 已退出")


if __name__ == '__main__':
    main()


