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

# 电机控制参数
FORWARD_COEFF = 100  # 前进/后退速度系数
TURN_COEFF = 50     # 转向速度系数（降为一半）


class MotorGroup2025:
    def __init__(self, ser: serial.Serial) -> None:
        self.ser = ser
        # 仅使用 0/1/2 三个电机
        self.m0 = RmdMotor(0, ser)  # 前进/后退
        self.m1 = RmdMotor(1, ser)  # 左右映射
        self.m2 = RmdMotor(2, ser)  # 上下映射
        
        # 电机限位角度（度）
        self.m1_limit = (-90, 90)  # 电机1的限位：左右90度
        self.m2_limit = (-30, 30)  # 电机2的限位：上下30度

    def get_angles(self) -> tuple:
        """获取各电机当前角度（度）"""
        # 更新所有电机状态
        self.m0.update_state()
        self.m1.update_state()
        self.m2.update_state()
        # 返回位置信息
        return (self.m0.position, self.m1.position, self.m2.position)

    def move_forward(self, speed_forward: float) -> None:
        self.m0.set_speed(speed_forward)

    def map_horizontal(self, value: float) -> None:
        # 获取当前角度
        self.m1.update_state()
        current_angle = self.m1.position
        # 检查限位
        if (value > 0 and current_angle >= self.m1_limit[1]) or \
           (value < 0 and current_angle <= self.m1_limit[0]):
            # 到达限位，停止对应方向的运动
            self.m1.stop()
            return
        # 在限位范围内，正常控制
        self.m1.set_speed(value)

    def map_vertical(self, value: float) -> None:
        # 获取当前角度
        self.m2.update_state()
        current_angle = self.m2.position
        # 检查限位
        if (value > 0 and current_angle >= self.m2_limit[1]) or \
           (value < 0 and current_angle <= self.m2_limit[0]):
            # 到达限位，停止对应方向的运动
            self.m2.stop()
            return
        # 在限位范围内，正常控制
        self.m2.set_speed(value)

    def angle_return(self) -> bool:
        """电机归零，对电机2使用较慢的速度"""
        finish = True

        # 一次性更新两个电机状态
        self.m1.update_state()
        self.m2.update_state()

        # 获取当前角度
        current_angle1 = self.m1.position
        current_angle2 = self.m2.position

        # 同时控制两个电机
        if abs(current_angle1) > 0.5 or abs(current_angle2) > 0.5:
            # 电机1控制
            if abs(current_angle1) > 0.5:
                direction1 = -1 if current_angle1 > 0 else 1
                self.m1.set_speed(direction1 * TURN_COEFF)
            else:
                self.m1.stop()
            
            # 电机2控制（使用较低的速度）
            if abs(current_angle2) > 0.5:
                direction2 = -1 if current_angle2 > 0 else 1
                self.m2.set_speed(direction2 * TURN_COEFF / 2)  # 使用一半速度
            else:
                self.m2.stop()
            
            finish = False
        else:
            # 两个电机都到位，停止
            self.m1.stop()
            self.m2.stop()
        
        return finish

    def set_current_position_as_zero_point(self) -> None:
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

    print(  f"{get_time()}-初始化完成：\n"
            f"  按下\"START\" 退出\n"
            f"  Y 切换 手动/空闲 模式（空闲=释放输出，可自由转动）\n"
            f"  A 将当前位置写入为零点（将重启更新）\n"
            f"  X 回零\n"
            f"  右扳机 控制电机0前\n"
            f"  左扳机 控制电机0后\n"
            f"  右摇杆X 控制电机1左右\n"
            f"  左摇杆Y 控制电机2上下"
         )

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

            # 模式切换：Y
            if xbox.is_button_pressed('Y'):
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
                # 归零完成后立即停止所有电机
                motors.stop()
                print(f"{get_time()}-角度归零完成")

            # 进入/退出空闲模式时的瞬时处理
            if was_manual_mode and not manual_mode:
                # 切到空闲：关闭输出
                motors.shutdown_all()
            was_manual_mode = manual_mode

            if manual_mode:
                # 获取扳机和摇杆的值
                right_trigger = xbox.get_trigger_value('RT')  # 右扳机：前进
                left_trigger = xbox.get_trigger_value('LT')   # 左扳机：后退
                right_stick_x = xbox.get_joystick_value('RX')  # 右摇杆X：左右
                left_stick_y = xbox.get_joystick_value('LY')   # 左摇杆Y：上下

                # 获取当前角度
                angles = motors.get_angles()
                # 显示当前控制值和角度
                print(f"\r扳机(RT={right_trigger:.2f}, LT={left_trigger:.2f}) " + 
                      f"摇杆(RX={right_stick_x:.2f}, LY={left_stick_y:.2f}) " +
                      f"角度(M0={angles[0]:.1f}°, M1={angles[1]:.1f}°, M2={angles[2]:.1f}°)", end="")

                # 电机0：前后（使用扳机）
                if abs(right_trigger) > 0.05 or abs(left_trigger) > 0.05:
                    # 只有当扳机有明显输入时才控制
                    # 右扳机控制前进（正值），左扳机控制后退（负值）
                    forward_speed = (right_trigger * FORWARD_COEFF) if right_trigger > 0.05 else (-left_trigger * FORWARD_COEFF)
                    motors.move_forward(forward_speed)
                else:
                    # 无输入时停止电机
                    motors.m0.stop()

                # 电机1：左右（使用右摇杆X轴）
                if abs(right_stick_x) > 0.05:
                    # 只有当摇杆有明显输入时才控制
                    motors.map_horizontal(right_stick_x * TURN_COEFF)
                else:
                    # 无输入时停止电机
                    motors.m1.stop()

                # 电机2：上下（使用左摇杆Y轴）
                if abs(left_stick_y) > 0.05:
                    # 只有当摇杆有明显输入时才控制
                    motors.map_vertical(left_stick_y * TURN_COEFF)
                else:
                    # 无输入时停止电机
                    motors.m2.stop()

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


