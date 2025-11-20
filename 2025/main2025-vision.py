import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import time
import serial
import threading

from transitions import Machine

from Interface.JoystickInterfaceV2 import XboxController
from Interface.SerialInterface import find_rmd_motor_port
from Interface.RmdInterfaceV2 import RmdMotor

from ToolKits.ToolBox import get_time
from ToolKits.Timer import busy_maintain_target_frequency

from config import BAUDRATE, TIMEOUT, get_config
from predict import UnetPackage

# 电机控制参数
FORWARD_COEFF = 100  # 前进/后退速度系数
TURN_COEFF = 50     # 转向速度系数（降为一半）
# 视觉侧输出的 x>0 表示需要顺时针旋转，因此需要在此处做一次符号映射
# 如果硬件接线导致方向相反，只需把该值改成 1.0
HORIZONTAL_CLOCKWISE_SIGN = -1.0


#############################################################总开关####################################################################################################
# 是否开启摄像头图像处理线程？
CAMERA_SWITCH = True
# CAMERA_SWITCH = False

# 是否开启机器人控制线程？
# ROBOT_SWITCH = True
ROBOT_SWITCH = False


def video_processing() -> None:
    """视觉线程：持续运行Unet视频推理"""
    m_unet_package = UnetPackage(
        mode='video',
        video_path=0,
        video_save_path='',
        video_fps=30,
    )
    while True:
        m_unet_package.video()


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
        """电机归零，使用位置闭环控制"""
        finish = True

        # 一次性更新两个电机状态
        self.m1.update_state()
        self.m2.update_state()

        # 获取当前角度
        current_angle1 = self.m1.position
        current_angle2 = self.m2.position

        # 电机1：使用位置闭环，精确归零
        if abs(current_angle1) > 0.1:  # 降低阈值提高精度
            self.m1.set_position(0, max_speed=TURN_COEFF/2)  # 降低速度提高精度
            finish = False
        
        # 电机2：使用位置闭环，精确归零
        if abs(current_angle2) > 0.1:
            self.m2.set_position(0, max_speed=TURN_COEFF/3)  # 使用更低的速度
            finish = False
        
        if finish:
            # 完全停止
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


class VisionRobotStateMachine:
    """三态状态机：空闲断电、手动控制、视觉自主控制"""

    states = ('Idle', 'ManualControl', 'VisionControl')

    def __init__(self) -> None:
        print(f"{get_time()}-main2025 启动：初始化手柄与电机...")

        self.xbox = XboxController()

        port = find_rmd_motor_port(0)
        if port is None:
            raise RuntimeError("未找到RMD电机串口")

        self.ser = serial.Serial(port, baudrate=BAUDRATE, timeout=TIMEOUT)
        self.motors = MotorGroup2025(self.ser)

        self.shutdown_event = threading.Event()

        self.machine = Machine(
            model=self,
            states=self.states,
            initial='Idle',
            auto_transitions=False,
            ignore_invalid_triggers=True,
        )

        self.machine.add_transition('enter_idle', '*', 'Idle')
        self.machine.add_transition('activate_manual', ['Idle', 'VisionControl'], 'ManualControl')
        self.machine.add_transition('activate_vision', ['Idle', 'ManualControl'], 'VisionControl')

        self.state_handlers = {
            'Idle': self.loop_idle,
            'ManualControl': self.loop_manual_control,
            'VisionControl': self.loop_vision_control,
        }

        self.state_thread = threading.Thread(target=self._state_loop, name="robot_state_loop")

    def start(self) -> None:
        self.state_thread.start()

    def join(self) -> None:
        self.state_thread.join()

    def request_shutdown(self, reason: str | None = None) -> None:
        if reason:
            print(reason)
        if self.machine.state != 'Idle':
            self._transition_to_idle()
        self.shutdown_event.set()

    # ---------------------------
    # 状态进入提示
    # ---------------------------
    def on_enter_Idle(self) -> None:
        print(
            f"{get_time()}-状态切换：空闲断电\n"
            f"  START -> 手动控制\n"
            f"  Y     -> 视觉自主\n"
            f"  BACK  -> 退出程序"
        )
        self.motors.stop()
        self.motors.shutdown_all()

    def on_enter_ManualControl(self) -> None:
        print(
            f"{get_time()}-状态切换：手动控制\n"
            f"  START -> 空闲断电\n"
            f"  Y     -> 视觉自主\n"
            f"  X     -> 电机回零\n"
            f"  A     -> 将当前位置写入零点\n"
            f"  BACK  -> 退出程序"
        )

    def on_enter_VisionControl(self) -> None:
        print(
            f"{get_time()}-状态切换：视觉自主控制\n"
            f"  START -> 切回手动\n"
            f"  B     -> 空闲断电\n"
            f"  X     -> 电机回零\n"
            f"  BACK  -> 退出程序"
        )

    # ---------------------------
    # 状态循环
    # ---------------------------
    def loop_idle(self) -> None:
        if self._handle_back_button():
            return
        if self.xbox.is_button_pressed('START'):
            self._transition_to_manual()
            return
        if self.xbox.is_button_pressed('Y'):
            self._transition_to_vision()
            return
        if self.xbox.is_button_pressed('A'):
            self._set_zero_point()

    def loop_manual_control(self) -> None:
        if self._handle_common_controls(next_idle_trigger='START', next_vision_trigger='Y'):
            return

        right_trigger = self.xbox.get_trigger_value('RT')
        left_trigger = self.xbox.get_trigger_value('LT')
        right_stick_x = self.xbox.get_joystick_value('RX')
        left_stick_y = self.xbox.get_joystick_value('LY')

        angles = self.motors.get_angles()
        print(
            f"\r手动控制 "
            f"扳机(RT={right_trigger:.2f}, LT={left_trigger:.2f}) "
            f"摇杆(RX={right_stick_x:.2f}, LY={left_stick_y:.2f}) "
            f"角度(M1={angles[1]:.1f}°, M2={angles[2]:.1f}°)",
            end=""
        )

        if abs(right_trigger) > 0.05 or abs(left_trigger) > 0.05:
            forward_speed = right_trigger * FORWARD_COEFF if right_trigger > 0.05 else (-left_trigger * FORWARD_COEFF)
            self.motors.move_forward(forward_speed)
        else:
            self.motors.m0.stop()

        if abs(right_stick_x) > 0.02:
            control_value = (right_stick_x * abs(right_stick_x)) * TURN_COEFF
            self.motors.map_horizontal(control_value)
        else:
            self.motors.m1.stop()

        if abs(left_stick_y) > 0.02:
            control_value = (left_stick_y * abs(left_stick_y)) * TURN_COEFF
            self.motors.map_vertical(control_value)
            self.motors.m2.previous_command['value'] = None
        else:
            self.motors.m2.stop()

    def loop_vision_control(self) -> None:
        if self._handle_common_controls(next_idle_trigger='B', next_vision_trigger=None, allow_vision_toggle=False):
            return

        forward_speed = self._safe_float(get_config('speed_pf'))

        speed_pt = get_config('speed_pt')

        # 将来自视觉算法的 (x, y) 指令拆解成：
        #   x -> 顺/逆时针旋转（水平）
        #   y -> 上/下俯仰（垂直）
        horizontal_input = 0.0
        vertical_input = 0.0
        if isinstance(speed_pt, dict):
            horizontal_input = self._safe_float(speed_pt.get('x', 0.0))
            vertical_input = self._safe_float(speed_pt.get('y', 0.0))
        elif isinstance(speed_pt, (list, tuple)):
            if len(speed_pt) >= 1:
                horizontal_input = self._safe_float(speed_pt[0])
            if len(speed_pt) >= 2:
                vertical_input = self._safe_float(speed_pt[1])

        # x > 0 (原“右侧”) -> 顺时针，因此做一次符号映射
        horizontal_speed = self._clamp(horizontal_input * HORIZONTAL_CLOCKWISE_SIGN, TURN_COEFF)
        vertical_speed = self._clamp(vertical_input, TURN_COEFF)

        self.motors.move_forward(self._clamp(forward_speed, FORWARD_COEFF))
        self.motors.map_horizontal(horizontal_speed)
        self.motors.map_vertical(vertical_speed)
        self.motors.m2.previous_command['value'] = None

        print(
            f"\r视觉控制 F={forward_speed:.1f} HX={horizontal_speed:.1f} HY={vertical_speed:.1f}",
            end=""
        )

    # ---------------------------
    # 内部辅助
    # ---------------------------
    def _state_loop(self) -> None:
        try:
            while not self.shutdown_event.is_set():
                t0 = time.perf_counter()
                handler = self.state_handlers.get(self.machine.state)
                if handler:
                    handler()
                busy_maintain_target_frequency(60, t0)
        except KeyboardInterrupt:
            self.request_shutdown(f"{get_time()}-收到中断信号，退出中...")
        finally:
            self._cleanup()

    def _cleanup(self) -> None:
        try:
            self.motors.stop()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        print(f"{get_time()}-main2025 已退出")

    def _handle_common_controls(
        self,
        next_idle_trigger: str | None,
        next_vision_trigger: str | None,
        allow_vision_toggle: bool = True,
    ) -> bool:
        if self._handle_back_button():
            return True
        if self.xbox.is_button_pressed('X'):
            self._perform_angle_return()
            return True
        if self.xbox.is_button_pressed('A'):
            self._set_zero_point()
        if next_idle_trigger and self.xbox.is_button_pressed(next_idle_trigger):
            self._transition_to_idle()
            return True
        if allow_vision_toggle and next_vision_trigger and self.xbox.is_button_pressed(next_vision_trigger):
            self._transition_to_vision()
            return True
        if self.machine.state == 'VisionControl' and self.xbox.is_button_pressed('START'):
            self._transition_to_manual()
            return True
        return False

    def _handle_back_button(self) -> bool:
        if self.xbox.is_button_pressed('BACK'):
            self.request_shutdown(f"{get_time()}-接收到 BACK，准备退出...")
            return True
        return False

    def _perform_angle_return(self) -> None:
        print(f"\n{get_time()}-触发角度归零...")
        while not self.motors.angle_return():
            busy_maintain_target_frequency(60, time.perf_counter())
        self.motors.stop()
        print(f"{get_time()}-角度归零完成")

    def _set_zero_point(self) -> None:
        print(f"{get_time()}-设定当前位置为零点（写入ROM并重启）...")
        self.motors.set_current_position_as_zero_point()
        print(f"{get_time()}-零点设定完成")

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(min(value, limit), -limit)

    @staticmethod
    def _safe_float(value) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _transition_to_idle(self) -> None:
        self.machine.trigger('enter_idle')

    def _transition_to_manual(self) -> None:
        self.machine.trigger('activate_manual')

    def _transition_to_vision(self) -> None:
        self.machine.trigger('activate_vision')


def robot_thread() -> None:
    robot = VisionRobotStateMachine()
    robot.start()
    try:
        robot.join()
    except KeyboardInterrupt:
        robot.request_shutdown(f"{get_time()}-收到中断信号，退出中...")
        robot.join()


def main() -> None:
    print(f"{get_time()}-main2025-vision 启动：创建线程...")

    video_thread = None
    if CAMERA_SWITCH:
        video_thread = threading.Thread(
            target=video_processing,
            name="video_thread",
            daemon=ROBOT_SWITCH,
        )
        video_thread.start()
        print(f"{get_time()}-video_thread 已启动 (Camera_switch=True)")
    else:
        print(f"{get_time()}-Camera_switch=False，跳过视觉线程启动")

    if not ROBOT_SWITCH:
        print(f"{get_time()}-Robot_switch=False，跳过机器人控制线程")
        return

    robot_thread_obj = threading.Thread(target=robot_thread, name="robot_thread")
    robot_thread_obj.start()

    robot_thread_obj.join()
    print(f"{get_time()}-robot_thread 已结束，程序退出。")


if __name__ == '__main__':
    main()


