import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import threading
import time
import inputs
from ToolKits.Timer import busy_maintain_target_frequency
from config import *
import math

class XboxController:
    def __init__(self, smooth=False) -> None:
        """
        初始化Xbox手柄

        初始化手柄状态，开启线程监听输入事件
        """
        # 尝试获取手柄列表
        devices = inputs.devices.gamepads

        if not devices:
            raise ValueError("Xbox手柄未连接")


        self.state = {
            'A': False,
            'B': False,
            'X': False,
            'Y': False,
            'LB': False,
            'RB': False,
            'BACK': False,
            'START': False,
            'TL': False, # 左摇杆按下
            'TR': False, # 右摇杆按下
            'LS': [0.0, 0.0],  # 左摇杆 [x, y]
            'RS': [0.0, 0.0],  # 右摇杆 [x, y]
            # 'LT': 0.0,  # 左扳机
            # 'RT': 0.0,   # 右扳机
            'LT': False,  # 左扳机
            'RT': False,   # 右扳机
            'DPAD_X': 0.0,  # 方向键 x
            'DPAD_Y': 0.0  # 方向键 y
        }

        self.button_press_history = {  # 记录按钮按下的历史状态
            'A': False,
            'B': False,
            'X': False,
            'Y': False,
            'LB': False,
            'RB': False,
            'BACK': False,
            'START': False,
            'TL': False,
            'TR': False,
            'LT': False,
            'RT': False,
        }

        self.shutdown_event = threading.Event()
        self.thread = threading.Thread(target=self.listen, daemon=True)
        self.thread.start()

    def listen(self) -> None:
        """
        监听输入事件

        通过线程监听输入事件，更新控制器状态
        """
        while not self.shutdown_event.is_set():
            events = inputs.get_gamepad()
            for event in events:
                self.process_event(event)
        print("Xbox controller is shutting down.")

    def process_event(self, event: inputs.InputEvent) -> None:
        """
        处理输入事件

        参数：
        - event: 输入事件
        """
        if event.ev_type == 'Key':
            self.__update_btn(event)

        elif event.ev_type == 'Absolute':
            self.__update_axis(event)

    def __update_btn(self, event: inputs.InputEvent) -> None:
        """
        更新按钮的状态

        参数：
        - event: 输入事件
        """
        
        self.button_press_history[self.__get_button_name(event)] = self.state[self.__get_button_name(event)] 
        self.state[self.__get_button_name(event)] = event.state == 1
        
    def __update_axis(self, event: inputs.InputEvent) -> None:
        """
        更新摇杆和扳机的状态
        
        参数：
        - event: 输入事件
        """
        if event.code == 'ABS_X':
            self.state['LS'][0] = event.state / 32767.0
        elif event.code == 'ABS_Y':
            self.state['LS'][1] = event.state / 32767.0
        elif event.code == 'ABS_RX':
            self.state['RS'][0] = event.state / 32767.0
        elif event.code == 'ABS_RY':
            self.state['RS'][1] = event.state / 32767.0
        elif event.code == 'ABS_Z':
            self.state['LT'] = event.state / 255.0
        elif event.code == 'ABS_RZ':
            self.state['RT'] = event.state / 255.0
        elif event.code == 'ABS_HAT0X':
            self.state['DPAD_X'] = event.state
        elif event.code == 'ABS_HAT0Y':
            self.state['DPAD_Y'] = event.state

    def __get_button_name(self, event: inputs.InputEvent) -> str:
        """
        获取按钮名称

        参数：
        - event: 输入事件

        返回：
        - str: 按钮名称
        """
        if event.code == 'BTN_START':
            return 'BACK'
        elif event.code == 'BTN_SELECT':
            return 'START'
        elif event.code == 'BTN_NORTH':
            return 'Y'
        elif event.code == 'BTN_SOUTH':
            return 'A'
        elif event.code == 'BTN_WEST':
            return 'X'
        elif event.code == 'BTN_EAST':
            return 'B'
        elif event.code == 'BTN_TL':
            return 'LB'
        elif event.code == 'BTN_TR':
            return 'RB'
        elif event.code == 'BTN_THUMBL':
            return 'TL'
        elif event.code == 'BTN_THUMBR':
            return 'TR'
        # elif event.code == 'BTN_TL2':
        #     return 'LT'
        # elif event.code == 'BTN_TR2':
        #     return 'RT'    
        
    def is_button_pressed(self, button: str) -> bool:
        """
        判断按钮是否被按下

        参数：
        - button: 按钮名称

        返回：
        - bool: 按钮是否被按下
        """
        pressed = self.state[button] == False and self.button_press_history[button] == True
        if pressed:
            self.button_press_history[button] = False
        return pressed
    
    def map_joystick_values_to_motion_values(self, is_SOFA=False):
        """
        将操纵杆值映射到运动值，应用阈值和系数进行速度和转向的计算。

        返回:
            包含前进速度和转向速度([vx, vy])的元组。
        """
        FC = SOFA_FORWARD_COEFF if is_SOFA else FORWARD_COEFF
        TC = SOFA_TURN_COEFF if is_SOFA else TURN_COEFF

        # 根据右摇杆的Y轴计算前进速度，应用阈值和系数
        speed_forward = self.state['RS'][1] * FC if self.state['RS'][1] > MIN_THRESHOLD or self.state['RS'][1] < -MIN_THRESHOLD else 0
        
        # 根据左摇杆的X和Y轴计算转向速度，应用阈值和系数
        vx = self.state['LS'][0] * TC if self.state['LS'][0] > MIN_THRESHOLD or self.state['LS'][0] < -MIN_THRESHOLD else 0
        vy = self.state['LS'][1] * TC if self.state['LS'][1] > MIN_THRESHOLD or self.state['LS'][1] < -MIN_THRESHOLD else 0
        
        # 如果必要，标准化转向速度以防止超过最大系数值
        length = math.sqrt(vx**2 + vy**2)
        if length > TC:
            vx = vx / length * TC
            vy = vy / length * TC
        speed_turn = [vx, vy]

        if abs(speed_forward) > FC:
            speed_forward = FC if speed_forward > 0 else -FC

        # time.sleep(0.1)
        return speed_forward, speed_turn
        

    def stop(self) -> None:
        """
        停止监听输入事件
        """
        self.shutdown_event.set()
        self.thread.join()

    def __repr__(self) -> str:
        """
        返回控制器状态
        """
        return str(self.state)

if __name__ == '__main__':
    xbox_controller = XboxController()
    try:
        while True:
            print(xbox_controller)
            if xbox_controller.is_button_pressed('A'):
                break
            busy_maintain_target_frequency(60, time.perf_counter())
    except KeyboardInterrupt:
        xbox_controller.stop()
