import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import pygame  # 用于游戏开发和控制器输入处理的Pygame库
import copy  # 创建对象的深拷贝
import time  # 时间相关功能
import math  # 数学运算
from ToolKits.ToolBox import get_time  # 自定义函数，获取当前时间
from config import *  # 从配置文件导入所有设置

# 定义控制器按钮到可读名称的映射
button_mapping = {
    0: 'A',
    1: 'B',
    2: 'X',
    3: 'Y',
    4: 'LB',
    5: 'RB',
    6: 'BACK',
    7: 'START'
}

# 定义控制器轴到可读名称和轴编号的映射
axis_mapping = {
    0: ('LS', 0),
    1: ('LS', 1),
    2: ('RS', 0),
    3: ('RS', 1),
    4: 'LT',
    5: 'RT'
}

class XBOX_JOYSTICK:
    def __init__(self):
        """
        初始化 JOYSTICK 类，包括控制器状态、事件监听和pygame初始化。
        """
        # 用默认值初始化控制器状态，包括按钮和摇杆
        self.state = {
            'A': False,
            'B': False,
            'X': False,
            'Y': False,
            'LB': False,
            'RB': False,
            'BACK': False,
            'START': False,
            'LS': [0.0, 0.0],  # 左摇杆 [x, y]
            'RS': [0.0, 0.0],  # 右摇杆 [x, y]
            'LT': 0.0,  # 左扳机
            'RT': 0.0   # 右扳机
        }

        self.last_state = copy.deepcopy(self.state)  # 保留上一次状态以便变化检测

        pygame.init()

        self.xbox_controller = self._get_xbox_controller()  # 尝试连接到Xbox控制器

    def _get_xbox_controller(self):
        """
        尝试获取连接的Xbox控制器对象。
        
        返回:
            pygame.joystick.Joystick: 连接的Xbox控制器对象。
        抛出:
            异常: 如果没有找到Xbox控制器。
        """
        printed_message = False  # 标志以确保连接消息只打印一次

        while True:  # 持续尝试找到控制器
            joystick_count = pygame.joystick.get_count()  # 获取操纵杆的数量

            if joystick_count > 0:
                xbox_controller = pygame.joystick.Joystick(0)  # 假设第一个操纵杆是Xbox控制器
                xbox_controller.init()  # 初始化操纵杆

                print(f"{get_time()}-找到Xbox控制器, 名称: {xbox_controller.get_name()}.")
                return xbox_controller  # 返回初始化的控制器
            
            elif not printed_message:
                print(f"{get_time()}-没有找到Xbox控制器, 等待连接...")
                printed_message = True
            
            pygame.time.delay(500)  # 每半秒尝试一次
            pygame.event.get()  # 处理事件队列

    def listening_joystick(self, events=None):
        """
        监听操纵杆事件，并相应地更新控制器状态。
        """
        self.last_state = copy.deepcopy(self.state)  # 使用当前状态更新last_state以进行变化检测

        if events is None:  # 如果没有传递事件列表，则获取所有事件
            events = pygame.event.get()

        for event in events:  # 遍历所有事件
            if event.type == pygame.JOYBUTTONDOWN:  # 按钮按下事件
                button_pressed = event.button
                if button_pressed in button_mapping:  # 检查按钮是否在映射中
                    button_name = button_mapping[button_pressed]
                    self.state[button_name] = True  # 更新状态以反映按钮按下

            if event.type == pygame.JOYBUTTONUP:  # 按钮释放事件
                print(f"Button {event.button} released")
                button_released = event.button
                if button_released in button_mapping:  # 检查按钮是否在映射中
                    button_name = button_mapping[button_released]
                    self.state[button_name] = False  # 更新状态以反映按钮释放

        # 根据当前输入更新轴状态
        for i in range(self.xbox_controller.get_numaxes()):  # 遍历所有轴
            axis_value = self.xbox_controller.get_axis(i)  # 获取当前轴值
            if i in axis_mapping:  # 检查轴是否在映射中
                if isinstance(axis_mapping[i], tuple):  # 如果映射是针对摇杆（元组）
                    stick_name, index = axis_mapping[i]
                    self.state[stick_name][index] = axis_value  # 更新特定摇杆轴值
                else:
                    self.state[axis_mapping[i]] = axis_value  # 更新扳机值

        # 为符合常见控制方案，将两个摇杆的Y轴值反向
        self.state['LS'][1] = -self.state['LS'][1]
        self.state['RS'][1] = -self.state['RS'][1]

    def button_pressed(self, button_name):
        """
        检查按钮从按下到释放的状态变化。

        参数:
            button_name (str): 要检查的按钮名称。

        返回:
            bool: 如果按钮自上次更新以来被按下，则为True；否则为False。
        """
        # 如果按钮在上一个状态中未按下，但现在按下，则返回True
        return self.last_state.get(button_name, False) == False and self.state.get(button_name, False) == True

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

        return speed_forward, speed_turn

    def out(self):
        """
        退出pygame并释放资源。
        """
        pygame.quit()  # 取消初始化所有pygame模块

if __name__ == "__main__":
    # 创建一个XBOX_JOYSTICK对象
    joystick = XBOX_JOYSTICK()

    # 主循环
    while True:
        # 监听操纵杆事件
        joystick.listening_joystick()

        print(joystick.state)  # 打印当前控制器状态

        # 退出循环的条件
        if joystick.button_pressed('BACK'):
            break

        # 暂停一段时间
        time.sleep(0.1)

    # 退出pygame并释放资源
    joystick.out()




#------------------- 1.0 代码布局 -------------------
# import pygame
# import copy
# import time
# import math
# from ToolKits.ToolBox import get_time
# from config import *

# # 创建一个字典来存储按键状态
# button_mapping = {
#     0: 'A',
#     1: 'B',
#     2: 'X',
#     3: 'Y',
#     4: 'LB',
#     5: 'RB',
#     6: 'BACK',
#     7: 'START'
# }

# # 创建一个字典来存储轴状态
# axis_mapping = {
#     0: ('LS', 0),
#     1: ('LS', 1),
#     2: ('RS', 0),
#     3: ('RS', 1),
#     4: 'LT',
#     5: 'RT'
# }




# class XBOX_JOYSTICK:
#     def __init__(self):
#         """
#         初始化 JOYSTICK 类，包括手柄状态、事件监听和 pygame 初始化。
#         """
#         self.state = {
#             'A': False,
#             'B': False,
#             'X': False,
#             'Y': False,
#             'LB': False,
#             'RB': False,
#             'BACK': False,
#             'START': False,
#             'LS': [0.0, 0.0],
#             'RS': [0.0, 0.0],
#             'LT': 0.0,
#             'RT': 0.0
#         }

#         self.last_state = copy.deepcopy(self.state)

#         pygame.init()

#         self.xbox_controller = self._get_xbox_controller()

#     def _get_xbox_controller(self):
#         """
#         获取连接的 Xbox 手柄控制器对象。

#         Returns:
#             pygame.joystick.Joystick: 连接的 Xbox 手柄控制器对象。
#         Raises:
#             Exception: 如果没有找到Xbox控制器。
#         """
#         printed_message = False

#         while True:
#             joystick_count = pygame.joystick.get_count()

#             if joystick_count > 0:
#                 xbox_controller = pygame.joystick.Joystick(0)
#                 xbox_controller.init()

#                 print(f"{get_time()}-找到 Xbox 控制器，名称为 {xbox_controller.get_name()}.")
#                 return xbox_controller
            
#             elif not printed_message:
                
#                 print(f"{get_time()}-未找到 Xbox 控制器，等待连接...")
#                 printed_message = True
            
#             pygame.time.delay(500)
#             pygame.event.get() 



#     def listening_joystick(self):
#         """
#         监听手柄事件，更新手柄状态。
#         """

#         # 复制当前状态以备后用
#         self.last_state = copy.deepcopy(self.state)

#         for event in pygame.event.get():
#             if event.type == pygame.JOYBUTTONDOWN:
#                 button_pressed = event.button
#                 if button_pressed in button_mapping:
#                     button_name = button_mapping[button_pressed]
#                     self.state[button_name] = True
#                 else:
#                     print(f"Button {button_pressed} pressed")

#             if event.type == pygame.JOYBUTTONUP:
#                 button_released = event.button
#                 if button_released in button_mapping:
#                     button_name = button_mapping[button_released]
#                     self.state[button_name] = False

#         # 获取手柄的轴状态
#         for i in range(self.xbox_controller.get_numaxes()):
#             axis_value = self.xbox_controller.get_axis(i)
#             if i in axis_mapping:
#                 if isinstance(axis_mapping[i], tuple):
#                     stick_name, index = axis_mapping[i]
#                     self.state[stick_name][index] = axis_value
#                 else:
#                     self.state[axis_mapping[i]] = axis_value

#         self.state['LS'][1] = -self.state['LS'][1]
#         self.state['RS'][1] = -self.state['RS'][1]



#     def button_pressed(self, button_name):
#         """
#         检查按钮状态的转换，返回 True 如果按钮由按下变为释放。

#         Args:
#             button_name (str): 要检查的按钮名称。

#         Returns:
#             bool: 如果按钮状态由按下变为释放，则返回 True;否则返回 False。
#         """
#         return self.last_state.get(button_name, False) == False and self.state.get(button_name, False) == True
    
#     def map_joystick_values_to_motion_values(self):
#         speed_forward = self.state['RS'][1] * FORWARD_COEFF if self.state['RS'][1] > MIN_THRESHOLD or self.state['RS'][1] < -MIN_THRESHOLD else 0
        
#         vx = self.state['LS'][0] * TURN_COEFF if self.state['LS'][0] > MIN_THRESHOLD or self.state['LS'][0] < -MIN_THRESHOLD else 0
#         vy = self.state['LS'][1] * TURN_COEFF if self.state['LS'][1] > MIN_THRESHOLD or self.state['LS'][1] < -MIN_THRESHOLD else 0
        
#         length = math.sqrt(vx**2 + vy**2)
#         if length > TURN_COEFF:
#             vx = vx / length * TURN_COEFF
#             vy = vy / length * TURN_COEFF
#         speed_turn = [vx, vy]

#         return speed_forward, speed_turn
    
#     def out(self):
#         """
#         退出 pygame,释放资源。
#         """
#         pygame.quit()
