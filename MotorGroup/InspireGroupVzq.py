# import sys
# import os
# parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# sys.path.append(parent_path)

# from Interface.InspireInterface import InspireMotor
# from Interface.RmdInterface import RmdMotor

# from ToolKits.ToolBox import change_config_value, get_time
# from config import *

# import math

# class MotorGroup:
#     def __init__(self, ROBOT_SERIAL):
#         """
#         初始化电机组，创建所有电机实例。
#         :param ROBOT_SERIAL: 包含两个串口对象的列表，分别用于Inspire和Rmd电机。
#         """
#         [RMD_SERIAL, INSPIRE_SERIAL] = ROBOT_SERIAL
#         self.group = {
#             'Interval': InspireMotor(Interval_MOTOR, INSPIRE_SERIAL),
#             # 'Right': InspireMotor(RIGHT_MOTOR, INSPIRE_SERIAL),
            
            
#         }

#         # 警告：检查并打印电机最大位置是否过小，可能影响正常工作
#         for key, motor in self.group.items():
#             if key in ['Interval']:
#                 if motor.max_position < TURN_COEFF:
#                     print(f'{get_time()}-警告：电机 {motor.ID} 最大位置小于转弯系数，可能会导致电机无法正常工作！')


#     def update_state(self):
#         """
#         更新电机组中每个电机的状态。
#         """
#         for motor in self.group.values():
#             motor.update_state()


#     def stop(self):
#         """
#         停止所有电机。
#         """
#         for motor in self.group.values():
#             motor.stop()


#     def keep_force(self, force=0.0):
#         """
#         对电机组中的所有电机施加恒定的力。
#         :param force: 需要施加的力。
#         """
#         for motor in self.group.values():
#             motor.force_control(force)


#     def reset_motors_position(self):
#         """
#         重置电机组中所有电机的位置。
#         """
#         for motor in self.group.values():
#             motor.reset_position()


#     def return_zero_position(self):
#         """
#         使电机组中指定的电机回到零位。
#         """
#         for key, motor in self.group.items():
#             if key in ['Interval']:
#                 motor.return_zero_position()


#     def force_stable(self):
#         """
#         通过调整速度控制，使直行电机达到稳定的力输出。
#         """
#         while True:
#             self.update_state()
#             current_avg = (abs(self.group['left_straight'].current) + abs(self.group['right_straight'].current)) / 2.0
            
#             if abs(current_avg - FORWARD_CURRENT) < 0.1:
#                 print(f'{get_time()}-电机组力保持完成')
#                 self.group['left_straight'].force_control(FORWARD_CURRENT)
#                 self.group['right_straight'].force_control(-FORWARD_CURRENT)
#                 break
#             else:
#                 speed = (FORWARD_CURRENT - current_avg) * 200
#                 if speed > 0:
#                     speed += 20
#                 else:
#                     speed -= 20
                
#                 self.group['left_straight'].speed_control(speed)
#                 self.group['right_straight'].speed_control(-speed)


#     def move(self, speed_forward):
#         """
#         控制直行电机向前移动。
#         :param speed_forward: 直行电机的速度。
#         """
#         self.group['straight'].speed_control(speed_forward)


#     def turn(self, position_turn):
#         """
#         控制电机组进行转向操作。
#         :param position_turn: 转向位置信息，元组形式，第一个元素代表左右方向，第二个元素代表前后方向。
#         """
#         # 解析转向参数，计算上下左右四个电机的目标位置
#         up = position_turn[1] if position_turn[1] > 0 else 0
#         down = -position_turn[1] if position_turn[1] < 0 else 0
#         right = position_turn[0] if position_turn[0] > 0 else 0
#         left = -position_turn[0] if position_turn[0] < 0 else 0

#         # 调整位置控制，使电机移动至目标位置
#         self.group['Up'].inspire_position_control(self.group['Up'].max_position - up)
#         self.group['Down'].inspire_position_control(self.group['Down'].max_position + down)
#         self.group['Right'].inspire_position_control(self.group['Right'].max_position - right)
#         self.group['Left'].inspire_position_control(self.group['Left'].max_position + left)


#     # 其他方法...

#     def __repr__(self) -> str:
#         """
#         返回电机组的字符串表示。
#         """
#         return f"MotorGroup with motors: {list(self.group.keys())}"