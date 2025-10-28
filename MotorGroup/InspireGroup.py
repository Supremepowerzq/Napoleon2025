import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

from Interface.InspireInterface import InspireMotor
from Interface.RmdInterface import RmdMotor

from ToolKits.ToolBox import change_config_value, get_time
from config import *

import math

class MotorGroup:
    def __init__(self, ROBOT_SERIAL):
        [RMD_SERIAL, INSPIRE_SERIAL] = ROBOT_SERIAL
        self.group = {
            'Up': InspireMotor(UP_MOTOR, INSPIRE_SERIAL),
            'Right': InspireMotor(RIGHT_MOTOR, INSPIRE_SERIAL),
            'Down': InspireMotor(DOWN_MOTOR, INSPIRE_SERIAL),
            'Left': InspireMotor(LEFT_MOTOR, INSPIRE_SERIAL),
            'straight': RmdMotor(0, RMD_SERIAL),
            'left_straight': RmdMotor(1, RMD_SERIAL),
            'right_straight': RmdMotor(2, RMD_SERIAL),
        }

        for key, motor in self.group.items():
            if key in ['Up', 'Right', 'Down', 'Left']:
                if motor.max_position < TURN_COEFF:
                    print(f'{get_time()}-警告：电机 {motor.ID} 最大位置小于转弯系数，可能会导致电机无法正常工作！')
    

        # 排序
        # self.group = dict(sorted(self.group.items(), key=lambda x: x[1].ID))


    def update_state(self):
        """
        更新每个电机状态。

        """
        for motor in self.group.values():
            motor.update_state()

    def stop(self):
        """
        停止所有电机。

        """
        for motor in self.group.values():
            motor.stop()

    def keep_force(self, force = 0.0):
        for motor in self.group.values():
            motor.force_control(force)

    def reset_motors_position(self):
        for motor in self.group.values():
            motor.reset_position()

    def return_zero_position(self):
        for key, motor in self.group.items():
            if key in ['Up', 'Right', 'Down', 'Left']:
                motor.return_zero_position()

    def force_stable(self):
        while True:
            self.update_state()
            current = (abs(self.group['left_straight'].current) + abs(self.group['right_straight'].current)) / 2.0
            
            if abs(current - FORWARD_CURRENT) < 0.1:
                print(f'{get_time()}-电机组力保持完成')
                self.group['left_straight'].force_control(FORWARD_CURRENT)
                self.group['right_straight'].force_control(-FORWARD_CURRENT)
                break
            else:
                speed = (FORWARD_CURRENT - current) * 200
                if speed > 0:
                    speed += 20
                else:
                    speed -= 20

            self.group['left_straight'].speed_control(speed)
            self.group['right_straight'].speed_control(-speed)

    def move(self, speed_forward):
        self.group['straight'].speed_control(speed_forward)

        # self.group['left_straight'].force_control(FORWARD_CURRENT)
        # self.group['right_straight'].force_control(-FORWARD_CURRENT)

        # self.group['left_straight'].update_state()
        # self.group['right_straight'].update_state()

        # if self.group['left_straight'].speed > 300:
        #     self.group['left_straight'].speed_control(300)
        # elif self.group['left_straight'].speed < -300:
        #     self.group['left_straight'].speed_control(-300)
        # if self.group['right_straight'].speed > 300:
        #     self.group['right_straight'].speed_control(300)
        # elif self.group['right_straight'].speed < -300:
        #     self.group['right_straight'].speed_control(-300)

        
        # self.group['left_straight'].speed_control(speed_forward*FORWARD_RATIO)
        # self.group['right_straight'].speed_control(-speed_forward*FORWARD_RATIO)

    def turn(self, position_turn):

        # p1 = -position_turn[0] * math.cos(math.pi/4) + position_turn[1] * math.sin(math.pi/4)
        # p2 = position_turn[0] * math.cos(math.pi/4) + position_turn[1] * math.sin(math.pi/4)
        # p3 = position_turn[0] * math.cos(math.pi/4) - position_turn[1] * math.sin(math.pi/4)
        # p4 = -position_turn[0] * math.cos(math.pi/4) - position_turn[1] * math.sin(math.pi/4)
        
        up = position_turn[1]
        down = -position_turn[1]
        right = position_turn[0]
        left = -position_turn[0]

        up = up if up > 0 else 0
        down = down if down > 0 else 0
        right = right if right > 0 else 0
        left = left if left > 0 else 0


        self.group['Up'].inspire_position_control(self.group['Up'].max_position-up)
        self.group['Down'].inspire_position_control(self.group['Down'].max_position-down)
        self.group['Right'].inspire_position_control(self.group['Right'].max_position-right)
        self.group['Left'].inspire_position_control(self.group['Left'].max_position-left)



    # def turn(self, speed_turn):
    #     v1 = -speed_turn[0] * math.cos(math.pi/4) + speed_turn[1] * math.sin(math.pi/4)
    #     v2 = speed_turn[0] * math.cos(math.pi/4) + speed_turn[1] * math.sin(math.pi/4)
    #     v3 = speed_turn[0] * math.cos(math.pi/4) - speed_turn[1] * math.sin(math.pi/4)
    #     v4 = -speed_turn[0] * math.cos(math.pi/4) - speed_turn[1] * math.sin(math.pi/4)

    #     v1 = v1 if v1 > 0 else 0
    #     v2 = v2 if v2 > 0 else 0
    #     v3 = v3 if v3 > 0 else 0
    #     v4 = v4 if v4 > 0 else 0

    #     # print(self.group['LeftUp'].check_arrive_zero_position(), self.group['RightUp'].check_arrive_zero_position(), self.group['RightDown'].check_arrive_zero_position(), self.group['LeftDown'].check_arrive_zero_position())
    #     if v1 > 0 and not self.group['RightDown'].check_arrive_zero_position():
    #         v3 = -v1
    #         v1 = 0
    #     if v3 > 0 and not self.group['LeftUp'].check_arrive_zero_position():
    #         v1 = -v3
    #         v3 = 0
    #     if v2 > 0 and not self.group['LeftDown'].check_arrive_zero_position():
    #         v4 = -v2
    #         v2 = 0
    #     if v4 > 0 and not self.group['RightUp'].check_arrive_zero_position():
    #         v2 = -v4
    #         v4 = 0

    #     self.group['LeftUp'].speed_control(-v1)
    #     self.group['RightUp'].speed_control(-v2)
    #     self.group['RightDown'].speed_control(-v3)
    #     self.group['LeftDown'].speed_control(-v4)



        
        





    
    def __repr__(self) -> str:
        pass