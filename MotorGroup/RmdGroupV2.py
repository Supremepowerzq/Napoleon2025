import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

from config import *
import serial
from Interface.RmdInterfaceV2 import RmdMotor as RmdMotorV2
from ToolKits.ToolBox import get_time
from ToolKits.Timer import busy_maintain_target_frequency

import time

MIN_SPEED_ADJUSTMENT = 30
SPEED_FACTOR = 200
CURRENT_TOLERANCE = 0.1

# 前进灵敏度(允许最大值)
FORWARD_COEFF = 200
# 转向灵敏度(允许最大值)
TURN_COEFF = 100
TC = TURN_COEFF

# 该参数用于控制转向电机的最大角度
ANGLE_LIMIT_MAX = ANGLE_LIMIT_MAX

R = R_Speed

class MotorGroup:
    def __init__(self, primary_serial: serial.Serial=None, secondary_serial: serial.Serial=None) -> None:
        # 初始化电机组
        if secondary_serial is None:
            secondary_serial = primary_serial
        self.group = {
            'up': RmdMotorV2(3, primary_serial),
            'down': RmdMotorV2(4, primary_serial),
            'left': RmdMotorV2(5, primary_serial),
            'right': RmdMotorV2(6, primary_serial),
            
            'left_straight': RmdMotorV2(1, secondary_serial),
            'right_straight': RmdMotorV2(2, secondary_serial),
            'straight': RmdMotorV2(0, secondary_serial)
        }

        self.update_state()

        # 排序
        self.group = dict(sorted(self.group.items(), key=lambda x: x[1].id))
        print(f"{get_time()}-电机组初始化完成")

    def release(self) -> None:
        """
        释放电机组。
        """
        for motor in self.group.values():
            motor.set_current(0.0)

    def stabilize_guidance_motors(self) -> bool:
        """
        使电机组中的引导电机力保持在一个稳定的值。
        必须要在循环中调用。
        """
        # 更新电机状态
        self.group['left_straight'].update_state()
        self.group['right_straight'].update_state()

        # 计算平均电流
        average_current = (abs(self.group['left_straight'].current) + abs(self.group['right_straight'].current)) / 2.0
        
        if abs(average_current - FORWARD_CURRENT) < CURRENT_TOLERANCE:
            print(f'{get_time()}-电机组力保持完成')
            self.group['left_straight'].set_current(FORWARD_CURRENT)
            self.group['right_straight'].set_current(-FORWARD_CURRENT)
            return True
        else:
            # 如果电流不在范围内,则根据当前电流与目标电流的差值调整速度
            speed_adjustment = (FORWARD_CURRENT - average_current) * SPEED_FACTOR
            speed_adjustment = speed_adjustment + MIN_SPEED_ADJUSTMENT if speed_adjustment > 0 else speed_adjustment - MIN_SPEED_ADJUSTMENT
            self.group['left_straight'].set_speed(speed_adjustment)
            self.group['right_straight'].set_speed(-speed_adjustment)
            return False

    def set_current_position_as_zero_point(self) -> None:
        for motor in self.group.values():
            motor.set_current_position_as_zero_point()

    def stop(self) -> None:
        """
        停止电机组。
        """
        for motor in self.group.values():
            motor.stop()

    def update_state(self) -> None:
         for motor in self.group.values():
            motor.update_state()  

    def angle_return(self) -> bool:
        finish = True
        for motor in [self.group['left'], self.group['right'], self.group['up'], self.group['down']]:
            if motor.angle_return() is False:
                print(f"{get_time()}-电机{motor.id}归零未完成")
                finish = False
            motor.update_state()
        return finish   

    def move(self, speed_forward: float) -> None:
        """
        使推进电机组以指定速度运动。
        """
        if speed_forward > 0:
            self.group['straight'].set_speed(speed_forward)
            self.group['left_straight'].set_speed(speed_forward*FORWARD_RATIO)
            # print(f"speed_forward > 0:left_straight speed: {speed_forward*FORWARD_RATIO}")
            self.group['right_straight'].set_speed(-speed_forward*FORWARD_RATIO)
            # print(f"speed_forward > 0:right_straight speed: {-speed_forward*FORWARD_RATIO}")
        elif speed_forward == 0:
            self.group['straight'].set_speed(0)
            self.group['left_straight'].set_speed(0)
            self.group['right_straight'].set_speed(0)
        else:    
            self.group['straight'].set_speed(speed_forward)
            self.group['left_straight'].set_speed(speed_forward*RETRACT_RATIO)
            # print(f"speed_forward < 0:left_straight speed: {-speed_forward*RETRACT_RATIO}")
            self.group['right_straight'].set_speed(-speed_forward*RETRACT_RATIO)
            # print(f"speed_forward < 0:right_straight speed: {-speed_forward*RETRACT_RATIO}")


    def limit_position(self, motor: RmdMotorV2, increment):
                
                new_position = motor.position + increment
                # print(f"{get_time()}-电机{motor}位置:{motor.position},增量:{increment},新位置:{new_position}")
                # print(f"{get_time()}-电机：{motor}-当前位置:{motor.position}, 新位置:{new_position}")
                if new_position >= ANGLE_LIMIT_MAX:
                    # adjustment = ANGLE_LIMIT_MAX - motor.position  # 调整量为到限位角度的差值
                    # # new_position = ANGLE_LIMIT_MAX
                    # adjustment = max(-TC,adjustment)

                    # if motor.id == '03':
                    #     print(f"{get_time()}-电机：{motor}已到限位角度")

                    #改为设置电机至ANGLE_LIMIT_MAX位置
                    motor.set_position(ANGLE_LIMIT_MAX, TC) #(位置，速度)
                    print('触发:限位~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
                    # time.sleep(0.1)
                    return 0
                
                if new_position < 0:
                    # adjustment = -motor.position  # 调整量为负的当前角度
                    # # new_position = 0
                    # adjustment = max(-TC,adjustment)

                    #改为设置电机至ANGLE_LIMIT_MAX位置
                    motor.set_position(0, TC) #(位置，速度)
                    print('触发:归0------------------------------------------------------------------')
                    return 0
                
                else:
                    #  if motor.id == '03':
                    #     print(f"{get_time()}-电机：{motor}未到限位角度")
                    return increment

    def turn(self, speed_turn: tuple) -> None:
        """
        控制机器人的转向电机和速度。

        参数:
        - speed_turn (tuple): 包含两个元素的元组，分别代表转向速度的水平和垂直分量。
        """
        # speed_x, speed_y = speed_turn

        # if speed_x >= 0:
        #     if self.group['left'].position > 0:
        #         v_right = None
        #         v_left = -abs(speed_x)
        #     if self.group['right'].position > 0:
        #         v_right = abs(speed_x)
        #         v_left = None
        #     if self.group['right'].position <= 0 and self.group['left'].position <= 0:
        #         v_right = abs(speed_x)
        #         v_left = None
        # else:
        #     if self.group['right'].position > 0:
        #         v_right = -abs(speed_x)
        #         v_left = None
        #     if self.group['left'].position > 0:
        #         v_right = None
        #         v_left = abs(speed_x)
        #     if self.group['right'].position <= 0 and self.group['left'].position <= 0:
        #         v_right = None
        #         v_left = abs(speed_x)

        # if speed_y >= 0:
        #     if self.group['down'].position > 0:
        #         v_up = None
        #         v_down = -abs(speed_y)
        #     if self.group['up'].position > 0:
        #         v_up = abs(speed_y) 
        #         v_down = None
        #     if self.group['up'].position <= 0 and self.group['down'].position <= 0:
        #         v_up = abs(speed_y)
        #         v_down = None
        # else:
        #     if self.group['up'].position > 0:
        #         v_up = -abs(speed_y)
        #         v_down = None
        #     if self.group['down'].position > 0:
        #         v_up = None
        #         v_down = abs(speed_y)
        #     if self.group['up'].position <= 0 and self.group['down'].position <= 0:
        #         v_up = None
        #         v_down = abs(speed_y)

        # if v_right is not None:
        #     self.group['right'].set_speed(v_right)
        # else:
        #     self.group['right'].set_position(0)
        # if v_left is not None:
        #     self.group['left'].set_speed(v_left)
        # else:
        #     self.group['left'].set_position(0)

        # if v_up is not None:
        #     self.group['up'].set_speed(v_up)
        # else:
        #     self.group['up'].set_position(0)
        # if v_down is not None:
        #     self.group['down'].set_speed(v_down)
        # else:
        #     self.group['down'].set_position(0)


        """控制机器人的转向和速度。
        参数:
        - speed_turn (tuple): 包含两个元素的元组，分别代表转向速度的水平和垂直分量。
        """
        speed_x, speed_y = speed_turn

        # def limit_position(motor, increment):
        #     new_position = motor.position + increment
        #     if new_position < 0:
        #         new_position = 0
        #         return new_position - motor.position
        #     elif new_position > ANGLE_LIMIT_MAX:
        #         new_position = ANGLE_LIMIT_MAX
        #         print(f"{get_time()}-已到限位角度")
        #         return new_position
        #     return new_position - motor.position
        #     # 6.23,实验发现到达限位角度后会自动回退一定角度,待修正

        # def limit_position(motor, increment):
        #     new_position = motor.position + increment
        #     if new_position < 0:
        #         adjustment = -motor.position  # 调整量为负的当前角度
        #         new_position = 0
        #         return adjustment
        #     elif new_position >= ANGLE_LIMIT_MAX:
        #         adjustment = ANGLE_LIMIT_MAX - motor.position  # 调整量为到限位角度的差值
        #         new_position = ANGLE_LIMIT_MAX
        #         print(f"{get_time()}-已到限位角度")
        #         return adjustment
        #     return increment

         # 初始化变量
        v_right = None
        v_left = None
        v_up = None
        v_down = None
        # 转向灵敏度映射电机转角度系数
        # R = R 开头已经引用
        # 水平转向
        if speed_x >= 0:
            if self.group['left'].position > 0:
                v_right = None
                v_left = self.limit_position(self.group['left'], -abs(speed_x)*R)
            elif 0 < self.group['right'].position < ANGLE_LIMIT_MAX:
                v_right = self.limit_position(self.group['right'], abs(speed_x)*R)
                v_left = None
            # elif self.group['right'].position <= 0 and self.group['left'].position <= 0:
            else:
                v_right = self.limit_position(self.group['right'], abs(speed_x)*R)
                v_left = None
        if speed_x < 0:
            if self.group['right'].position > 0:
                v_right = self.limit_position(self.group['right'], -abs(speed_x)*R)
                v_left = None
            elif 0 < self.group['left'].position < ANGLE_LIMIT_MAX:
                v_right = None
                v_left = self.limit_position(self.group['left'], abs(speed_x)*R)
            # if self.group['right'].position <= 0 and self.group['left'].position <= 0:
            else:
                v_right = None
                v_left = self.limit_position(self.group['left'], abs(speed_x)*R)

        # 垂直转向
        if speed_y >= 0:
            if self.group['down'].position > 0:
                v_up = None
                v_down = self.limit_position(self.group['down'], -abs(speed_y)*R)

            elif 0 < self.group['up'].position < ANGLE_LIMIT_MAX:
                v_up = self.limit_position(self.group['up'], abs(speed_y)*R)
                v_down = None
            # if self.group['up'].position <= 0 and self.group['down'].position <= 0:
            else:
                v_up = self.limit_position(self.group['up'], abs(speed_y)*R)
                v_down = None

        if speed_y < 0:
            if self.group['up'].position > 0:
                v_up = self.limit_position(self.group['up'], -abs(speed_y)*R)
                v_down = None
            elif 0 < self.group['down'].position < ANGLE_LIMIT_MAX:
                v_up = None
                v_down = self.limit_position(self.group['down'], abs(speed_y)*R)
            # if self.group['up'].position <= 0 and self.group['down'].position <= 0:
            else:
                v_up = None
                v_down = self.limit_position(self.group['down'], abs(speed_y)*R)




        # 设置电机速度或位置
        if v_right is not None:
            self.group['right'].set_speed(v_right)
        if v_right is None:
            self.group['right'].set_position(0)
        
        if v_left is not None:
            self.group['left'].set_speed(v_left)
        if v_left is None:
            self.group['left'].set_position(0)

        if v_up is not None:
            self.group['up'].set_speed(v_up)
        if v_up is None:
            self.group['up'].set_position(0)
        
        if v_down is not None:
            self.group['down'].set_speed(v_down)
        if v_down is None:
            self.group['down'].set_position(0)

        # time.sleep(0.02)
        # self.group['right'].set_speed(0)
        # self.group['left'].set_speed(0)
        # self.group['up'].set_speed(0)
        # self.group['down'].set_speed(0)


    
    def __repr__(self) -> str:
        motor_states = [str(motor) for motor in self.group.values()]
        return "\n".join(motor_states)

    def rotate_move(self, rotate_forward: float) -> None:
        """
        rotate to move
        """
        self.group['straight'].set_speed(rotate_forward)
        self.group['left_straight'].set_speed(-rotate_forward*RR)
        self.group['right_straight'].set_speed(rotate_forward*RR)



    def rotate(self, rotate_forward: float) -> None:
        """
        rotate
        """
        # self.group['straight'].set_speed(rotate_forward)
        self.group['left_straight'].set_speed(rotate_forward*FORWARD_RATIO)
        self.group['right_straight'].set_speed(rotate_forward*FORWARD_RATIO)    


from Interface.SerialInterface import find_rmd_motor_port
import math

if __name__ == "__main__":
    port_0 = find_rmd_motor_port(0)
    print(port_0)
    port_1 = find_rmd_motor_port(3)
    print(port_1)

    ser_0 = serial.Serial(port_0, baudrate=BAUDRATE, timeout=None)
    ser_1 = serial.Serial(port_1, baudrate=BAUDRATE, timeout=None)

    motor_group = MotorGroup(ser_1, ser_0)
    motor_group.stop()
    print(motor_group)


