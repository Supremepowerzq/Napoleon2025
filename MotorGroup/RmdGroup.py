import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

from Interface.RmdInterface import RmdMotor
from config import *
from ToolKits.ToolBox import change_config_value, get_time

class PIController:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.prev_error = 0
        self.integral = 0

    def update(self, set, current, dt):
        error = set - current
        self.integral += error * dt
        output = self.kp * error + self.ki * self.integral
        self.prev_error = error
        return output

controller = PIController(1.5, 0.1)

class MotorGroup:
    def __init__(self, ROBOT_SERIAL):
        """
        初始化 MotorGroup,管理多个电机的控制。

        电机映射关系：
        - 'left': 4号电机,反转,左侧,用于拉回操作。
        - 'right': 3号电机,正转,右侧,用于拉回操作。
        - 'up': 5号电机,正转,上侧,用于拉回操作。
        - 'down': 6号电机,反转,下侧,用于拉回操作。
        - 'left_straight': 1号电机,正转,用于回收操作。
        - 'right_straight': 2号电机,反转,用于回收操作。

        Args:
            ser: 串口通信对象。
        """
        [RMD_SERIAL, INSPIRE_SERIAL] = ROBOT_SERIAL
        self.group = {
            'down': RmdMotor(6, RMD_SERIAL),
            'right': RmdMotor(3, RMD_SERIAL),
            'up': RmdMotor(4, RMD_SERIAL),
            'left': RmdMotor(5, RMD_SERIAL),
            'left_straight': RmdMotor(1, RMD_SERIAL),
            'right_straight': RmdMotor(2, RMD_SERIAL),
            'straight': RmdMotor(0, RMD_SERIAL)
        }

        # 排序
        self.group = dict(sorted(self.group.items(), key=lambda x: x[1].ID))


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
        finish = True
        for motor in self.group.values():
            if motor.ID in ['03', '04', '05', '06']:
                velocity = controller.update(0, motor.position, TARGET_PERIOD)
                # 限制velocity在 [-300, 300]
                velocity = max(min(velocity, 300), -300)
                min_vel = 30
                if velocity > 0 and velocity < min_vel:
                    velocity = min_vel
                elif velocity < 0 and velocity > -min_vel:
                    velocity = -min_vel
                motor.speed_control(velocity)
                print('returning zero position: ', motor.ID, motor.position, velocity)
                if abs(motor.position) > 4.0:
                    finish *= False

        return finish

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
        # self.group['left_straight'].speed_control(-speed_forward*FORWARD_RATIO)
        # self.group['right_straight'].speed_control(speed_forward*FORWARD_RATIO)

    def turn(self, speed_turn):
        
        v1 = speed_turn[0] if speed_turn[0] > 0 else 0
        v3 = 0 if speed_turn[0] > 0 else -speed_turn[0]

        v2 = speed_turn[1] if speed_turn[1] > 0 else 0
        v4 = 0 if speed_turn[1] > 0 else -speed_turn[1]

        if self.group['right'].position > 0 and v3 > 0:
            v1 = -v3
            v3 = 0
        if self.group['left'].position < 0 and v1 > 0:
            v3 = -v1
            v1 = 0
        if self.group['up'].position > 0 and v4 > 0:
            v2 = -v4
            v4 = 0
        if self.group['down'].position < 0 and v2 > 0:
            v4 = -v2
            v2 = 0

        self.group['right'].speed_control(v1)
        self.group['up'].speed_control(v2)
        
        self.group['down'].speed_control(-v4)
        self.group['left'].speed_control(-v3)

        
        





    
    def __repr__(self) -> str:
        pass