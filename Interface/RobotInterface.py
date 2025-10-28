from config import *
import time

from ToolKits.ToolBox import get_time
import ToolKits.ToolBox as ToolBox
import MotorGroup.RmdGroup as RmdGroup
from config import *
import MotorGroup.InspireGroup as InspireGroup

import pandas as pd
import cv2

import time

FULL_RMD_VERSION = 0
INSPIRE_VERSION = 1


def print_init_info():
        long_time = 0.2
        short_time = 0.1
        print(f"{get_time()}-操作指南：")
        time.sleep(long_time)
        print(f"{get_time()}-按下 BACK 键退出系统")
        time.sleep(long_time)
        print(f"{get_time()}-按下 START 键开启/关闭用户控制模式")
        time.sleep(long_time)
        print(f"{get_time()}-按下 LB 和 RB 键开启/关闭调试模式")
        time.sleep(long_time)
        print(f"{get_time()}-在非调试模式下：")
        time.sleep(long_time)
        print(f"{get_time()}-  按下 B 键显示所有电机状态")
        time.sleep(short_time)
        print(f"{get_time()}-  按下 A 键转向电机归零")
        time.sleep(short_time)
        print(f"{get_time()}-  按下 X 开关图像显示")
        time.sleep(short_time)
        print(f"{get_time()}-  按下 Y 键开启/关闭数据保存")
        time.sleep(long_time)
        print(f"{get_time()}-在调试模式下：")
        time.sleep(long_time)
        print(f"{get_time()}-  按下 X 和 B 键切换调试电机")
        time.sleep(short_time)
        print(f"{get_time()}-  按下 Y 键重置原点")
        time.sleep(short_time)

        print(f"{get_time()}-当前系统运行正常，初始化完成")



class ControlManager:
    def __init__(self, ROBOT_SERIAL, RECORD_PATHS, version=FULL_RMD_VERSION):
        print(f"{get_time()}-机器人控制管理器初始化中...")

        self.version = version

        self.user_control = False
        self.system_running = False
        self.angle_returning = False
        self.data_saving = False

        self.index = 0
        self.debug_mode = False
        self.debug_motors = ['Up', 'Right', 'Down', 'Left']
        self.debug_index = 0

        self.show_image = False

        self.time_out = 0

        print(f"{get_time()}-当前数据保存文件夹：{RECORD_PATHS}")
        ToolBox.create_directories(RECORD_PATHS)

        self.record_paths = RECORD_PATHS

        if version == FULL_RMD_VERSION:
            self.MotorGroup = RmdGroup.MotorGroup(ROBOT_SERIAL)
            print(f"{get_time()}-当前版本： FULL_RMD_VERSION")
        elif version == INSPIRE_VERSION:
            self.MotorGroup = InspireGroup.MotorGroup(ROBOT_SERIAL)
            print(f"{get_time()}-当前版本： INSPIRE_VERSION")

        print(f"{get_time()}-机器人控制管理器初始化完成")
        print(f"{get_time()}-当前控制频率：{TARGET_FREQUENCY}Hz")
        print(f"{get_time()}-当前电机组：{self.MotorGroup.group.keys()}")

        print_init_info()


    def Instruction_ToggleImageShow(self):
        if self.show_image:
            self.show_image = False
            print(f"{get_time()}-图像显示关闭")
            cv2.destroyWindow("Image")
        else:
            if not self.user_control:
                print(f"{get_time()}-请先开启用户控制模式")
                return
            self.show_image = True
            print(f"{get_time()}-图像显示开启")
            cv2.namedWindow("Image", cv2.WINDOW_AUTOSIZE)

    def Instruction_ImageShow(self, image, micro_image):
        if self.show_image:
            if image is not None and micro_image is not None:
                # 拼接图像
                img_1 = cv2.resize(image, (400, 400))
                img_2 = cv2.resize(micro_image, (400, 400))

                img = cv2.hconcat([img_1, img_2])
                cv2.imshow("Image", img)
                cv2.waitKey(1)
            if image is None:
                cv2.imshow("Image", micro_image)    
                cv2.waitKey(1)
            if micro_image is None:
                cv2.imshow("Image", image)    
                cv2.waitKey(1)

    def Instruction_Idle(self):
        try:
            if self.version == FULL_RMD_VERSION:
                self.MotorGroup.keep_force(0.0)
        except Exception as e:
            print(f"{get_time()}-机器人待机状态出错，退出系统")
            print(f"{get_time()}-错误信息：{e}")
            self.system_running = False

    def Instruction_Move(self, speed_forward):
        try:
            self.MotorGroup.move(speed_forward)
        except Exception as e:
            print(f"{get_time()}-机器人移动出错，关闭用户控制模式")
            print(f"{get_time()}-错误信息：{e}")
            self.user_control = False

    def Instruction_Turn(self, position_turn):
        try:
            if not self.angle_returning:
                self.MotorGroup.turn(position_turn)
        except Exception as e:
            print(f"{get_time()}-机器人转向出错，关闭用户控制模式")
            print(f"{get_time()}-错误信息：{e}")
            self.user_control = False

    def Instruction_ToggleDataSaving(self):
        if self.data_saving:
            self.data_saving = False
            print(f"{get_time()}-数据保存关闭")
        else:
            self.data_saving = True
            print(f"{get_time()}-数据保存开启")
            current_time = get_time()
            self.ControlDataFile = self.record_paths['ControlData'] + current_time + '.csv'
            self.ImageDataPath = self.record_paths['ImageData'] + current_time + '\\'
            self.MicroImageDataPath = self.record_paths['MicroImageData'] + current_time + '\\'
            ToolBox.create_directories(self.ImageDataPath)
            ToolBox.create_directories(self.MicroImageDataPath)
            self.index = 0

    def Instruction_DataSave(self, speed_forward, position_turn, image, micro_image):
        if not self.data_saving:
            return
        
        data = [self.index, speed_forward, position_turn[0], position_turn[1]]
        data = pd.DataFrame(data).T
        data.to_csv(self.ControlDataFile, mode='a', header=False, index=False)

        print(f"{get_time()}-数据保存成功，当前数据序号：{self.index}", end='')

        if image is not None:
            cv2.imwrite(self.ImageDataPath + str(self.index) + '.png', image)
        else:
            print(" image图像保存失败", end='')

        if micro_image is not None:
            cv2.imwrite(self.MicroImageDataPath + str(self.index) + '.png', micro_image)
        else:
            print(" micro_image图像保存失败", end='')

        print()
        self.index += 1

    def Instruction_AngleReturn(self):
        print(f"{get_time()}-角度归零，关闭用户控制模式")
        self.user_control = False
        self.MotorGroup.return_zero_position()

    def Instruction_ToggleDebugMode(self):
        if self.debug_mode:
            self.debug_mode = False
            print(f"{get_time()}-调试模式关闭")
        else:
            self.debug_mode = True
            print(f"{get_time()}-调试模式开启，将自动关闭用户控制模式")
            print(f"{get_time()}-当前调试电机：{self.debug_motors[self.debug_index]}")

    def Instruction_Debug(self, speed):
        try:
            self.MotorGroup.group[self.debug_motors[self.debug_index]].debug_speed_control(speed)
        except Exception as e:
            print(f"{get_time()}-调试模式出错，关闭调试模式")
            print(f"{get_time()}-错误信息：{e}")
            self.debug_mode = False

    def increase_debug_index(self):
        self.debug_index += 1
        if self.debug_index >= len(self.debug_motors):
            self.debug_index = 0
        print(f"{get_time()}-当前调试电机：{self.debug_motors[self.debug_index]}")
    
    def decrease_debug_index(self):
        self.debug_index -= 1
        if self.debug_index < 0:
            self.debug_index = len(self.debug_motors) - 1
        print(f"{get_time()}-当前调试电机：{self.debug_motors[self.debug_index]}")

    def Instruction_SystemStart(self):
        self.system_running = True

    def Instruction_SystemStop(self):
        self.system_running = False
        print(f"{get_time()}-系统停止")
        self.Instruction_StopAll()
        cv2.destroyAllWindows()

    def Instruction_ToggleUserControl(self):
        if self.user_control:
            self.user_control = False
            print(f"{get_time()}-用户控制模式关闭")
            # self.Instruction_StopAll()
            self.MotorGroup.keep_force(0.0)
            if self.show_image:
                cv2.destroyWindow("Image")
                self.show_image = False
                
        else:
            if self.debug_mode:
                print(f"{get_time()}-调试模式开启，无法开启用户控制模式")
            else:
                self.user_control = True
                print(f"{get_time()}-等待力矩稳定...")
                self.MotorGroup.force_stable()
                print(f"{get_time()}-用户控制模式开启")

    def Instruction_ResetOrigin(self):
        print(f"{get_time()}-重置原点")
        try:
            self.MotorGroup.reset_motors_position()
            print(f"{get_time()}-重置原点完成")
        except Exception as e:
            print(f"{get_time()}-重置原点出错")
            print(f"{get_time()}-错误信息：{e}")
        
    def Instruction_StopAll(self):
        print(f"{get_time()}-停止所有电机")
        try:
            self.MotorGroup.stop()
        except Exception as e:
            print(f"{get_time()}-停止所有电机出错")
            print(f"{get_time()}-错误信息：{e}")

    def Instruction_ShowStates(self):
        print(f"{get_time()}-显示所有电机状态")
        try:
            for motor in self.MotorGroup.group.values():
                print(repr(motor))
        except Exception as e:
            print(f"{get_time()}-显示所有电机状态出错")
            print(f"{get_time()}-错误信息：{e}")

    def loop_start(self):
        self.start_time = time.time()
    
    def loop_end(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time < TARGET_PERIOD:
            self.time_out = 0
            # if self.user_control:
            #     print(f"{get_time()}-循环频率：{1.0 / elapsed_time}Hz")
            time.sleep(TARGET_PERIOD - elapsed_time)
        else:
            self.time_out = self.time_out + 1
            if self.time_out > 10:
                print(f"{get_time()}-循环时间：{elapsed_time}秒")
                raise Exception("循环时间超过目标周期，请降低目标频率。")

