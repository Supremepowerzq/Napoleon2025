    # from transitions import Machine
from transitions.extensions import GraphMachine as Machine
from Interface.JoystickInterfaceV2 import XboxController
from Interface.SerialInterface import find_rmd_motor_port
import threading
import time
from ToolKits.Timer import busy_maintain_target_frequency
import sys
import cv2
import serial
import os
import tempfile
from MotorGroup.RmdGroupV2 import MotorGroup
from config import *
from ToolKits.ToolBox import get_time
from RobotUI import RobotUI
import ttkbootstrap as ttk
from Interface.CameraInterface import RealSenseL515

from predict import UnetPackage
from time import sleep

from config import get_config
from Interface.InspireInterfaceVz import InspireMotor
#############################################################总开关####################################################################################################
# 是否开启摄像头图像处理线程？
Camera_switch = True
# Camera_switch = False

Robot_switch = True
# Robot_switch = False

InsRobot_switch = True
# InsRobot_switch = False
#############################################################总开关####################################################################################################
IM = None
ser = None
scale_value_force_tight =  8 # 压力0-20.00
scale_value_force_loose =  4 # 压力0-20.00

def main() -> None:

    global a
    a = False
    global b
    b = False
    global current_action
    current_action = 'A'
    # # Unet视觉部分
    # # 创建UnetPackage实例，并设置所需的参数
    # unet_package = UnetPackage(
    #     mode='video',  # 设置模式为video
    #     video_path= 2,  # 设置视频路径或摄像头索引，视频源或者0，1，2...
    #     video_save_path='',  # 设置视频保存路径,空为不保存
    #     video_fps=30  # 设置视频帧率
    # )
    # # 调用视频处理方法
    # unet_package.video()

    # print(f"{get_time()}-进入机器人初始化")
    # robot = RobotStateMachine()
    # robot.initialize()

    # while robot.state_loop_thread.is_alive():
    #     time_start = time.perf_counter()
    #     if robot.xbox_joystick.is_button_pressed('BACK'):
    #         robot.InitiateShutdown()
    #     # if robot.xbox_joystick.is_button_pressed('Y'):
    #     #     print(f"{get_time()}-按下Y键.")
    #     #     robot.show_graph = not robot.show_graph
    #     #     print(f"{get_time()}-显示状态图: {robot.show_graph}")
    #     busy_maintain_target_frequency(60, time_start)

    # global speed_pf
    # global speed_pt

    # speed_pf = 0
    # speed_pt = [0, 0] 
    # 创建并启动线程
      
    if Camera_switch :
     video_thread = threading.Thread(target=video_processing)
     video_thread.start()

    if Robot_switch :
        robot_thread = threading.Thread(target=robot_initialization)
        robot_thread.start()

    if InsRobot_switch :
        insrobot_thread = threading.Thread(target=insrobot_initialization)
        insrobot_thread.start()
         
    

    # 如果需要等待两个线程都完成再结束主程序，可以使用下面的代码
    # video_thread.join()
    # robot_thread.join()    
    # insrobot_thread.join()  # 等待线程完成初始化

    print(f"{get_time()}-退出程序")
    sys.exit()

# 添加图像引导状态，GUI待实现

def video_processing():
    """视频处理线程"""
    m_unet_package = UnetPackage( mode='video',  # 设置模式为video
    # 设置视频路径或摄像头索引，视频源或者0，1，2... 
    #1为笔记本相机，2为深度相机
    video_path= 1,  
    # video_path=r'assets\test1.mp4',
    # video_path=r'assets\test2.mp4',
    # video_path='assets\demo6.18-480.mp4',
    # video_path='assets\demo6.26-x2.mp4',
    # video_save_path='assets\result6.28-demo6.18-480.mp4',
    video_save_path='',  # 设置视频保存路径,空为不保存
    video_fps=30,  # 设置视频帧率
    )
    while True:  
     # 调用视频处理方法
        m_unet_package.video()



def robot_initialization():
    """机器人初始化线程"""
    print(f"{get_time()}-进入机器人初始化")
    robot = RobotStateMachine()
    robot.initialize()

    while robot.state_loop_thread.is_alive():
        time_start = time.perf_counter()
        if robot.xbox_joystick.is_button_pressed('BACK'):
            robot.InitiateShutdown()
        # if robot.xbox_joystick.is_button_pressed('Y'):
        #     print(f"{get_time()}-按下Y键.")
        #     robot.show_graph = not robot.show_graph
        #     print(f"{get_time()}-显示状态图: {robot.show_graph}")
        busy_maintain_target_frequency(60, time_start) 

def insrobot_initialization():
    """Ins机器人初始化线程"""
    global IM, ser
    print(f"{get_time()}-进入Ins机器人初始化")
    IM=InspireMotor()
    ser = IM.open_serial('COM5', 115200) # 改成自己的串口号和波特率，波特率默认115200
    time.sleep(1)
    
    # print('设置电缸速度以及位置信息')
    scale_value_speed = 100 # 速度0-100
    scale_value_position = 5

    # V1五腔管数据
    # scale_value_position_A = 5 # 位置0-10.00
    # scale_value_position_B = 4.65

    # # V2五腔管数据
    # scale_value_position_A = 5.25 # 位置0-10.00
    # scale_value_position_B = 5.1

    # # 测试
    # scale_value_position_A = 6.25 # 位置0-10.00
    # scale_value_position_B = 3.25


    scale_value_speed = IM.scale_value_speed(scale_value_speed)

    # scale_value_position_A = IM.scale_value_position(scale_value_position_A) 
    # scale_value_position_B = IM.scale_value_position(scale_value_position_B) 

    # scale_value_force_tight =  12 # 压力0-20.00
    # scale_value_force_loose =  6 # 压力0-20.00
    global scale_value_force_tight,scale_value_force_loose

    scale_value_force_tight =  IM.scale_value_force(scale_value_force_tight)
    scale_value_force_loose =  IM.scale_value_force(scale_value_force_loose)


class RobotStateMachine:
    # 定义状态
    states = ['Initializing', 
    'Idle', 
    'ReleasingMotors', 
    'AngleReturning', 
    'ManualControl', 
    'AutomaticControl', 
    'ShuttingDown', 
    'DebugMode',
    'PicAtControl']


    
# Initializing：初始化状态，进行机器人系统初始化。
# Idle：空闲状态，等待用户指令进入其他状态。
# ReleasingMotors：释放电机状态，允许用户释放电机并设置零点。
# ManualControl：手动控制状态，用户通过手柄控制机器人。
# AngleReturning：角度归零状态，机器人自动进行角度归零。
# AutomaticControl：自动控制状态，机器人自动执行预定任务。
# ShuttingDown：关闭状态，进行资源清理并结束程序。
# DebugMode：调试模式，用户进行系统调试。

# PicAtControl添加图像引导的自动控制模式。

    def __init__(self) -> None:
        # 初始化状态机
        self.machine = Machine(model=self, states=RobotStateMachine.states, initial='Initializing', show_auto_transitions=True,
                                title="Robot State Machine", show_conditions=True)

        # 添加转换
        self.machine.add_transition(trigger='CompleteInitialization', source='Initializing', dest='Idle')
        self.machine.add_transition(trigger='ResetToInitializing', source='*', dest='Initializing')

        self.machine.add_transition(trigger='ReleaseMotors', source='Idle', dest='ReleasingMotors')
        self.machine.add_transition(trigger='ReturnToIdle', source=['ReleasingMotors', 'ManualControl', 'AngleReturning', 'AutomaticControl', 'DebugMode','PicAtControl'], dest='Idle')
        
        self.machine.add_transition(trigger='InitiateAngleReturn', source=['Idle', 'ManualControl','PicAtControl'], dest='AngleReturning')

        self.machine.add_transition(trigger='ActivateManualControl', source=['Idle', 'AngleReturning'], dest='ManualControl')
        
        self.machine.add_transition(trigger='ActivateAutomaticControl', source='Idle', dest='AutomaticControl')
        self.machine.add_transition(trigger='DeactivateAutomaticControl', source='AutomaticControl', dest='Idle')
        
        self.machine.add_transition(trigger='InitiateShutdown', source='*', dest='ShuttingDown')
        
        self.machine.add_transition(trigger='EnterDebugMode', source=['Idle', 'ManualControl'], dest='DebugMode')
        self.machine.add_transition(trigger='ExitDebugMode', source='DebugMode', dest='Idle')
        
        self.machine.add_transition(trigger='ActivatePicAtControl', source=['Idle', 'AngleReturning'], dest='PicAtControl')
        self.machine.add_transition(trigger='DeactivatePicAtControl', source=['PicAtControl', 'AngleReturning'], dest='Idle')

        self.state_loop = {
            'Initializing': self.do_nothing,
            'Idle': self.loop_idle,
            'ReleasingMotors': self.loop_release_motors,
            'ManualControl': self.loop_manual_control,
            'AngleReturning': self.loop_angle_returning,
            'AutomaticControl': self.loop_auto_control,
            'ShuttingDown': self.do_nothing,
            'DebugMode': self.loop_debug,
            'PicAtControl': self.loop_picauto_control,
        }

        self.shutdown_event = threading.Event()  # 用于控制线程退出的事件
        self.state_loop_thread = threading.Thread(target=self.state_processing_loop, daemon=True)
        # self.image_loop_thread = threading.Thread(target=self.image_processing_loop, daemon=True)
        self.last_state = None

        self.show_graph = False


    def initialize(self) -> None:
        print(f"{get_time()}-机器人正在初始化...")
        # 初始化手柄
        self.xbox_joystick = XboxController()
        
        # self.UnetPackage = UnetPackage()
        
        self.serial_list = []
        # 初始化串口
        # self.serial_list.append(serial.Serial(find_rmd_motor_port(3), baudrate=BAUDRATE, timeout=TIMEOUT))
        self.serial_list.append(serial.Serial(find_rmd_motor_port(0), baudrate=BAUDRATE, timeout=TIMEOUT))
        print(f"{get_time()}-RMD电机串口初始化完成.")
        # 初始化电机组   
        self.motor_group = MotorGroup(self.serial_list[0])
        # self.motor_group = MotorGroup(self.serial_list[0], self.serial_list[1])

        # self.camera = RealSenseL515()
        
        self.state_loop_thread.start()
        
        # 开机时将电机现在的位置设置为零点(这样比较安全，防止断丝)
        # self.motor_group.set_current_position_as_zero_point()

        # # 开机时将电机返回零点(这样会有断丝的风险，应为之前设置的零点不一定适合现在的位置)
        # self.motor_group.angle_return()


        # self.image_loop_thread.start() 
        self.CompleteInitialization()
         # 开机时将电机返回零点(这样会有断丝的风险，应为之前设置的零点不一定适合现在的位置)
        self.motor_group.angle_return()

    def on_enter_Idle(self) -> None:
        print(f"------------------------------")
        print(f"{get_time()}-机器人处于空闲状态.")
        print(f"！！！激活前请确认电机角度归零！！！")
        print(f"        按下START键以激活手动控制.")
        print(f"        按下TL键以激活图像自动控制.")
        print(f"        按下TR键以释放电机.")
        print(f"        按下X键以进行角度归零.")
        # print(f"------------------------------")
        self.motor_group.stop()

    def loop_idle(self) -> None:

        # self.shutdown_event.set()
        # self.motor_group.stop()

        if self.xbox_joystick.is_button_pressed('START'):
            self.ActivateManualControl()
        if self.xbox_joystick.is_button_pressed('TR'):
            self.ReleaseMotors()
        if self.xbox_joystick.is_button_pressed('X'):
            self.last_state = self.state
            self.InitiateAngleReturn()
        if self.xbox_joystick.is_button_pressed('TL'):
            self.ActivatePicAtControl()

    def on_enter_PicAtControl(self) -> None:
        print(f"------------------------------")
        print(f"{get_time()}-机器人处于图像自控状态.")
        print(f"        按下START键以返回空闲状态.")
        print(f"        按下X键以进行角度归零.")
        # print(f"------------------------------")
        

    def on_enter_ReleasingMotors(self) -> None:
        print(f"------------------------------")
        print(f"{get_time()}-机器人正在释放电机.")
        print(f"        按下A键以设置当前位置为零点.")
        print(f"        按下TR键以返回空闲状态.")
        # print(f"------------------------------")
        self.motor_group.release()

    def loop_release_motors(self) -> None:
        if self.xbox_joystick.is_button_pressed('A'):
            self.motor_group.set_current_position_as_zero_point()
        if self.xbox_joystick.is_button_pressed('TR'):
            self.ReturnToIdle()

    def on_enter_ManualControl(self) -> None:
        print(f"------------------------------")
        print(f"{get_time()}-机器人处于手动控制状态.")
        print(f"        按下Y键以调整摩擦轮松紧状态.")
        print(f"        按下LB键以进入旋转转向状态.")
        print(f"        按下RB键以进入旋转前进状态.")
        print(f"        按下B键以返回手动控制状态.")
        print(f"        按下START键以返回空闲状态.")
        print(f"        按下X键以进行角度归零.")
        # print(f"------------------------------")

    #  # 手动控制循环----------------------------------原版无旋转
    # def loop_manual_control(self) -> None:
    #     if self.xbox_joystick.is_button_pressed('START'):
    #         self.ReturnToIdle()
    #     if self.xbox_joystick.is_button_pressed('X'):
    #         self.last_state = self.state
    #         self.InitiateAngleReturn()
    #     speed_forward, speed_turn = self.xbox_joystick.map_joystick_values_to_motion_values()
    #     self.motor_group.turn(speed_turn)
    #     self.motor_group.move(speed_forward)


    def loop_manual_control(self) -> None:

        # IM=InspireMotor()
        # ser = IM.open_serial('COM4', 115200) # 改成自己的串口号和波特率，波特率默认115200
        # time.sleep(1)
        # # print('设置电缸速度以及位置信息')
        # scale_value_speed = 100 # 速度0-100
        # scale_value_position_A = 5.00 # 位置0-10.00
        # scale_value_position_B = 5.00

        global a
        global b
        if self.xbox_joystick.is_button_pressed('LB'):
            a = True
            b = False
            print(f"{get_time()}-机器人处于控制旋转转向状态.")
            print("按下B键关闭机器人控制旋转模式")
        elif self.xbox_joystick.is_button_pressed('RB'):
            b = True
            a = False
            print(f"{get_time()}-机器人控制旋转并移动模式状态开启.")
            print("按下B键关闭机器人控制旋转模式")
        elif self.xbox_joystick.is_button_pressed('B'):
            a = False
            b = False
            print(f"{get_time()}-机器人控制旋转模式状态关闭.")
            print("机器人处于手动控制模式")
            
        global current_action
        if self.xbox_joystick.is_button_pressed('Y'):
            
            if current_action == 'A':
                # 执行A动作
                # IM.set_speed_and_position(ser, 1, scale_value_speed, scale_value_position_A) #位置控制
                IM.force(ser,1,scale_value_force_tight) #力控制
                print("tight——不可旋转")
                current_action = 'B'  # 切换到B动作准备状态
                time.sleep(1)  # 等待1秒，避免重复触发
            else:
                # 执行B动作
                # IM.set_speed_and_position(ser, 1, scale_value_speed, scale_value_position_B) #位置控制
                IM.force(ser,1,scale_value_force_loose) #力控制
                print("loose——可旋转")
                current_action = 'A'  # 切换回A动作准备状态
                time.sleep(1)  # 等待1秒，避免重复触发
            


        if self.xbox_joystick.is_button_pressed('START'):
            a = False
            b = False
            self.ReturnToIdle()
        if self.xbox_joystick.is_button_pressed('X'):
            self.last_state = self.state
            self.InitiateAngleReturn()

        #根据控制状态执行不同的动作
        if a:
            rotate_forward, speed_turn = self.xbox_joystick.map_joystick_values_to_motion_values()
            self.motor_group.rotate(rotate_forward)
        elif b:
            rotate_forward, speed_turn = self.xbox_joystick.map_joystick_values_to_motion_values()
            self.motor_group.turn(speed_turn)
            self.motor_group.rotate_move(rotate_forward)

        else:
            speed_forward, speed_turn = self.xbox_joystick.map_joystick_values_to_motion_values()
            self.motor_group.turn(speed_turn)
            self.motor_group.move(speed_forward)    
            speed_forward, speed_turn = 0,[0, 0]
        # speed_forward, speed_turn = 0,[0, 0]
        # self.motor_group.turn(speed_turn)
        # self.motor_group.move(speed_forward)    



     # 图像控制循环----------------------------------
    def loop_picauto_control(self) -> None:
        if self.xbox_joystick.is_button_pressed('START'):
            self.ReturnToIdle()
        if self.xbox_joystick.is_button_pressed('X'):
            self.last_state = self.state
            self.InitiateAngleReturn()
        # # 改为图像处理后的参数
        # speed_forward, speed_turn = self.xbox_joystick.map_joystick_values_to_motion_values()
        # 改为图像处理后的参数
        # print(f"{get_time()}-mainV2尝试获取映射赋值")
        # global speed_pf
        # global speed_pt
        # 获取单个配置值  
        speed_forward1 = get_config('speed_pf')  
        speed_turn1 = get_config('speed_pt') 
        # print("获取config中的值,将之赋给电机")
        # print(speed_forward1)  
        # print(speed_turn1)
        # speed_forward1, speed_turn1 = self.UnetPackage.map_pic_values_to_motion_values()
        # print(f"{get_time()}-mainV2已获取:")
        # print(f"给电机的值Speed Forward: {speed_forward1}, Speed Turn: {speed_turn1}")
        self.motor_group.turn(speed_turn1)
        # self.motor_group.move(speed_forward1)    
        # speed_forward1, speed_turn1 = 0,[0, 0]***************************
        # self.motor_group.turn(speed_turn1)
    
    def on_enter_AngleReturning(self) -> None:
        print(f"------------------------------")
        print(f"{get_time()}-机器人正在进行角度归零.")
        # print(f"------------------------------")

    def loop_angle_returning(self) -> None:  
        if self.motor_group.angle_return():
            print(f"------------------------------")
            print(f"{get_time()}-电机角度归零完成.")
            # print(f"------------------------------")
            if self.last_state == 'Idle':
                self.ReturnToIdle()
            elif self.last_state == 'ManualControl':
                self.ActivateManualControl()
            elif self.last_state == 'PicAtControl':
                self.ActivatePicAtControl()


    def on_enter_AutomaticControl(self) -> None:
        print(f"{get_time()}-机器人处于自动控制状态.")
    
    def loop_auto_control(self) -> None:
        pass

    def on_enter_ShuttingDown(self) -> None:
        print(f"------------------------------")
        print(f"{get_time()}-机器人正在关闭.")
        # print(f"------------------------------")
        self.shutdown_event.set()

        self.motor_group.stop()

        for i in self.serial_list:
            i.close()

        cv2.destroyAllWindows()  # 关闭所有OpenCV窗口
        self.state_loop_thread.join() # 等待状态处理线程退出
        # self.image_loop_thread.join() # 等待图像处理线程退出

    def on_enter_DebugMode(self) -> None:
        print(f"{get_time()}-机器人进入调试模式.")

    def loop_debug(self) -> None:
        pass

    def do_nothing(self) -> None:
        pass

    def state_processing_loop(self) -> None:
        while not self.shutdown_event.is_set():
            time_start = time.perf_counter()

            # if self.xbox_joystick.is_button_pressed('B'):
            #     print(self.motor_group)
            # 按下B可以打印电机组信息，但是和后面的旋转控制冲突了，注释掉了

            action = self.state_loop.get(self.state)
            if action:
                try:
                    action()
                except Exception as e:
                    print(f"{get_time()}-状态处理出现异常: {e}")
                    self.ReturnToIdle()

            else:
                print(f"{get_time()}-未定义状态{self.state}的处理方法.")
            busy_maintain_target_frequency(30, time_start)

            time_end = time.perf_counter()
            # print(f"{get_time()}-状态处理循环执行帧率 {1 / (time_end - time_start):.2f}.")
        print(f"{get_time()}-状态处理循环结束.")

    # def image_processing_loop(self) -> None:
    #     while not self.shutdown_event.is_set():
    #         time_start = time.perf_counter()

    #         if self.xbox_joystick.is_button_pressed('Y'):
    #             self.update_graph()
    #             cv2.imwrite('state_diagram.png', self.graph)
                
    #         # if self.show_graph:
    #         #     cv2.imshow('State Diagram', self.graph)
    #         #     cv2.waitKey(1)
    #         # else:
    #         #     cv2.destroyAllWindows()
            

    #         busy_maintain_target_frequency(30, time_start)
    #     print(f"{get_time()}-图像处理循环结束.")

    def update_graph(self):
        try:
            with tempfile.TemporaryDirectory() as tmpdirname:
                self.machine.get_graph().draw(f'{tmpdirname}/state_diagram.png', prog='dot', format='png')
                self.graph = cv2.imread(f'{tmpdirname}/state_diagram.png')
                # self.graph = cv2.resize(self.graph, (0, 0), fx=0.3, fy=0.6)
        except Exception as e:
            pass

    

if __name__ == '__main__':
    main()

    # # IM=InspireMotor()
    # # print('打开串口！')                               # 打印提示字符“打开串口”
    # # ser = IM.open_serial('COM4', 115200) # 改成自己的串口号和波特率，波特率默认115200
    # # time.sleep(1)
    # # print('设置电缸速度以及位置信息')
    # # scale_value_speed = 100 # 速度0-100
    # # scale_value_position = 10.00 # 位置0-10.00

    # scale_value_speed = IM.scale_value_speed(scale_value_speed)
    # scale_value_position = IM.scale_value_position(scale_value_position) 
    # IM.set_speed_and_position(ser, 7, scale_value_speed, scale_value_position)  # ID号改为对应电缸的ID号 0-16384对应0-100%的速度或者位置的标幺值
    # time.sleep(1)
    # IM.set_speed_and_position(ser, 7, 16384,0) #全速归零
    # # print('设置力控目标值')
    # # force(ser,1, 10)
    # # time.sleep(1)
    # # print('快速定位+软接触模式下，设置力控大小，速度，预接触位置，软接触速度')
    # # softcon(ser,4, 1000,20000,10000,163)
    # # time.sleep(1)
    # print('读取电缸状态信息')
    # IM.read_state(ser, 7)
    # # time.sleep(10) # 由于力校准时间较长，请不要漏过这个sleep并尝试重新与手通讯，可能导致插件崩溃


