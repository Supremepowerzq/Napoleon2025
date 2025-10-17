import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import ToolKits.ToolBox as ToolBox
from ToolKits.ToolBox import get_time
import Interface.JoystickInterface as JoystickInterface
import Interface.RobotInterface as RobotInterface
import Interface.SerialInterface as SerialInterface
import Interface.CameraInterface as CameraInterface
import cv2
import Segmentation.image_rectify as image_rectify

from config import *

import math

if __name__ == '__main__':
    print(f"------------------------------------------------------------------------------------------")
    print(f"------------------------------------------------------------------------------------------")
    print(f"{get_time()}-系统运行日志：")

    XBOX_JOYSTICK = JoystickInterface.XBOX_JOYSTICK()

    ROBOT_SERIAL = SerialInterface.get_serial()

    sorted_points = []
    try:
        # CAMERA = CameraInterface.RealSenseL515()
        raise Exception("略过深度相机初始化")
    except:
        print(f"{get_time()}-警告：未检测到深度相机!")
        CAMERA = None
    
    try:
        MICRO_CAMERA = cv2.VideoCapture(CameraInterface.find_micro_camera(), cv2.CAP_DSHOW)
        # raise Exception("略过微型相机初始化")
    except:
        print(f"{get_time()}-警告：未检测到微型相机!")
        MICRO_CAMERA = None

    CONTROL_MANAGER = RobotInterface.ControlManager(ROBOT_SERIAL, RECORD_PATHS, RobotInterface.FULL_RMD_VERSION)
    CONTROL_MANAGER.Instruction_SystemStart()

    while CONTROL_MANAGER.system_running:
        CONTROL_MANAGER.loop_start()
        XBOX_JOYSTICK.listening_joystick()

        if XBOX_JOYSTICK.button_pressed('BACK'):
            CONTROL_MANAGER.Instruction_SystemStop()

        if XBOX_JOYSTICK.button_pressed('START'):
            CONTROL_MANAGER.Instruction_ToggleUserControl()

        if XBOX_JOYSTICK.button_pressed('LB') and XBOX_JOYSTICK.button_pressed('RB'):
            CONTROL_MANAGER.Instruction_ToggleDebugMode()

        if CONTROL_MANAGER.debug_mode:
            CONTROL_MANAGER.user_control = False

            if XBOX_JOYSTICK.button_pressed('X'):
                CONTROL_MANAGER.decrease_debug_index()
            if XBOX_JOYSTICK.button_pressed('B'):
                CONTROL_MANAGER.increase_debug_index()
            if XBOX_JOYSTICK.button_pressed('Y'):
                CONTROL_MANAGER.Instruction_ResetOrigin()

            speed_forward, _ = XBOX_JOYSTICK.map_joystick_values_to_speeds()
            speed_forward = speed_forward / FORWARD_COEFF * INSPIRE_MAX_SPEED
            
            CONTROL_MANAGER.Instruction_Debug(speed_forward)

        else:
            if XBOX_JOYSTICK.button_pressed('B'):
                CONTROL_MANAGER.Instruction_ShowStates()
            if XBOX_JOYSTICK.button_pressed('A'):
                CONTROL_MANAGER.Instruction_AngleReturn()
            if XBOX_JOYSTICK.button_pressed('Y'):
                CONTROL_MANAGER.Instruction_ToggleDataSaving()
            if XBOX_JOYSTICK.button_pressed('X'):
                CONTROL_MANAGER.Instruction_ToggleImageShow()
        
        if CONTROL_MANAGER.user_control:
            speed_forward, position_turn = XBOX_JOYSTICK.map_joystick_values_to_motion_values()

            CONTROL_MANAGER.Instruction_Move(speed_forward)   
            CONTROL_MANAGER.Instruction_Turn(position_turn)
            
            if CAMERA is not None:
                image, _ = CAMERA.get_aligned_frames()
                if len(sorted_points) != 4:
                    print(f"{get_time()}-正在寻找标定点...")
                    m_contours = image_rectify.image_HSV_red_detect(image)
                    m_centers = image_rectify.image_moments(m_contours)
                    sorted_points = image_rectify.image_centers_sort(m_centers)
                    print(f"{get_time()}-标定点坐标：{sorted_points}")
                    continue
                rectify_image = image_rectify.image_perspective_transform(image, sorted_points)
            else:
                rectify_image = None
            if MICRO_CAMERA is not None:
                _, micro_image = MICRO_CAMERA.read()

                center = (micro_image.shape[1] / 2, micro_image.shape[0] / 2)
                M = cv2.getRotationMatrix2D(center, MICRO_CAMERA_ANGLE, 1.0)
                rotated_micro_image = cv2.warpAffine(micro_image, M, (micro_image.shape[1], micro_image.shape[0]))

            else:
                rotated_micro_image = None

            CONTROL_MANAGER.Instruction_ImageShow(rectify_image, rotated_micro_image)
            CONTROL_MANAGER.Instruction_DataSave(speed_forward, position_turn, rectify_image, rotated_micro_image)

        else:
            CONTROL_MANAGER.Instruction_Idle()

        CONTROL_MANAGER.loop_end()

    for ser in ROBOT_SERIAL:
        if ser is not None:
            ser.close()   
    XBOX_JOYSTICK.out()

    sys.exit()






