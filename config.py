import numpy as np
import time

# config_dict = {  
#     'speed_pf': 0,  
#     'speed_pt': [0, 0]  
# }  
  
# # 提供函数来更新配置  
# def update_config(key, value):  
#     config_dict[key] = value  
  
# # 提供函数来获取配置  
# def get_config(key):  
#     return config_dict[key]  
  
# # 也可以提供一个函数来获取整个配置字典  
# def get_all_config():  
#     return config_dict

config_dict = {
    'speed_pf': [0],
    'speed_pt': [[0,0]]
}

# 更新允许标记
allow_update = {key: True for key in config_dict}

# 提供函数来更新配置
def update_config(key, value):
    if allow_update[key]:
        # print('update可更新')
        # print(key, value)
        config_dict[key].append(value)
        # if key == 'speed_pt':
        #    print(config_dict[key])

        allow_update[key] = False  # 更新后禁止再次更新，直到值被获取
    # else:
    #     print('update不可更新')
    

# 提供函数来获取配置
def get_config(key):
    if allow_update[key]:
        # print('get-可更新状态')
        if key == 'speed_pf':
            value = 0
        elif key == 'speed_pt':
            value = [0, 0]
    else:
        # print('get-不可更新状态')
        # if config_dict[key]:
        value = config_dict[key].pop(-1)  # 获取并移除第一个值
        allow_update[key] = True
        # else:  # 当config_dict[key]为空时
        #     if key == 'speed_pf':
        #         value = 0
        #     elif key == 'speed_pt':
        #         value = [0, 0]
    # time.sleep(0.1)    
    return value    
        

    
# 控制频率
TARGET_FREQUENCY = 100
TARGET_PERIOD = 1.0 / TARGET_FREQUENCY

# 通讯相关
BAUDRATE = 115200
TIMEOUT = 0.1

#最大限位角
# ANGLE_LIMIT_MAX = 400
ANGLE_LIMIT_MAX = 270 #180°回头弯用
# ANGLE_LIMIT_MAX = 90 #图控最大值
# 该参数用于控制转向电机的最大角度

# 前进比 （该系数为1号电机和2好电机相对于0号电机的转动比例）
FORWARD_RATIO = 1.8   # 1 / 26
# 后退比 （该系数为1号电机和2好电机相对于0号电机的转动比例）（待修改，后退时摩擦轮应该较慢才不会出现弯曲）
RETRACT_RATIO = 0.9    

# 前进电流
FORWARD_CURRENT = 0.6
# 前进灵敏度(允许最大值)
FORWARD_COEFF = 110
# 转向灵敏度(允许最大值)
# TURN_COEFF = 200
# TURN_COEFF = 20 #图控用 sleep=0.1
# TURN_COEFF = 100 #手动用
# TURN_COEFF = 10 #图控用 sleep=0.02

# TURN_COEFF = 15 #图控用 sleep=0.02*****
TURN_COEFF = 20 #图控用 sleep=0.02

# TURN_COEFF_SLOW =10 #慢速但是精准，用于靠近目标点不出现跳动*****
TURN_COEFF_SLOW =10 #慢速但是精准，用于靠近目标点不出现跳动



# 转向灵敏度映射电机转角度系数0-1
R_Speed= 1
# 即实际转角速度等于转向灵敏度*手柄输入系数*转向灵敏度映射电机转角度系数

# 手柄检测启动的最小阈值
MIN_THRESHOLD = 0.03

# 直线电机相关
INSPIRE_MAX_SPEED = 10 # mm/s
INSPIRE_MAX_POSITION = 30 # mm

INSPIRE_MAX_STEP = 2000 # 步

INSPIRE_STEP_LENGTH = INSPIRE_MAX_POSITION / INSPIRE_MAX_STEP # mm/step


RECORD_PATHS = {
    'ControlData': f"Dataset\\SystemData\\ControlData_{TARGET_FREQUENCY}\\",
    'ImageData': f"Dataset\\SystemData\\ImageData_{TARGET_FREQUENCY}\\",
    'MicroImageData': f"Dataset\\SystemData\\MicroImageData_{TARGET_FREQUENCY}\\",
}

MICRO_CAMERA_ANGLE = -10.0


UP_MOTOR = 2
DOWN_MOTOR = 3
RIGHT_MOTOR = 4
LEFT_MOTOR = 1

UP_INIT_POSITION = 24.27
DOWN_INIT_POSITION = 27.029999999999998
RIGHT_INIT_POSITION = 23.685
LEFT_INIT_POSITION = 24.675

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>伸缩电缸相关 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Interval_MOTOR = 7

Interval_INIT_POSITION = 1000

RR = 2 #3

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Sofa相关 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
MESH_PATH = "C:\\Files\\Eric\\Projects\\CatheterProject\\Sofa\\mesh\\SIL945-50.vtk"
MODEL_PATH = "C:\\Files\\Eric\\Projects\\CatheterProject\\Sofa\\model\\SIL945-50.stl"

L = 50.0
R = 1.5
N = 50

display_size = (800, 600)

SOFA_FORWARD_COEFF = 20
SOFA_TURN_COEFF = 10

DETECTION = True
RAY_NUM = 32
RAY_LENGTH = 200



point1 = (501, 90)
point2 = (1100, 700)
roi_range = np.s_[point1[1]:point2[1], point1[0]:point2[0]]

lower_catheter = np.array([70, 10, 50])
upper_catheter = np.array([140, 110, 110])

lower_wall = np.array([0, 10, 0])
upper_wall = np.array([50, 50, 90])
