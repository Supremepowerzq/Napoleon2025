import serial  # 调用串口通信库
import time    # 调用时间库
import sys
import os
# 从上级目录导入工具包
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)
from ToolKits.ToolBox import clamp, change_config_value, get_time

# 导入配置模块
from config import *

import serial
import time

class InspireMotor:
    # 寄存器地址说明，对应微型伺服电缸-BLA系列用户手册8页，2.4寄存器说明
    regdict = {
    'deviceType'      : 0x01,   # 设备类型
    'SN'              : 0X02,   # 固件版本
    'ID'              : 0x06,   # ID
    'baudrate'        : 0x07,   # 波特率设置
    'clearErrors'     : 0x08,   # 清除故障命令
    'emergencyStop'   : 0x09,   # 急停命令
    'suspend'         : 0x0A,   # 暂停运动
    'restorePar'      : 0x0B,   # 还原参数命令
    'save'            : 0x0C,   # 保存命令
    'forceAct'        : 0x0E,   # 过温保护设置
    'warmUpSta'       : 0x0F,   # 回温启动设置
    'overCurproSet'   : 0X10,   # 过流保护设置
    'travelLimit'     : 0x13,   # 行程上线
    'controlModel'    : 0x20,   # 控制模式
    'targetValue'     : 0x22,   # 力控目标值（力控模式下有效）
    'targetSpeed'     : 0x23,   # 目标速度（速度模式下有效）
    'targetLocation'  : 0x24,   # 目标位置（速度、定位、伺服模式下有效）
    'softContact'     : 0x25,   # 软接触速度
    'actualLocation'  : 0x26,   # 实际位置
    'current'         : 0x27,   # 电流值，单位mA
    'actualSpeed'     : 0x28,   # 实际速度值
    'actualForce'     : 0x29,   # 实际受力值，单位：g
    'faultCodes'      : 0x2A,   # 故障码
    'actualTem'       : 0x2B,   # 实际温度值
}

    def __init__(self, port='COM5', baudrate=115200):
        self.ser = None
        if port is not None:
            self.open_serial(port, baudrate)

    # def mm_to_hex_bytes(self, mm_value):
    #     scaled_value = int(mm_value / 10 * 16384)
    #     hex_value = format(scaled_value, '04x')
    #     low_byte = int(hex_value[2:], 16)
    #     high_byte = int(hex_value[:2], 16)
    #     return low_byte, high_byte

# 函数说明：设置串口号和波特率并且打开串口；参数：port为串口号，baudrate为波特率
    def open_serial(self, port, baudrate):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baudrate
        ser.open()            # 打开串口
        return ser

# 函数说明：读电缸状态信息；参数：id为电缸ID号，speed为设置电缸速度， val为设置电缸位置数据
    def read_state(self,ser, id):
        bytes = [0x55, 0xAA]              # 帧头
        bytes.append(0x03)                # 数据长度
        bytes.append(id)                  # ID号
        bytes.append(0x30)                # CMD_RD_STATUS 读寄存器命令标志
        bytes.append(0x00)
        bytes.append(0x00)
        checksum = 0x00                   # 校验和初始化为0
        for i in range(2, len(bytes)):
            checksum += bytes[i]          # 对数据进行加和处理
        checksum &= 0xFF                  # 对校验和取低八位
        bytes.append(checksum)            # 低八位校验和
        ser.write(bytes)                  # 向串口写入数据
        time.sleep(0.01)                  # 延时10ms
        recv = ser.read_all()             # 从端口读字节数据
        if len(recv) == 0:                # 如果返回的数据长度为0，直接返回
            return []
        num = (recv[2] & 0xFF) - 3      # 寄存器数据所返回的数量
        val = []
        for i in range(num):
            val.append(recv[7 + i])
        print('读到的寄存器值依次为：', end='')
        for i in range(num):
            print(val[i], end=' ')
        print()

# 函数说明：写电缸寄存器操作函数；参数：id为电缸ID号，add为控制表索引，num为该帧数据的部分长度，val为所要写入寄存器的数据
    def write_register(self, ser, id, add, num, val):
        bytes = [0x55, 0xAA]            # 帧头
        bytes.append(num*2 + 3)         # 帧长度
        bytes.append(id)                # ID号
        bytes.append(0x32)              # CMD_WR 写寄存器命令标志
        bytes.append(add & 0xff)        # 寄存器地址低字节
        bytes.append((add >> 8) & 0xff) # 寄存器地址高字节
        bytes.append(add & 0xff)        # 控制表索引

        for i in range(num):
            bytes.append(val[i])
        checksum = 0x00                 # 校验和初始化为0
        for i in range(2, len(bytes)):
            checksum += bytes[i]        # 对数据进行加和处理
        checksum &= 0xFF                # 对校验和取低八位
        bytes.append(checksum)          # 低八位校验和
        ser.write(bytes)                # 向串口写入数据
        time.sleep(0.01)                # 延时10ms
        ser.read_all()             # 把返回帧读掉，不处理

# 函数说明：定位运动函数；参数：id为电缸ID号， speed为设置速度数据, val为设置电缸位置数据
# 首先进入位置控制模式，然后设置速度和目标位置
    def set_speed_and_position(self, ser, id, speed, position):
        bytes = [0x55, 0xAA]              # 帧头
        bytes.append(0x05)                # 数据长度
        bytes.append(id)                  # ID号
        bytes.append(0x31)                # CMD_WR_REGISTER 写寄存器命令标志
        bytes.append(0x20)                # 设置工作模式寄存器地址低字节
        bytes.append(0x00)                # 设置工作模式寄存器地址低字节
        bytes.append(0x00)                # 位置模式
        bytes.append(0x00)                # 位置模式
        checksum = 0x00                   # 校验和初始化为0
        for i in range(2, len(bytes)):
            checksum += bytes[i]          # 对数据进行加和处理
        checksum &= 0xFF                  # 对校验和取低八位
        bytes.append(checksum)            # 低八位校验和
        ser.write(bytes)                  # 向串口写入数据
        time.sleep(0.01)                  # 延时10ms

        bytes = [0x55, 0xAA]              # 帧头
        bytes.append(0x07)                # 数据长度
        bytes.append(id)                  # ID号
        bytes.append(0x31)                # CMD_WR_REGISTER 写寄存器命令标志
        bytes.append(0x23)                # 设置速度寄存器地址
        bytes.append(0x00)                # 设置速度寄存器地址
        bytes.append(speed & 0xff)        # 设置速度
        bytes.append((speed >> 8) & 0xff) # 设置速度
        bytes.append(position & 0xff)          # 目标位置
        bytes.append((position >> 8) & 0xff)   # 目标位置
        checksum = 0x00                   # 校验和初始化为0
        for i in range(2, len(bytes)):
            checksum += bytes[i]          # 对数据进行加和处理
        checksum &= 0xFF                  # 对校验和取低八位
        bytes.append(checksum)            # 低八位校验和
        ser.write(bytes)             # 向串口写入数据
        time.sleep(0.01)                  # 延时10ms
        ser.read_all()               # 把返回帧读掉，不处理

    # def set_servo_mode(self, id, position):
    #     if not self.ser.is_open:
    #         raise IOError("Serial port is not open.")
    #     # 设置工作模式为伺服模式
    #     self.write_register(id, self.REG_DICT['controlModel'], [0x00, 0x01])
    #     # 设置位置
    #     low, high = self.mm_to_hex_bytes(position)
    #     self.write_register(id, self.REG_DICT['targetLocation'], [low, high])

    def force(self,ser, id, force):
        bytes = [0x55, 0xAA]              # 帧头
        bytes.append(0x05)                # 数据长度
        bytes.append(id)                  # ID号
        bytes.append(0x31)                # CMD_WR_REGISTER 写寄存器命令标志
        bytes.append(0x20)                # 控制模式寄存器地址低字节
        bytes.append(0x00)                # 控制模式寄存器地址高字节
        bytes.append(0x04)                # 设置控制模式为力控模式
        bytes.append(0x00)                # 设置控制模式为力控模式
        checksum = 0x00                   # 校验和初始化为0
        for i in range(2, len(bytes)):
            checksum += bytes[i]          # 对数据进行加和处理
        checksum &= 0xFF                  # 对校验和取低八位
        bytes.append(checksum)            # 低八位校验和
        ser.write(bytes)                  # 向串口写入数据
        time.sleep(0.01)                  # 延时10ms

        bytes = [0x55, 0xAA]              # 帧头
        bytes.append(0x05)                # 数据长度
        bytes.append(id)                  # ID号
        bytes.append(0x31)                # CMD_WR_REGISTER 写寄存器命令标志
        bytes.append(0x22)                # 控制模式寄存器地址低字节
        bytes.append(0x00)                # 控制模式寄存器地址高字节
        bytes.append(force & 0xff)        # 力控目标值
        bytes.append((force >> 8) & 0xff) # 力控目标值
        checksum = 0x00                   # 校验和初始化为0
        for i in range(2, len(bytes)):
            checksum += bytes[i]          # 对数据进行加和处理
        checksum &= 0xFF                  # 对校验和取低八位
        bytes.append(checksum)            # 低八位校验和
        ser.write(bytes)                  # 向串口写入数据

        ser.read_all()                    # 把返回帧读掉，不处理

        # if not self.ser.is_open:

        #     raise IOError("Serial port is not open.")

        # # 设置控制模式为力控模式

        # self.write_register(id, self.regdict['controlModel'], [0x04, 0x00])

        # # 设置力控目标值

        # self.write_register(id, self.regdict['targetValue'], [force & 0xff, (force >> 8) & 0xff])


    # def softcon(self, id, force, speed, pos, val):

    #     if not self.ser.is_open:

    #         raise IOError("Serial port is not open.")

    #     # 设置控制模式为软接触模式

    #     self.write_register(id, self.regdict['controlModel'], [0x05, 0x00])

    #     # 设置力控、速度、预接触位置和软接触速度

    #     self.write_register(id, self.regdict['targetValue'], [force & 0xff, (force >> 8) & 0xff])

    #     self.write_register(id, self.regdict['targetSpeed'], [speed & 0xff, (speed >> 8) & 0xff])

    #     self.write_register(id, self.regdict['targetLocation'], [pos & 0xff, (pos >> 8) & 0xff])

    #     self.write_register(id, self.regdict['softContact'], [val & 0xff, (val >> 8) & 0xff])


    def read_register(self, ser, id, add, num):
        mute = False
        bytes = [0x55, 0xAA]            # 帧头
        bytes.append(0x04)              # 帧长度
        bytes.append(id)                # id
        bytes.append(0x32)              # CMD_RD 读寄存器命令标志
        bytes.append(add & 0xff)        # 寄存器地址低字节
        bytes.append((add >> 8) & 0xff) # 寄存器地址高字节
        bytes.append(num)
        checksum = 0x00                 # 校验和赋值为0
        for i in range(2, len(bytes)):
            checksum += bytes[i]        # 对数据进行加和处理
        checksum &= 0xFF                # 对校验和取低八位
        bytes.append(checksum)          # 低八位校验和
        ser.write(bytes)                # 向串口写入数据
        time.sleep(0.01)                # 延时10ms
        recv = ser.read_all()           # 从端口读字节数据
        if len(recv) == 0:              # 如果返回的数据长度为0，直接返回
            return []
        num = (recv[2] & 0xFF) - 3      # 寄存器数据所返回的数量
        val = []
        for i in range(num):
            val.append(recv[6 + i])
        if not mute:
            print('读到的寄存器值依次为：', end='')
            for i in range(num):
                print(val[i], end=' ')
            print()
        return val

    def scale_value_speed(self, x):
        m = 163.84
        y = m * x
        # 转换为整数
        return int(y)  
    def scale_value_position(self, x):
        m = 1638.4
        y = m * x
        # 转换为整数
        return int(y)
    def scale_value_force(self, x):
        m = 163.84*0.5
        y = m * x
        # 转换为整数
        return int(y)    



if __name__ == '__main__':
    
    IM=InspireMotor()
    print('打开串口！')                               # 打印提示字符“打开串口”
    ser = IM.open_serial('COM5', 115200) # 改成自己的串口号和波特率，波特率默认115200
    time.sleep(1)
    print('设置电缸速度以及位置信息')
    scale_value_speed = 100 # 速度0-100
    scale_value_position = 4 # 位置0-10.00
    scale_value_force = 10 # 压力0-20.00

    scale_value_speed = IM.scale_value_speed(scale_value_speed)
    scale_value_position = IM.scale_value_position(scale_value_position) 
    scale_value_force = IM.scale_value_force(scale_value_force)

    # IM.set_speed_and_position(ser, 1, scale_value_speed, scale_value_position)  # ID号改为对应电缸的ID号 0-16384对应0-100%的速度或者位置的标幺值
    # time.sleep(1)

    # IM.set_speed_and_position(ser, 7, 16384,0) #全速归零

    print('设置力控目标值')
    IM.force(ser,1,scale_value_force)
    time.sleep(1)

    # print('快速定位+软接触模式下，设置力控大小，速度，预接触位置，软接触速度')
    # softcon(ser,4, 1000,20000,10000,163)
    # time.sleep(1)

    print('读取电缸状态信息')
    IM.read_state(ser, 1)
    # time.sleep(10) # 由于力校准时间较长，请不要漏过这个sleep并尝试重新与手通讯，可能导致插件崩溃