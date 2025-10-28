import serial
from serial.tools import list_ports
from config import *
from Interface.RmdInterface import RmdMotor
from Interface.InspireInterface import InspireMotor
from ToolKits.ToolBox import get_time

from Interface.RmdInterfaceV2 import RmdMotor as RmdMotorV2

def find_available_com_port(BAUDRATE, TIMEOUT):
    print(f"{get_time()}-正在查找可用串口...")
    available_ports = list_ports.comports()

    rmd_port = None
    inspire_port = None

    for port, desc, hwid in sorted(available_ports):
        try:
            if port != rmd_port and port != inspire_port:
                ser = serial.Serial(port, baudrate=BAUDRATE, timeout=TIMEOUT)
                rmdmotor = RmdMotor(0, ser)
                if rmdmotor.check():
                    print(f"{get_time()}-找到Rmd电机串口: {port}")
                    rmd_port = port
                ser.close()
        except:
            pass

        try:
            if port != inspire_port and port != rmd_port:
                ser = serial.Serial(port, baudrate=BAUDRATE, timeout=TIMEOUT)
                inspiremotor = InspireMotor(1, ser)
                if inspiremotor.check():
                    print(f"{get_time()}-找到Inspire电机串口: {port}")
                    inspire_port = port
                ser.close()
        except:
            pass
    
    if rmd_port is not None or inspire_port is not None:
        return [rmd_port, inspire_port]
    return None

def get_serial():

    ports = find_available_com_port(BAUDRATE, TIMEOUT)
    if ports is None:
        raise Exception("未找到可用串口")
    if ports[0] is not None:
        rmd_serial = serial.Serial(ports[0], baudrate=BAUDRATE, timeout=TIMEOUT)
    else:
        rmd_serial = None
    if ports[1] is not None:
        inspire_serial = serial.Serial(ports[1], baudrate=BAUDRATE, timeout=TIMEOUT)
    else:
        inspire_serial = None

    
    return [rmd_serial, inspire_serial]


def find_rmd_motor_port(id: int) -> str | None:
    """
    查找RMD电机的串口。

    遍历所有可用的串口，尝试通过串口与RMD电机通信，以确定该串口是否为RMD电机的串口。
    
    返回:
        str: RMD电机串口的名称，如果未找到则为None。
    """
    print(f"{get_time()}-正在查找RMD电机串口...")
    available_ports = list_ports.comports()

    for port_info in sorted(available_ports, key=lambda p: p.device, reverse=True):
        print(f"{get_time()}-正在尝试打开串口{port_info.device}...")
        try:
            with serial.Serial(port_info.device, baudrate=BAUDRATE, timeout=TIMEOUT) as serial_port:
                rmd_motor = RmdMotorV2(id, serial_port)
                # 此处应有向电机发送命令并等待响应的代码，以验证是否正确连接到RMD电机
                print(f"{get_time()}-找到RMD电机串口: {port_info.device}")
                return port_info.device
        except Exception as e:
            pass
            # print(f"{get_time()}-尝试打开串口{port_info.device}失败：{e}")

    print(f"{get_time()}-未找到RMD电机串口")
    return None


