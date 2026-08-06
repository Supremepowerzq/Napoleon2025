import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import time
import serial
from typing import List
# import tqdm  # 已移除，避免依赖问题
# import pyinstrument  # 已移除，避免依赖问题
import threading
import queue
from ToolKits.Timer import maintain_target_frequency, busy_maintain_target_frequency
# from matplotlib import pyplot as plt  # 已移除，避免版本冲突
import numpy as np
from config import *
from ToolKits.ToolBox import get_time

debug = False

# 前进灵敏度(允许最大值)
FORWARD_COEFF = 200
# 转向灵敏度(允许最大值)
TURN_COEFF = 100

TC = TURN_COEFF

class RmdMotor:
    def __init__(self, motor_id: int, serial_port: serial.Serial) -> None:
        """
        初始化电机对象。

        参数:
        - motor_id (int): 电机的ID号。
        - serial_port (serial.Serial): 用于与电机通信的串口对象。
        """

        self.id = f"{motor_id:02X}"
        self.serial = serial_port

        self.previous_position = 0  # 新增：用于跟踪上一次的位置
        self.overflows = 0  # 新增：用于跟踪位置溢出的次数

        self.update_state()

        self.previous_command = {
            'type': None,
            'value': None
        }

    def __update_previous_command(self, command_type: str, command_value: float) -> None:
        """
        更新上一个指令的记录。

        参数:
        - command_type (str): 指令类型。
        - command_value (float): 指令值。
        """
        self.previous_command['type'] = command_type
        self.previous_command['value'] = command_value

    def __check_previous_command(self, command_type: str, command_value: float) -> bool:
        """
        检查上一个指令是否与当前指令相同。

        参数:
        - command_type (str): 指令类型。
        - command_value (float): 指令值。

        返回:
        - bool: 如果上一个指令与当前指令相同，则返回True。
        """
        return self.previous_command['type'] == command_type and self.previous_command['value'] == command_value

    def set_current(self, current: float) -> None:
        """
        电流设置指令。

        参数:
        - current (float): 期望的电流，单位为安培。
        """
        # 检查上一个指令是否与当前指令相同
        if self.__check_previous_command('current', current):
            return
        # 更新上一个指令的记录
        self.__update_previous_command('current', current)

        current_hex = self.__encode_signed_hex(int(current*100), 4)
        send_buffer = ['3E', self.id, '08', 'A1', '00', '00', '00'] + list(reversed(current_hex)) + ['00', '00']
        receive_buffer = self.write(send_buffer)
        self.__decode_received_buffer(receive_buffer, send_buffer[3])

    def set_speed(self, speed: float) -> None:
        """
        速度设置指令。

        参数:
        - speed (float): 期望的速度，单位为度/秒。
        """
        speed = min(TC*R,speed)

        # # 检查上一个指令是否与当前指令相同
        # if self.__check_previous_command('speed', speed):   #该步的目的是为了减少通讯的次数，但是这样会影响电机的实时反馈
        #     return
        # 更新上一个指令的记录
        self.__update_previous_command('speed', speed)

        speed_hex = self.__encode_signed_hex(int(speed*100), 8)
        send_buffer = ['3E', self.id, '08', 'A2', '00', '00', '00'] + list(reversed(speed_hex))
        receive_buffer = self.write(send_buffer)
        self.__decode_received_buffer(receive_buffer, send_buffer[3])

    def set_position(self, position: float, max_speed: float = TC ) -> None:
        """
        位置设置指令。

        参数:
        - position (float): 期望的位置，单位为度。
        - max_speed (float): 期望的最大速度，单位为度/秒。
        """
        # # 检查上一个指令是否与当前指令相同
        # if self.__check_previous_command('position', position):
        #     return
        # 更新上一个指令的记录
        self.__update_previous_command('position', position)

        max_speed_hex = self.__encode_signed_hex(int(max_speed), 4)
        position_hex = self.__encode_signed_hex(int(position*100), 8)
        send_buffer = ['3E', self.id, '08', 'A4', '00'] + list(reversed(max_speed_hex)) + list(reversed(position_hex))
        receive_buffer = self.write(send_buffer)
        self.__decode_received_buffer(receive_buffer, send_buffer[3])

    def custom_set_position(self, position: float, max_speed: float = TC ) -> None:
        """
        自定义位置设置指令。应对了位置溢出的情况。
        必须要在循环中调用。

        参数:
        - position (float): 期望的位置，单位为度。
        - max_speed (float): 期望的最大速度，单位为度/秒。
        """
        if self.overflows == 0:
            # print(f"{get_time()}-电机{self.id}的位置溢出次数为0，使用普通位置设置指令。")
            self.set_position(position, max_speed)
        else:
            target_speed = max_speed if self.overflows < 0 else -max_speed
            self.set_speed(target_speed)

    def shutdown(self) -> None:
        """
        关闭电机输出，同时清除电机运行状态， 不在任何闭环模式下。
        """
        send_buffer = ['3E', self.id, '08', '80', '00', '00', '00', '00', '00', '00', '00']
        self.write(send_buffer)
        if debug: print(f"Motor {self.id} has been shut down.")

    def stop(self) -> None:
        """
        停止电机，将电机速度停下来，并使电机保持不动，不会因为外力移动。
        """
        send_buffer = ['3E', self.id, '08', '81', '00', '00', '00', '00', '00', '00', '00']
        self.write(send_buffer)
        if debug: print(f"Motor {self.id} has been stopped.")

    def set_current_position_as_zero_point(self) -> None:
        """
        将编码器当前多圈位置写入ROM作为电机零点。

        此命令会将当前编码器的多圈位置保存到电机的ROM中，
        并将其设置为电机的零点位置。这样，即使在电机断电后，
        此零点位置也会被保留。
        """
        send_buffer = ['3E', self.id, '08', '64', '00', '00', '00', '00', '00', '00', '00']
        self.write(send_buffer)
        print(f"{get_time()}-电机{self.id}的当前位置已被设置为零点。", end='')

        self.restart()
        time.sleep(0.5) # 等待电机重启
        self.update_state()
        print(f"更新后的位置为{self.position}°。")

    def restart(self) -> None:
        """
        电机重启指令，此时电机不会返回响应。
        """
        send_buffer = ['3E', self.id, '08', '76', '00', '00', '00', '00', '00', '00', '00']
        self.write(send_buffer, buffer_size=0)
        if debug: print(f"Motor {self.id} has been restarted.")

    def update_state(self) -> None:
        """
        更新电机状态，包括电机的温度、电流、速度和位置。
        """
        send_buffer = ['3E', self.id, '08', '9C', '00', '00', '00', '00', '00', '00', '00']
        receive_buffer = self.write(send_buffer)
        self.__decode_received_buffer(receive_buffer, send_buffer[3])

    def engage_release(self) -> None:
        """
        目前无效。

        该命令用于开启系统抱闸。系统会松开抱闸，电机会处于可运动状态不受抱闸制动器限制。
        """
        send_buffer = ['3E', self.id, '08', '77', '00', '00', '00', '00', '00', '00', '00']
        self.write(send_buffer)

    def engage_break(self) -> None:
        """
        目前无效。

        该命令用于关闭系统抱闸。抱闸会锁住电机，此时电机无法再运行。系统断电后抱闸制动器也是处于这个状态。
        """
        send_buffer = ['3E', self.id, '08', '78', '00', '00', '00', '00', '00', '00', '00']
        self.write(send_buffer)

    def angle_return(self, max_speed: float = 100) -> bool:
        """
        电机角度返回指令。

        该命令用于使电机回到零点位置。
        """
        self.custom_set_position(0, max_speed)
        return abs(self.position) < 5
    

    def write(self, send_buffer: List[str], buffer_size: int = 13, retries: int = 3) -> List[str]:
        """
        向电机发送指令，并返回电机的响应。

        参数:
        - send_buffer (List[str]): 待发送的指令，以十六进制字符串列表的形式提供。
        - buffer_size (int): 期望接收的响应字节数。
        - retries (int): 剩余重试次数，默认为3。

        返回:
        List[str]: 电机的响应，以十六进制字符串列表的形式返回。
        """
        # Flush and send
        self.serial.flushInput()
        self.serial.flushOutput()
        # Debug: print outgoing buffer
        # (已移除调试打印)
        self.serial.write(self.__calculate_crc(send_buffer))
        self.serial.flush()
        receive_buffer = []
        time_start = time.perf_counter()
        # Increase per-call timeout to allow motor processing (was 0.1s)
        per_call_timeout = 0.3
        while len(receive_buffer) < buffer_size:
            if time.perf_counter() - time_start > per_call_timeout:
                if retries > 0:
                    print(f"{get_time()}-motor {self.id} response timeout, retries left: {retries}", flush=True)
                    return self.write(send_buffer, buffer_size, retries-1)  # recursive retry
                else:
                    raise TimeoutError("Motor response timeout, no retries left.")
            response = self.serial.read().hex().upper()
            if response:
                receive_buffer.append(response)
        # Debug: print received buffer
        # (已移除调试打印)
        return receive_buffer        
    
    def __decode_received_buffer(self, receive_buffer: List[str], instruction_type: str) -> None:
        """
        解析电机的响应，并更新电机的状态。

        参数:
        - receive_buffer (List[str]): 电机的响应，以十六进制字符串列表的形式提供。
        - instruction_type (str): 电机的指令类型。

        返回:
        None
        """
        if receive_buffer is None:
            return
        if receive_buffer[3] != instruction_type:
            raise ValueError(f"Received instruction type {receive_buffer[3]} does not match the expected instruction type {instruction_type}.")

        # 解析温度
        temperature_hex = receive_buffer[4]
        self.temperature = self.__decode_signed_hex(temperature_hex)

        # 解析电流
        current_hex = receive_buffer[6] + receive_buffer[5]
        self.current = self.__decode_signed_hex(current_hex) * 0.01

        # 解析速度
        speed_hex = receive_buffer[8] + receive_buffer[7]
        self.speed = self.__decode_signed_hex(speed_hex)

        # 解析位置
        position_hex = receive_buffer[10] + receive_buffer[9]
        self.position = self.__decode_signed_hex(position_hex)

        # if self.id == '03' or self.id == '04':
        #     self.position = self.position * (-1)

        # 检测位置超出并更新计数器
        position_change = self.position - self.previous_position
        if position_change > 32768:
            self.overflows -= 1
        elif position_change < -32768:
            self.overflows += 1
        self.previous_position = self.position
    
    def __decode_signed_hex(self, value: str) -> int:
        """
        解码表示有符号整数的十六进制字符串。

        该函数将十六进制字符串转换为其对应的整数值。如果十六进制字符串代表的是一个负数（使用二进制补码表示），
        该函数将返回对应的负整数值。

        参数:
        - value (str): 待解码的十六进制字符串。

        返回值:
        int: 解码后的有符号整数。
        """
        # 计算位数：每个十六进制数字代表4个二进制位。
        num_bits = len(value) * 4
        
        # 计算给定位数的最大正值。
        max_positive_value = 2 ** (num_bits - 1) - 1
        
        # 将十六进制字符串转换为整数。
        int_value = int(value, 16)
        
        # 如果int_value表示的是负数，进行调整。
        if int_value > max_positive_value:
            int_value -= 2 ** num_bits

        return int_value
    
    def __encode_signed_hex(self, value: int, hex_length: int = 4) -> List[str]:
        """
        将有符号整数编码为十六进制字符串。

        该函数将有符号整数转换为其对应的十六进制字符串。如果整数是负数，该函数将使用二进制补码表示。

        参数:
        - value (int): 待编码的有符号整数。
        - hex_length (int): 期望的十六进制字符串长度。

        返回值:
        List[str]: 编码后的十六进制字符串。
        """
        # 按字段自身位宽生成二进制补码。旧实现无论 hex_length 是 4 还是 8
        # 都先转为 int16，导致 A4 的 32 位多圈位置在 position < -327.68°
        # 时溢出为正数（例如 -36000 被错误编码为 +29536）。
        if hex_length <= 0 or hex_length % 2 != 0:
            raise ValueError(f"hex_length 必须是正偶数，当前为 {hex_length}")

        bit_width = 4 * hex_length
        min_value = -(1 << (bit_width - 1))
        max_value = (1 << (bit_width - 1)) - 1
        if not min_value <= value <= max_value:
            raise OverflowError(
                f"{value} 超出 {bit_width} 位有符号整数范围 "
                f"[{min_value}, {max_value}]"
            )

        encoded_value = value & ((1 << bit_width) - 1)
        hex_value = f"{encoded_value:0{hex_length}X}"
        return [hex_value[i:i+2] for i in range(0, len(hex_value), 2)]

    def __modbusCrc(self, msg: str) -> int:
        """
        计算给定消息的Modbus CRC校验码。

        参数:
        - msg (str): 需要计算CRC的消息，以字节串形式提供。

        返回:
        - int: 计算出的CRC校验码。
        """
        crc = 0xFFFF
        for n in range(len(msg)):
            crc ^= msg[n]
            for i in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1 
        return crc 
    
    def __calculate_crc(self, send_buffer: List[str]) -> List[int]:
        """
        计算给定消息的Modbus CRC校验码。

        参数:
        - send_buffer (List[str]): 需要计算CRC的消息，以十六进制字符串列表的形式提供。

        返回:
        - List[int]: 包含原始数据和CRC校验码的整数列表。
        """

        # 将数据列表转换为字节串
        data_bytes = bytes.fromhex(''.join(send_buffer))

        # 计算字节串的CRC校验码
        crc = self.__modbusCrc(data_bytes)
        
        # 将CRC校验码转换为两个字节的小端序形式，并分解为两个整数
        crc_low, crc_high = crc.to_bytes(2, byteorder='little')

        # 将原始数据和CRC校验码转换为整数列表并返回
        return [int(hex_value, 16) for hex_value in send_buffer] + [crc_low, crc_high]
    
    def __repr__(self) -> str:
        return f"RmdMotor(id={self.id}, t={self.temperature}°C, I={self.current}A, v={self.speed}dps, p={self.position}°)"
    
    

def calculate_position_difference(current_position, previous_position):
    """
    计算两个位置之间的差异，考虑到值的环绕。

    参数:
    - current_position: 当前位置
    - previous_position: 上一个位置
    返回:
    - 位置差异值
    """
    max_value = 65535
    half_max = max_value // 2

    diff = current_position - previous_position

    if diff > half_max:
        # 值减小了，但因为环绕看起来像是增大了
        diff -= max_value + 1  # +1 因为范围是从0到65535
    elif diff < -half_max:
        # 值增大了，但因为环绕看起来像是减小了
        diff += max_value + 1

    return diff


if __name__ == "__main__":
    ser = serial.Serial('COM4', baudrate=115200, timeout=None)
    speed = 0
    motor_1 = RmdMotor(1, ser)
    motor_2 = RmdMotor(2, ser)
    motor_1.set_speed(speed)
    motor_2.set_speed(speed)

    motor_1.stop()
    motor_2.stop()

    print(motor_1)
    print(motor_2)

    # for i in range(120):
    #     motor.update_state()
    #     print(motor.position, motor.overflows)
    #     time.sleep(0.1)

    # while abs(motor.position) > 10:
    #     motor.custom_set_position(0, 3000)
    #     print(motor.position, motor.overflows)
    #     time.sleep(0.1)

    # motor.stop()

    # motor.set_speed(-3000)

    # for i in range(300):
    #     motor.update_state()
    #     print(motor.position, motor.overflows)
    #     time.sleep(0.1)

    # motor.stop()

    # motor.set_position(0)

    # while abs(motor.position) > 10:
    #     motor.update_state()
    #     print(motor.position, motor.overflows)
    #     time.sleep(0.1)

    # motor.stop()
    # motor.set_current_position_as_zero_point()
    # print(motor.position)


    
# if __name__ == "__main__":

#     ser = serial.Serial('COM10', baudrate=115200, timeout=None)
#     motor = RmdMotor(1, ser)
#     motor.enable()

#     speeds = []  # 用于收集速度数据
#     positions = []  # 用于收集位置数据
#     times = []  # 用于收集时间数据

#     time_start = time.perf_counter()
#     while motor.test_num > 0:
#         _time_start = time.perf_counter()

#         motor.set_speed(3000)

#         speeds.append(motor.speed)
#         positions.append(motor.position)
#         times.append(time.perf_counter() - time_start)

#         maintain_target_frequency(30, _time_start)
#     time_end = time.perf_counter()


#     motor.disable()
    
#     elpased_time = 1 / 30
#     position_differences = [calculate_position_difference(positions[i], positions[i-1]) for i in range(1, len(positions))]
#     cumulative_position_changes = np.cumsum(position_differences)
#     cumulative_position_changes = np.insert(cumulative_position_changes, 0, 0)


    
#     speeds2 = np.diff(cumulative_position_changes) / elpased_time / 2.0

#     # 创建一个图像和3个子图
#     fig, axs = plt.subplots(3, 1, figsize=(10, 12))  # 3行1列的子图

#     # 绘制速度图
#     axs[0].plot(times, speeds, label='Speed')
#     axs[0].set_title('Motor Speed Over Time')
#     axs[0].set_xlabel('Time (iterations)')
#     axs[0].set_ylabel('Speed')
#     axs[0].legend()

#     # 绘制位置图
#     axs[1].plot(times, cumulative_position_changes, label='Position')
#     axs[1].set_title('Motor Position Over Time')
#     axs[1].set_xlabel('Time (iterations)')
#     axs[1].set_ylabel('Position')
#     axs[1].legend()

#     # 绘制速度变化（位置的导数）图
#     axs[2].plot(times[1:len(speeds2)+1], speeds2, label='Derived Speed')
#     axs[2].set_title('Derived Speed from Position Over Time')
#     axs[2].set_xlabel('Time (iterations)')
#     axs[2].set_ylabel('Derived Speed')
#     axs[2].legend()

#     plt.tight_layout()  # 自动调整子图间距
#     plt.show()




# class RmdMotor:
#     def __init__(self, motor_id: int, serial_port: serial.Serial):

#         self.id = f"{motor_id:02X}"
#         self.serial = serial_port

#         self.update_state()

#         self.target_Frequence = TARGET_FREQUENCY

#         self.command_queue = queue.Queue()  # 创建一个用于存储指令的队列

#         # 通信线程控制
#         self.comm_thread = threading.Thread(target=self.comm_loop, daemon=True)
#         self.running = True

#         self.test_num = 100

#     def comm_loop(self):
#         """
#         串口通信循环，用于在独立线程中处理与电机的通信。
#         """
#         while self.running:
#             if not self.command_queue.empty():
#                 time_start = time.perf_counter()

#                 command = self.command_queue.get()  # 从队列中获取指令
#                 if command['action'] == 'set_speed':
#                     speed = command['value']
#                     self.actual_set_speed(speed)  # 实际发送速度设置指令给电机的方法

#                 busy_maintain_target_frequency(self.target_Frequence, time_start)

#                 if abs(frequency_diff:= 1 / (time.perf_counter() - time_start) - self.target_Frequence) > 5:
#                     print(f"frequency_diff: {frequency_diff} Hz")


#     def enable(self) -> None:
#         """
#         启用电机。
#         """
#         self.comm_thread.start()

#     def disable(self) -> None:
#         """
#         禁用电机。
#         """
#         self.running = False
#         self.comm_thread.join()


#     def set_speed(self, speed: float):
#         """
#         将速度设置指令添加到队列。
#         """
#         self.command_queue.put({'action': 'set_speed', 'value': speed})

#     def actual_set_speed(self, speed: float):
#         """
#         实际发送速度设置指令给电机的方法。
#         """
#         speed_hex = self.__encode_signed_hex(int(speed*100), 8)
#         send_buffer = ['3E', self.id, '08', 'A2', '00', '00', '00'] + list(reversed(speed_hex))
#         print(send_buffer)
#         receive_buffer = self.write(send_buffer)
#         self.__decode_received_buffer(receive_buffer, send_buffer[3])

#         self.test_num -= 1



#     def update_state(self) -> None:
#         """
#         更新电机状态，包括电机的温度、电流、速度和位置。
#         """
#         send_buffer = ['3E', self.id, '08', '9C', '00', '00', '00', '00', '00', '00', '00']
#         receive_buffer = self.write(send_buffer)
#         self.__decode_received_buffer(receive_buffer, send_buffer[3])


#     def write(self, send_buffer: List[str], buffer_size: int = 13) -> List[str]:
#         """
#         向电机发送指令，并返回电机的响应。
        
#         参数:
#         - send_buffer (List[str]): 待发送的指令，以十六进制字符串列表的形式提供。
#         - buffer_size (int): 期望接收的响应字节数。

#         返回:
#         List[str]: 电机的响应，以十六进制字符串列表的形式返回。
#         """
#         self.serial.write(self.__calculate_crc(send_buffer))

#         receive_buffer = []
#         while len(receive_buffer) < buffer_size:
#             response = self.serial.read().hex().upper()
#             if response:
#                 receive_buffer.append(response)

#         return receive_buffer
    
#     def __decode_received_buffer(self, receive_buffer: List[str], instruction_type: str) -> None:
#         """
#         解析电机的响应，并更新电机的状态。

#         参数:
#         - receive_buffer (List[str]): 电机的响应，以十六进制字符串列表的形式提供。
#         - instruction_type (str): 电机的指令类型。

#         返回:
#         None
#         """
#         if receive_buffer[3] != instruction_type:
#             raise ValueError(f"Received instruction type {receive_buffer[3]} does not match the expected instruction type {instruction_type}.")

#         # 解析温度
#         temperature_hex = receive_buffer[4]
#         self.temperature = self.__decode_signed_hex(temperature_hex)

#         # 解析电流
#         current_hex = receive_buffer[6] + receive_buffer[5]
#         self.current = self.__decode_signed_hex(current_hex) * 0.01

#         # 解析速度
#         speed_hex = receive_buffer[8] + receive_buffer[7]
#         self.speed = self.__decode_signed_hex(speed_hex)

#         # 解析位置
#         position_hex = receive_buffer[10] + receive_buffer[9]
#         self.position = self.__decode_signed_hex(position_hex)
    
#     def __decode_signed_hex(self, value: str) -> int:
#         """
#         解码表示有符号整数的十六进制字符串。

#         该函数将十六进制字符串转换为其对应的整数值。如果十六进制字符串代表的是一个负数（使用二进制补码表示），
#         该函数将返回对应的负整数值。

#         参数:
#         - value (str): 待解码的十六进制字符串。

#         返回值:
#         int: 解码后的有符号整数。
#         """
#         # 计算位数：每个十六进制数字代表4个二进制位。
#         num_bits = len(value) * 4
        
#         # 计算给定位数的最大正值。
#         max_positive_value = 2 ** (num_bits - 1) - 1
        
#         # 将十六进制字符串转换为整数。
#         int_value = int(value, 16)
        
#         # 如果int_value表示的是负数，进行调整。
#         if int_value > max_positive_value:
#             int_value -= 2 ** num_bits

#         return int_value
    
#     def __encode_signed_hex(self, value: int, hex_length: int = 4) -> List[str]:
#         """
#         将有符号整数编码为十六进制字符串。

#         该函数将有符号整数转换为其对应的十六进制字符串。如果整数是负数，该函数将使用二进制补码表示。

#         参数:
#         - value (int): 待编码的有符号整数。
#         - hex_length (int): 期望的十六进制字符串长度。

#         返回值:
#         List[str]: 编码后的十六进制字符串。
#         """
#         # 计算整数的二进制补码。
#         if value < 0:
#             value = (1 << 4 * hex_length) + value

#         hex_value = f"{value:0{hex_length}X}"
#         return [hex_value[i:i+2] for i in range(0, len(hex_value), 2)]



#     def __modbusCrc(self, msg: str) -> int:
#         """
#         计算给定消息的Modbus CRC校验码。

#         参数:
#             msg (str): 需要计算CRC的消息，以字节串形式提供。

#         返回:
#             int: 计算出的CRC校验码。
#         """
#         crc = 0xFFFF
#         for n in range(len(msg)):
#             crc ^= msg[n]
#             for i in range(8):
#                 if crc & 1:
#                     crc >>= 1
#                     crc ^= 0xA001
#                 else:
#                     crc >>= 1 
#         return crc 
    
#     def __calculate_crc(self, send_buffer: List[str]) -> List[int]:
#         # 将数据列表转换为字节串
#         data_bytes = bytes.fromhex(''.join(send_buffer))

#         # 计算字节串的CRC校验码
#         crc = self.__modbusCrc(data_bytes)
        
#         # 将CRC校验码转换为两个字节的小端序形式，并分解为两个整数
#         crc_low, crc_high = crc.to_bytes(2, byteorder='little')

#         # 将原始数据和CRC校验码转换为整数列表并返回
#         return [int(hex_value, 16) for hex_value in send_buffer] + [crc_low, crc_high]
    
#     def __repr__(self) -> str:
#         return f"RmdMotor(id={self.id}, t={self.temperature}°C, I={self.current}A, v={self.speed}dps, p={self.position}°)"
