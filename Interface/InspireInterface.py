import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

from ToolKits.ToolBox import clamp, change_config_value, get_time
import time
import serial
from serial.tools import list_ports
import importlib

from config import *



class InspireMotor:
    def __init__(self, ID, ser):

        ID = hex(ID).upper().replace('0X', '')
        while len(ID) % 2 != 0:
            ID = '0' + ID

        self.ID = ID
        self.ser = ser

        self.control_mode = None

        self.tempreture = None
        self.current = None
        self.position = None
        self.force = None
        self.init_force = None

        if self.ID == '0'+str(UP_MOTOR):
            self.max_position = UP_INIT_POSITION
        elif self.ID == '0'+str(RIGHT_MOTOR):
            self.max_position = RIGHT_INIT_POSITION
        elif self.ID == '0'+str(DOWN_MOTOR):
            self.max_position = DOWN_INIT_POSITION
        elif self.ID == '0'+str(LEFT_MOTOR):
            self.max_position = LEFT_INIT_POSITION



    def _write_serial_data(self, send_buffer):

        buffer_size = 20
        send_buffer = calculate_crc(send_buffer)

        # print('send_buffer: ', end='')
        # for i in range(len(send_buffer)):
        #     print(hex(send_buffer[i]), end=' ')
        # print()
        
        self.ser.write(send_buffer)
        receive_buffer = self.ser.read(buffer_size).hex()
        # print('receive_buffer: ', receive_buffer)

        target_sequence = ['aa', '55', '0f', self.ID]
        try:
            start_index = receive_buffer.index(target_sequence[0]+target_sequence[1]+target_sequence[2])
            if start_index != 0:
                receive_buffer = receive_buffer[start_index:]
                rereaed_size = buffer_size - len(receive_buffer)
                receive_buffer.extend(self.ser.read(rereaed_size).hex())
        except:
            receive_buffer = None

        return receive_buffer

    
    def _decode_field(self, value):
        """
        解码16进制字段为整数值。

        Args:
            value (str): 16进制值的字符串表示。

        Returns:
            int: 解码后的整数值。
        """
        num_bits = len(value) * 4  # 每个十六进制字符对应4位二进制
        max_value = 2 ** (num_bits - 1) - 1
        int_value = int(value, 16)

        if int_value > max_value:
            int_value -= 2 ** num_bits

        return int_value

    def _decode_serial_data(self, receive_buffer):
        """
        解码从串口接收的数据。

        Args:
            receive_buffer (str): 从串口接收的16进制数据字符串。
        """

        if receive_buffer != None:
            split_values = [receive_buffer[i:i+2].upper() for i in range(0, len(receive_buffer), 2)]

            self.position = self._decode_field(split_values[10] + split_values[9])
            self.current = self._decode_field(split_values[12] + split_values[11])   # 电流单位为毫安
            self.force = self._decode_field(split_values[14] + split_values[13])     # 力单位为g
            self.init_force = self._decode_field(split_values[16] + split_values[15])# 初始力单位为g
            self.tempreture = self._decode_field(split_values[17])                  # 温度单位为摄氏度

            self.position = self.position * INSPIRE_STEP_LENGTH

    def force_control(self, force):
        pass

    def speed_control(self, speed):
        if self.control_mode != 'speed':
            self.control_mode = 'speed'
            send_buffer = ['55', 'AA', '05', self.ID, '32', '25', '00', '02', '00']
            receive_buffer = self._write_serial_data(send_buffer)
            self._decode_serial_data(receive_buffer)

        if speed > INSPIRE_MAX_SPEED:
            speed = INSPIRE_MAX_SPEED
        elif speed < -INSPIRE_MAX_SPEED:
            speed = -INSPIRE_MAX_SPEED

        speed = int(speed / INSPIRE_STEP_LENGTH)

        # target_position = self.max_position if speed > 0 else 0
        target_position = self.position + speed * TARGET_PERIOD
        target_position = clamp(target_position, 0, self.max_position)
        target_position = int(target_position / INSPIRE_STEP_LENGTH)
        target_position = decimal_to_hexadecimal(target_position)

        target_speed = abs(speed)
        target_speed = decimal_to_hexadecimal(target_speed)

        send_buffer = ['55', 'AA', '07', self.ID, '32', '28', '00'] + list(reversed(target_speed)) + list(reversed(target_position))
        receive_buffer = self._write_serial_data(send_buffer)

        self._decode_serial_data(receive_buffer)

    def debug_speed_control(self, speed):
        if self.control_mode != 'speed':
            self.control_mode = 'speed'
            send_buffer = ['55', 'AA', '05', self.ID, '32', '25', '00', '02', '00']
            receive_buffer = self._write_serial_data(send_buffer)
            self._decode_serial_data(receive_buffer)

        if speed > INSPIRE_MAX_SPEED:
            speed = INSPIRE_MAX_SPEED
        elif speed < -INSPIRE_MAX_SPEED:
            speed = -INSPIRE_MAX_SPEED

        speed = int(speed / INSPIRE_STEP_LENGTH)

        # target_position = self.max_position if speed > 0 else 0
        target_position = INSPIRE_MAX_POSITION if speed > 0 else 0
        target_position = int(target_position / INSPIRE_STEP_LENGTH)
        target_position = decimal_to_hexadecimal(target_position)

        target_speed = abs(speed)
        target_speed = decimal_to_hexadecimal(target_speed)

        send_buffer = ['55', 'AA', '07', self.ID, '32', '28', '00'] + list(reversed(target_speed)) + list(reversed(target_position))
        receive_buffer = self._write_serial_data(send_buffer)

        self._decode_serial_data(receive_buffer)


    def position_control(self, target_position, max_speed):
        if self.control_mode != 'speed':
            self.control_mode = 'speed'
            send_buffer = ['55', 'AA', '05', self.ID, '32', '25', '00', '02', '00']
            receive_buffer = self._write_serial_data(send_buffer)
            self._decode_serial_data(receive_buffer)

        max_speed = clamp(max_speed, 0, INSPIRE_MAX_SPEED)
        max_speed = abs(int(max_speed / INSPIRE_STEP_LENGTH))
        target_speed = decimal_to_hexadecimal(max_speed)

        target_position = clamp(target_position, 0, self.max_position)
        target_position = int(target_position / INSPIRE_STEP_LENGTH)
        target_position = decimal_to_hexadecimal(target_position)

        send_buffer = ['55', 'AA', '07', self.ID, '32', '28', '00'] + list(reversed(target_speed)) + list(reversed(target_position))
        receive_buffer = self._write_serial_data(send_buffer)

        self._decode_serial_data(receive_buffer)

    def inspire_position_control(self, target_position):
        if self.control_mode != 'position':
            self.control_mode = 'position'
            send_buffer = ['55', 'AA', '05', self.ID, '32', '25', '00', '00', '00']
            receive_buffer = self._write_serial_data(send_buffer)
            self._decode_serial_data(receive_buffer)       

        target_position = clamp(target_position, 0, self.max_position)
        target_position = int(target_position / INSPIRE_STEP_LENGTH)
        target_position = decimal_to_hexadecimal(target_position)


        send_buffer = ['55', 'AA', '05', self.ID, '32', '29', '00'] + list(reversed(target_position))
        receive_buffer = self._write_serial_data(send_buffer)

        self._decode_serial_data(receive_buffer)


    def stop(self):
        send_buffer = ['55', 'AA', '05', self.ID, '32', '1A', '00', '01', '00']
        receive_buffer = self._write_serial_data(send_buffer)
        self._decode_serial_data(receive_buffer)

    def update_state(self):
        send_buffer = ['55', 'AA', '01', self.ID, '30']
        receive_buffer = self._write_serial_data(send_buffer)
        self._decode_serial_data(receive_buffer) 

    def check(self):
        send_buffer = ['55', 'AA', '01', self.ID, '30']
        receive_buffer = self._write_serial_data(send_buffer)
        return receive_buffer != None

    def reset_position(self):
        print(f"{get_time()}-警告：电机 {self.ID} 正在更新最大位置...")
        self.update_state()
        if self.ID == '0'+str(UP_MOTOR):
            print(f"{get_time()}-警告：电机 {self.ID} 位置已重置为 {self.position} mm")
            change_config_value('UP_INIT_POSITION', self.position)
        elif self.ID == '0'+str(RIGHT_MOTOR):
            print(f"{get_time()}-警告：电机 {self.ID} 位置已重置为 {self.position} mm")
            change_config_value('RIGHT_INIT_POSITION', self.position)
        elif self.ID == '0'+str(DOWN_MOTOR):
            print(f"{get_time()}-警告：电机 {self.ID} 位置已重置为 {self.position} mm")
            change_config_value('DOWN_INIT_POSITION', self.position)
        elif self.ID == '0'+str(LEFT_MOTOR):
            print(f"{get_time()}-警告：电机 {self.ID} 位置已重置为 {self.position} mm")
            change_config_value('LEFT_INIT_POSITION', self.position)
        self.max_position = self.position

        if self.max_position < TURN_COEFF:
            print(f'{get_time()}-警告：电机 {self.ID} 最大位置小于转弯系数，可能会导致电机无法正常工作！')
        

    def check_arrive_zero_position(self):
        self.update_state()
        if self.ID == '0'+str(UP_MOTOR):
            return abs(self.position - UP_INIT_POSITION) < 0.5
        elif self.ID == '0'+str(RIGHT_MOTOR):
            return abs(self.position - RIGHT_INIT_POSITION) < 0.5
        elif self.ID == '0'+str(DOWN_MOTOR):
            return abs(self.position - DOWN_INIT_POSITION) < 0.5
        elif self.ID == '0'+str(LEFT_MOTOR):
            return abs(self.position - LEFT_INIT_POSITION) < 0.5

    def return_zero_position(self):
        if self.ID == '0'+str(UP_MOTOR):
            self.position_control(UP_INIT_POSITION, INSPIRE_MAX_SPEED)
        elif self.ID == '0'+str(RIGHT_MOTOR):
            self.position_control(RIGHT_INIT_POSITION, INSPIRE_MAX_SPEED)
        elif self.ID == '0'+str(DOWN_MOTOR):
            self.position_control(DOWN_INIT_POSITION, INSPIRE_MAX_SPEED)
        elif self.ID == '0'+str(LEFT_MOTOR):
            self.position_control(LEFT_INIT_POSITION, INSPIRE_MAX_SPEED)


    def __repr__(self):
        """
        电机状态的字符串表示。

        Returns:
            str: 电机状态的字符串表示。
        """
        return f"ID: {self.ID}, 位置: {self.position}, 电流: {self.current}, 力: {self.force}, 初始力: {self.init_force}, 温度: {self.tempreture}"
    


def calculate_crc(send_buffer):
    
    _send_buffer = send_buffer[2:]

    crc = 0
    for i in range(len(_send_buffer)):
        crc += int(_send_buffer[i], 16)
    crc = str(hex(crc)).replace('0x', '')

    send_buffer = send_buffer + [crc[-2:]]

    send_buffer = [int('0x' + value, 16) for value in send_buffer]

    return send_buffer

def decimal_to_hexadecimal(number):
    hex_value = hex(abs(number)).replace('0x', '')

    while len(hex_value) % 4 != 0:
        hex_value = '0' + hex_value


    if number < 0:
        int_value = int(hex_value, 16)
        hex_value = hex(~int_value + 1 & 0xFFFFFFFF).replace('0x', '')

    split_values = [hex_value[i:i+2] for i in range(0, len(hex_value), 2)]
    split_values = [value.upper() for value in split_values]

    return split_values




if __name__ =='__main__':
    ser = serial.Serial('COM5', baudrate=115200, timeout=0.05)

    motor_1 = InspireMotor(1, ser)
    motor_2 = InspireMotor(2, ser)
    motor_3 = InspireMotor(3, ser)
    motor_4 = InspireMotor(4, ser)


    speed = 1
    motor_1.position_control(30, speed)
    # time.sleep(1)
    # motor_1.stop()

    # motor_1.speed_control(speed)
    # motor_2.speed_control(speed)
    # motor_3.speed_control(speed)
    # motor_4.speed_control(speed)
    # p = 0
    # speed = 5
    # while p >= 0 and p <= 30:
    #     motor.position_control(p, speed)

    #     p += speed * 0.05

    #     time.sleep(0.05)
    
    
    