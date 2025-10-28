import time

class RmdMotor:
    def __init__(self, ID, ser):
        """
        初始化电机对象。

        参数:
            ID (int): 电机的ID号。
            ser: 用于与电机通信的串口对象。
        """

        # 将ID转换为十六进制字符串，并确保长度为偶数
        ID = hex(ID).upper().replace('0X', '')
        while len(ID) % 2 != 0:
            ID = '0' + ID

        # 初始化电机属性
        self.ID = ID  # 电机ID
        self.ser = ser  # 串口对象
        # 初始化电机的状态信息，初始值设为None
        self.tempreture = None  # 温度
        self.current = None  # 电流
        self.speed = None  # 速度
        self.position = None  # 位置
        # 初始化PID参数，初始值设为None
        self.pid_parameters = {
            'current_KP': None,
            'current_KI': None,
            'velocity_KP': None,
            'velocity_KI': None,
            'position_KP': None,
            'position_KI': None
        }

    def _write_serial_data(self, send_buffer, wait=False):
        """
        向串口发送数据，并接收回复。

        参数:
            send_buffer (list): 要发送的数据列表。
            wait (bool): 是否等待回复。

        返回:
            str: 接收到的数据的十六进制字符串。
        """

        buffer_size = 13  # 接收缓冲区大小
        send_buffer = calculate_crc(send_buffer)  # 计算并添加CRC校验

        self.ser.write(send_buffer)  # 发送数据
        if wait: time.sleep(0.2)  # 如果需要等待，暂停0.2秒
        receive_buffer = self.ser.read(buffer_size).hex()  # 读取接收缓冲区的数据

        # 查找接收数据中的目标序列
        target_sequence = ['3e', self.ID, '08']
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
        将十六进制字段解码为整数。

        参数:
            value (str): 十六进制值的字符串表示。

        返回:
            int: 解码后的整数值。
        """
        # 计算值的位数，并据此计算最大值和解码后的整数值
        num_bits = len(value) * 4  # 每个十六进制字符对应4位
        max_value = 2 ** (num_bits - 1) - 1
        int_value = int(value, 16)

        if int_value > max_value:
            int_value -= 2 ** num_bits

        return int_value

    def _decode_serial_data(self, receive_buffer):
        """
        解码从串口接收的数据。

        参数:
            receive_buffer (str): 从串口接收的十六进制数据字符串。
        """

        if receive_buffer != None:
            # 将接收的数据分割为两位一组的十六进制字符串，并解码各状态值
            split_values = [receive_buffer[i:i+2].upper() for i in range(0, len(receive_buffer), 2)]

            # 解码并更新电机状态信息
            self.tempreture = self._decode_field(split_values[4])  # 温度
            self.current = self._decode_field(split_values[6] + split_values[5]) * 0.01  # 电流
            self.speed = self._decode_field(split_values[8] + split_values[7])  # 速度
            self.position = self._decode_field(split_values[10] + split_values[9])  # 位置

    def force_control(self, force):
        """
        执行力控制命令。

        参数:
            force (float): 期望施加的力，单位为A（安培）。

        注意:
            该方法通过发送控制命令来实现电机的力控制，并更新电机的状态。
        """
        force = int(force * 100)  # 将力的单位转换为适合传输的格式
        hex_force = decimal_to_hexadecimal(force)  # 将十进制力值转换为十六进制
        # 准备发送的数据，包含力控制命令及其参数
        send_buffer = ['3E', self.ID, '08', 'A1', '00', '00', '00'] + list(reversed(hex_force))
        # 由于转矩控制只用到前四个字节，将后两字节设为0
        send_buffer[9] = '00'
        send_buffer[10] = '00'
        
        # 发送数据并接收回复，然后解码回复数据
        receive_buffer = self._write_serial_data(send_buffer)
        self._decode_serial_data(receive_buffer)

    def speed_control(self, velocity):
        """
        执行速度控制命令。

        参数:
            velocity (float): 期望的速度，单位为度/秒。

        注意:
            该方法通过发送控制命令来实现电机的速度控制，并更新电机的状态。
        """
        velocity = int(velocity * 100)  # 将速度的单位转换为适合传输的格式
        hex_velocity = decimal_to_hexadecimal(velocity)  # 将十进制速度值转换为十六进制
        # 准备发送的数据，包含速度控制命令及其参数
        send_buffer = ['3E', self.ID, '08', 'A2', '00', '00', '00'] + list(reversed(hex_velocity))
        
        # 发送数据并接收回复，然后解码回复数据
        receive_buffer = self._write_serial_data(send_buffer)
        self._decode_serial_data(receive_buffer)

    def stop(self):
        """
        发送停止电机的命令。

        注意:
            该方法通过发送停止命令来停止电机的运行，并更新电机的状态。
        """
        # 准备发送的停止命令数据
        send_buffer = ['3E', self.ID, '08', '81', '00', '00', '00', '00', '00', '00', '00']
        # 发送数据并接收回复
        receive_buffer = self._write_serial_data(send_buffer)

    def update_state(self):
        """
        更新电机的状态信息。

        注意:
            该方法通过发送特定的查询命令来请求电机的当前状态，并更新本地状态信息。
        """
        # 准备发送的查询状态命令数据
        send_buffer = ['3E', self.ID, '08', '9C', '00', '00', '00', '00', '00', '00', '00']
        # 发送数据并接收回复，然后解码回复数据
        receive_buffer = self._write_serial_data(send_buffer)
        self._decode_serial_data(receive_buffer)

    def check(self):
        """
        检查电机的通信状态。

        返回:
            bool: 如果电机响应，则返回True，否则返回False。
        """
        # 准备发送的检查命令数据
        send_buffer = ['3E', self.ID, '08', '9C', '00', '00', '00', '00', '00', '00', '00']
        # 发送数据并接收回复
        receive_buffer = self._write_serial_data(send_buffer)
        # 如果收到回复，则通信正常
        return receive_buffer != None

    def read_pid_parameters(self):
        """
        读取电机的PID参数。

        注意:
            该方法通过发送读取PID参数的命令来获取电机的PID参数，并更新本地PID参数信息。
        """
        # 准备发送的读取PID参数命令数据
        send_buffer = ['3E', self.ID, '08', '30', '00', '00', '00', '00', '00', '00', '00']
        # 发送数据并接收回复
        receive_buffer = self._write_serial_data(send_buffer)
        # 解析并更新PID参数
        split_values = [receive_buffer[i:i+2].upper() for i in range(0, len(receive_buffer), 2)]
        self.pid_parameters = {
            'current_KP': self._decode_field(split_values[5]),
            'current_KI': self._decode_field(split_values[6]),
            'velocity_KP': self._decode_field(split_values[7]),
            'velocity_KI': self._decode_field(split_values[8]),
            'position_KP': self._decode_field(split_values[9]),
            'position_KI': self._decode_field(split_values[10])
        }

    def reset_position(self):
        """
        重置电机的位置。

        注意:
            该方法发送重置位置的命令，并在必要时等待回复以确认操作成功。
        """
        # 准备发送的重置位置命令数据
        send_buffer = ['3E', self.ID, '08', '64', '00', '00', '00', '00', '00', '00', '00']
        # 发送数据并接收回复，等待确保操作完成
        receive_buffer = self._write_serial_data(send_buffer, wait=True)
        # 如果没有收到回复，则表示重置位置操作出错
        if receive_buffer == None:
            print(f"motor {self.ID} reset multi position error")
        # 重置系统状态
        self.reset_system()

    def reset_system(self):
        """
        重置电机系统。

        注意:
            该方法发送重置系统的命令，用于在特定情况下重启电机控制器。
        """
        # 准备发送的重置系统命令数据
        send_buffer = ['3E', self.ID, '08', '76', '00', '00', '00', '00', '00', '00', '00']
        # 发送数据并接收回复
        receive_buffer = self._write_serial_data(send_buffer)

    def __repr__(self):
        """
        返回电机状态的字符串表示。

        返回:
            str: 电机状态的字符串描述，包括ID、温度、电流、速度和位置信息。
        """
        return f"Motor {self.ID}, Temperature: {self.tempreture}°C, Current: {self.current}A, Speed: {self.speed}dps, Position: {self.position}°"

def modbusCrc(msg: str) -> int:
    """
    计算给定消息的Modbus CRC校验码。

    参数:
        msg (str): 需要计算CRC的消息，以字节串形式提供。

    返回:
        int: 计算出的CRC校验码。
    """
    crc = 0xFFFF  # 初始化CRC校验码
    for n in range(len(msg)):  # 遍历消息中的每个字节
        crc ^= msg[n]  # 将CRC校验码与字节异或
        for i in range(8):  # 对每个位进行处理
            if crc & 1:  # 如果CRC的最低位为1
                crc >>= 1  # 右移一位
                crc ^= 0xA001  # 与多项式异或
            else:
                crc >>= 1  # 如果最低位为0，只需右移一位
    return crc  # 返回计算出的CRC校验码

def calculate_crc(send_buffer):
    """
    计算发送缓冲区数据的CRC校验码，并将校验码附加到数据末尾。

    参数:
        send_buffer (list): 要发送的数据列表，每个元素为十六进制字符串。

    返回:
        list: 添加了CRC校验码的发送数据列表。
    """
    hex_str = ''.join(send_buffer)  # 将数据列表转换为十六进制字符串
    bytes_str = bytes.fromhex(hex_str)  # 将十六进制字符串转换为字节串

    crc = modbusCrc(bytes_str)  # 计算字节串的CRC校验码
    crc_bytes = crc.to_bytes(2, byteorder='little')  # 将CRC校验码转换为字节串
    
    # 将原始数据和CRC校验码转换为整数列表
    send_buffer = [int('0x' + value, 16) for value in send_buffer] + [crc_bytes[0], crc_bytes[1]]

    return send_buffer  # 返回添加了CRC校验码的发送数据列表

def decimal_to_hexadecimal(number):
    """
    将十进制数转换为十六进制字符串，并确保其长度为8的倍数。

    参数:
        number (int): 需要转换的十进制数。

    返回:
        list: 十六进制字符串的分割列表，每个元素包含两个字符。
    """
    hex_value = hex(abs(number)).replace('0x', '')  # 将十进制数转换为十六进制字符串

    # 确保十六进制字符串的长度为8的倍数
    while len(hex_value) % 8 != 0:
        hex_value = '0' + hex_value

    # 如果原始数为负数，计算其补码表示
    if number < 0:
        int_value = int(hex_value, 16)  # 将十六进制字符串转换为整数
        hex_value = hex(~int_value + 1 & 0xFFFFFFFF).replace('0x', '')  # 计算补码

    # 将十六进制字符串分割为每两个字符一组，并转换为大写
    split_values = [hex_value[i:i+2] for i in range(0, len(hex_value), 2)]
    split_values = [value.upper() for value in split_values]

    return split_values  # 返回分割后的十六进制字符串列表




#------------------- 1.0 代码布局 -------------------
# import time

# class RmdMotor:
#     def __init__(self, ID, ser):
#         """
#         初始化Motor对象。

#         Args:
#             ID (int): 电机的ID。
#             ser: 串口通信对象。
#         """

#         ID = hex(ID).upper().replace('0X', '')
#         while len(ID) % 2 != 0:
#             ID = '0' + ID

#         self.ID = ID
#         self.ser = ser

#         self.tempreture = None
#         self.current = None
#         self.speed = None
#         self.position = None

#         self.pid_parameters = {
#             'current_KP': None,
#             'current_KI': None,
#             'velocity_KP': None,
#             'velocity_KI': None,
#             'position_KP': None,
#             'position_KI': None
#         }


#     def _write_serial_data(self, send_buffer, wait=False):
#         """
#         向串口写入数据，并处理接收缓冲区。

#         Args:
#             send_buffer (list): 发送的数据列表。

#         Returns:
#             str: 接收到的数据的16进制字符串。
#         """

#         buffer_size = 13
#         send_buffer = calculate_crc(send_buffer)

#         # print('send_buffer: ', send_buffer)
#         self.ser.write(send_buffer)
#         if wait: time.sleep(0.2)
#         receive_buffer = self.ser.read(buffer_size).hex()
#         # print('receive_buffer: ', receive_buffer)

#         target_sequence = ['3e', self.ID, '08']
#         try:
#             start_index = receive_buffer.index(target_sequence[0]+target_sequence[1]+target_sequence[2])
#             if start_index != 0:
#                 receive_buffer = receive_buffer[start_index:]
#                 rereaed_size = buffer_size - len(receive_buffer)
#                 receive_buffer.extend(self.ser.read(rereaed_size).hex())
#         except:
#             receive_buffer = None

#         return receive_buffer
    
#     def _decode_field(self, value):
#         """
#         解码16进制字段为整数值。

#         Args:
#             value (str): 16进制值的字符串表示。

#         Returns:
#             int: 解码后的整数值。
#         """
#         num_bits = len(value) * 4  # 每个十六进制字符对应4位二进制
#         max_value = 2 ** (num_bits - 1) - 1
#         int_value = int(value, 16)

#         if int_value > max_value:
#             int_value -= 2 ** num_bits

#         return int_value

#     def _decode_serial_data(self, receive_buffer):
#         """
#         解码从串口接收的数据。

#         Args:
#             receive_buffer (str): 从串口接收的16进制数据字符串。
#         """

#         if receive_buffer != None:
#             split_values = [receive_buffer[i:i+2].upper() for i in range(0, len(receive_buffer), 2)]

#             self.tempreture = self._decode_field(split_values[4])                 # 温度单位为摄氏度
#             self.current = self._decode_field(split_values[6] + split_values[5]) * 0.01  # 电流单位为 A
#             self.speed = self._decode_field(split_values[8] + split_values[7])    # 速度单位为 dps
#             self.position = self._decode_field(split_values[10] + split_values[9]) # 位置单位为度
        

#     def force_control(self, force):
#         """
#         执行力控制。

#         Args:
#             force (float): 所期望的力(电流 单位为A)。

#         Note:
#             该方法发送控制命令并更新电机状态。
#         """
#         force = int(force * 100)
#         hex_force = decimal_to_hexadecimal(force)
#         send_buffer = ['3E', self.ID, '08', 'A1', '00', '00', '00'] + list(reversed(hex_force))

#         # 转矩控制只有前四个字节有效
#         send_buffer[9] = '00'
#         send_buffer[10] = '00'
        
#         receive_buffer = self._write_serial_data(send_buffer)
#         self._decode_serial_data(receive_buffer)

#     def speed_control(self, velocity):
#         """
#         执行速度控制。

#         Args:
#             velocity (float): 所期望的速度(单位 度/s)。

#         Note:
#             该方法发送控制命令并更新电机状态。
#         """
#         velocity = int(velocity * 100)
#         hex_velocity = decimal_to_hexadecimal(velocity)
#         send_buffer = ['3E', self.ID, '08', 'A2', '00', '00', '00'] + list(reversed(hex_velocity))
        
#         receive_buffer = self._write_serial_data(send_buffer)

#         self._decode_serial_data(receive_buffer)
        

#     def stop(self):
#         """
#         停止电机。

#         Note:
#             该方法发送控制命令并更新电机状态。
#         """
#         send_buffer = ['3E', self.ID, '08', '81', '00', '00', '00', '00', '00', '00', '00']
#         receive_buffer = self._write_serial_data(send_buffer)

    
#     def update_state(self):
#         """
#         更新电机状态。

#         Note:
#             该方法更新电机状态。
#         """
#         send_buffer = ['3E', self.ID, '08', '9C', '00', '00', '00', '00', '00', '00', '00']
#         receive_buffer = self._write_serial_data(send_buffer)

#         self._decode_serial_data(receive_buffer)

#     def check(self):
#         send_buffer = ['3E', self.ID, '08', '9C', '00', '00', '00', '00', '00', '00', '00']
#         receive_buffer = self._write_serial_data(send_buffer)
#         return receive_buffer != None


#     def read_pid_parameters(self): 
#         """
#         读取电机PID参数。

#         Note:
#             该方法读取电机PID参数。
#         """
#         send_buffer = ['3E', self.ID, '08', '30', '00', '00', '00', '00', '00', '00', '00']
#         receive_buffer = self._write_serial_data(send_buffer)

#         split_values = [receive_buffer[i:i+2].upper() for i in range(0, len(receive_buffer), 2)]

#         self.pid_parameters = {
#             'current_KP': self._decode_field(split_values[5]),
#             'current_KI': self._decode_field(split_values[6]),
#             'velocity_KP': self._decode_field(split_values[7]),
#             'velocity_KI': self._decode_field(split_values[8]),
#             'position_KP': self._decode_field(split_values[9]),
#             'position_KI': self._decode_field(split_values[10])
#         }

#     def reset_position(self):

#         send_buffer = ['3E', self.ID, '08', '64', '00', '00', '00', '00', '00', '00', '00']
#         receive_buffer = self._write_serial_data(send_buffer, wait=True)

#         if receive_buffer == None:
#             print("motor ", self.ID, " reset multi position error")

#         self.reset_system()

#     def reset_system(self):

#         send_buffer = ['3E', self.ID, '08', '76', '00', '00', '00', '00', '00', '00', '00']
#         receive_buffer = self._write_serial_data(send_buffer)


#     def __repr__(self):
#         """
#         电机状态的字符串表示。

#         Returns:
#             str: 电机状态的字符串表示。
#         """

#         return f"Motor {self.ID}, Temperature: {self.tempreture}°C, Current: {self.current}A, Speed: {self.speed}dps, Position: {self.position}°"


# def modbusCrc(msg:str) -> int:
#     crc = 0xFFFF
#     for n in range(len(msg)):
#         crc ^= msg[n]
#         for i in range(8):
#             if crc & 1:
#                 crc >>= 1
#                 crc ^= 0xA001
#             else:
#                 crc >>= 1
#     return crc

# def calculate_crc(send_buffer):
#     hex_str = ''.join(send_buffer)

#     bytes_str = bytes.fromhex(hex_str)

#     crc = modbusCrc(bytes_str).to_bytes(2, byteorder='little')
    
#     send_buffer = [int('0x' + value, 16) for value in send_buffer] + [crc[0], crc[1]]

#     return send_buffer

# def decimal_to_hexadecimal(number):
#     hex_value = hex(abs(number)).replace('0x', '')

#     while len(hex_value) % 8 != 0:
#         hex_value = '0' + hex_value


#     if number < 0:
#         int_value = int(hex_value, 16)
#         hex_value = hex(~int_value + 1 & 0xFFFFFFFF).replace('0x', '')

#     split_values = [hex_value[i:i+2] for i in range(0, len(hex_value), 2)]
#     split_values = [value.upper() for value in split_values]

#     return split_values