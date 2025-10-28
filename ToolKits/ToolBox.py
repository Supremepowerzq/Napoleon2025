import os
import datetime
import chardet
import importlib
import sys

def get_time():
    current_time = datetime.datetime.now()
    formatted_time = current_time.strftime("%Y-%m-%d_%H-%M-%S")
    return formatted_time

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def create_directories(directory_paths):
    if isinstance(directory_paths, str):
        # 如果是字符串，将其放入一个字典中，以默认键名作为标识
        directory_paths = {'default_key': directory_paths}

    for key, path in directory_paths.items():
        # 检查路径是否存在，如果不存在则创建
        if not os.path.exists(path):
            os.makedirs(path)
            print(f"{get_time()}-目录 {path} 创建成功 (标识: {key})")


def change_config_value(key, value):
    try:
        import config
    except:
        pass

    # 读取 key 的当前值
    current_value = eval(f"config.{key}")

    # 读取config.py文件的内容，自动检测编码
    with open('config.py', 'rb') as file:
        result = chardet.detect(file.read())
        file_encoding = result['encoding']

    # 使用检测到的编码打开文件
    with open('config.py', 'r', encoding=file_encoding) as file:
        config_content = file.read()

    # 修改变量的值
    config_content = config_content.replace(f"{key} = {current_value}", f"{key} = {value}")

    with open('config.py', 'w', encoding=file_encoding) as file:
        file.write(config_content)

    importlib.reload(sys.modules['config'])




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
    

if __name__ == '__main__':
    print(f"{get_time()}")