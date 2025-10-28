import cv2
import numpy as np
import os
import shutil
import datetime

import chardet
import importlib
import sys


# 创建一个鼠标回调函数，用于检测鼠标点击事件并获取点击位置的像素值
def create_mouse_callback(m_frame):
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 定义一个鼠标左键按下去的事件
            # 获取鼠标点击位置的像素值
            pixel_value = m_frame[y, x]
            # 将BGR格式转换为HSV格式
            HSV = cv2.cvtColor(np.uint8([[pixel_value]]), cv2.COLOR_BGR2HSV)
            H, S, V = HSV[0][0]  # 获取HSV值

            print(f"Clicked pixel position: ({x}, {y})")
            print(f"B G R values: {pixel_value}")
            print(f"H S V values: H={H}, S={S}, V={V}")
            # HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # print("HSV information(H, S, V) : ", HSV[y, x], "coordinate : ", x, y)
    return mouse_callback


# 检查指定路径目录是否存在, 如果不存在则创建该目录
def check_directory_exist(directory_path):
    if not os.path.exists(directory_path):
        print(f"File {directory_path} does not exist. Now create it.")
        # 使用os.makedirs函数创建该目录
        os.makedirs(directory_path)
    else:
        print(f"File {directory_path} exists.")


# 递归删除指定目录下的所有文件和文件夹
def delete_directory(directory_path):
    if os.path.exists(directory_path):
        print(f"File {directory_path} exists. Now remove it.")
        # 使用shutil.rmtree函数递归删除该目录及其所有子目录和文件
        shutil.rmtree(directory_path)
    else:
        print(f"File {directory_path} does not exist.")


# 用于递归地删除指定路径下的所有文件和文件夹
def remove_files(directory_path):
    if not os.path.exists(directory_path):
        print(f"File {directory_path} does not exist.")
        return
    else:
        print(f"File {directory_path} exists.")
        # 使用 os.walk 函数来遍历指定路径 path 及其子目录中的所有文件和文件夹, topdown=False 表示从底层向上遍历目录结构
        for root, dirs, files in os.walk(directory_path, topdown=False):
            # 在每个目录中遍历文件列表 files，即当前目录下的所有文件
            for file_name in files:
                file_path = os.path.join(root, file_name)
                os.remove(file_path)
            # 在每个目录中遍历子目录列表 dirs，即当前目录下的所有子目录
            for dir_name in dirs:
                dir_path = os.path.join(root, dir_name)
                os.rmdir(dir_path)


# 实现从输入的一个路径名称中，提取最后一级目录名
def extract_last_directory(path):
    # 使用os.path.basename函数获取路径中的最后一个目录名,
    # os.path.normpath函数用于规范化路径 : 即使路径中存在反斜杠 (\) 或者混合使用反斜杠和正斜杠，都会转化为标准的路径格式。
    last_directory = os.path.basename(os.path.normpath(path))
    return last_directory


# 基于当前时间生成一个唯一的文件名
def generate_unique_filename():
    # 使用datetime模块中的datetime类获取当前时间
    current_time = datetime.datetime.now()
    # 使用strftime函数将当前时间格式化为指定格式
    current_time_formatted = current_time.strftime("%Y%m%d_%H%M%S")
    return current_time_formatted

