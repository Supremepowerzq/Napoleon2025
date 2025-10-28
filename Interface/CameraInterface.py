import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import pyrealsense2 as rs
import numpy as np
import cv2
import ToolKits.ToolBox as ToolBox
from ToolKits.ToolBox import get_time
import Segmentation.image_rectify as image_rectify

def find_micro_camera():
    print(f"{get_time()}-正在查找微型相机...")
    for i in range(10):
        try:
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            ret, frame = cap.read()
            if frame.shape==(400, 400, 3):
                print(f"{get_time()}-找到微型相机，编号为{i}")
                return i
            cap.release()
        except:
            continue
    raise Exception("未找到微型相机")

class RealSenseL515:
    def __init__(self) -> None:
        """
        初始化RealSenseL515相机对象。
        """

        print(f"{get_time()}-正在初始化深度相机...")
        # 创建RealSenseL515数据流（pipeline）以管理数据流
        self.pipeline = rs.pipeline()

        # 配置数据流参数
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # 使用配置的设置启动数据流，并获取数据流配置
        self.cfg = self.pipeline.start(self.config)

        # 获取深度传感器及其深度比例尺度
        self.depth_sensor = self.cfg.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # 获取深度和彩色数据流的配置文件
        self.profile_depth = self.cfg.get_stream(rs.stream.depth)
        self.profile_color = self.cfg.get_stream(rs.stream.color)

        # 获取用于对齐彩色和深度数据流的外部参数
        self.extrinsics = self.profile_color.get_extrinsics_to(self.profile_depth)

        # 创建用于对齐彩色和深度帧的对齐器
        self.align = rs.align(rs.stream.color)


        # 初始化相机内参
        self.camera_intrinsics = None

        print(f"{get_time()}-深度相机初始化完成！")

    def get_aligned_frames(self):
        """
        从RealSenseL515相机中捕获并返回对齐后的深度和彩色帧。

        返回:
            aligned_depth_frame: 对齐后的深度帧。
            color_frame: 对齐后的彩色帧。
        """
        # 等待新的帧数据可用
        frames = self.pipeline.wait_for_frames()

        # 对齐彩色和深度帧
        aligned_frames = self.align.process(frames)

        # 获取对齐后的深度和彩色帧
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # 提取彩色帧的相机内参（仅在第一次获取帧时提取）
        if self.camera_intrinsics is None:
            self.camera_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image = depth_image * self.depth_scale

        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image


    def get_camera_intrinsics(self):
        """
        获取彩色帧的相机内参。

        返回:
            相机内参对象。
        """
        return self.camera_intrinsics
    



if __name__ == "__main__":
    Camera = RealSenseL515()
    cv2.namedWindow("color", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("rectify", cv2.WINDOW_AUTOSIZE)

    sorted_points = []

    while 1:
        color_image, depth_image = Camera.get_aligned_frames()
        if sorted_points == []:
            m_contours = image_rectify.image_HSV_red_detect(color_image)
            m_centers = image_rectify.image_moments(m_contours)
            sorted_points = image_rectify.image_centers_sort(m_centers)
            print(sorted_points)
            continue

        result_image = image_rectify.image_perspective_transform(color_image, sorted_points)

        cv2.imshow("color", color_image)
        cv2.imshow("rectify", result_image)

        key = cv2.waitKey(100)
        if key & 0xFF == ord('q'):
            break