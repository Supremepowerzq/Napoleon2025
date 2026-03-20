import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)
#----------------------------------------------------#
#   将单张图片预测、摄像头检测和FPS测试功能
#   整合到了一个py文件中，通过指定mode进行模式的修改。
#----------------------------------------------------#
import time
import cv2
import numpy as np
import torch
import matplotlib
import warnings
from PIL import Image
from DLpredict import Unet_ONNX, Unet

from config import *
from ToolKits.ToolBox import get_time

from config import update_config
from config import get_config

# import cProfile

# ---------------- 深度推理相关导入 ----------------
try:
    # 添加Depth-Anything-V2-main目录到路径
    depth_path = os.path.join(os.path.dirname(__file__), 'Depth-Anything-V2-main')
    if depth_path not in sys.path:
        sys.path.insert(0, depth_path)
    from depth_anything_v2.dpt import DepthAnythingV2
    DEPTH_AVAILABLE = True
except ImportError as e:
    print(f"警告：无法导入 Depth-Anything-V2: {e}，将禁用深度推理功能")
    DEPTH_AVAILABLE = False

class UnetPackage:
    def __init__(self, model_path=None,
        mode='video', count=False, name_classes=None,
        video_path=0, video_save_path="",
        video_fps=30, test_interval=1000,
        fps_image_path="img/street.jpg",
        dir_origin_path="img/",
        dir_save_path="img_out/",
        simplify=True,
        onnx_save_path="model_data/models.onnx",
        # 深度推理相关参数
        enable_depth=False,
        depth_encoder='vits',
        depth_input_size=518,
        depth_grayscale=False,
        circle_center_x=400,
        circle_center_y=230,
        circle_radius=115,
        target_ratio=16/9,
        # speed_fowrd = 0,
        # speed_turn= []
       ):
     
        self.mode = mode
        self.count = count
        self.name_classes = name_classes or ["LaserTube", "Large"]
        self.video_path = video_path
        self.video_save_path = video_save_path
        self.video_fps = video_fps
        self.test_interval = test_interval
        self.fps_image_path = fps_image_path
        self.dir_origin_path = dir_origin_path
        self.dir_save_path = dir_save_path
        self.simplify = simplify
        self.onnx_save_path = onnx_save_path

        # 深度推理相关属性
        self.enable_depth = enable_depth and DEPTH_AVAILABLE
        self.depth_encoder = depth_encoder
        self.depth_input_size = depth_input_size
        self.depth_grayscale = depth_grayscale
        self.circle_center_x = circle_center_x
        self.circle_center_y = circle_center_y
        self.circle_radius = circle_radius
        self.target_ratio = target_ratio

        # 深度模型初始化
        self.depth_model = None
        if self.enable_depth:
            self._init_depth_model()

        # 初始化UNet模型
        if self.mode != "predict_onnx":
            self.unet = Unet()
        else:
            self.unet = Unet_ONNX()

        # self.speed_forward = 0
        # self.speed_turn = [0, 0]

    def _init_depth_model(self):
        """初始化深度推理模型"""
        if not DEPTH_AVAILABLE:
            print("深度推理功能不可用：无法导入 Depth-Anything-V2")
            self.enable_depth = False
            return

        try:
            # 设备选择
            self.depth_device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

            # 模型配置
            model_configs = {
                'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
                'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
                'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
                'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
            }

            print(f"初始化深度模型：{self.depth_encoder} | 设备：{self.depth_device} | 圆形参数：({self.circle_center_x},{self.circle_center_y}) 半径{self.circle_radius}")

            # 加载模型
            self.depth_model = DepthAnythingV2(**model_configs[self.depth_encoder])
            weight_path = os.path.join('E:/zq/TJU/Depth-Anything-V2-main/checkpoints', f'depth_anything_v2_{self.depth_encoder}.pth')

            if not os.path.exists(weight_path):
                print(f"错误：深度权重文件不存在 → {weight_path}")
                self.enable_depth = False
                return

            self.depth_model.load_state_dict(torch.load(weight_path, map_location=self.depth_device, weights_only=True))
            self.depth_model = self.depth_model.to(self.depth_device).eval()

            # 深度图可视化相关
            self.depth_cmap = matplotlib.colormaps.get_cmap('Spectral_r')

            print("深度模型初始化成功")

        except Exception as e:
            print(f"深度模型初始化失败：{e}")
            self.enable_depth = False

    def infer_depth(self, image):
        """
        对输入图像进行深度推理
        :param image: 输入图像（BGR格式）
        :return: 深度图（可视化后的图像）
        """
        if not self.enable_depth or self.depth_model is None:
            return None

        try:
            # 转换为RGB
            if len(image.shape) == 3:
                rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                rgb_image = image

            # 不对输入帧做拉伸，也不尝试恢复到其它尺寸，直接在输入帧大小上处理并返回同尺寸的深度可视化。
            # 将输入中黑色背景视为无效区域，用中性灰(128)填充以避免模型对黑边敏感。
            h, w = rgb_image.shape[:2]
            # 生成掩码：非黑像素视为有效区域（适用于 crop_to_circle 返回的圆形图）
            if len(rgb_image.shape) == 3:
                gray_for_mask = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            else:
                gray_for_mask = rgb_image
            crop_mask = np.zeros((h, w), dtype=np.uint8)
            crop_mask[gray_for_mask > 0] = 255

            # 生成 roi_image：将无效区域设为中性灰（128）
            roi_image = rgb_image.copy()
            if roi_image.ndim == 3:
                roi_image[crop_mask == 0] = 128
            else:
                roi_image[crop_mask == 0] = 128

            # 对 roi_image 直接推理（保持尺寸或内部由模型决定）
            with torch.no_grad():
                depth = self.depth_model.infer_image(roi_image, self.depth_input_size)

            # 归一化并转为 uint8（若 depth 全为常数，保护性处理）
            try:
                dmin = float(depth.min())
                dmax = float(depth.max())
                if dmax > dmin:
                    depth_norm = (depth - dmin) / (dmax - dmin) * 255.0
                else:
                    depth_norm = np.zeros_like(depth, dtype=np.float32)
                depth_uint8 = np.clip(depth_norm, 0, 255).astype(np.uint8)
            except Exception:
                depth_uint8 = depth.astype(np.uint8)

            # 将深度图（可能与输入尺寸不同）缩放到输入帧尺寸
            if depth_uint8.shape[:2] != (h, w):
                try:
                    depth_resized = cv2.resize(depth_uint8, (w, h), interpolation=cv2.INTER_LINEAR)
                except Exception:
                    # 回退：裁剪或填充到目标尺寸
                    dh, dw = depth_uint8.shape[:2]
                    depth_resized = np.zeros((h, w), dtype=np.uint8)
                    depth_resized[:min(dh, h), :min(dw, w)] = depth_uint8[:min(dh, h), :min(dw, w)]
            else:
                depth_resized = depth_uint8

            # 生成可视化颜色（与原逻辑保持一致）
            if self.depth_grayscale:
                depth_color = np.repeat(depth_resized[..., np.newaxis], 3, axis=-1)
            else:
                depth_color = (self.depth_cmap(depth_resized)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)

            # 输出深度可视化：仅在有效掩码处写入颜色，其他区域保持黑色（或可改为其它背景）
            depth_vis = np.zeros((h, w, 3), dtype=np.uint8)
            mask3 = crop_mask.astype(bool)
            depth_vis[mask3] = depth_color[mask3]

            return depth_vis

        except Exception as e:
            print(f"深度推理错误：{e}")
            return None

    def crop_to_circle(self, image, fps=None, show_crosshair=True, show_circles=True, red_circles=None):
        """
        将图像裁剪为圆形区域，黑边尽量少，圆形居中显示
        同时在圆形中心绘制十字准星，左上角显示帧率，右下角显示四个圆
        red_circles: 要变红的圆圈索引列表，如 [0, 2] 表示第0和第2个圆变红
        """
        try:
            h, w = image.shape[:2]
            # 圆形直径 = 高度（高度是固定的）
            diameter = h
            radius = diameter // 2
            
            # 圆心在图像中心
            center_x = w // 2
            center_y = h // 2
            
            # 计算裁剪区域：从圆心向左右各radius距离，确保圆形居中
            left = max(0, center_x - radius)
            right = min(w, center_x + radius)
            
            # 裁剪矩形区域
            cropped = image[:, left:right]
            
            # 创建圆形掩码
            ch, cw = cropped.shape[:2]
            
            # 如果宽度不足（宽 < 高），需要在周围补黑色以保持正方形
            if cw < ch:
                padding = (ch - cw) // 2
                # 创建黑色背景
                result = np.zeros((ch, ch, 3), dtype=np.uint8)
                # 将裁剪的内容放在中央
                result[:, padding:padding+cw] = cropped
                cw = ch  # 更新宽度为高度，保持正方形
                circle_center_x = ch // 2
            else:
                result = cropped.copy()
                circle_center_x = cw // 2
            
            circle_center_y = ch // 2
            circle_radius = radius
            
            # 创建圆形掩码
            mask = np.zeros((ch, cw), dtype=np.uint8)
            cv2.circle(mask, (circle_center_x, circle_center_y), circle_radius, 255, -1)
            
            # 应用掩码（黑边）
            result[mask == 0] = 0
            
            # 在圆形中心绘制十字准星
            if show_crosshair:
                cross_length = 20
                cv2.line(result, (circle_center_x - cross_length, circle_center_y), 
                        (circle_center_x + cross_length, circle_center_y), (0, 0, 255), 2, cv2.LINE_AA)
                cv2.line(result, (circle_center_x, circle_center_y - cross_length), 
                        (circle_center_x, circle_center_y + cross_length), (0, 0, 255), 2, cv2.LINE_AA)
            
            # 左上角显示帧率
            if fps is not None:
                cv2.putText(result, "FPS:%.2f" % (fps), (0, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.putText(result, "BIOMACH", (0,result.shape[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)               
            
            # 右下角显示四个圆
            if show_circles:
                circle_centers = [(cw - 40, ch - 20),
                                 (cw - 20, ch - 40),
                                 (cw - 60, ch - 40),
                                 (cw - 40, ch - 60)]
                circle_radius_small = 8
                
                # 初始化所有圆为绿色
                for idx, center in enumerate(circle_centers):
                    # 检查该圆是否应该变红
                    if red_circles and idx in red_circles:
                        cv2.circle(result, center, circle_radius_small, (0, 0, 255), -1, cv2.LINE_AA)
                    else:
                        cv2.circle(result, center, circle_radius_small, (0, 255, 0), -1, cv2.LINE_AA)
            
            return result
        except Exception as e:
            print(f'crop_to_circle error: {e}')
            return image

    # ---------------- 深度推理相关函数 ----------------
    def extract_circular_roi(self, image, center_x, center_y, radius):
        """
        仅提取圆形内的有效像素（彻底去掉黑边），返回纯圆形区域的图像
        :param image: 已拉伸的输入图像
        :param center_x/y: 圆心坐标（拉伸后画面）
        :param radius: 圆半径
        :return:
            - roi_image: 纯圆形有效像素组成的图像（无黑边，尺寸=2*radius×2*radius）
            - mask: 圆形掩码（用于后续还原）
        """
        # 1. 创建圆形掩码
        mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        cv2.circle(mask, (center_x, center_y), radius, 255, -1)

        # 2. 裁剪出圆形所在的正方形区域（仅包含圆形的最小正方形）
        x1 = max(0, center_x - radius)
        y1 = max(0, center_y - radius)
        x2 = min(image.shape[1], center_x + radius)
        y2 = min(image.shape[0], center_y + radius)
        crop_image = image[y1:y2, x1:x2]
        crop_mask = mask[y1:y2, x1:x2]  # 裁剪后的掩码

        # 3. 仅保留掩码内的有效像素（黑边设为中性灰，避免干扰）
        roi_image = np.copy(crop_image)
        roi_image[crop_mask == 0] = 128  # 中性灰（模型对中性灰无响应）

        return roi_image, crop_mask

    def resize_with_padding(self, image, target_ratio=16/9):
        """强制宽高比处理函数"""
        orig_height, orig_width = image.shape[:2]
        orig_ratio = orig_width / orig_height

        if orig_ratio > target_ratio:
            new_height = orig_height
            new_width = int(new_height * target_ratio)
            resized = cv2.resize(image, (new_width, new_height))
            pad_width = (orig_width - new_width) // 2
            padded = cv2.copyMakeBorder(resized, 0, 0, pad_width, pad_width, cv2.BORDER_CONSTANT, value=[0, 0, 0])
        else:
            new_width = orig_width
            new_height = int(new_width / target_ratio)
            resized = cv2.resize(image, (new_width, new_height))
            pad_height = (orig_height - new_height) // 2
            padded = cv2.copyMakeBorder(resized, pad_height, pad_height, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])

        return padded

    def resize_with_padding(self, image, target_ratio=16/9):
        """强制宽高比处理函数"""
        orig_height, orig_width = image.shape[:2]
        orig_ratio = orig_width / orig_height

        if orig_ratio > target_ratio:
            new_height = orig_height
            new_width = int(new_height * target_ratio)
            resized = cv2.resize(image, (new_width, new_height))
            pad_width = (orig_width - new_width) // 2
            padded = cv2.copyMakeBorder(resized, 0, 0, pad_width, pad_width, cv2.BORDER_CONSTANT, value=[0, 0, 0])
        else:
            new_width = orig_width
            new_height = int(new_width / target_ratio)
            resized = cv2.resize(image, (new_width, new_height))
            pad_height = (orig_height - new_height) // 2
            padded = cv2.copyMakeBorder(resized, pad_height, pad_height, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])

        return padded

    def predict(self, image_path):
        try:
            image = Image.open(image_path)
        except Exception as e:
            print(f'Open Error! {e}')
            return None
        else:
            return self.unet.detect_image(image, count=self.count, name_classes=self.name_classes)
        
    # 计算线性插值的比例因子
    def linear_interpolation(self, value, low, high, low_speed, high_speed):
        if abs(value) <= abs(low):
            return low_speed
        elif  abs(value) >= abs(high):
            return high_speed
        else:
            # print('runnning linear_interpolation')
            return low_speed + (high_speed - low_speed) * (value - low) / (high - low)    
  
    def video(self):
        lower_green0 = np.array([76, 46, 28])   #导管颜色hsv-unet 青
        upper_green0 = np.array([98, 255, 255]) 
        lower_green = np.array([98, 157, 88])   #结石颜色hsv-unet  蓝
        upper_green = np.array([125, 255, 255]) 

        capture = cv2.VideoCapture(self.video_path)
        if self.video_save_path != "":
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            out = cv2.VideoWriter(self.video_save_path, fourcc, self.video_fps, size)

        ref, frame = capture.read()
        if not ref:
            raise ValueError("未能正确读取摄像头（视频），请注意是否正确安装摄像头（是否正确填写视频路径）。")

        fps = 0.0
        while True:
            t1 = time.time()
            ref, frame = capture.read()
            
            if not ref:
                break
            
            # ===== 第一步：对原始frame进行宽度放大并取右侧部分 =====
            try:
                h, w = frame.shape[:2]
                # 宽度放大系数：1.6倍（从4/3改为1.6，扩大裁剪范围，裁掉更多干扰）
                new_w = int(w * 1.43)
                # 按新宽度拉伸（水平拉伸），然后裁切右侧原始宽度的区域
                scaled = cv2.resize(frame, (new_w, h), interpolation=cv2.INTER_LINEAR)
                frame_cropped_raw = scaled[:, new_w - w:new_w]  # 裁剪后但未圆形处理的原始帧
            except Exception as e:
                print(f"Frame cropping error: {e}")
                frame_cropped_raw = frame
            
            # ===== 第二步：显示Original窗口（圆形版本，不显示十字和圆圈）=====
            try:
                circular = self.crop_to_circle(frame_cropped_raw, show_crosshair=False, show_circles=False)
                # cv2.imshow('Original', circular)  # 注释掉单独显示
            except Exception:
                circular = frame_cropped_raw
                # cv2.imshow('Original', frame_cropped_raw)  # 注释掉单独显示

            # ===== 深度推理：使用Original circular图像帧作为输入 =====
            depth_display = None
            if self.enable_depth:
                try:
                    # 使用圆形裁剪后的图像作为深度推理输入
                    depth_result = self.infer_depth(circular)
                    if depth_result is not None:
                        # 调整深度图尺寸以匹配原始图像尺寸
                        depth_display = cv2.resize(depth_result, (circular.shape[1], circular.shape[0]))
                        # cv2.imshow('Depth', depth_display)  # 注释掉单独显示
                except Exception as e:
                    print(f"深度推理显示错误：{e}")

            # ===== 第三步：对裁剪后的原始帧进行unet目标检测 =====
            # 为了与深度推理保持一致，也对输入进行预处理：将黑色背景设为中性灰
            frame_rgb = cv2.cvtColor(frame_cropped_raw, cv2.COLOR_BGR2RGB)
            h, w = frame_rgb.shape[:2]
            # 创建圆形掩码，将圆形外的区域设为中性灰
            center_x, center_y = w // 2, h // 2
            radius = min(center_x, center_y, w - center_x, h - center_y)  # 确保圆形不超出边界
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(mask, (center_x, center_y), radius, 255, -1)

            # 生成预处理图像：圆形外的区域设为中性灰（128）
            roi_image = frame_rgb.copy()
            roi_image[mask == 0] = 128

            frame = Image.fromarray(np.uint8(roi_image))
            frame = np.array(self.unet.detect_image(frame))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            #

            fps  = (fps + (1./(time.time()-t1))) / 2
            # print("fps= %.2f" % (fps))
            #将输入帧从BGR颜色空间转换为HSV颜色空间。
            # 使用两个不同的绿色范围创建两个掩码。
            # 对每个掩码分别查找轮廓。
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, lower_green0, upper_green0)
            contours1, hierarchy1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            mask2 = cv2.inRange(hsv, lower_green, upper_green)
            contours2, hierarchy2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            #获取图像帧的高度和宽度。
            # 计算图像帧的总像素数。#
            frame_height, frame_width = frame.shape[:2]
            total_pixels = frame_width * frame_height
 
            # 初始化percentage1，确保在所有代码路径中都有定义
            percentage1 = 0.0

            #寻找导管最大的轮廓，并绘制最右侧的绿色圆圈
            # A: 初始化最大面积 max0 和索引 max_num0。
            # B: 判断轮廓列表是否为空。
            # C: 遍历轮廓列表。
            # D: 判断当前轮廓面积是否大于最大面积。
            # F: 更新最大面积和索引。
            # G: 找到最大面积轮廓的最右点。
            # H: 在图像上绘制绿色圆圈标记最右点。
            # I: 在图像上绘制最大面积轮廓。
            # E: 结束。
            max0 = 0.0
            max_num0 = 0
            if len(contours1) > 0:
                for i in range(len(contours1)):
                    c = cv2.contourArea(contours1[i])
                    if c > max0:
                        max0 = c
                        max_num0 = i

                rightmost_point = tuple(contours1[max_num0][contours1[max_num0][:, :, 0].argmax()][0])
                dx = rightmost_point[0]
                dy = rightmost_point[1]
                radius = 15
                #绘制导管右侧的圆圈
                # frame = cv2.circle(frame, (dx, dy), radius, (0, 255, 0), 2)
                result = cv2.drawContours(frame, contours1[max_num0], -1, (255, 160, 0), 2)

            #寻找结石最大的轮廓，并绘制质心处的红色实心圆圈
            # A: 初始化最大面积 max0 和索引 max_num0。
            # B: 判断轮廓列表是否为空。
            # C: 遍历轮廓列表。
            # D: 判断当前轮廓面积是否大于最大面积。
            # F: 更新最大面积和索引。
            # G: 找到最大面积轮廓的质心。
            # H: 在图像上绘制红色圆圈标记。
            # I: 在图像上绘制最大面积轮廓。
            # E: 结束。
            max = 0.0
            max_num = 0
            # 设置最小有效轮廓面积阈值，避免检测到噪声
            min_contour_area = 10.0  # 可以根据需要调整

            if len(contours2) > 0:
                for i in range(len(contours2)):
                    c = cv2.contourArea(contours2[i])
                    if c > max:
                        max = c
                        max_num = i

                # 只有当最大轮廓面积超过阈值时才认为检测到有效目标
                if max > min_contour_area:
                    moments = cv2.moments(contours2[max_num])
                    if moments['m00'] != 0:
                        cx = int(moments['m10'] / moments['m00'])
                        cy = int(moments['m01'] / moments['m00'])
                        # 绘制最大轮廓
                        result = cv2.drawContours(frame, contours2[max_num], -1, (255, 255, 0), 3, cv2.LINE_AA)
                        #绘制质心#
                # cv2.circle(img, center, radius, color, thickness=None, lineType=None, shift=None)
                #                 各参数的具体说明如下：
                # img: 输入图像，也是输出图像（即在原图上直接绘制）。它是一个 NumPy 数组。
                # center: 圆心的坐标，格式为 (x, y)，其中 x 和 y 是圆心的像素位置。
                # radius: 圆的半径，单位是像素。
                # color: 圆的颜色，以 BGR（蓝、绿、红）格式表示。例如 (0, 0, 255) 表示红色。
                # thickness (可选): 线条的粗细。如果为正值，则表示线条的宽度；如果为负值（如 -1），则表示填充圆形。默认值为 1。
                # lineType (可选): 线条类型。可以选择以下几种：
                # cv2.LINE_4: 4 连通线（默认）
                # cv2.LINE_8: 8 连通线
                # cv2.LINE_AA: 抗锯齿线（更平滑）
                # shift (可选): 圆心坐标和半径的小数位数。通常不需要设置，默认值为 0。

                # frame = cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1, cv2.LINE_AA) #红色圆点
                        # 在轮廓质心位置绘制质心标记（中心圆）
                        frame = cv2.circle(frame, (cx, cy), 20, (255, 255, 0), -1, cv2.LINE_AA)#中心圆
                    else:
                        # 如果轮廓面积为零，可以设置默认值或跳过处理
                        # cx, cy = 0, 0  # 或者其他合理的默认值
                        # print('cx, cy = 中心 ')
                         # 获取图像帧的宽度和高度
                        height, width = frame.shape[:2]
                         # 计算屏幕中心点（用于后续的目标跟踪计算）
                        center_x, center_y = width // 2, height // 2
                        cx = center_x
                        cy = center_y

                        # 计算屏幕中心点（用于后续的目标跟踪计算）
                        center_x, center_y = width // 2, height // 2
                        cx = center_x
                        cy = center_y
                else:
                    # 轮廓面积太小，认为没有有效目标
                    # print('cx, cy = 中心 (轮廓面积太小)')
                    # 获取图像帧的宽度和高度
                    height, width = frame.shape[:2]

                    # 计算屏幕中心点（用于后续的目标跟踪计算）
                    center_x, center_y = width // 2, height // 2
                    cx = center_x
                    cy = center_y
            else:
                # 没有找到任何轮廓
                # print('cx, cy = 中心 (无轮廓)')
                # 获取图像帧的宽度和高度
                height, width = frame.shape[:2]

                # 计算屏幕中心点（用于后续的目标跟踪计算）
                center_x, center_y = width // 2, height // 2
                # 将质心设置为屏幕中心（无目标时使用中心坐标）
                cx = center_x
                cy = center_y



                text_position = (0, frame.shape[1]-30)
                if percentage1 < 30:
                    cv2.putText(frame, "F", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                elif percentage1 > 60:
                    cv2.putText(frame, "R", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                else:
                    cv2.putText(frame, "IN FOCUS", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

            # cv2.putText(frame, "BIOMACH", (0,frame.shape[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            # frame = cv2.putText(frame, "FPS:%.2f" % (fps), (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)  # FPS将在crop_to_circle中绘制
            
            # 获取图像帧的宽度和高度
            height, width = frame.shape[:2]

            # 计算屏幕中心点（用于后续的目标跟踪计算）
            center_x, center_y = width // 2, height // 2

            # 十字准星、FPS和四个圆现在在crop_to_circle中处理，不需要在这里绘制
            
            # 初始化圆圈变红的标志（如果没有检测到目标则都是False）
            red_left = False
            red_right = False
            red_up = False
            red_down = False

            # if len(contours1) > 0 and len(contours2) > 0:
            if  len(contours2) > 0:
                #相对于激光管最右侧点判断移动方向：
                # Dx = dx - cx
                # Dy = dy - cy
                #相对于屏幕中心点判断移动方向：
                Dx = center_x - cx
                Dy = center_y - cy
                # print('Dx:',Dx,'Dy:',Dy)

                # 在检测到目标后立即设置标志，确保没有目标时保持False
                PixelR = 10 #红圆精准度
                red_left = Dx > PixelR
                red_right = Dx < -PixelR
                red_up = Dy < -PixelR
                red_down = Dy > PixelR

                # 把图像处理的值赋给机器人,线性插值法
                # percentage1,(Dx，Dy) to speed_forward, speed_turn=[vx, vy] 其中vx, vy<=100
                FC = FORWARD_COEFF
                
                if 0 <= percentage1 <= 30:
                    speed_FR = FC - (FC / 30) * percentage1
                    # print(f"{get_time()}-speed_FR为+")
                    # return speed_FR  
                if 60 <= percentage1 <= 100:
                    speed_FR = - (FC / 40) * (percentage1 - 60)
                    # return speed_FR 
                    # print(f"{get_time()}-speed_FR为-")
                if 30 < percentage1 < 60:
                    speed_FR=0
                    # print(f"{get_time()}-speed_FR为0")       

                if abs(speed_FR) > FC:
                            speed_FR = FC if speed_FR > 0 else -FC
                # 安全机制
                # if abs(speed_forward) > TC:
                #             speed_forward = FC if speed_forward > 0 else -FC 
                TC = TURN_COEFF
                TC_S = TURN_COEFF_SLOW

                Multiple = 3.0 #在几倍精度像素数中变为慢速

                vx = 0
                vy = 0
                #Vx, Vy 速度范围限定在-50~+50，-0.83~+0.83之间

                # 重新定义circle_centers和circle_radius用于着色反馈
                circle_centers = [(frame.shape[1] - 40, frame.shape[0] - 20),
                                 (frame.shape[1] - 20, frame.shape[0] - 40),
                                 (frame.shape[1] - 60, frame.shape[0] - 40),
                                 (frame.shape[1] - 40, frame.shape[0] - 60)]
                circle_radius = 8

                # X方向控制 (左右移动) - 基于PixelR和abs(Dx)的分段线性映射
                vx = 0.0
                M_x= 4.5 # 最大倍数，当abs(Dx) >= M_x * PixelR时达到最大速度
                # vx_max = 50.0
                vx_max = 20.0

                if red_left:
                    abs_dx = abs(Dx)
                    if abs_dx < PixelR:
                        vx = 0.0  # 死区内无响应
                    elif abs_dx < M_x * PixelR:
                        # 线性插值：从PixelR到M_x*PixelR，速度从-5增加到-50.0
                        vx = -5.0 + (-vx_max) * (abs_dx - PixelR) / ((M_x - 1) * PixelR)
                    else:
                        vx = -vx_max  # 达到最大速度

                if red_right:
                    abs_dx = abs(Dx)
                    if abs_dx < PixelR:
                        vx = 0.0  # 死区内无响应
                    elif abs_dx < M_x * PixelR:
                        # 线性插值：从PixelR到M*PixelR，速度从5增加到50.0
                        vx = 5 + (vx_max) * (abs_dx - PixelR) / ((M_x - 1) * PixelR)
                    else:
                        vx = vx_max  # 达到最大速度

                # Y方向控制 (上下移动) - 基于PixelR和abs(Dy)的分段线性映射，取消换向检测
                vy = 0.0
                M_y = 8.0  # 最大倍数，当abs(Dy) >= M_y * PixelR时达到最大速度
                # vy_max = 0.83
                vy_max = 0.6

                if red_up:
                    abs_dy = abs(Dy)
                    if abs_dy < PixelR:
                        vy = 0.0  # 死区内无响应
                    elif abs_dy < M_y * PixelR:
                        # 线性插值：从PixelR到M_y*PixelR，速度从0增加到-0.83
                        vy = -0.05 - vy_max * (abs_dy - PixelR) / ((M_y - 1) * PixelR)
                    else:
                        vy = -vy_max  # 达到最大速度

                if red_down:
                    abs_dy = abs(Dy)
                    if abs_dy < PixelR:
                        vy = 0.0  # 死区内无响应
                    elif abs_dy < M_y * PixelR:
                        # 线性插值：从PixelR到M_y*PixelR，速度从0增加到0.83
                        vy = 0.05 + vy_max * (abs_dy - PixelR) / ((M_y - 1) * PixelR)
                    else:
                        vy = vy_max  # 达到最大速度



                # if (-PixelR <= Dx ) and (Dx <= PixelR ):
                #     vx= 0
                #     # print("vx= 0")
                # if (-PixelR <= Dy ) and (Dy <= PixelR ):  
                #     vy= 0  
                #     # print("vy= 0")
                


                # global speed_pf
                # global speed_pt

                # speed_pf = speed_FR
                speed_pt = [vx, vy]

                # speed_pf = get_config('speed_pf')
                # speed_pt = get_config('speed_pt') 
               

                # print(f"video中赋值：Speed_pf: {speed_FR}, Speed_pt: {[vx, vy]}")
                update_config('speed_pf', speed_FR)  
                update_config('speed_pt', speed_pt)
                # print(f"{get_time()}-图像计算中值已更新为:")
                # print('speed_pf',speed_FR)   
                # print('speed_pt',speed_pt)

                
                # speed_pf = get_config('speed_pf')  
                # speed_pt = get_config('speed_pt') 
                # print("获取config中的值-更新后")
                # print(speed_pf)  
                # print(speed_pt)     
               
                # speed_forward1, speed_turn1 = self.map_pic_values_to_motion_values()
                # print(f"{get_time()}-mainV2已获取:")
                # print(f"给电机的值Speed Forward: {speed_forward1}, Speed Turn: {speed_turn1}")


            overlay = np.zeros_like(frame, dtype=np.uint8)
            cv2.drawContours(overlay, contours2, max_num, (0, 255, 0), -1, cv2.LINE_AA)
            alpha = 0.2
            frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

            # 收集需要变红的圆圈索引 (根据之前的red_left, red_right, red_up, red_down)
            red_circle_indices = []
            if red_left:
                red_circle_indices.append(2)  # 左
            if red_right:
                red_circle_indices.append(1)  # 右
            if red_up:
                red_circle_indices.append(0)  # 上
            if red_down:
                red_circle_indices.append(3)  # 下

            # 将 video 画面裁剪为圆形（直接使用已裁剪的frame，不需要再裁剪）
            try:
                # 裁剪为圆形，黑边尽量少，并绘制十字准星、FPS、四个圆和红色反馈
                circular_frame = self.crop_to_circle(frame, fps=fps, show_crosshair=True, show_circles=True, red_circles=red_circle_indices)
                # cv2.imshow("video", circular_frame)  # 注释掉单独显示
            except Exception as e:
                # 如果出错则直接显示原始帧
                print(f"Error in video display: {e}")
                circular_frame = frame

            # ===== 拼接三个窗口为一个大窗口 =====
            try:
                # 获取图像尺寸
                h, w = circular.shape[:2]

                # 如果深度不可用，创建一个与circular相同大小的黑色图像作为占位符
                if depth_display is None:
                    depth_display = np.zeros((h, w, 3), dtype=np.uint8)

                # 创建大画布，宽度为三个图像宽度之和，高度为单个图像高度
                combined_width = w * 3
                combined_height = h
                combined_image = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)

                # 将三个图像按左中右顺序拼接
                combined_image[:, 0:w] = circular  # Original (左)
                combined_image[:, w:2*w] = circular_frame  # Video (中)
                combined_image[:, 2*w:3*w] = depth_display  # Depth (右)

                # 显示拼接后的窗口
                cv2.imshow("Combined View: Original | Video | Depth", combined_image)

            except Exception as e:
                print(f"拼接窗口显示错误：{e}")
                # 如果拼接失败，显示原来的video窗口
                cv2.imshow("video", circular_frame)
            if self.video_save_path != "":
                out.write(frame)

            c = cv2.waitKey(1) & 0xff
            if c == 27:
                capture.release()
                break
        capture.release()
        if self.video_save_path != "":
            out.release()
        cv2.destroyAllWindows()       

    # def map_pic_values_to_motion_values(self):
    #     print(f"{get_time()}-正在映射赋值map_pic_values_to_motion_values") 
    #     print(f"返回的值：Speed Forward: {speed_pf}, Speed Turn: {speed_pt}") 
    #     return speed_pf, speed_pt

    def fps(self):
        t1 = time.time()
        img = Image.open(self.fps_image_path)
        tact_time = self.unet.get_FPS(img, self.test_interval)
        print(str(tact_time) + ' seconds, ' + str(1/tact_time) + 'FPS, @batch_size 1')
        t2 = time.time()
        print('FPS:', 1 / (t2 - t1))

    def directory(self):
        from utils.utils import detect_directory
        detect_directory(self.unet, self.dir_origin_path, self.dir_save_path, self.count, self.name_classes)

    def export_onnx(self):
        self.unet.convert_to_onnx(self.simplify, self.onnx_save_path)

    def run(self):
        if self.mode == "predict":
            self.predict()
        elif self.mode == "video":
            self.video()
        elif self.mode == "fps":
            self.fps()
        elif self.mode == "dir_predict":
            self.directory()
        elif self.mode == "predict_onnx":
            self.predict_onnx()
        elif self.mode == "export_onnx":
            self.export_onnx()
        else:
            raise ValueError("Please specify a correct mode.")  


if __name__ == "__main__":

    # # cProfile.run('main()')
    # #-------------------------------------------------------------------------#
    # #   如果想要修改对应种类的颜色，到__init__函数里修改self.colors即可
    # #-------------------------------------------------------------------------#
    # 检查深度推理功能可用性
    if DEPTH_AVAILABLE:
        print("Depth-Anything-V2 已成功导入，深度推理功能可用")
    else:
        print("Depth-Anything-V2 导入失败，深度推理功能将被禁用")
    # #----------------------------------------------------------------------------------------------------------#
    # #   mode用于指定测试的模式：
    # #   'predict'           表示单张图片预测，如果想对预测过程进行修改，如保存图片，截取对象等，可以先看下方详细的注释
    # #   'video'             表示视频检测，可调用摄像头或者视频进行检测，详情查看下方注释。
    # #   'fps'               表示测试fps，使用的图片是img里面的street.jpg，详情查看下方注释。
    # #   'dir_predict'       表示遍历文件夹进行检测并保存。默认遍历img文件夹，保存img_out文件夹，详情查看下方注释。
    # #   'export_onnx'       表示将模型导出为onnx，需要pytorch1.7.1以上。
    # #   'predict_onnx'      表示利用导出的onnx模型进行预测，相关参数的修改在unet.py_346行左右处的Unet_ONNX
    # #----------------------------------------------------------------------------------------------------------#
    # mode = "video"
    # #-------------------------------------------------------------------------#
    # #   count               指定了是否进行目标的像素点计数（即面积）与比例计算
    # #   name_classes        区分的种类，和json_to_dataset里面的一样，用于打印种类和数量
    # #
    # #   count、name_classes仅在mode='predict'时有效
    # #-------------------------------------------------------------------------#
    # count           = False
    # # name_classes    = ["background","aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
    # # name_classes    = ["background","cat","dog"]
    # name_classes    = ["LaserTube","Large"]
    # #----------------------------------------------------------------------------------------------------------#
    # #   video_path          用于指定视频的路径，当video_path=0时表示检测摄像头
    # #                       想要检测视频，则设置如video_path = "xxx.mp4"即可，代表读取出根目录下的xxx.mp4文件。
    # #   video_save_path     表示视频保存的路径，当video_save_path=""时表示不保存
    # #                       想要保存视频，则设置如video_save_path = "yyy.mp4"即可，代表保存为根目录下的yyy.mp4文件。
    # #   video_fps           用于保存的视频的fps
    # #
    # #   video_path、video_save_path和video_fps仅在mode='video'时有效
    # #   保存视频时需要ctrl+c退出或者运行到最后一帧才会完成完整的保存步骤。
    # #----------------------------------------------------------------------------------------------------------#
    # video_path= 2
    # # video_path      = r"F:\Napoleon\videodemo6.18-Subtitles.mp4"
    # # video_path      = r"F:\Napoleon\demo6.18-480.mp4"
    # # video_path      = r"assets\demo6.18-480.mp4"
    # # video_path      = r"assets\5.10.6.mp4"

    # video_save_path = ""
    # # video_save_path = r"F:\Napoleon\result.videodemo6.18-Subtitles.mp4"
    # # video_save_path = r"F:\Napoleon\result.demo6.18-480.mp4"
    # # video_save_path = r"F:\F:\unet-pytorch-main\ZQ\videoresults\5.10.6.mp4"
    
    # video_fps       = 30
    # #----------------------------------------------------------------------------------------------------------#
    # #   test_interval       用于指定测量fps的时候，图片检测的次数。理论上test_interval越大，fps越准确。
    # #   fps_image_path      用于指定测试的fps图片
    # #   
    # #   test_interval和fps_image_path仅在mode='fps'有效
    # #----------------------------------------------------------------------------------------------------------#
    # test_interval = 1000
    # fps_image_path  = "img/street.jpg"
    # #-------------------------------------------------------------------------#
    # #   dir_origin_path     指定了用于检测的图片的文件夹路径
    # #   dir_save_path       指定了检测完图片的保存路径
    # #   
    # #   dir_origin_path和dir_save_path仅在mode='dir_predict'时有效
    # #-------------------------------------------------------------------------#
    # dir_origin_path = "img/"
    # dir_save_path   = "img_out/"
    # #-------------------------------------------------------------------------#
    # #   simplify            使用Simplify onnx
    # #   onnx_save_path      指定了onnx的保存路径
    # #-------------------------------------------------------------------------#
    # simplify        = True
    # onnx_save_path  = "model_data/models.onnx"

    # if mode != "predict_onnx":
    #     unet = Unet()
    # else:
    #     yolo = Unet_ONNX()
    #     # 这里命名上的“yolo”是为了表示这是一个与ONNX相关的处理部分，
    #     # 它应该是与ONNX模型预测相关的自定义类Unet_ONNX的实例，用于处理或执行基于ONNX格式的Unet模型的推理任务。

    # # if mode == "predict":
    # #     '''
    # #     predict.py有几个注意点
    # #     1、该代码无法直接进行批量预测，如果想要批量预测，可以利用os.listdir()遍历文件夹，利用Image.open打开图片文件进行预测。
    # #     具体流程可以参考get_miou_prediction.py，在get_miou_prediction.py即实现了遍历。
    # #     2、如果想要保存，利用r_image.save("img.jpg")即可保存。
    # #     3、如果想要原图和分割图不混合，可以把blend参数设置成False。
    # #     4、如果想根据mask获取对应的区域，可以参考detect_image函数中，利用预测结果绘图的部分，判断每一个像素点的种类，然后根据种类获取对应的部分。
    # #     seg_img = np.zeros((np.shape(pr)[0],np.shape(pr)[1],3))
    # #     for c in range(self.num_classes):
    # #         seg_img[:, :, 0] += ((pr == c)*( self.colors[c][0] )).astype('uint8')
    # #         seg_img[:, :, 1] += ((pr == c)*( self.colors[c][1] )).astype('uint8')
    # #         seg_img[:, :, 2] += ((pr == c)*( self.colors[c][2] )).astype('uint8')
    # #     '''
    # #     while True:
    # #         img = input('Input image filename:')
    # #         try:
    # #             image = Image.open(img)
    # #         except:
    # #             print('Open Error! Try again!')
    # #             continue
    # #         else:
    # #             r_image = unet.detect_image(image, count=count, name_classes=name_classes)
    # #             r_image.show()

    # if mode == "video":

    #     # lower_green0 = np.array([35,23,76])   #导管颜色hsv-unet green
    #     # upper_green0 = np.array([70,213,177]) 

    #     # lower_green = np.array([0,144,57])   #结石颜色hsv-unet  red
    #     # upper_green = np.array([7,242,255]) 

    #     lower_green0 = np.array([76,46,28])   #导管颜色hsv-unet 青
    #     upper_green0 = np.array([98,255,255]) 

    #     lower_green = np.array([98,157,88])   #结石颜色hsv-unet  蓝
    #     upper_green = np.array([125,255,255]) 


    #     capture=cv2.VideoCapture(video_path)
    #     if video_save_path!="":
    #         fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #         size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    #         out = cv2.VideoWriter(video_save_path, fourcc, video_fps, size)

    #     ref, frame = capture.read()
    #     if not ref:
    #         raise ValueError("未能正确读取摄像头（视频），请注意是否正确安装摄像头（是否正确填写视频路径）。")

    #     fps = 0.0
    #     while(True):
    #         t1 = time.time()
    #         # 读取某一帧
    #         ref, frame = capture.read()
    #         if not ref:
    #             break
    #         cv2.imshow('Original', frame)  #正常读取显示图像帧

    #         # 格式转变，BGRtoRGB
    #         frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    #         # 转变成Image
    #         frame = Image.fromarray(np.uint8(frame))
    #         # 进行检测
    #         frame = np.array(unet.detect_image(frame))
    #         # RGBtoBGR满足opencv显示格式
    #         frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
            
    #         fps  = ( fps + (1./(time.time()-t1)) ) / 2
    #         print("fps= %.2f"%(fps))
    #         # frame = cv2.putText(frame, "fps= %.2f"%(fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
    #         # cv2.imshow("video",frame) unets预测显示窗口

    #         ##############  trackingdemo3.1

    #         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
    #         # 导管的掩码处理
    #         mask1 = cv2.inRange(hsv, lower_green0, upper_green0)
    #         contours1, hierarchy1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
            

    #         # 结石的掩码处理
    #         mask2 = cv2.inRange(hsv, lower_green, upper_green)
    #         contours2, hierarchy2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
    #         # mask = cv2.inRange(hsv, lower_green, upper_green)
    #         # contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #          # 获取视频帧的宽度和高度
    #         frame_height, frame_width = frame.shape[:2]
    #         # 计算整个画面的像素总数
    #         total_pixels = frame_width * frame_height
        
    #         #找最大面积的那个-导管
    #         max0 = 0.0
    #         max_num0 = 0    
    #         if len(contours1) > 0:       # 找到面积最大的轮廓
    #             for i in range(len(contours1)):
    #                 c = cv2.contourArea(contours1[i])
    #                 if c > max0:
    #                     max0 = c
    #                     max_num0 = i
        
    #         # M = cv2.moments(contours1[max_num0])#中点
    #         # cx = int(M['m10']/M['m00'])#这两行是计算中心点坐标
    #         # cy = int(M['m01']/M['m00'])
    #         # # print((cx, cy))
    #         # frame = cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)
    #         # result=cv2.drawContours(frame, contours1[max_num0], -1, (0,255,0), 3)#最终图像
            
    #             # 找到导管轮廓中最右侧的点
    #             rightmost_point = tuple(contours1[max_num0][contours1[max_num0][:, :, 0].argmax()][0])
    #             # 使用最右侧点的坐标作为中心点坐标
    #             dx = rightmost_point[0]
    #             dy = rightmost_point[1]
    #             # 修改圆的半径（假设半径为10）
    #             radius = 15
    #             # 绘制圆
    #             frame = cv2.circle(frame, (dx, dy), radius, (0, 255, 0), 2)
    #             result=cv2.drawContours(frame, contours1[max_num0], -1, (255,160,0), 2)#最终图像

    #             # # 计算绿色轮廓包围的像素面积
    #             # gre_area = cv2.contourArea(contours1[max_num0])
    #             # # 计算绿色轮廓包围的像素面积占整个画面面积的百分比
    #             # percentage2 = (gre_area / total_pixels) * 100
    #             # # 在左上角显示百分比
    #             # cv2.putText(frame, f'{percentage2:.2f}%', (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #         # else:
    #         #     cv2.putText(frame, "No catheter", (0, 445), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    #         #找最大面积的那个-结石
    #         max = 0.0
    #         max_num = 0    
    #         if len(contours2) > 0:       # 找到面积最大的轮廓
    #             for i in range(len(contours2)):
    #                 c = cv2.contourArea(contours2[i])
    #                 if c > max:
    #                     max = c
    #                     max_num = i
            
    #             M = cv2.moments(contours2[max_num])#中点
    #             cx = int(M['m10']/M['m00'])#这两行是计算中心点坐标
    #             cy = int(M['m01']/M['m00'])
    #             # print((cx, cy))
    #             frame = cv2.circle(frame, (cx, cy), 3, (0,0,255), -1)
    #             result=cv2.drawContours(frame, contours2[max_num], -1, (255,255,0), 3)#最终图像

    #             # 计算红色轮廓包围的像素面积
    #             red_area = cv2.contourArea(contours2[max_num])
    #             # 计算红色轮廓包围的像素面积占整个画面面积的百分比
    #             percentage1 = (red_area / total_pixels) * 100
    #             # 在左下角显示百分比
    #             cv2.putText(frame, f'{percentage1:.2f}%', (0, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    #             # 判断百分比值并设置要显示的文本
    #             # 设置文本在图像上的位置（左下角）  
    #             # 假设图像宽度为width，高度为height，你可能需要替换这些变量为实际的宽度和高度值  
    #             # 或者使用其他方法来确定文本的位置  
    #             text_position = (0, 445)  # 假设(10, height-10)是左下角附近的一个位置  
    #             if percentage1 < 30:  
    #                 cv2.putText(frame, "F", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) 
    #             elif percentage1 > 60:  
    #                 cv2.putText(frame, "R", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)  
    #             elif 30 <= percentage1 <= 60:  
    #                 cv2.putText(frame, "IN FOCUS", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    #         # else:
    #         #     cv2.putText(frame, "No Target", (0, 445), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2) 

    #         cv2.putText(frame, " BIOM", (0, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)      
    #         # 在左上角显示fps
    #         frame = cv2.putText(frame, "FPS:%.2f"%(fps), (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    #         # 定义四个圆的中心点和半径  
    #         circle_centers = [(frame.shape[1] - 40, frame.shape[0] - 20),  # 右下角第一个圆  下
    #                         (frame.shape[1] - 20, frame.shape[0] - 40),  # 右下角第二个圆    右
    #                         (frame.shape[1] - 60, frame.shape[0] - 40),  # 右下角第三个圆    左
    #                         (frame.shape[1] - 40, frame.shape[0] - 60)]  # 右下角第四个圆    上
    #         circle_radius = 8  

    #         # 绘制四个绿色的圆  
    #         for center in circle_centers:  
    #             cv2.circle(frame, center, circle_radius, (0, 255, 0), -1)  # 绘制实心圆
    #         Dx=0
    #         Dy=0
    #         if len(contours1) > 1 and len(contours2) > 1:
        
    #             # 根据结石中心点和导管右侧点的位置判断哪些圆应该变红
    #             # 计算当前十字准星中心点到轮廓中心的向量
    #             Dx = dx - cx  
    #             Dy = dy - cy
    #             # print('dx:',dx)
    #             # print('dy:',dy)  

    #         # 初始化标志，用于跟踪哪些方向需要变红  
    #         red_left = False  
    #         red_right = False  
    #         red_up = False  
    #         red_down = False  
            
    #         # 根据 dx 和 dy 的值来设置标志  
    #         if Dx > 5:  # 向左移动  
    #             red_left = True
    #         elif Dx < -5:  # 向右移动  
    #             red_right = True  
    #         if Dy > 5:  # 向下移动  
    #             red_down = True
    #         elif Dy < -5:  # 向上移动  
    #             red_up = True

    #         if red_left:  
    #             cv2.circle(frame, circle_centers[2], circle_radius, (0, 0, 255), -1)    
    #         if red_right:  
    #             cv2.circle(frame, circle_centers[1], circle_radius, (0, 0, 255), -1)    
    #         if red_up:  
    #             cv2.circle(frame, circle_centers[0], circle_radius, (0, 0, 255), -1)    
    #         if red_down:  
    #             cv2.circle(frame, circle_centers[3], circle_radius, (0, 0, 255), -1)  

    #     # 创建一个与帧相同大小的全零图像作为透明度图层
    #         overlay = np.zeros_like(frame, dtype=np.uint8)
    #         # 绘制红色轮廓区域的透明度图层
    #         cv2.drawContours(overlay, contours2, max_num, (0, 255, 0), thickness=cv2.FILLED)
    #         # 绘制绿色轮廓区域的透明度图层
    #         cv2.drawContours(overlay, contours1, max_num0, (100, 0, 255), thickness=cv2.FILLED)
    #         # 将透明度图层与原始帧进行融合
    #         alpha = 0.2  # 设置透明度（0为完全透明，1为完全不透明）
    #         result = cv2.addWeighted(frame, 1-alpha, overlay, alpha, 0)
    #         # # 显示带有叠加轮廓的帧
    #         # cv2.imshow('Frame with Overlay', frame_with_overlay)

    #         cv2.imshow('result',result)
    #     # 写入输出视频
    #         # out.write(frame)
            

    #         # # 添加延迟以降低视频的播放速度
    #         # if cv2.waitKey(1) == ord('q'):  # 延迟 30 毫秒
    #         #     cv2.imwrite('picture.jpg',frame)
    #         #     break
    #     ################################## trackingdemo3.1结束
            
    #         c= cv2.waitKey(1) & 0xff 

    #         if video_save_path!="":
    #             # out.write(frame)
    #             out.write(result)

    #         if c==27:
    #             capture.release()
    #             break
    #     print("Video Detection Done!")
    #     capture.release()

    #     if video_save_path!="":
    #         print("Save processed video to the path :" + video_save_path)
    #         out.release()

    #     cv2.destroyAllWindows()

    # # elif mode == "fps":
    # #     img = Image.open('img/street.jpg')
    # #     tact_time = unet.get_FPS(img, test_interval)
    # #     print(str(tact_time) + ' seconds, ' + str(1/tact_time) + 'FPS, @batch_size 1')
        
    # # elif mode == "dir_predict":
    # #     import os
    # #     from tqdm import tqdm

    # #     img_names = os.listdir(dir_origin_path)
    # #     for img_name in tqdm(img_names):
    # #         if img_name.lower().endswith(('.bmp', '.dib', '.png', '.jpg', '.jpeg', '.pbm', '.pgm', '.ppm', '.tif', '.tiff')):
    # #             image_path  = os.path.join(dir_origin_path, img_name)
    # #             image       = Image.open(image_path)
    # #             r_image     = unet.detect_image(image)
    # #             if not os.path.exists(dir_save_path):
    # #                 os.makedirs(dir_save_path)
    # #             r_image.save(os.path.join(dir_save_path, img_name))
    # # elif mode == "export_onnx":
    # #     unet.convert_to_onnx(simplify, onnx_save_path)
                
    # # elif mode == "predict_onnx":
    # #     while True:
    # #         img = input('Input image filename:')
    # #         try:
    # #             image = Image.open(img)
    # #         except:
    # #             print('Open Error! Try again!')
    # #             continue
    # #         else:
    # #             r_image = yolo.detect_image(image)
    # #             r_image.show()
    # else:
    #     raise AssertionError("Please specify the correct mode: 'predict', 'video', 'fps' or 'dir_predict'.")
    unet_package = UnetPackage( mode='video',  # 设置模式为video
        # 设置视频路径或摄像头索引，视频源或者0，1，2...
        #1为笔记本相机，2为深度相机
         video_path= 1,
        #  video_path='assets\demo6.18-480.mp4',
        #  video_path='assets\demo6.26-x2.mp4',
        #  video_path=r'assets\test2-compressed.mp4',
        #  video_save_path='assets\result-test2-compressed.mp4',
        #  video_save_path='assets\result6.28-demo6.18-480.mp4',  # 设置视频保存路径,空为不保存
         video_fps=30,  # 设置视频帧率
        # 深度推理相关参数
         enable_depth=True,  # 启用深度推理功能
         depth_encoder='vits',  # 深度模型编码器类型
         depth_input_size=518,  # 深度模型输入尺寸
         depth_grayscale=False,  # 是否使用灰度深度图
        )
     # 调用视频处理方法
    unet_package.video()
