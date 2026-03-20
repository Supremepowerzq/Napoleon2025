import sys
import os
import builtins
from pathlib import Path
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

# =========================
# 0) 路径设置
# =========================
ROOT = Path(__file__).resolve().parent

# ★改成你真实的 Depth-Anything-V2-main 路径（使用仓库内自带版本）
REPO_ROOT = Path(r"E:\zq\TJU\Napoleon2025\2026\Depth-Anything-V2-main")
METRIC_ROOT = REPO_ROOT / "metric_depth"
sys.path.insert(0, str(REPO_ROOT))  # 添加主目录到路径
sys.path.insert(0, str(METRIC_ROOT))  # 添加metric_depth目录到路径

from depth_anything_v2.dpt import DepthAnythingV2
DEPTH_AVAILABLE = True

# ★改成你真实的权重路径（pth 文件在哪里就写哪里）
LOAD_FROM = Path(r"E:\zq\TJU\checkpoints\depth_anything_v2_metric_hypersim_vitl.pth")

# =========================
# 1) 深度模型参数（从 predict_wqx.py 移植）
# =========================
ENCODER = "vitl"
INPUT_SIZE = 518
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# ---- Z 标定（你说：预测1000mm实际10mm → 先用0.01）----
A_SCALE = 0.01
B_BIAS_MM = 0.0

# ---- 圆外弱化（给深度模型用）----
BLUR_KSIZE = 51
OUTSIDE_STRENGTH = 0.10

# ---- 深度门控（防跳）----
Z_MIN_MM = 5.0
Z_MAX_MM = 300.0
Z_JUMP_MM = 30.0
INLIER_MIN = 0.25

# ---- 推进逻辑：>15mm 前进，<=10mm 停止（中间也停，保守）----
Z_ADVANCE_MM = 15.0
Z_STOP_MM = 10.0

# ---- 安全限速（非常保守，先小一点）----
SAFE_MAX_FORWARD = 8   # 推进最大值（正号前进）
SAFE_MAX_TURN = 8      # 转向最大值

# =========================
# 2) 深度推理工具函数（从 predict_wqx.py 移植）
# =========================
cmap = matplotlib.colormaps.get_cmap("Spectral")

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def make_circle_mask(side: int) -> np.ndarray:
    cx = side // 2
    cy = side // 2
    r = side // 2
    Y, X = np.ogrid[:side, :side]
    return (((X - cx) ** 2 + (Y - cy) ** 2) <= r * r).astype(np.uint8)

def apply_roi_circle_preprocess(img, mask, blur_ksize=51, outside_strength=0.1):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if blur_ksize % 2 == 0:
        blur_ksize += 1
    gray3 = cv2.GaussianBlur(gray3, (blur_ksize, blur_ksize), 0)
    m = mask[..., None].astype(np.float32)
    out = img * m + gray3 * (1 - m) * outside_strength
    return np.clip(out, 0, 255).astype(np.uint8)

def robust_depth_from_mask(depth_m, mask, erode_r=3):
    m = (mask > 0).astype(np.uint8)
    if erode_r > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erode_r + 1, 2 * erode_r + 1))
        m = cv2.erode(m, k, iterations=1)

    vals = depth_m[m > 0]
    if vals.size < 80:
        return None, 0.0

    z_med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - z_med))) + 1e-6
    inliers = vals[np.abs(vals - z_med) < 3.5 * 1.4826 * mad]
    if inliers.size < 30:
        return z_med, 0.0

    return float(np.median(inliers)), float(inliers.size / vals.size)

# =========================
# 3) 加载 DepthAnythingV2 metric
# =========================
model_cfg = {
    "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]}
}
depth_anything = DepthAnythingV2(**model_cfg[ENCODER])
depth_anything.load_state_dict(torch.load(str(LOAD_FROM), map_location="cpu", weights_only=True))
depth_anything = depth_anything.to(DEVICE).eval()
torch.backends.cudnn.benchmark = True
torch.set_float32_matmul_precision("high")

class UnetPackage:
    def __init__(self, model_path=None,
        mode='video', count=False, name_classes=None,
        video_path=1, video_save_path="",
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
        """初始化深度推理模型（使用全局模型）"""
        if not DEPTH_AVAILABLE:
            print("深度推理功能不可用：无法导入 Depth-Anything-V2")
            self.enable_depth = False
            return

        try:
            # 使用全局已加载的深度模型
            self.depth_model = depth_anything
            self.depth_device = DEVICE

            # 深度图可视化相关
            self.depth_cmap = cmap  # 使用全局的 cmap

            print(f"使用全局深度模型：{ENCODER} | 设备：{self.depth_device}")

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

        # ===== 深度状态变量（用于Z门控/防抖）=====
        Z_prev_mm = None
        Z_filt_mm = None

        # 圆形掩码（用于深度推理预处理）
        h_init, w_init = frame.shape[:2]
        side = min(h_init, w_init)
        circle_mask = make_circle_mask(side)

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
                    # 使用全局深度模型 depth_anything 进行推理
                    # 预处理
                    h_depth, w_depth = circular.shape[:2]
                    side_depth = min(h_depth, w_depth)
                    circle_mask_depth = make_circle_mask(side_depth)
                    circular_for_depth = apply_roi_circle_preprocess(circular, circle_mask_depth, BLUR_KSIZE, OUTSIDE_STRENGTH)

                    # 深度推理
                    depth_result_m = depth_anything.infer_image(circular_for_depth, INPUT_SIZE).astype(np.float32)

                    # 可视化
                    dmin = float(depth_result_m.min())
                    dmax = float(depth_result_m.max())
                    if dmax > dmin:
                        depth_norm = (depth_result_m - dmin) / (dmax - dmin) * 255.0
                    else:
                        depth_norm = np.zeros_like(depth_result_m, dtype=np.float32)
                    depth_uint8 = np.clip(depth_norm, 0, 255).astype(np.uint8)

                    # 调整为输入尺寸
                    if depth_uint8.shape[:2] != (h_depth, w_depth):
                        depth_resized = cv2.resize(depth_uint8, (w_depth, h_depth), interpolation=cv2.INTER_LINEAR)
                    else:
                        depth_resized = depth_uint8

                    # 颜色映射
                    depth_color = (cmap(depth_resized)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
                    depth_display = depth_color

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

            # ===== 初始化默认速度（未检测到目标时使用）=====
            speed_FR = 0
            vx = 0
            vy = 0
            z_ok = False  # 默认深度不可信

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

                # ===== 使用深度模型计算目标深度并计算速度 =====
                # 使用原始裁剪帧 frame_cropped_raw 进行深度推理
                h_frame, w_frame = frame_cropped_raw.shape[:2]
                side_frame = min(h_frame, w_frame)

                # 创建与 frame_cropped_raw 相同尺寸的圆形掩码
                circle_mask_frame = np.zeros((h_frame, w_frame), dtype=np.uint8)
                cx, cy = w_frame // 2, h_frame // 2
                r = side_frame // 2
                cv2.circle(circle_mask_frame, (cx, cy), r, 255, -1)

                # 预处理：圆外弱化
                frame_for_depth = apply_roi_circle_preprocess(frame_cropped_raw, circle_mask_frame, BLUR_KSIZE, OUTSIDE_STRENGTH)

                # 深度推理（米）
                depth_m = depth_anything.infer_image(frame_for_depth, INPUT_SIZE).astype(np.float32)

                # 使用结石轮廓创建掩码获取稳健深度
                stone_mask_depth = np.zeros((h_frame, w_frame), dtype=np.uint8)
                cv2.drawContours(stone_mask_depth, [contours2[max_num]], -1, 255, thickness=cv2.FILLED, lineType=cv2.LINE_AA)
                stone_mask_depth = (stone_mask_depth * circle_mask_frame).astype(np.uint8)

                Z_pred_m, inlier = robust_depth_from_mask(depth_m, stone_mask_depth, erode_r=3)

                z_ok = False
                Z_pred_mm = 0.0
                Z_mm = 0.0

                if Z_pred_m is not None:
                    Z_pred_mm = Z_pred_m * 1000.0
                    Z_mm = A_SCALE * Z_pred_mm + B_BIAS_MM

                    # 门控：范围 + inlier + 跳变
                    if (Z_MIN_MM <= Z_mm <= Z_MAX_MM) and (inlier >= INLIER_MIN):
                        if Z_prev_mm is None or abs(Z_mm - Z_prev_mm) <= Z_JUMP_MM:
                            z_ok = True
                        else:
                            # 跳变太大 → 用上一帧
                            Z_mm = Z_prev_mm
                            z_ok = True

                if z_ok:
                    Z_prev_mm = Z_mm

                    # 额外防抖：一阶低通
                    if Z_filt_mm is None:
                        Z_filt_mm = Z_mm
                    else:
                        alpha = 0.2
                        Z_filt_mm = (1 - alpha) * Z_filt_mm + alpha * Z_mm

                    # ===== 基于深度的推进逻辑（从 predict_wqx.py 移植）=====
                    if Z_filt_mm > Z_ADVANCE_MM:
                        speed_FR = +SAFE_MAX_FORWARD
                    elif Z_filt_mm <= Z_STOP_MM:
                        speed_FR = 0
                    else:
                        speed_FR = 0  # 保守
                else:
                    # 深度不可信 → 必须停
                    speed_FR = 0

                # ===== 转向逻辑（保持原有）=====
                TC = SAFE_MAX_TURN
                TC_S = builtins.max(1, int(SAFE_MAX_TURN // 3))

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

                speed_pf = speed_FR
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

                # ===== 在 Video 窗口（圆形裁剪后）中显示深度值和速度值（与 FPS 错开，避免重合）=====
                line_h = 26  # 行高，避免与 FPS(0,20) 及彼此重叠
                if len(contours2) > 0 and z_ok:
                    cv2.putText(
                        circular_frame,
                        f"Zfilt={Z_filt_mm:.1f}mm  inlier={inlier:.2f}",
                        (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA
                    )
                    # 显示深度信息：Zpred, Zcal, Zfilt, inlier（第二行）
                    cv2.putText(
                        circular_frame,
                        f"Zpred={Z_pred_mm:.1f}mm  Zcal={Z_mm:.1f}mm",
                        (0, 20 + line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA
                    )
                    
                    # 显示速度值（第三行）
                    cv2.putText(
                        circular_frame,
                        f"speed_FR={speed_FR}",
                        (0, 20 + 2 * line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA
                    )
                elif len(contours2) > 0:
                    cv2.putText(
                        circular_frame,
                        "Depth invalid (gated) -> speed_FR=0",
                        (0, 20 + line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA
                    )
                else:
                    cv2.putText(
                        circular_frame,
                        "No stone detected -> speed_FR=0",
                        (10, 20 + line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA
                    )

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
                else:
                    # 深度图先缩放到与 circular 完全一致尺寸，再裁剪，保证圆心和半径一致
                    if depth_display.shape[:2] != (h, w):
                        depth_display = cv2.resize(depth_display, (w, h), interpolation=cv2.INTER_LINEAR)

                # 对深度窗口进行圆形裁剪（手写版本，圆心和半径可调）
                # 圆心 (center_x, center_y) 和半径 radius 可调整以对齐深度图
                h_d, w_d = depth_display.shape[:2]
                # ===== 可调参数：修改以下三个值来对齐深度图圆 =====
                depth_center_x = w_d // 2 -25  # 圆心X
                depth_center_y = h_d // 2   # 圆心Y
                depth_radius = min(h_d, w_d) // 2 -65  # 半径（不要超出 min(h_d,w_d)/2）
                # ==============================================
                mask_d = np.zeros((h_d, w_d), dtype=np.uint8)
                cv2.circle(mask_d, (depth_center_x, depth_center_y), depth_radius, 255, -1)
                depth_display_circular = depth_display.copy()
                depth_display_circular[mask_d == 0] = 0  # 圆形外设为黑色

                # 创建大画布，宽度为三个图像宽度之和，高度为单个图像高度
                combined_width = w * 3
                combined_height = h
                combined_image = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)

                # 将三个图像按左中右顺序拼接
                combined_image[:, 0:w] = circular  # Original (左)
                combined_image[:, w:2*w] = circular_frame  # Video (中)
                combined_image[:, 2*w:3*w] = depth_display_circular  # Depth (右)

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

        # 释放 CUDA 缓存
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            print("CUDA 缓存已释放")       

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
