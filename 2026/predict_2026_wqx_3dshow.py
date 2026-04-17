import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)
import time
import cv2
import numpy as np
import torch
import matplotlib
import warnings
from pathlib import Path
from PIL import Image
from DLpredict import Unet_ONNX, Unet

from config import *
from ToolKits.ToolBox import get_time
from config import update_config
from config import get_config

# ============================================================
# Depth-Anything-V2 Metric 版本导入
# ============================================================
REPO_ROOT = Path(r"G:\zq\Depth-Anything-V2-main")
METRIC_ROOT = REPO_ROOT / "metric_depth"
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(METRIC_ROOT))

try:
    from metric_depth.depth_anything_v2.dpt import DepthAnythingV2
    DEPTH_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Cannot import Depth-Anything-V2 metric: {e}, depth inference disabled")
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
        enable_depth=False,
        depth_encoder='vitl',
        depth_input_size=518,
        depth_grayscale=False,
        # 圆形裁剪参数
        circle_center_x=400,
        circle_center_y=230,
        circle_radius=115,
        target_ratio=16/9,
        # ROI和去畸变参数
        roi_x=803, roi_y=146, roi_side=788,
        #此处为笔记本路径
        # undistort_maps_path=r"E:\p3\wqx\undistort_maps_roi788.npz",
        # depth_weight_path=r"E:\zq\TJU\Depth-Anything-V2-main\checkpoints\depth_anything_v2_metric_hypersim_vitl.pth",
        # # depth_weight_path=r"E:\zq\TJU\Depth-Anything-V2-main\checkpoints\kidney_dav2_best.pth",
        #此处为台式机路径
        undistort_maps_path=r"G:\wqx\undistort_maps_roi788.npz",
        depth_weight_path=r"G:\zq\Depth-Anything-V2-main\checkpoints\depth_anything_v2_metric_hypersim_vitl.pth",
        # depth_weight_path=r"E:\zq\TJU\Depth-Anything-V2-main\checkpoints\kidney_dav2_best.pth",
        # 深度标定参数
        depth_scale=0.01,
        depth_bias=0.0,
        # 深度门控参数
        depth_min_mm=5.0,
        depth_max_mm=300.0,
        depth_jump_mm=30.0,
        inlier_min=0.25,
        # 推进控制参数
        depth_advance_mm=20.0,
        # depth_stop_mm=15.0,
        depth_stop_mm=12.0,
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

        self.enable_depth = enable_depth and DEPTH_AVAILABLE
        self.depth_encoder = depth_encoder
        self.depth_input_size = depth_input_size
        self.depth_grayscale = depth_grayscale

        # 圆形裁剪参数
        self.circle_center_x = circle_center_x
        self.circle_center_y = circle_center_y
        self.circle_radius = circle_radius
        self.target_ratio = target_ratio

        # ROI和去畸变参数
        self.roi_x = roi_x
        self.roi_y = roi_y
        self.roi_side = roi_side
        self.undistort_maps_path = undistort_maps_path
        self.depth_weight_path = depth_weight_path

        # 深度标定参数
        self.depth_scale = depth_scale
        self.depth_bias = depth_bias

        # 深度门控参数
        self.depth_min_mm = depth_min_mm
        self.depth_max_mm = depth_max_mm
        self.depth_jump_mm = depth_jump_mm
        self.inlier_min = inlier_min

        # 推进控制参数
        self.depth_advance_mm = depth_advance_mm
        self.depth_stop_mm = depth_stop_mm

        # 深度模型
        self.depth_model = None
        self.depth_cmap = None
        self.maps = None
        self.map1 = None
        self.map2 = None
        self.Knew = None
        self.fx = None
        self.fy = None
        self.cx_k = None
        self.cy_k = None
        self.circle_mask = None

        # 深度状态变量
        self.depth_prev_mm = None
        self.depth_filt_mm = None

        # 加载去畸变maps（独立于深度模型）
        self._init_undistort_maps()

        if self.enable_depth:
            self._init_depth_model()

        if self.mode != "predict_onnx":
            self.unet = Unet()
        else:
            self.unet = Unet_ONNX()

    def _init_undistort_maps(self):
        """加载去畸变maps（独立于深度模型）"""
        try:
            if os.path.exists(self.undistort_maps_path):
                maps_data = np.load(str(self.undistort_maps_path))
                self.map1 = maps_data["map1"]
                self.map2 = maps_data["map2"]
                self.Knew = maps_data["Knew"]
                self.fx, self.fy = float(self.Knew[0, 0]), float(self.Knew[1, 1])
                self.cx_k, self.cy_k = float(self.Knew[0, 2]), float(self.Knew[1, 2])
                print(f"Undistort maps loaded: {self.undistort_maps_path}")
            else:
                print(f"Warning: Undistort maps not found: {self.undistort_maps_path}")
                self.map1 = None
                self.map2 = None
                self.Knew = None
        except Exception as e:
            print(f"Undistort maps load failed: {e}")
            self.map1 = None
            self.map2 = None
            self.Knew = None

        # 创建圆形掩码
        self.circle_mask = self.make_circle_mask(self.roi_side)

    def _init_depth_model(self):
        """初始化Depth-Anything-V2 Metric深度估计模型"""
        if not DEPTH_AVAILABLE:
            print("Depth inference unavailable: cannot import Depth-Anything-V2 metric")
            self.enable_depth = False
            return

        try:
            self.depth_device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

            # Metric版本只有vitl配置
            model_cfg = {
                "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]}
            }

            if self.depth_encoder not in model_cfg:
                print(f"Warning: encoder {self.depth_encoder} not available in metric version, using vitl")
                self.depth_encoder = 'vitl'

            print(f"Loading Depth-Anything-V2 Metric: {self.depth_encoder} | Device: {self.depth_device}")

            # 加载深度模型
            self.depth_model = DepthAnythingV2(**model_cfg[self.depth_encoder], max_depth=20.0)
            if os.path.exists(self.depth_weight_path):
                self.depth_model.load_state_dict(
                    torch.load(self.depth_weight_path, map_location="cpu", weights_only=True)
                )
                self.depth_model = self.depth_model.to(self.depth_device).eval()
                torch.backends.cudnn.benchmark = True
                torch.set_float32_matmul_precision("high")
            else:
                print(f"Warning: Depth weight not found: {self.depth_weight_path}")
                self.enable_depth = False
                return

            self.depth_cmap = matplotlib.colormaps.get_cmap("Spectral")
            print("Depth model initialized successfully")

        except Exception as e:
            print(f"Depth model init failed: {e}")
            self.enable_depth = False

    def make_circle_mask(self, side):
        """创建圆形掩码"""
        cx = side // 2
        cy = side // 2
        r = side // 2
        Y, X = np.ogrid[:side, :side]
        return (((X - cx) ** 2 + (Y - cy) ** 2) <= r * r).astype(np.uint8)

    def apply_roi_circle_preprocess(self, img, mask, blur_ksize=51, outside_strength=0.1):
        """圆形ROI预处理：高斯模糊弱化圆外区域"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if blur_ksize % 2 == 0:
            blur_ksize += 1
        gray3 = cv2.GaussianBlur(gray3, (blur_ksize, blur_ksize), 0)
        m = mask[..., None].astype(np.float32)
        out = img * m + gray3 * (1 - m) * outside_strength
        return np.clip(out, 0, 255).astype(np.uint8)

    def normalize_depth_to_uint8(self, depth):
        """深度归一化为uint8"""
        dmin, dmax = float(depth.min()), float(depth.max())
        if dmax - dmin < 1e-6:
            return np.zeros_like(depth, dtype=np.uint8)
        return ((depth - dmin) / (dmax - dmin) * 255).astype(np.uint8)

    def robust_depth_from_mask(self, depth_m, mask, erode_r=3):
        """稳健深度估计：使用中位数+MAD统计"""
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

    def infer_depth_metric(self, und_for_model):
        """使用Metric深度模型进行推理"""
        if not self.enable_depth or self.depth_model is None:
            return None

        try:
            with torch.no_grad():
                depth = self.depth_model.infer_image(und_for_model, self.depth_input_size)
            return depth.astype(np.float32)
        except Exception as e:
            print(f"Depth inference error: {e}")
            return None

    def crop_to_circle(self, image, fps=None, show_crosshair=True, show_circles=True, red_circles=None, depth_info=None):
        """将图像裁剪为圆形显示区域，添加十字准星、FPS、方向指示圆"""
        try:
            h, w = image.shape[:2]
            
            # 圆形直径 = 高度（与predict_2026.py保持一致）
            diameter = h
            radius = diameter // 2
            
            # 计算原始图像中心
            center_x = w // 2
            center_y = h // 2
            
            # 从图像中心裁剪出圆形区域（宽度方向）
            left = max(0, center_x - radius)
            right = min(w, center_x + radius)
            cropped = image[:, left:right]
            
            ch, cw = cropped.shape[:2]
            
            # 如果宽度不足（宽 < 高），需要在周围补黑色以保持正方形
            if cw < ch:
                padding = (ch - cw) // 2
                # 创建黑色背景的正方形画布
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
            
            # 创建圆形掩码并应用
            mask = np.zeros((ch, cw), dtype=np.uint8)
            cv2.circle(mask, (circle_center_x, circle_center_y), circle_radius, 255, -1)
            result[mask == 0] = 0

            if show_crosshair:
                cross_length = 20
                # 十字准星画在画布正中心（也是圆形中心）
                cv2.line(result, (circle_center_x - cross_length, circle_center_y),
                        (circle_center_x + cross_length, circle_center_y), (0, 0, 255), 2, cv2.LINE_AA)
                cv2.line(result, (circle_center_x, circle_center_y - cross_length),
                        (circle_center_x, circle_center_y + cross_length), (0, 0, 255), 2, cv2.LINE_AA)

            if fps is not None:
                cv2.putText(result, "FPS:%.2f" % (fps), (0, 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.putText(result, "BIOMACH", (0, ch - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            if show_circles:
                circle_centers = [(cw - 40, ch - 20),
                                 (cw - 20, ch - 40),
                                 (cw - 60, ch - 40),
                                 (cw - 40, ch - 60)]
                circle_radius_small = 8

                for idx, center in enumerate(circle_centers):
                    if red_circles and idx in red_circles:
                        cv2.circle(result, center, circle_radius_small, (0, 0, 255), -1, cv2.LINE_AA)
                    else:
                        cv2.circle(result, center, circle_radius_small, (0, 255, 0), -1, cv2.LINE_AA)

            # 如果有深度信息，显示在右上角（仅用于深度窗口）
            if depth_info is not None:
                # 显示深度信息（Speed单独处理颜色）
                if depth_info.get('z_ok', False):
                    cv2.putText(result, f"Zcal={depth_info['z_mm']:.1f}mm", (cw - 180, 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(result, f"Zfilt={depth_info['z_filt']:.1f}mm", (cw - 180, 42),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(result, f"inlier={depth_info['inlier']:.2f}", (cw - 180, 64),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                    # Speed根据值显示不同颜色
                    if 'speed_FR' in depth_info:
                        speed_color = (0, 0, 255) if depth_info['speed_FR'] == 0 else (0, 255, 0)
                        cv2.putText(result, f"Speed={int(depth_info['speed_FR'])}", (cw - 180, 86),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, speed_color, 2, cv2.LINE_AA)
                else:
                    cv2.putText(result, "Depth Invalid", (cw - 180, 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                    if 'speed_FR' in depth_info:
                        speed_color = (0, 0, 255) if depth_info['speed_FR'] == 0 else (0, 255, 0)
                        cv2.putText(result, f"Speed={int(depth_info['speed_FR'])}", (cw - 180, 42),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, speed_color, 2, cv2.LINE_AA)

            return result
        except Exception as e:
            print(f'crop_to_circle error: {e}')
            return image

    def extract_circular_roi(self, image, center_x, center_y, radius):
        """提取圆形区域ROI，去除黑边"""
        mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        cv2.circle(mask, (center_x, center_y), radius, 255, -1)

        x1 = max(0, center_x - radius)
        y1 = max(0, center_y - radius)
        x2 = min(image.shape[1], center_x + radius)
        y2 = min(image.shape[0], center_y + radius)
        crop_image = image[y1:y2, x1:x2]
        crop_mask = mask[y1:y2, x1:x2]

        roi_image = np.copy(crop_image)
        roi_image[crop_mask == 0] = 128

        return roi_image, crop_mask

    def resize_with_padding(self, image, target_ratio=16/9):
        """强制宽高比resize"""
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
        """单张图像预测"""
        try:
            image = Image.open(image_path)
        except Exception as e:
            print(f'Open Error! {e}')
            return None
        else:
            return self.unet.detect_image(image, count=self.count, name_classes=self.name_classes)

    def linear_interpolation(self, value, low, high, low_speed, high_speed):
        """线性插值映射"""
        if abs(value) <= abs(low):
            return low_speed
        elif abs(value) >= abs(high):
            return high_speed
        else:
            return low_speed + (high_speed - low_speed) * (value - low) / (high - low)

    def clamp(self, v, lo, hi):
        """限幅"""
        return max(lo, min(hi, v))

    def video(self):
        """视频流实时推理：UNet分割 + Metric深度估计 + 速度控制"""
        lower_green0 = np.array([76, 46, 28])   # 导管颜色 HSV
        upper_green0 = np.array([98, 255, 255])
        lower_green = np.array([98, 157, 88])    # 结石颜色 HSV
        upper_green = np.array([125, 255, 255])

            # 深度预处理参数
        blur_ksize = 51
        outside_strength = 0.10

        # 检查去畸变maps是否可用
        maps_loaded = self.map1 is not None and self.map2 is not None
        if not maps_loaded:
            print("Warning: Undistort maps not loaded, skipping undistortion")

        # 摄像头初始化（与predict_wqx.py一致）
        capture = cv2.VideoCapture(self.video_path, cv2.CAP_DSHOW)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self.video_save_path != "":
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            out = cv2.VideoWriter(self.video_save_path, fourcc, self.video_fps, size)

        ref, frame = capture.read()
        if not ref:
            raise ValueError("Camera/Video read failed.")

        fps = 0.0

        # 安全限速参数
        safe_max_forward = 80
        # safe_max_turn = 8

        print("Keys: ESC=Quit")
        if self.enable_depth:
            print(f"Depth: scale={self.depth_scale}, advance>{self.depth_advance_mm}mm, stop<={self.depth_stop_mm}mm")

        frame_count = 0
        while True:
            t1 = time.time()
            ref, frame_full = capture.read()
            if not ref:
                print("Camera read failed, retrying...")
                continue

            frame_count += 1
            if frame_count == 1:
                print(f"Camera initialized, frame size: {frame_full.shape}")

            # ============================================================
            # Step 1: ROI裁剪 + 去畸变（与predict_wqx.py一致）
            # ============================================================
            try:
                # ROI裁剪
                roi_raw = frame_full[self.roi_y:self.roi_y + self.roi_side,
                                     self.roi_x:self.roi_x + self.roi_side]
                
                if roi_raw.shape[0] != self.roi_side or roi_raw.shape[1] != self.roi_side:
                    continue
                
                # 去畸变（仅在maps加载成功时执行）
                if maps_loaded:
                    und = cv2.remap(roi_raw, self.map1, self.map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    frame_cropped_raw = und
                else:
                    frame_cropped_raw = roi_raw
            except Exception as e:
                print(f"Frame processing error: {e}")
                frame_cropped_raw = frame_full

            # ============================================================
            # Step 2: Original显示窗口（去畸变后的图像）
            # ============================================================
            try:
                circular_original = self.crop_to_circle(frame_cropped_raw, show_crosshair=False, show_circles=False)
            except Exception:
                circular_original = frame_cropped_raw

            # ============================================================
            # Step 3: 深度推理（仅在启用时）
            # ============================================================
            depth_m = None
            depth_vis = np.zeros((self.roi_side, self.roi_side, 3), dtype=np.uint8)

            if self.enable_depth and self.depth_model is not None:
                try:
                    # 深度预处理
                    und_for_model = self.apply_roi_circle_preprocess(frame_cropped_raw, self.circle_mask, blur_ksize, outside_strength)

                    # 深度推理
                    depth_m = self.infer_depth_metric(und_for_model)

                    if depth_m is not None:
                        # 深度可视化
                        depth_u8 = self.normalize_depth_to_uint8(depth_m)
                        depth_color = (self.depth_cmap(depth_u8)[:, :, :3] * 255).astype(np.uint8)[:, :, ::-1]
                        depth_vis = (depth_color * self.circle_mask[..., None]).astype(np.uint8)
                except Exception as e:
                    print(f"Depth error: {e}")

            # ============================================================
            # Step 4: UNet语义分割（使用去畸变后的图像）
            # ============================================================
            frame_rgb = cv2.cvtColor(frame_cropped_raw, cv2.COLOR_BGR2RGB)
            h, w = frame_rgb.shape[:2]
            center_x, center_y = w // 2, h // 2
            
            # 使用圆形掩码处理：保持圆形内原始亮度，圆形外设为0（黑色）
            # 这样可以避免亮度降低影响UNet识别效果
            roi_image = frame_rgb.copy()
            roi_image[self.circle_mask == 0] = 0

            # UNet处理用于获取分割结果（用于HSV颜色分割）
            frame_unet_input = Image.fromarray(np.uint8(roi_image))
            frame_unet_output = np.array(self.unet.detect_image(frame_unet_input))
            frame_unet_output = cv2.cvtColor(frame_unet_output, cv2.COLOR_RGB2BGR)

            # 使用原始图像作为显示基础，保持原始亮度
            # UNet结果仅用于HSV分割获取轮廓，不用于显示
            frame = frame_cropped_raw.copy()

            fps = (fps + (1. / (time.time() - t1))) / 2

            # ============================================================
            # Step 5: HSV颜色分割 - 导管与结石（使用UNet结果进行分割）
            # ============================================================
            hsv = cv2.cvtColor(frame_unet_output, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, lower_green0, upper_green0)
            contours1, hierarchy1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            mask2 = cv2.inRange(hsv, lower_green, upper_green)
            contours2, hierarchy2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            frame_height, frame_width = frame.shape[:2]
            total_pixels = frame_width * frame_height
            percentage1 = 0.0

            # 默认值
            speed_FR = 0
            vx = 0
            vy = 0
            cx = w // 2
            cy = h // 2
            
            # 初始化深度信息（默认值）
            depth_info = {
                'z_ok': False,
                'z_mm': 0,
                'z_filt': 0,
                'inlier': 0,
                'speed_FR': 0
            }

            # ============================================================
            # Step 6: 导管处理（忽略轮廓绘制）
            # ============================================================
            # 导管仅用于确定末端位置，不绘制轮廓

            # ============================================================
            # Step 7: 结石轮廓处理 - 找最大轮廓，计算质心，绘制（与predict_2026.py一致）
            # ============================================================
            max = 0.0
            max_num = 0
            min_contour_area = 10.0

            if len(contours2) > 0:
                for i in range(len(contours2)):
                    c = cv2.contourArea(contours2[i])
                    if c > max:
                        max = c
                        max_num = i

                # 计算结石面积占比
                percentage1 = (max / total_pixels) * 100

                # 只有当最大轮廓面积超过阈值时才认为检测到有效目标
                if max > min_contour_area:
                    moments = cv2.moments(contours2[max_num])
                    if moments['m00'] != 0:
                        cx = int(moments['m10'] / moments['m00'])
                        cy = int(moments['m01'] / moments['m00'])
                        # 绘制最大轮廓
                        result = cv2.drawContours(frame, contours2[max_num], -1, (255, 255, 0), 3, cv2.LINE_AA)
                        # 绘制质心圆（半径20）
                        frame = cv2.circle(frame, (cx, cy), 20, (255, 255, 0), -1, cv2.LINE_AA)
                    else:
                        cx = w // 2
                        cy = h // 2
                else:
                    cx = w // 2
                    cy = h // 2
            else:
                cx = w // 2
                cy = h // 2

            # ============================================================
            # Step 8: F/R/IN FOCUS判断（不显示文字，避免被掩膜遮挡）
            # ============================================================
            # 文字显示已移除，避免被圆形掩膜遮挡

            # ============================================================
            # Step 9: 深度估计（仅当检测到结石时，与predict_wqx.py一致）
            # ============================================================
            if len(contours2) > 0 and depth_m is not None:
                # 创建结石mask
                stone_mask = np.zeros((h, w), dtype=np.uint8)
                cv2.drawContours(stone_mask, [contours2[max_num]], -1, 255, thickness=cv2.FILLED, lineType=cv2.LINE_AA)
                stone_mask = (stone_mask * self.circle_mask).astype(np.uint8)

                # 稳健深度估计
                Z_pred_m, inlier = self.robust_depth_from_mask(depth_m, stone_mask, erode_r=3)

                z_ok = False
                Z_mm = None

                if Z_pred_m is not None:
                    Z_pred_mm = Z_pred_m * 1000.0
                    Z_mm = self.depth_scale * Z_pred_mm + self.depth_bias

                    # 深度门控
                    if (self.depth_min_mm <= Z_mm <= self.depth_max_mm) and (inlier >= self.inlier_min):
                        if self.depth_prev_mm is None or abs(Z_mm - self.depth_prev_mm) <= self.depth_jump_mm:
                            z_ok = True
                        else:
                            Z_mm = self.depth_prev_mm
                            z_ok = True

                if z_ok:
                    self.depth_prev_mm = Z_mm

                    # 一阶低通滤波
                    if self.depth_filt_mm is None:
                        self.depth_filt_mm = Z_mm
                    else:
                        alpha = 0.2
                        self.depth_filt_mm = (1 - alpha) * self.depth_filt_mm + alpha * Z_mm

                    # 推进速度控制：基于滤波后深度，当z距离在8-15mm时线性插值
                    if self.depth_filt_mm >= self.depth_advance_mm:
                        speed_FR = safe_max_forward  # 全速前进
                    elif self.depth_filt_mm <= self.depth_stop_mm:
                        speed_FR = 0  # 停止
                    else:
                        # 在深度8-15mm范围内，从0到最大前进速度线性插值
                        speed_FR = int(safe_max_forward * (self.depth_filt_mm - self.depth_stop_mm) / (self.depth_advance_mm - self.depth_stop_mm))

                    # 更新深度信息（包含速度信息，用于在深度窗口显示）
                    depth_info = {
                        'z_ok': True,
                        'z_mm': Z_mm,
                        'z_filt': self.depth_filt_mm,
                        'inlier': inlier,
                        'speed_FR': speed_FR
                    }
                else:
                    # 深度不可信
                    speed_FR = 0
                    depth_info = {
                        'z_ok': False,
                        'z_mm': 0,
                        'z_filt': 0,
                        'inlier': 0,
                        'speed_FR': 0
                    }

            # ============================================================
            # Step 10: 转向速度计算（严格按照predict_2026.py的逻辑）
            # ============================================================
            red_left = False
            red_right = False
            red_up = False
            red_down = False

            if len(contours2) > 0:
                # 计算结石质心与画面中心的偏移量
                Dx = center_x - cx
                Dy = center_y - cy

                PixelR = 10
                red_left = Dx > PixelR
                red_right = Dx < -PixelR
                red_up = Dy < -PixelR
                red_down = Dy > PixelR

                # 获取控制参数
                FC = FORWARD_COEFF
                TC = TURN_COEFF
                TC_S = TURN_COEFF_SLOW
                Multiple = 3.0
                M_x = 4.5
                M_y = 8.0
                # vx_max = 20.0
                # vy_max = 0.6
                vx_max = 30.0
                vy_max = 0.4

                # X方向控制（左右移动）
                vx = 0.0
                if red_left:
                    abs_dx = abs(Dx)
                    if abs_dx < PixelR:
                        vx = 0.0
                    elif abs_dx < M_x * PixelR:
                        vx = -5.0 + (-vx_max) * (abs_dx - PixelR) / ((M_x - 1) * PixelR)
                    else:
                        vx = -vx_max

                if red_right:
                    abs_dx = abs(Dx)
                    if abs_dx < PixelR:
                        vx = 0.0
                    elif abs_dx < M_x * PixelR:
                        vx = 5 + (vx_max) * (abs_dx - PixelR) / ((M_x - 1) * PixelR)
                    else:
                        vx = vx_max

                # Y方向控制（上下移动）
                vy = 0.0
                if red_up:
                    abs_dy = abs(Dy)
                    if abs_dy < PixelR:
                        vy = 0.0
                    elif abs_dy < M_y * PixelR:
                        vy = -0.05 - vy_max * (abs_dy - PixelR) / ((M_y - 1) * PixelR)
                    else:
                        vy = -vy_max

                if red_down:
                    abs_dy = abs(Dy)
                    if abs_dy < PixelR:
                        vy = 0.0
                    elif abs_dy < M_y * PixelR:
                        vy = 0.05 + vy_max * (abs_dy - PixelR) / ((M_y - 1) * PixelR)
                    else:
                        vy = vy_max

                # 写入config（与predict_2026.py一致，不额外限幅）
                speed_FR = self.clamp(speed_FR, -safe_max_forward, safe_max_forward)
                update_config('speed_pf', speed_FR)
                update_config('speed_pt', [vx, vy])
            else:
                # 没有检测到结石，速度置零
                speed_FR = 0
                update_config('speed_pf', 0)
                update_config('speed_pt', [0, 0])
                # 更新深度信息（无速度）
                depth_info = {
                    'z_ok': False,
                    'z_mm': 0,
                    'z_filt': 0,
                    'inlier': 0,
                    'speed_FR': 0
                }

            # ============================================================
            # Step 11: 结石区域边框绘制（取消填充，改用边框避免变暗）
            # ============================================================
            if len(contours2) > 0:
                # 直接绘制边框，不进行填充叠加，避免影响图像亮度
                cv2.drawContours(frame, contours2, max_num, (0, 255, 0), 2, cv2.LINE_AA)

            # ============================================================
            # Step 12: 方向指示圆反馈
            # ============================================================
            red_circle_indices = []
            if red_left:
                red_circle_indices.append(2)
            if red_right:
                red_circle_indices.append(1)
            if red_up:
                red_circle_indices.append(0)
            if red_down:
                red_circle_indices.append(3)

            # UNet分割窗口：裁剪为圆形并添加UI元素
            try:
                circular_unet = self.crop_to_circle(frame, fps=fps, show_crosshair=True,
                                                     show_circles=True, red_circles=red_circle_indices)
            except Exception as e:
                print(f"UNet crop error: {e}")
                circular_unet = frame

            # 深度窗口：圆形裁剪并添加深度和速度信息
            try:
                # 在深度图上绘制结石轮廓和质心（白色）
                depth_with_stone = depth_vis.copy()
                if len(contours2) > 0 and max > min_contour_area:
                    cv2.drawContours(depth_with_stone, contours2[max_num], -1, (255, 255, 255), 3, cv2.LINE_AA)
                    cv2.circle(depth_with_stone, (cx, cy), 20, (255, 255, 255), -1, cv2.LINE_AA)
                
                circular_depth = self.crop_to_circle(depth_with_stone, fps=fps, show_crosshair=True,
                                                     show_circles=False, depth_info=depth_info)
            except Exception as e:
                print(f"Depth crop error: {e}")
                circular_depth = depth_vis

            # ============================================================
            # Step 13: 生成3D点云视角
            # ============================================================
            target_h, target_w = circular_original.shape[:2]
            point_cloud_view = self.render_isometric_point_cloud(
                depth_m, depth_vis, contours2, max_num, cx, cy,
                target_size=(target_h, target_w),
                downsample=5,
                z_exaggeration=1.5,      # Z轴拉伸系数
                rotation_deg=0,        # XY平面绕Z轴旋转（0=无旋转）
                tilt_deg=-30,          # 俯仰角度（上下倾角）
                azimuth_deg=-30         # 方位角度（左右旋转）
            )

            # ============================================================
            # Step 14: 拼接四窗口：Original | UNet | Depth | 3D Point Cloud
            # ============================================================
            try:
                # 调整 circular_unet 和 circular_depth 的尺寸以匹配 circular_original
                if circular_unet.shape[:2] != (target_h, target_w):
                    circular_unet = cv2.resize(circular_unet, (target_w, target_h))

                if circular_depth.shape[:2] != (target_h, target_w):
                    circular_depth = cv2.resize(circular_depth, (target_w, target_h))

                # 3D点云窗口已在render_isometric_point_cloud中调整为target尺寸

                combined_width = target_w * 4
                combined_height = target_h
                combined_image = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)

                combined_image[:, 0:target_w] = circular_original
                combined_image[:, target_w:2*target_w] = circular_unet
                combined_image[:, 2*target_w:3*target_w] = circular_depth
                combined_image[:, 3*target_w:4*target_w] = point_cloud_view

                # 缩小拼接窗口以便显示
                display_scale = 0.8
                display_w = int(combined_width * display_scale)
                display_h = int(combined_height * display_scale)
                combined_image_display = cv2.resize(combined_image, (display_w, display_h))

                cv2.imshow("Combined View: Original | UNet | Depth | 3D PointCloud", combined_image_display)

            except Exception as e:
                print(f"Combine window error: {e}")
                try:
                    cv2.imshow("UNet", circular_unet)
                except Exception:
                    pass

            if self.video_save_path != "":
                out.write(frame)

            c = cv2.waitKey(1) & 0xff
            if c == 27:
                break

        # 退出前停止电机
        update_config('speed_pf', 0)
        update_config('speed_pt', [0, 0])
        capture.release()
        if self.video_save_path != "":
            out.release()
        cv2.destroyAllWindows()

    def make_circle_mask_for_size(self, w, h, center_x, center_y, radius):
        """为指定尺寸创建圆形掩码"""
        Y, X = np.ogrid[:h, :w]
        return (((X - center_x) ** 2 + (Y - center_y) ** 2) <= radius ** 2).astype(np.uint8)

    def render_isometric_point_cloud(self, depth_m, depth_color, contours2, max_num, cx, cy,
                                     target_size=None, downsample=6,
                                     z_exaggeration=1.5, rotation_deg=0, tilt_deg=30,
                                     azimuth_deg=0):
        """
        使用可调视角渲染深度图的3D点云

        视角参数说明：
        - z_exaggeration: Z轴拉伸系数，默认1.5
        - rotation_deg: XY平面绕Z轴旋转，默认0°
        - tilt_deg: 俯仰角度（上下倾角），默认30°
          - 0°: 正对着XY平面（俯视图）
          - 30°: 标准正等轴测图视角
        - azimuth_deg: 方位角度（左右旋转），默认0°
          - 与tilt_deg垂直，控制水平方向的倾斜
          - 0°: 无方位倾斜
          - 正值: 向右偏移
          - 负值: 向左偏移
        """
        if depth_m is None:
            h = depth_color.shape[0] if depth_color is not None else 788
            w = depth_color.shape[1] if depth_color is not None else 788
            target_h, target_w = target_size if target_size else (h, w)
            view = np.zeros((target_h, target_w, 3), dtype=np.uint8)
            cv2.circle(view, (target_w//2, target_h//2), target_h//2 - 2, (100, 100, 100), 1, cv2.LINE_AA)
            cv2.putText(view, '3D PointCloud', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            return view

        h, w = depth_m.shape
        target_h, target_w = target_size if target_size else (h, w)

        # 投影参数
        cos_30 = np.sqrt(3) / 2
        sin_30 = 0.5

        # 角度转弧度
        rot_rad = np.deg2rad(rotation_deg)
        tilt_rad = np.deg2rad(tilt_deg)
        azimuth_rad = np.deg2rad(azimuth_deg)
        cos_rot, sin_rot = np.cos(rot_rad), np.sin(rot_rad)
        cos_tilt, sin_tilt = np.cos(tilt_rad), np.sin(tilt_rad)
        cos_azimuth, sin_azimuth = np.cos(azimuth_rad), np.sin(azimuth_rad)

        # 创建圆形掩码
        circle_mask = self.make_circle_mask_for_size(w, h, w//2, h//2, h//2)

        # 深度归一化
        depth_normalized = depth_m.copy()
        dmin, dmax = depth_normalized.min(), depth_normalized.max()
        if dmax - dmin > 1e-6:
            depth_normalized = (depth_normalized - dmin) / (dmax - dmin)
        else:
            depth_normalized = np.zeros_like(depth_normalized)

        # 第一步：计算所有投影点的范围
        projected_points = []
        for j in range(0, h, downsample):
            for i in range(0, w, downsample):
                if circle_mask[j, i] > 0:
                    # 像素坐标 (i, j) -> 归一化世界坐标 [-1, 1]
                    # i是列(水平X)，j是行(垂直Y，图像中向下为正)
                    x_norm = (i - w/2) / (w/2)
                    y_norm = (j - h/2) / (h/2)

                    # 镜像修正：图像中Y向下，但3D空间中Y应向"上"（或向内）
                    # 去掉这个负号会导致上下镜像
                    y_norm = -y_norm  # 关键修正：翻转Y轴

                    z_val = depth_normalized[j, i] * z_exaggeration

                    # 应用XY平面旋转（绕Z轴）
                    x_rot = x_norm * cos_rot - y_norm * sin_rot
                    y_rot = x_norm * sin_rot + y_norm * cos_rot

                    # 应用方位角（左右旋转）：绕Y轴旋转X和Z
                    x_az = x_rot * cos_azimuth + z_val * sin_azimuth
                    z_az = -x_rot * sin_azimuth + z_val * cos_azimuth

                    # 应用俯仰角（上下倾角）：绕X轴旋转
                    iso_x = x_az
                    iso_y = y_rot * cos_tilt - z_az * sin_tilt

                    projected_points.append((iso_x, iso_y, i, j))

        # 初始化输出图像（黑色背景）
        view = np.zeros((target_h, target_w, 3), dtype=np.uint8)

        if not projected_points:
            cv2.circle(view, (target_w//2, target_h//2), target_h//2 - 2, (100, 100, 100), 1, cv2.LINE_AA)
            cv2.putText(view, '3D PointCloud', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            return view

        # 计算投影范围
        iso_x_vals = [p[0] for p in projected_points]
        iso_y_vals = [p[1] for p in projected_points]
        min_iso_x, max_iso_x = min(iso_x_vals), max(iso_x_vals)
        min_iso_y, max_iso_y = min(iso_y_vals), max(iso_y_vals)

        # 计算缩放因子
        margin = 0.1
        range_x = max_iso_x - min_iso_x
        range_y = max_iso_y - min_iso_y
        display_range = 1.0 - 2 * margin
        scale_x = display_range / (range_x + 1e-6)
        scale_y = display_range / (range_y + 1e-6)
        scale = min(scale_x, scale_y)
        offset_x = -min_iso_x * scale + margin
        offset_y = -min_iso_y * scale + margin + 0.05

        # 第二步：使用NumPy向量化绘制所有投影点（高效方式）
        pts_x = np.array([p[0] for p in projected_points])
        pts_y = np.array([p[1] for p in projected_points])
        pts_px = np.array([p[2] for p in projected_points], dtype=np.int32)
        pts_py = np.array([p[3] for p in projected_points], dtype=np.int32)

        # 计算屏幕坐标
        screen_x = ((pts_x * scale + offset_x) * target_w).astype(np.int32)
        screen_y = target_h - ((pts_y * scale + offset_y) * target_h).astype(np.int32)

        # 创建有效点掩码
        valid_mask = (screen_x >= 0) & (screen_x < target_w) & (screen_y >= 0) & (screen_y < target_h)

        # 批量绘制有效点
        valid_x = screen_x[valid_mask]
        valid_y = screen_y[valid_mask]
        valid_px = pts_px[valid_mask]
        valid_py = pts_py[valid_mask]

        if len(valid_x) > 0:
            # 获取所有颜色
            colors = depth_color[valid_py, valid_px].astype(np.int32)

            # 使用OpenCV的putText批量绘制（绘制小矩形代替点）
            for k in range(len(valid_x)):
                sx, sy, color = int(valid_x[k]), int(valid_y[k]), colors[k]
                # 绘制单个像素点
                view[sy, sx] = np.clip(color, 0, 255).astype(np.uint8)
                # 绘制2x2区域使点更明显
                if sx + 1 < target_w and sy + 1 < target_h:
                    view[sy+1, sx] = np.clip(color, 0, 255).astype(np.uint8)
                    view[sy, sx+1] = np.clip(color, 0, 255).astype(np.uint8)

        # 第三步：绘制结石轮廓（应用相同的视角变换）
        if len(contours2) > 0 and max_num >= 0:
            try:
                contour = contours2[max_num]
                contour_points_iso = []
                for point in contour:
                    px, py = point[0]
                    if 0 <= px < w and 0 <= py < h:
                        x_norm = (px - w/2) / (w/2)
                        y_norm = (py - h/2) / (h/2)
                        y_norm = -y_norm  # Y轴镜像修正

                        z_val = depth_normalized[py, px] * z_exaggeration

                        # 应用XY平面旋转（绕Z轴）
                        x_rot = x_norm * cos_rot - y_norm * sin_rot
                        y_rot = x_norm * sin_rot + y_norm * cos_rot

                        # 应用方位角（左右旋转）：绕Y轴旋转X和Z
                        x_az = x_rot * cos_azimuth + z_val * sin_azimuth
                        z_az = -x_rot * sin_azimuth + z_val * cos_azimuth

                        # 应用俯仰角（上下倾角）
                        iso_x = x_az
                        iso_y = y_rot * cos_tilt - z_az * sin_tilt

                        sx = int((iso_x * scale + offset_x) * target_w)
                        sy = target_h - int((iso_y * scale + offset_y) * target_h)
                        if 0 <= sx < target_w and 0 <= sy < target_h:
                            contour_points_iso.append([sx, sy])

                if len(contour_points_iso) > 2:
                    contour_points_iso = np.array(contour_points_iso, dtype=np.int32)
                    cv2.polylines(view, [contour_points_iso], False, (0, 255, 255), 3, cv2.LINE_AA)
            except Exception:
                pass

        # 第四步：绘制坐标系（原点对齐到左下角区域）
        # 目标布局：Y轴向上，X轴右上，Z轴右下
        origin_x, origin_y = 50, target_h - 40
        axis_len = 50
        cos_30 = np.sqrt(3) / 2
        sin_30 = 0.5

        # Y轴：向上（屏幕上方）
        cv2.line(view, (origin_x, origin_y), (origin_x, origin_y - axis_len), (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(view, 'Y', (origin_x - 8, origin_y - axis_len - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

        # X轴：右上方
        x_end_x = origin_x + int(axis_len * cos_30)
        x_end_y = origin_y - int(axis_len * sin_30)
        cv2.line(view, (origin_x, origin_y), (x_end_x, x_end_y), (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(view, 'X', (x_end_x + 3, x_end_y + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        # Z轴：右下方
        z_end_x = origin_x + int(axis_len * cos_30)
        z_end_y = origin_y + int(axis_len * sin_30)
        cv2.line(view, (origin_x, origin_y), (z_end_x, z_end_y), (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(view, 'Z', (z_end_x + 3, z_end_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

        # 标题
        cv2.putText(view, '3D PointCloud', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        # 绘制圆形边框
        cv2.circle(view, (target_w//2, target_h//2), target_h//2 - 2, (100, 100, 100), 1, cv2.LINE_AA)

        return view

    def fps(self):
        """测试UNet模型推理FPS"""
        t1 = time.time()
        img = Image.open(self.fps_image_path)
        tact_time = self.unet.get_FPS(img, self.test_interval)
        print(str(tact_time) + ' seconds, ' + str(1/tact_time) + 'FPS, @batch_size 1')
        t2 = time.time()
        print('FPS:', 1 / (t2 - t1))

    def directory(self):
        """批量目录图像预测"""
        from utils.utils import detect_directory
        detect_directory(self.unet, self.dir_origin_path, self.dir_save_path, self.count, self.name_classes)

    def export_onnx(self):
        """导出UNet模型为ONNX格式"""
        self.unet.convert_to_onnx(self.simplify, self.onnx_save_path)

    def run(self):
        """根据mode选择运行模式"""
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
    if DEPTH_AVAILABLE:
        print("Depth-Anything-V2 Metric imported successfully, depth inference enabled")
    else:
        print("Depth-Anything-V2 Metric import failed, depth inference will be disabled")

    unet_package = UnetPackage(
        mode='video',
        video_path=0,
        video_fps=30,
        enable_depth=True,
        depth_encoder='vitl',
        depth_input_size=518,
        depth_grayscale=False,
        # 深度标定
        depth_scale=0.01,
        depth_bias=0.0,
        # 深度门控
        depth_min_mm=5.0,
        depth_max_mm=300.0,
        depth_jump_mm=30.0,
        inlier_min=0.25,
        # 推进控制
        depth_advance_mm=15.0,
        depth_stop_mm=10.0,
    )
    unet_package.video()
