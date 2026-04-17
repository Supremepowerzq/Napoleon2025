# -*- coding: utf-8 -*-
"""
main2026-RL-P0.py - RL数据采集P0阶段
=========================================
功能：深度区域分割 + 无目标自主导航

核心算法：
1. DepthPathFinder - 深度图区域分割与岔路口检测
2. 无目标导航模式 - 基于深度最深区域引导机器人前进

Author: ZQ
Date: 2026-04-13
"""

import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import time
import cv2
import numpy as np
import threading
import queue
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass
from collections import deque

from transitions import Machine
from Interface.JoystickInterfaceV2 import XboxController
from Interface.SerialInterface import find_rmd_motor_port
from Interface.RmdInterfaceV2 import RmdMotor

from ToolKits.ToolBox import get_time
from ToolKits.Timer import busy_maintain_target_frequency
from config import BAUDRATE, TIMEOUT, get_config, update_config

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import matplotlib
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

from ActionRecorder import ActionRecorder, ActionPlayer

# ============================================================
# 总开关
# ============================================================
CAMERA_SWITCH = True
ROBOT_SWITCH = True

# ============================================================
# 电机控制参数
# ============================================================
FORWARD_COEFF = 100
TURN_COEFF = 50
HORIZONTAL_CLOCKWISE_SIGN = 1.0

# ============================================================
# 深度导航参数
# ============================================================
# 深度分割参数
DEPTH_BLUR_KERNEL = 5
DEPTH_THRESHOLD_RATIO = 0.6
MIN_REGION_AREA = 200
MAX_REGIONS = 10

# 导航参数
SAFE_DEPTH_THRESHOLD_MM = 50.0
SAFE_FORWARD_SPEED = 30
SAFE_TURN_SPEED = 15
MIN_SAFE_DEPTH_MM = 15.0
MAX_FORWARD_DEPTH_MM = 100.0

# 岔路选择参数
PATH_SELECT_MODE = "deepest"  # deepest / center / widest
PATH_VERTICAL_SPLIT = 3
PATH_HORIZONTAL_SPLIT = 5

# ============================================================
# 圆形ROI参数 (与predict_2026_wqx.py保持一致)
# ============================================================
ROI_SIDE = 788
CIRCLE_CENTER_X = 400
CIRCLE_CENTER_Y = 230
CIRCLE_RADIUS = 115


# ============================================================
# 深度路径分析器 - DepthPathFinder
# ============================================================

@dataclass
class PathRegion:
    """检测到的可通行区域"""
    contour: np.ndarray
    centroid: Tuple[int, int]
    area: float
    mean_depth_mm: float
    min_depth_mm: float
    max_depth_mm: float
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    direction: str  # "left", "center", "right"


class DepthPathFinder:
    """
    深度路径分析器
    功能：
    1. 深度图预处理
    2. 深度区域分割
    3. 岔路口检测与选择
    4. 安全导航方向计算
    """

    def __init__(
        self,
        roi_side: int = ROI_SIDE,
        blur_kernel: int = DEPTH_BLUR_KERNEL,
        depth_threshold_ratio: float = DEPTH_THRESHOLD_RATIO,
        min_region_area: int = MIN_REGION_AREA,
        vertical_split: int = PATH_VERTICAL_SPLIT,
        horizontal_split: int = PATH_HORIZONTAL_SPLIT,
        min_safe_depth_mm: float = MIN_SAFE_DEPTH_MM,
        max_forward_depth_mm: float = MAX_FORWARD_DEPTH_MM,
    ):
        self.roi_side = roi_side
        self.blur_kernel = blur_kernel
        self.depth_threshold_ratio = depth_threshold_ratio
        self.min_region_area = min_region_area
        self.vertical_split = vertical_split
        self.horizontal_split = horizontal_split
        self.min_safe_depth_mm = min_safe_depth_mm
        self.max_forward_depth_mm = max_forward_depth_mm

        self._circle_mask = self._create_circle_mask()

        # 深度平滑历史
        self._depth_history: deque = deque(maxlen=5)

        # 上一帧的方向(用于稳定性)
        self._last_direction: Optional[str] = None
        self._direction_stable_count: int = 0

    def _create_circle_mask(self) -> np.ndarray:
        """创建圆形掩码"""
        side = self.roi_side
        cx = side // 2
        cy = side // 2
        r = side // 2
        Y, X = np.ogrid[:side, :side]
        return (((X - cx) ** 2 + (Y - cy) ** 2) <= r * r).astype(np.uint8)

    def _preprocess_depth(self, depth: np.ndarray) -> np.ndarray:
        """
        深度图预处理
        1. 应用圆形掩码
        2. 高斯平滑
        3. 时间滤波
        """
        if depth is None or depth.size == 0:
            return np.zeros((self.roi_side, self.roi_side), dtype=np.float32)

        h, w = depth.shape
        if h != self.roi_side or w != self.roi_side:
            depth = cv2.resize(depth, (self.roi_side, self.roi_side))

        depth_masked = depth * self._circle_mask

        depth_masked[depth_masked == 0] = np.nan

        depth_blur = cv2.GaussianBlur(
            depth_masked.astype(np.float32),
            (self.blur_kernel, self.blur_kernel),
            0
        )

        depth_blur = np.nan_to_num(depth_blur, nan=0.0)

        if len(self._depth_history) > 0:
            depth_median = np.median(np.stack(self._depth_history), axis=0)
            depth_blur = 0.7 * depth_blur + 0.3 * depth_median

        self._depth_history.append(depth_blur.copy())

        depth_blur = depth_blur * self._circle_mask

        return depth_blur

    def _find_deep_regions(self, depth: np.ndarray) -> List[PathRegion]:
        """
        深度区域分割 - 找出深度较深的区域
        使用阈值分割+轮廓检测的方法
        """
        if depth is None or np.max(depth) == 0:
            return []

        depth_valid = depth[self._circle_mask > 0]
        if len(depth_valid) == 0 or np.max(depth_valid) == 0:
            return []

        max_depth = np.max(depth_valid)
        min_valid_depth = np.min(depth_valid[depth_valid > 0]) if len(depth_valid[depth_valid > 0]) > 0 else 0

        threshold = min_valid_depth + (max_depth - min_valid_depth) * self.depth_threshold_ratio

        _, depth_thresh = cv2.threshold(
            depth.astype(np.uint8),
            int(threshold),
            255,
            cv2.THRESH_BINARY
        )

        kernel = np.ones((3, 3), np.uint8)
        depth_thresh = cv2.morphologyEx(depth_thresh, cv2.MORPH_CLOSE, kernel)
        depth_thresh = cv2.morphologyEx(depth_thresh, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            depth_thresh * self._circle_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        regions = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_region_area:
                continue

            mask_temp = np.zeros((self.roi_side, self.roi_side), dtype=np.uint8)
            cv2.drawContours(mask_temp, [contour], -1, 255, -1)

            mask_temp = cv2.bitwise_and(mask_temp, mask_temp, mask=self._circle_mask)

            depth_region = depth[mask_temp > 0]
            if len(depth_region) == 0:
                continue

            valid_depths = depth_region[depth_region > 0]
            if len(valid_depths) == 0:
                continue

            mean_depth = float(np.mean(valid_depths))
            min_depth = float(np.min(valid_depths))
            max_depth_val = float(np.max(valid_depths))

            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            center_x = self.roi_side // 2
            if cx < center_x * 0.6:
                direction = "left"
            elif cx > center_x * 1.4:
                direction = "right"
            else:
                direction = "center"

            region = PathRegion(
                contour=contour,
                centroid=(cx, cy),
                area=area,
                mean_depth_mm=mean_depth,
                min_depth_mm=min_depth,
                max_depth_mm=max_depth_val,
                bbox=(x, y, w, h),
                direction=direction
            )
            regions.append(region)

        return regions

    def _find_passable_paths(self, depth: np.ndarray) -> List[Dict]:
        """
        在深度图下半部分寻找可通行的岔路口
        将图像分为左/中/右三个区域，分析每个区域的最深深度
        """
        h, w = depth.shape
        roi_y1 = int(h * 0.4)
        roi_y2 = int(h * 0.95)

        bottom_region = depth[roi_y1:roi_y2, :]
        bottom_mask = self._circle_mask[roi_y1:roi_y2, :]

        third_w = w // 3
        paths = []

        for i, name in enumerate(["left", "center", "right"]):
            region_x1 = i * third_w
            region_x2 = (i + 1) * third_w

            region = bottom_region[:, region_x1:region_x2]
            mask = bottom_mask[:, region_x1:region_x2]

            masked_depth = region[mask > 0]
            if len(masked_depth) == 0:
                continue

            valid_depths = masked_depth[masked_depth > 0]
            if len(valid_depths) == 0:
                continue

            mean_depth = float(np.mean(valid_depths))
            max_depth_val = float(np.max(valid_depths))

            depth_ratio = np.sum(valid_depths > self.min_safe_depth_mm) / len(valid_depths)

            paths.append({
                "name": name,
                "x1": region_x1,
                "x2": region_x2,
                "mean_depth": mean_depth,
                "max_depth": max_depth_val,
                "safe_ratio": depth_ratio,
                "index": i,
                "center_x": (region_x1 + region_x2) // 2,
            })

        return paths

    def select_best_path(self, paths: List[Dict]) -> Optional[Dict]:
        """
        选择最佳通行路径
        策略：
        1. 优先选择深度最大的区域（最可能通往深处）
        2. 考虑安全比率（深度大于阈值的像素比例）
        3. 中间优先（如果深度相近，优先中间）
        """
        if not paths:
            return None

        for p in paths:
            score = p["mean_depth"] * 0.7 + p["max_depth"] * 0.3
            if p["safe_ratio"] < 0.3:
                score *= 0.5
            if p["name"] == "center":
                score *= 1.2
            p["score"] = score

        paths_sorted = sorted(paths, key=lambda x: x["score"], reverse=True)

        best = paths_sorted[0]
        direction_map = {"left": -1, "center": 0, "right": 1}
        best["turn_direction"] = direction_map.get(best["name"], 0)

        return best

    def compute_navigation(
        self,
        depth: np.ndarray,
        mode: str = "deepest"
    ) -> Dict[str, Any]:
        """
        计算导航指令

        Returns:
            {
                "has_path": bool,
                "best_path": Dict or None,
                "forward_speed": float,
                "turn_direction": float,  # -1=left, 0=center, 1=right
                "turn_speed": float,
                "min_safe_depth": float,
                "regions": List[PathRegion],
                "paths": List[Dict],
                "direction": str,
            }
        """
        result = {
            "has_path": False,
            "best_path": None,
            "forward_speed": 0.0,
            "turn_direction": 0.0,
            "turn_speed": 0.0,
            "min_safe_depth": 0.0,
            "regions": [],
            "paths": [],
            "direction": "stop",
        }

        if depth is None:
            return result

        depth_preprocessed = self._preprocess_depth(depth)

        regions = self._find_deep_regions(depth_preprocessed)
        result["regions"] = regions

        paths = self._find_passable_paths(depth_preprocessed)
        result["paths"] = paths

        if not paths:
            return result

        best_path = self.select_best_path(paths)
        result["best_path"] = best_path

        if best_path is None:
            return result

        result["has_path"] = True
        result["min_safe_depth"] = best_path["mean_depth"]

        if best_path["mean_depth"] < self.min_safe_depth_mm:
            result["forward_speed"] = 0.0
            result["turn_speed"] = SAFE_TURN_SPEED * best_path["turn_direction"]
            result["direction"] = "stop_safe"
        elif best_path["mean_depth"] > self.max_forward_depth_mm:
            depth_ratio = (best_path["mean_depth"] - self.max_forward_depth_mm) / self.max_forward_depth_mm
            depth_ratio = min(depth_ratio, 1.0)
            result["forward_speed"] = SAFE_FORWARD_SPEED * (0.3 + 0.7 * depth_ratio)
            result["turn_speed"] = SAFE_TURN_SPEED * best_path["turn_direction"] * 0.5
            result["direction"] = f"forward_{best_path['name']}"
        else:
            depth_range = self.max_forward_depth_mm - self.min_safe_depth_mm
            depth_norm = (best_path["mean_depth"] - self.min_safe_depth_mm) / depth_range
            depth_norm = max(0, min(1, depth_norm))

            speed_scale = 0.3 + 0.7 * depth_norm
            result["forward_speed"] = SAFE_FORWARD_SPEED * speed_scale
            result["turn_speed"] = SAFE_TURN_SPEED * best_path["turn_direction"]
            result["direction"] = f"forward_{best_path['name']}"

        turn_dir = best_path["turn_direction"]
        if self._last_direction == best_path["name"]:
            self._direction_stable_count += 1
        else:
            self._direction_stable_count = 0
            self._last_direction = best_path["name"]

        if self._direction_stable_count < 3:
            result["turn_speed"] *= 0.5

        return result


# ============================================================
# 深度导航可视化
# ============================================================

class DepthNavVisualizer:
    """
    深度导航可视化
    在深度图上绘制分割结果和导航指示
    """

    def __init__(self, roi_side: int = ROI_SIDE):
        self.roi_side = roi_side

    def draw_navigation_overlay(
        self,
        depth_vis: np.ndarray,
        nav_result: Dict[str, Any],
        regions: List[PathRegion] = None,
        paths: List[Dict] = None,
    ) -> np.ndarray:
        """
        绘制导航叠加层

        Args:
            depth_vis: 深度可视化图 [H, W, 3]
            nav_result: 导航结果字典
            regions: 深度区域列表
            paths: 岔路分析结果

        Returns:
            可视化后的图像
        """
        if depth_vis is None:
            return np.zeros((self.roi_side, self.roi_side, 3), dtype=np.uint8)

        vis = depth_vis.copy()
        h, w = vis.shape[:2]

        if regions is None:
            regions = nav_result.get("regions", [])
        if paths is None:
            paths = nav_result.get("paths", [])

        for region in regions:
            color = self._get_direction_color(region.direction)
            cv2.drawContours(vis, [region.contour], -1, color, 2)
            cx, cy = region.centroid
            cv2.circle(vis, (cx, cy), 5, color, -1)
            text = f"{region.mean_depth_mm:.0f}mm"
            cv2.putText(vis, text, (cx + 10, cy),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

        third_w = w // 3
        for i, name in enumerate(["左", "中", "右"]):
            x1 = i * third_w
            x2 = (i + 1) * third_w
            cv2.rectangle(vis, (x1, h//2), (x2, h-10), (100, 100, 100), 1)

        if nav_result.get("has_path") and nav_result.get("best_path"):
            best = nav_result["best_path"]
            x_pos = best["center_x"]
            y_pos = h - 30

            arrow_color = self._get_direction_color(best["name"])
            if best["name"] == "left":
                arrow_points = np.array([
                    [x_pos + 20, y_pos],
                    [x_pos - 10, y_pos - 15],
                    [x_pos - 10, y_pos + 15]
                ], np.int32)
            elif best["name"] == "right":
                arrow_points = np.array([
                    [x_pos - 20, y_pos],
                    [x_pos + 10, y_pos - 15],
                    [x_pos + 10, y_pos + 15]
                ], np.int32)
            else:
                arrow_points = np.array([
                    [x_pos, y_pos - 20],
                    [x_pos - 15, y_pos + 10],
                    [x_pos + 15, y_pos + 10]
                ], np.int32)

            cv2.fillPoly(vis, [arrow_points], arrow_color)

            info_text = f"{best['name']} | {best['mean_depth']:.0f}mm | 速度:{nav_result['forward_speed']:.0f}"
            cv2.putText(vis, info_text, (10, h - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(vis, "无安全路径", (w//2 - 50, h - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        return vis

    def _get_direction_color(self, direction: str) -> Tuple[int, int, int]:
        """获取方向对应的颜色"""
        colors = {
            "left": (255, 100, 100),
            "center": (100, 255, 100),
            "right": (100, 100, 255),
        }
        return colors.get(direction, (200, 200, 200))


# ============================================================
# 视频处理线程 (整合深度导航)
# ============================================================

class IntegratedVisionProcessor:
    """
    整合视觉处理器
    1. UNet目标检测
    2. 深度估计
    3. 深度区域分析
    4. 无目标导航计算
    """

    def __init__(
        self,
        video_path: int = 1,
        enable_depth: bool = True,
        depth_encoder: str = 'vits',
        depth_input_size: int = 518,
        enable_unet: bool = True,
        show_debug: bool = True,
    ):
        self.video_path = video_path
        self.enable_depth = enable_depth and DEPTH_AVAILABLE if 'DEPTH_AVAILABLE' in globals() else enable_depth
        self.depth_encoder = depth_encoder
        self.depth_input_size = depth_input_size
        self.enable_unet = enable_unet
        self.show_debug = show_debug

        self.depth_path_finder = DepthPathFinder()
        self.visualizer = DepthNavVisualizer()

        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_vis: Optional[np.ndarray] = None
        self._latest_nav_result: Dict[str, Any] = {}
        self._has_target: bool = False
        self._target_2d: Optional[Tuple[int, int]] = None
        self._target_3d: Optional[np.ndarray] = None
        self._stone_centroid: Tuple[int, int] = (ROI_SIDE // 2, ROI_SIDE // 2)

        self._lock = threading.Lock()
        self._running = False

        self._unet_package = None
        self._depth_model = None
        self._depth_cmap = None
        self._circle_mask = None
        self._initialized = False

    def _lazy_init(self):
        """延迟初始化（避免模块导入时卡住）"""
        if self._initialized:
            return

        print(f"{get_time()}-开始初始化视觉模块...")

        if self.enable_unet:
            try:
                from predict_2026_wqx import UnetPackage
                self._unet_package = UnetPackage(
                    mode='video',
                    video_path=self.video_path,
                    enable_depth=self.enable_depth,
                    depth_encoder=self.depth_encoder,
                    depth_input_size=self.depth_input_size,
                )
                print(f"{get_time()}-UnetPackage初始化完成 (深度={self.enable_depth})")
            except Exception as e:
                print(f"{get_time()}-UnetPackage初始化失败: {e}")
                self.enable_unet = False

        if self.enable_depth:
            self._depth_cmap = matplotlib.colormaps.get_cmap("Spectral") if MATPLOTLIB_AVAILABLE else None
            self._create_circle_mask_local()

        self._initialized = True
        print(f"{get_time()}-视觉模块初始化完成")

    def _create_circle_mask_local(self):
        """创建本地圆形掩码"""
        side = ROI_SIDE
        cx, cy, r = side // 2, side // 2, side // 2
        Y, X = np.ogrid[:side, :side]
        self._circle_mask = (((X - cx) ** 2 + (Y - cy) ** 2) <= r * r).astype(np.uint8)

    def _process_frame(self, frame: np.ndarray, depth: Optional[np.ndarray] = None) -> Tuple[np.ndarray, Dict]:
        """
        处理单帧

        Returns:
            (display_frame, nav_result)
        """
        if self._unet_package is not None:
            display_frame, depth_result = self._unet_package.process_single_frame(frame, depth)
        else:
            display_frame = frame.copy()
            depth_result = None

        self._analyze_depth_navigation(depth if depth is not None else depth_result)

        return display_frame, self._latest_nav_result

    def _analyze_depth_navigation(self, depth: Optional[np.ndarray]):
        """分析深度图进行无目标导航"""
        if depth is None:
            self._latest_nav_result = {
                "has_path": False,
                "forward_speed": 0.0,
                "turn_speed": 0.0,
                "has_target": self._has_target,
            }
            return

        nav_result = self.depth_path_finder.compute_navigation(depth)
        nav_result["has_target"] = self._has_target

        self._latest_nav_result = nav_result

        if nav_result.get("has_path"):
            update_config('speed_pf', nav_result["forward_speed"])
            update_config('speed_pt', [nav_result["turn_speed"], 0.0])
        else:
            update_config('speed_pf', 0.0)
            update_config('speed_pt', [0.0, 0.0])

    def _normalize_depth_to_uint8(self, depth: np.ndarray) -> np.ndarray:
        """深度归一化为uint8"""
        dmin, dmax = float(depth.min()), float(depth.max())
        if dmax - dmin < 1e-6:
            return np.zeros_like(depth, dtype=np.uint8)
        return ((depth - dmin) / (dmax - dmin) * 255).astype(np.uint8)

    def get_depth_visualization(self) -> Optional[np.ndarray]:
        """获取深度可视化图"""
        with self._lock:
            return self._latest_depth_vis.copy() if self._latest_depth_vis is not None else None

    def get_latest_result(self) -> Dict[str, Any]:
        """获取最新处理结果"""
        with self._lock:
            return {
                "has_target": self._has_target,
                "target_2d": self._target_2d,
                "target_3d": self._target_3d,
                "nav_result": self._latest_nav_result.copy(),
                "stone_centroid": self._stone_centroid,
            }

    def stop(self):
        """停止处理"""
        self._running = False


# ============================================================
# 主程序 - 简化版状态机（用于演示）
# ============================================================

class SimpleRobotController:
    """
    简化版机器人控制器
    包含：无目标导航 + 视觉追踪两种模式
    """

    states = ['Idle', 'ManualControl', 'VisionTracking', 'NoTargetNav', 'PowerOff']
    triggers = ['start_manual', 'start_vision', 'start_notarget_nav', 'stop', 'back']

    def __init__(self):
        self.machine = Machine(
            model=self,
            states=SimpleRobotController.states,
            initial='Idle',
            auto_transitions=False
        )

        for s in SimpleRobotController.states:
            self.machine.add_transition('start_manual', s, 'ManualControl')
            self.machine.add_transition('start_vision', s, 'VisionTracking')
            self.machine.add_transition('start_notarget_nav', s, 'NoTargetNav')
            self.machine.add_transition('stop', s, 'Idle')
            self.machine.add_transition('back', s, 'Idle')

        self.machine.add_transition('power_off', '*', 'PowerOff')

        self.xbox = XboxController()

        self.serial_port = find_rmd_motor_port(0)
        if isinstance(self.serial_port, str):
            self.ser = serial.Serial(self.serial_port, baudrate=BAUDRATE, timeout=TIMEOUT)
        else:
            print(f"{get_time()}-未找到RMD电机串口")
            self.ser = None

        self.motors = RmdMotor(self.ser) if self.ser else None

        self.shutdown_event = threading.Event()
        self.state_thread = threading.Thread(target=self._state_loop, daemon=True)

        self.vision_processor: Optional[IntegratedVisionProcessor] = None

        print(f"{get_time()}-机器人控制器初始化完成")

    def start(self):
        """启动控制器"""
        self.state_thread.start()

    def _state_loop(self):
        """状态循环"""
        print(f"{get_time()}-进入状态循环")
        while not self.shutdown_event.is_set():
            t0 = time.perf_counter()

            try:
                handler = {
                    'Idle': self._loop_idle,
                    'ManualControl': self._loop_manual,
                    'VisionTracking': self._loop_vision,
                    'NoTargetNav': self._loop_notarget_nav,
                    'PowerOff': self._loop_poweroff,
                }.get(self.state)

                if handler:
                    handler()
            except Exception as e:
                print(f"{get_time()}-状态循环异常: {e}")

            busy_maintain_target_frequency(60, t0)

        print(f"{get_time()}-状态循环结束")

    def _loop_idle(self):
        """空闲状态"""
        if self.xbox.is_button_pressed('START'):
            self.start_manual()
        elif self.xbox.is_button_pressed('A'):
            self.start_vision()
        elif self.xbox.is_button_pressed('B'):
            self.start_notarget_nav()
        elif self.xbox.is_button_pressed('BACK'):
            self.power_off()

        print(f"\r[{self.state}] 按键: START=手动  A=视觉追踪  B=无目标导航  BACK=关机", end="")

    def _loop_manual(self):
        """手动控制"""
        if self.xbox.is_button_pressed('START'):
            self.stop()

        forward, turn_lr, turn_ud = self._read_joystick()

        if self.motors:
            self.motors.move(forward)
            self.motors.turn([turn_lr, turn_ud])

        print(f"\r[{self.state}] 手动 F={forward:.1f} LR={turn_lr:.1f} UD={turn_ud:.1f}", end="")

    def _loop_vision(self):
        """视觉追踪模式"""
        if self.xbox.is_button_pressed('START'):
            self.stop()

        speed_pf = get_config('speed_pf')
        speed_pt = get_config('speed_pt')

        if isinstance(speed_pt, list) and len(speed_pt) >= 2:
            vx, vy = speed_pt[0], speed_pt[1]
        else:
            vx, vy = 0.0, 0.0

        forward = speed_pf if speed_pf else 0.0

        if self.motors:
            self.motors.move(forward)
            self.motors.turn([vx * HORIZONTAL_CLOCKWISE_SIGN, vy])

        result = self.vision_processor.get_latest_result() if self.vision_processor else {}
        has_target = result.get("has_target", False)
        nav = result.get("nav_result", {})

        target_info = f"目标({'有' if has_target else '无'})"
        if nav.get("has_path"):
            target_info += f" | 导航:{nav.get('direction', '?')} | 速度:{nav.get('forward_speed', 0):.0f}"

        print(f"\r[{self.state}] {target_info} F={forward:.1f}", end="")

    def _loop_notarget_nav(self):
        """无目标导航模式"""
        if self.xbox.is_button_pressed('START'):
            self.stop()

        if self.vision_processor:
            result = self.vision_processor.get_latest_result()
            nav = result.get("nav_result", {})
            has_target = result.get("has_target", False)

            if has_target:
                print(f"\r[{self.state}] 检测到目标，切换到视觉追踪模式", end="")
                time.sleep(0.5)
                self.start_vision()
                return

            forward = nav.get("forward_speed", 0.0)
            turn = nav.get("turn_speed", 0.0)

            if self.motors:
                self.motors.move(forward)
                self.motors.turn([turn * HORIZONTAL_CLOCKWISE_SIGN, 0.0])

            depth = nav.get("min_safe_depth", 0)
            direction = nav.get("direction", "stop")
            has_path = nav.get("has_path", False)

            path_info = f"{'有路径' if has_path else '无路径'} | {direction} | 深度:{depth:.0f}mm | F={forward:.0f} T={turn:.0f}"
            print(f"\r[{self.state}] {path_info}", end="")
        else:
            print(f"\r[{self.state}] 视觉处理器未初始化", end="")

    def _loop_poweroff(self):
        """关机"""
        print(f"\n{get_time()}-正在关闭...")
        self.shutdown_event.set()

    def _read_joystick(self) -> Tuple[float, float, float]:
        """读取手柄输入"""
        forward = self.xbox.get_thumb_y() * FORWARD_COEFF
        turn_lr = self.xbox.get_thumb_rx() * TURN_COEFF
        turn_ud = self.xbox.get_thumb_ry() * TURN_COEFF * 0.5
        return forward, turn_lr, turn_ud

    def close(self):
        """关闭资源"""
        self.shutdown_event.set()
        if self.ser:
            self.ser.close()


# ============================================================
# 演示模式 (无需硬件)
# ============================================================

class DemoDepthNavigation:
    """
    演示模式 - 仅使用深度图进行导航演示
    """

    def __init__(self, video_path: int = 1):
        self.video_path = video_path
        self.depth_path_finder = DepthPathFinder()
        self.visualizer = DepthNavVisualizer()

        self._running = False
        self._window_name = "Depth Navigation Demo"

    def run(self):
        """运行演示"""
        print(f"{get_time()}-启动深度导航演示模式")
        print(f"{get_time()}-按键: Q=退出  S=截取深度分析图")

        cv2.namedWindow(self._window_name)
        cv2.namedWindow("Depth Analysis")

        capture = cv2.VideoCapture(self.video_path)
        if not capture.isOpened():
            print(f"{get_time()}-无法打开摄像头")
            return

        fps = 0.0
        frame_count = 0
        screenshot_taken = False

        while True:
            t1 = time.time()

            ret, frame = capture.read()
            if not ret:
                print(f"{get_time()}-摄像头读取失败")
                break

            frame_count += 1

            depth = self._generate_demo_depth(frame.shape)

            nav_result = self.depth_path_finder.compute_navigation(depth)

            depth_u8 = self._normalize_depth(depth)
            depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)

            roi_frame = self._crop_to_circle(frame)
            depth_vis = self._crop_to_circle(depth_color)

            nav_overlay = self.visualizer.draw_navigation_overlay(
                depth_vis, nav_result
            )

            info_frame = self._create_info_panel(nav_result)

            combined = np.hstack([roi_frame, nav_overlay, info_frame])

            cv2.imshow(self._window_name, combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self._save_screenshot(combined, nav_result)
                screenshot_taken = True

            fps = (fps + (1. / (time.time() - t1))) / 2

            status = f"{'有路径' if nav_result['has_path'] else '无路径'} | {nav_result['direction']} | F={nav_result['forward_speed']:.0f} T={nav_result['turn_speed']:.0f}"
            print(f"\rFPS:{fps:.1f} | {status} | 帧:{frame_count} | {'[截图已保存]' if screenshot_taken else ''}", end="")

        capture.release()
        cv2.destroyAllWindows()
        print(f"\n{get_time()}-演示结束")

    def _generate_demo_depth(self, shape) -> np.ndarray:
        """生成演示用深度图（可用真实深度替代）"""
        h, w = shape[:2]
        h = min(h, ROI_SIDE)
        w = min(w, ROI_SIDE)

        y, x = np.ogrid[:h, :w]
        cx, cy, r = w // 2, h // 2, min(w, h) // 2

        base_depth = np.ones((h, w), dtype=np.float32) * 30.0

        dist = np.sqrt((x - cx)**2 + (y - cy)**2)
        circle_mask = (dist <= r).astype(np.float32)

        depth_gradient = (1 - dist / r) * 50.0
        base_depth = (base_depth + depth_gradient) * circle_mask

        noise = np.random.normal(0, 2, (h, w)).astype(np.float32)
        base_depth = base_depth + noise * circle_mask

        side = ROI_SIDE
        cx_s, cy_s, r_s = side // 2, side // 2, side // 2
        Y, X = np.ogrid[:side, :side]
        circle_mask_s = (((X - cx_s) ** 2 + (Y - cy_s) ** 2) <= r_s * r_s).astype(np.float32)

        if h != ROI_SIDE or w != ROI_SIDE:
            base_depth = cv2.resize(base_depth, (ROI_SIDE, ROI_SIDE))
            circle_mask = cv2.resize(circle_mask, (ROI_SIDE, ROI_SIDE))

        return base_depth

    def _crop_to_circle(self, image: np.ndarray) -> np.ndarray:
        """裁剪为圆形"""
        side = ROI_SIDE
        h, w = image.shape[:2]

        if h != side or w != side:
            image = cv2.resize(image, (side, side))

        cx, cy, r = side // 2, side // 2, side // 2
        Y, X = np.ogrid[:side, :side]
        circle_mask = (((X - cx) ** 2 + (Y - cy) ** 2) <= r * r).astype(np.uint8)

        if len(image.shape) == 3:
            result = cv2.bitwise_and(image, image, mask=circle_mask)
        else:
            result = cv2.bitwise_and(image, image, mask=circle_mask)

        return result

    def _normalize_depth(self, depth: np.ndarray) -> np.ndarray:
        """归一化深度图"""
        d_min, d_max = float(depth.min()), float(depth.max())
        if d_max - d_min < 1e-6:
            return np.zeros_like(depth, dtype=np.uint8)
        return ((depth - d_min) / (d_max - d_min) * 255).astype(np.uint8)

    def _create_info_panel(self, nav_result: Dict) -> np.ndarray:
        """创建信息面板"""
        h, w = ROI_SIDE, 300
        panel = np.zeros((h, w, 3), dtype=np.uint8) + 30

        y = 30
        cv2.putText(panel, "=== 导航信息 ===", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += 35

        status = "有安全路径" if nav_result.get("has_path") else "无安全路径"
        color = (0, 255, 0) if nav_result.get("has_path") else (0, 0, 255)
        cv2.putText(panel, f"状态: {status}", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        y += 25

        cv2.putText(panel, f"方向: {nav_result.get('direction', 'stop')}", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        y += 25

        cv2.putText(panel, f"前进速度: {nav_result.get('forward_speed', 0):.1f}", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        y += 25

        cv2.putText(panel, f"转向速度: {nav_result.get('turn_speed', 0):.1f}", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        y += 25

        cv2.putText(panel, f"最小安全深度: {nav_result.get('min_safe_depth', 0):.1f}mm", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        y += 35

        cv2.putText(panel, "=== 岔路分析 ===", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += 35

        paths = nav_result.get("paths", [])
        if paths:
            for p in paths:
                name_map = {"left": "左", "center": "中", "right": "右"}
                text = f"{name_map.get(p['name'], p['name'])}: {p['mean_depth']:.0f}mm"
                score = p.get('score', 0)
                if nav_result.get("best_path") and p['name'] == nav_result["best_path"]["name"]:
                    cv2.putText(panel, f"> {text}", (10, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
                else:
                    cv2.putText(panel, f"  {text}", (10, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
                y += 22
        else:
            cv2.putText(panel, "  未检测到岔路", (10, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)

        return panel

    def _save_screenshot(self, frame: np.ndarray, nav_result: Dict):
        """保存截图"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"depth_nav_screenshot_{timestamp}.png"
        cv2.imwrite(filename, frame)
        print(f"\n{get_time()}-截图已保存: {filename}")


# ============================================================
# 主入口
# ============================================================

def main():
    print("=" * 60)
    print("  main2026-RL-P0 - 深度区域分割 + 无目标导航")
    print("=" * 60)
    print("功能说明:")
    print("  1. DepthPathFinder: 深度图区域分割算法")
    print("  2. 无目标导航: 当UNet未检测到目标时")
    print("     - 分析深度图找出最深区域")
    print("     - 将图像分为左/中/右三个岔路")
    print("     - 选择最安全的通行方向")
    print("     - 计算前进速度和转向角度")
    print()
    print("使用方法:")
    print("  演示模式: python main2026-RL-P0.py --demo")
    print("  完整模式: python main2026-RL-P0.py --full")
    print("  深度分析: python main2026-RL-P0.py --depth-only")
    print()
    print("按键说明:")
    print("  Q = 退出程序")
    print("  S = 保存当前截图")
    print("=" * 60)

    import argparse
    parser = argparse.ArgumentParser(description='深度区域分割导航演示')
    parser.add_argument('--demo', action='store_true', help='演示模式(无需硬件)')
    parser.add_argument('--full', action='store_true', help='完整模式(需要硬件)')
    parser.add_argument('--depth-only', action='store_true', help='仅深度分析模式')
    parser.add_argument('--camera', type=int, default=1, help='摄像头索引')
    args = parser.parse_args()

    if args.demo or args.depth_only:
        demo = DemoDepthNavigation(video_path=args.camera)
        demo.run()

    elif args.full:
        controller = SimpleRobotController()
        controller.start()

        try:
            while not controller.shutdown_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            print(f"\n{get_time()}-收到中断信号")
        finally:
            controller.close()

    else:
        print("请指定运行模式: --demo / --full / --depth-only")


if __name__ == "__main__":
    main()
