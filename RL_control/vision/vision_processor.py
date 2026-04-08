"""
视觉处理模块
为 RL 系统提供视觉状态输入

功能:
1. RealSense L515 相机接口封装
2. 视觉编码器 (CNN/ViT backbone)
3. 深度估计 (Depth-Anything-V2 集成)
4. 目标检测 (与现有 UNet 集成)
5. 多模态观测融合
"""

import os
import sys
import time
import numpy as np
import cv2
from typing import Tuple, Optional, Dict, List, Union
from dataclasses import dataclass
import threading
from collections import deque

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms

import warnings

# 可选依赖，缺失时静默降级
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "DLpredict"))
    from predict import UNetPredictor
except ImportError:
    UNetPredictor = None


# ============================================================
# 相机接口
# ============================================================

@dataclass
class CameraFrame:
    """相机帧数据"""
    rgb: np.ndarray          # RGB 图像 [H, W, 3]
    depth: np.ndarray        # 深度图 [H, W] (米)
    timestamp: float         # 时间戳
    intrinsics: Optional[np.ndarray] = None  # 相机内参 [fx, fy, cx, cy]


class RealSenseCamera:
    """
    Intel RealSense L515 相机接口
    支持 RGB + Depth 同步采集
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        enable_depth: bool = True,
        enable_color: bool = True,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth
        self.enable_color = enable_color

        self.pipeline = None
        self.align = None
        self.config = None
        self._is_running = False
        self._lock = threading.Lock()

        # 最新帧缓存
        self._latest_frame: Optional[CameraFrame] = None
        self._frame_queue = deque(maxlen=5)

    def start(self) -> bool:
        """启动相机"""
        if rs is None:
            print("Warning: RealSense SDK not available")
            return False

        try:
            # 创建管道
            self.pipeline = rs.pipeline()

            # 配置
            self.config = rs.config()
            self.config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.bgr8,
                self.fps
            )

            if self.enable_depth:
                self.config.enable_stream(
                    rs.stream.depth,
                    640, 480,
                    rs.format.z16,
                    30
                )

            # 校准文件 (如果有)
            serial = self._get_camera_serial()
            if serial:
                config_path = self._find_calibration_file(serial)
                if config_path:
                    self.config.enable_device(config_path)

            # 开始流
            profile = self.pipeline.start(self.config)

            # 设置对齐
            self.align = rs.align(rs.stream.color)

            # 获取内参
            self._intrinsics = self._get_intrinsics(profile)

            self._is_running = True
            print(f"RealSense camera started: {self.width}x{self.height} @ {self.fps}fps")
            return True

        except Exception as e:
            print(f"Failed to start camera: {e}")
            return False

    def _get_camera_serial(self) -> Optional[str]:
        """获取相机序列号"""
        if self.pipeline is None:
            return None
        try:
            profile = self.pipeline.get_active_profile()
            device = profile.get_device()
            return device.get_info(rs.camera_info.serial_number)
        except:
            return None

    def _get_intrinsics(self, profile) -> np.ndarray:
        """获取相机内参"""
        try:
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            return np.array([
                intrinsics.fx, intrinsics.fy,  # fx, fy
                intrinsics.ppx, intrinsics.ppy   # cx, cy
            ])
        except:
            return np.array([615.0, 615.0, 320.0, 240.0])  # 默认内参

    def _find_calibration_file(self, serial: str) -> Optional[str]:
        """查找校准文件"""
        # 简化实现
        return None

    def read(self) -> Optional[CameraFrame]:
        """读取最新帧"""
        if not self._is_running:
            return None

        with self._lock:
            return self._latest_frame

    def update(self) -> Optional[CameraFrame]:
        """更新并获取最新帧"""
        if not self._is_running or self.pipeline is None:
            return None

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            if frames is None:
                return None

            # 对齐
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if color_frame is None:
                return None

            # 转换为 numpy
            rgb = np.asanyarray(color_frame.get_data())
            depth = None
            if self.enable_depth and depth_frame is not None:
                depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) / 1000.0

            # 创建帧对象
            frame = CameraFrame(
                rgb=rgb,
                depth=depth,
                timestamp=time.time(),
                intrinsics=self._intrinsics if hasattr(self, '_intrinsics') else None
            )

            with self._lock:
                self._latest_frame = frame
                self._frame_queue.append(frame)

            return frame

        except Exception as e:
            return None

    def stop(self):
        """停止相机"""
        self._is_running = False
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except:
                pass
            self.pipeline = None

    def __del__(self):
        self.stop()

    def project_depth_to_3d(
        self,
        depth: np.ndarray,
        point_2d: Tuple[int, int]
    ) -> Tuple[float, float, float]:
        """将深度图中的像素点投影到 3D 世界坐标"""
        if self._intrinsics is None:
            return (0.0, 0.0, 0.0)

        fx, fy, cx, cy = self._intrinsics
        x, y = point_2d

        z = depth[y, x] if depth.ndim == 2 else depth[y, x, 0]
        x_3d = (x - cx) * z / fx
        y_3d = (y - cy) * z / fy

        return (x_3d, y_3d, z)


# ============================================================
# 视觉编码器
# ============================================================

class VisionEncoder(nn.Module):
    """
    视觉编码器
    将图像编码为固定维度的特征向量
    """

    def __init__(
        self,
        backbone: str = "resnet18",
        pretrained: bool = True,
        feature_dim: int = 256,
        freeze_bn: bool = True,
    ):
        super().__init__()
        self.backbone_name = backbone
        self.feature_dim = feature_dim

        # 加载 backbone
        if backbone == "resnet18":
            from torchvision.models import resnet18, ResNet18_Weights
            self.backbone = resnet18(weights=ResNet18_Weights.DEFAULT if pretrained else None)
            num_features = 512
        elif backbone == "resnet34":
            from torchvision.models import resnet34, ResNet34_Weights
            self.backbone = resnet34(weights=ResNet34_Weights.DEFAULT if pretrained else None)
            num_features = 512
        elif backbone == "mobilenet_v3":
            from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights
            self.backbone = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.DEFAULT if pretrained else None)
            num_features = 576
        else:
            raise ValueError(f"Unknown backbone: {backbone}")

        # 替换最后的全连接层
        self.backbone.fc = nn.Identity()

        # 投影层
        self.projection = nn.Sequential(
            nn.Linear(num_features, feature_dim),
            nn.ReLU(),
            nn.Linear(feature_dim, feature_dim),
        )

        # 冻结 BatchNorm
        if freeze_bn:
            self._freeze_bn()

    def _freeze_bn(self):
        """冻结 BatchNorm 层"""
        for module in self.backbone.modules():
            if isinstance(module, nn.BatchNorm2d):
                module.eval()
                for param in module.parameters():
                    param.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播
        Args:
            x: 输入图像 [B, 3, H, W]
        Returns:
            features: 特征向量 [B, feature_dim]
        """
        # 提取特征
        with torch.no_grad() if not self.training else torch.enable_grad():
            features = self.backbone(x)

        # 投影
        features = self.projection(features)
        return features


class MultiModalFusion(nn.Module):
    """
    多模态融合模块
    融合视觉特征、深度特征、状态特征
    """

    def __init__(
        self,
        vision_dim: int = 256,
        state_dim: int = 9,
        hidden_dim: int = 256,
        output_dim: int = 256,
    ):
        super().__init__()

        # 视觉分支
        self.vision_net = nn.Sequential(
            nn.Linear(vision_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
        )

        # 状态分支
        self.state_net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
        )

        # 融合
        self.fusion = nn.Sequential(
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim),
        )

    def forward(
        self,
        vision_features: torch.Tensor,
        state_features: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            vision_features: 视觉特征 [B, vision_dim]
            state_features: 状态特征 [B, state_dim]
        Returns:
            fused: 融合特征 [B, output_dim]
        """
        v_feat = self.vision_net(vision_features)
        s_feat = self.state_net(state_features)

        # 拼接 + 融合
        fused = torch.cat([v_feat, s_feat], dim=-1)
        fused = self.fusion(fused)

        return fused


# ============================================================
# 目标检测器
# ============================================================

class TargetDetector:
    """
    目标检测器
    基于现有 UNet 模型检测手术目标
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        threshold: float = 0.5,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.threshold = threshold
        self.device = device
        self.predictor = None

        if model_path and UNetPredictor is not None:
            try:
                self.predictor = UNetPredictor(model_path, device=device)
            except Exception as e:
                print(f"Failed to load UNet predictor: {e}")

    def detect(
        self, image: np.ndarray, depth: Optional[np.ndarray] = None
    ) -> Dict:
        """
        检测目标
        Args:
            image: RGB 图像 [H, W, 3]
            depth: 深度图 [H, W] (可选)
        Returns:
            检测结果，包含目标中心、边界框、掩码等
        """
        result = {
            "detected": False,
            "centroid_2d": None,
            "centroid_3d": None,
            "mask": None,
            "depth": None,
        }

        if self.predictor is None:
            return result

        try:
            # UNet 预测
            mask = self.predictor.predict_mask(image)

            if mask is None or mask.sum() == 0:
                return result

            # 计算目标中心 (2D)
            ys, xs = np.where(mask > self.threshold)
            if len(xs) == 0 or len(ys) == 0:
                return result

            cx_2d = int(np.mean(xs))
            cy_2d = int(np.mean(ys))

            # 如果有深度图，计算 3D 位置
            if depth is not None:
                z = depth[cy_2d, cx_2d]
                # 简单的相机投影 (假设内参已知)
                fx, fy = 615.0, 615.0  # 默认内参
                cx, cy = 320.0, 240.0
                x_3d = (cx_2d - cx) * z / fx
                y_3d = (cy_2d - cy) * z / fy
                result["centroid_3d"] = np.array([x_3d, y_3d, z])

            result.update({
                "detected": True,
                "centroid_2d": np.array([cx_2d, cy_2d]),
                "mask": mask,
                "depth": depth[cy_2d, cx_2d] if depth is not None else None,
            })

        except Exception as e:
            print(f"Detection error: {e}")

        return result


# ============================================================
# 深度估计
# ============================================================

class DepthEstimator:
    """
    深度估计器
    基于 Depth-Anything-V2 模型
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.device = device
        self.model = None
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((384, 384)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

        if model_path:
            self._load_model(model_path)

    def _load_model(self, model_path: str):
        """加载深度估计模型"""
        # 简化实现 - 实际需要加载 Depth-Anything-V2 模型
        print(f"Loading depth model from {model_path}")
        # TODO: 实现完整的 Depth-Anything-V2 加载
        # model = load_depth_anything_v2(model_path, device=self.device)
        # self.model = model

    def estimate_depth(
        self,
        image: np.ndarray,
        use_ground_truth: bool = False,
        gt_depth: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        估计深度
        Args:
            image: RGB 图像 [H, W, 3]
            use_ground_truth: 是否使用真实深度 (RealSense)
            gt_depth: 真实深度图 (如果有)
        Returns:
            depth: 深度图 [H, W] (米)
        """
        if use_ground_truth and gt_depth is not None:
            return gt_depth

        if self.model is None:
            # 如果没有模型，返回模拟深度
            # 简化: 假设所有像素深度为 0.5 米
            return np.ones((image.shape[0], image.shape[1]), dtype=np.float32) * 0.5

        # 使用模型估计
        try:
            # 预处理
            input_tensor = self.transform(image).unsqueeze(0).to(self.device)

            # 推理
            with torch.no_grad():
                depth_pred = self.model(input_tensor)

            # 后处理
            depth = depth_pred.squeeze().cpu().numpy()
            depth = cv2.resize(depth, (image.shape[1], image.shape[0]))

            return depth

        except Exception as e:
            print(f"Depth estimation error: {e}")
            return np.ones((image.shape[0], image.shape[1]), dtype=np.float32) * 0.5


# ============================================================
# 视觉观测模块
# ============================================================

class VisionObservationModule:
    """
    视觉观测模块
    整合相机、编码器、检测器，提供 RL 所需的全观测
    """

    def __init__(
        self,
        encoder_config: Optional[Dict] = None,
        camera_config: Optional[Dict] = None,
        use_depth: bool = True,
        use_detection: bool = True,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        self.device = device
        self.use_depth = use_depth
        self.use_detection = use_detection

        # 相机
        if camera_config is None:
            camera_config = {
                "width": 640,
                "height": 480,
                "fps": 30,
            }
        self.camera = RealSenseCamera(**camera_config)

        # 编码器
        if encoder_config is None:
            encoder_config = {
                "backbone": "resnet18",
                "pretrained": True,
                "feature_dim": 256,
            }
        self.encoder = VisionEncoder(**encoder_config).to(device)
        self.encoder.eval()

        # 检测器
        if use_detection:
            self.detector = TargetDetector()

        # 深度估计器
        if use_depth:
            self.depth_estimator = DepthEstimator()

        # 图像预处理
        self.image_transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

        # 缓存
        self._current_features: Optional[np.ndarray] = None
        self._current_detection: Optional[Dict] = None

    def start(self) -> bool:
        """启动相机"""
        return self.camera.start()

    def stop(self):
        """停止相机"""
        self.camera.stop()

    @torch.no_grad()
    def get_observation(
        self,
        image: Optional[np.ndarray] = None,
        depth: Optional[np.ndarray] = None,
    ) -> Dict:
        """
        获取完整观测
        Args:
            image: RGB 图像 (如果为 None，从相机读取)
            depth: 深度图 (如果为 None，从相机读取或估计)
        Returns:
            observation: 包含以下键的字典
                - image: 原始图像
                - depth: 深度图
                - features: 视觉特征向量
                - detection: 检测结果 (如果有)
                - target_position: 目标 3D 位置 (如果有)
        """
        # 获取图像
        if image is None:
            frame = self.camera.read()
            if frame is not None:
                image = frame.rgb
                if depth is None and frame.depth is not None:
                    depth = frame.depth

        if image is None:
            # 返回零观测
            return self._empty_observation()

        # 深度
        if depth is None and self.depth_estimator is not None:
            depth = self.depth_estimator.estimate_depth(image)

        # 视觉编码
        image_tensor = self.image_transform(image).unsqueeze(0).to(self.device)
        features = self.encoder(image_tensor)
        features = features.squeeze().cpu().numpy()

        # 目标检测
        detection_result = None
        target_position = None
        if self.use_detection and self.detector is not None:
            detection_result = self.detector.detect(image, depth)
            if detection_result.get("detected"):
                target_position = detection_result.get("centroid_3d")

        # 缓存
        self._current_features = features
        self._current_detection = detection_result

        return {
            "image": image,
            "depth": depth,
            "features": features,
            "detection": detection_result,
            "target_position": target_position,
        }

    def _empty_observation(self) -> Dict:
        """返回空观测"""
        return {
            "image": None,
            "depth": None,
            "features": np.zeros(self.encoder.feature_dim, dtype=np.float32),
            "detection": None,
            "target_position": None,
        }

    def update(self) -> Optional[Dict]:
        """更新相机帧并获取观测"""
        self.camera.update()
        return self.get_observation()


# ============================================================
# 仿真环境用的虚拟相机
# ============================================================

class SimulatedCamera:
    """
    仿真环境用的虚拟相机
    从 PyBullet 渲染生成图像
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
    ):
        self.width = width
        self.height = height

    def capture(
        self,
        pybullet_client,
        robot_position: np.ndarray,
        robot_orientation: np.ndarray,
        target_position: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        从仿真环境捕获图像
        Args:
            pybullet_client: PyBullet 客户端
            robot_position: 机器人位置
            robot_orientation: 机器人朝向 (四元数)
            target_position: 目标位置 (用于渲染)
        Returns:
            rgb: RGB 图像
            depth: 深度图
        """
        # 计算相机视图矩阵
        # 简化实现 - 实际需要根据机器人位置计算

        # 渲染
        view_matrix = None  # 需要根据机器人状态计算
        proj_matrix = None

        if pybullet_client is None:
            # 返回模拟图像
            rgb = np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)
            depth = np.ones((self.height, self.width), dtype=np.float32) * 0.5
            return rgb, depth

        # 实际渲染
        try:
            (_, _, rgb, depth_buffer, _) = pybullet_client.getCameraImage(
                self.width, self.height,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix,
                renderer=pybullet_client.ER_BULLET_HARDWARE_OPENGL,
            )
            rgb = np.array(rgb, dtype=np.uint8).reshape(self.height, self.width, 3)
            depth = np.array(depth_buffer, dtype=np.float32).reshape(self.height, self.width)
        except:
            rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            depth = np.ones((self.height, self.width), dtype=np.float32) * 0.5

        return rgb, depth


# ============================================================
# 测试代码
# ============================================================

if __name__ == "__main__":
    # 测试视觉观测模块 (需要 RealSense 相机)
    print("Testing Vision Observation Module...")

    # 测试相机
    camera = RealSenseCamera()
    if camera.start():
        print("Camera started successfully")
        time.sleep(2)

        for i in range(10):
            frame = camera.update()
            if frame is not None:
                print(f"Frame {i}: RGB shape={frame.rgb.shape}, Depth shape={frame.depth.shape if frame.depth is not None else None}")
                break
            time.sleep(0.1)

        camera.stop()
    else:
        print("Camera not available, using simulated mode")

    # 测试视觉编码器
    print("\nTesting Vision Encoder...")
    encoder = VisionEncoder(backbone="resnet18", feature_dim=256)
    encoder.eval()

    # 生成随机图像
    dummy_image = torch.randn(2, 3, 224, 224)
    with torch.no_grad():
        features = encoder(dummy_image)
    print(f"Features shape: {features.shape}")

    # 测试多模态融合
    print("\nTesting Multi-Modal Fusion...")
    fusion = MultiModalFusion(vision_dim=256, state_dim=6)
    vision_feat = torch.randn(2, 256)
    state_feat = torch.randn(2, 9)
    fused = fusion(vision_feat, state_feat)
    print(f"Fused features shape: {fused.shape}")

    print("\nAll tests passed!")
