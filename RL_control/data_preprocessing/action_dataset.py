"""
动作录制数据预处理模块
将录制的数据转换为 RL 训练可用的格式

支持:
1. 加载和解析动作录制 JSON 文件
2. 数据清洗和异常值处理
3. 数据标准化和归一化
4. 生成轨迹数据 (trajectories)
5. 数据增强 (时间偏移、噪声注入、MixUp)
6. 生成状态-动作对数据集
"""

import os
import json
import numpy as np
import pandas as pd
from pathlib import Path
from typing import List, Dict, Tuple, Optional, Callable
from dataclasses import dataclass, field
from collections import defaultdict
import warnings

try:
    from tqdm import tqdm
except ImportError:
    tqdm = lambda x, **kwargs: x


# ============================================================
# 数据结构
# ============================================================

@dataclass
class MotorState:
    """电机状态"""
    timestamp: float
    motor_0: float
    motor_1: float
    motor_2: float

    def to_array(self) -> np.ndarray:
        return np.array([self.motor_0, self.motor_1, self.motor_2])

    @classmethod
    def from_dict(cls, data: Dict) -> "MotorState":
        return cls(
            timestamp=data["timestamp"],
            motor_0=data["motor_0"],
            motor_1=data["motor_1"],
            motor_2=data["motor_2"],
        )


@dataclass
class Trajectory:
    """一条轨迹"""
    states: List[MotorState] = field(default_factory=list)
    initial_state: Optional[MotorState] = None
    final_state: Optional[MotorState] = None
    metadata: Dict = field(default_factory=dict)

    @property
    def duration(self) -> float:
        if not self.states:
            return 0.0
        return self.states[-1].timestamp - self.states[0].timestamp

    @property
    def num_steps(self) -> int:
        return len(self.states)

    def get_state_sequence(self) -> np.ndarray:
        """获取状态序列 [T, 3]"""
        return np.array([s.to_array() for s in self.states])

    def get_time_sequence(self) -> np.ndarray:
        """获取时间序列 [T]"""
        return np.array([s.timestamp for s in self.states])

    def compute_velocities(self) -> np.ndarray:
        """计算速度序列 [T-1, 3]"""
        if len(self.states) < 2:
            return np.zeros((0, 3))

        states = self.get_state_sequence()
        times = self.get_time_sequence()
        dt = np.diff(times)
        dt[dt == 0] = 1e-6  # 避免除零

        velocities = np.diff(states, axis=0) / dt[:, np.newaxis]
        return velocities

    def resample(self, target_hz: float = 60.0) -> "Trajectory":
        """重采样到目标频率"""
        if len(self.states) < 2:
            return self

        times = self.get_time_sequence()
        states = self.get_state_sequence()

        # 目标时间序列
        t_start = times[0]
        t_end = times[-1]
        target_times = np.arange(t_start, t_end, 1.0 / target_hz)

        # 插值
        resampled_states = np.zeros((len(target_times), 3))
        for i in range(3):
            resampled_states[:, i] = np.interp(target_times, times, states[:, i])

        # 构建新轨迹
        new_trajectory = Trajectory()
        new_trajectory.metadata = self.metadata.copy()
        new_trajectory.metadata["resampled_hz"] = target_hz

        for t, state in zip(target_times, resampled_states):
            new_trajectory.states.append(
                MotorState(timestamp=t, motor_0=state[0], motor_1=state[1], motor_2=state[2])
            )

        return new_trajectory

    def smooth(self, window_size: int = 3) -> "Trajectory":
        """平滑轨迹 (移动平均)"""
        if len(self.states) <= window_size:
            return self

        states = self.get_state_sequence()

        # 边界处理: 扩展
        pad_size = window_size // 2
        padded = np.vstack([
            np.tile(states[0], (pad_size, 1)),
            states,
            np.tile(states[-1], (pad_size, 1))
        ])

        # 移动平均
        smoothed = np.zeros_like(states)
        for i in range(len(states)):
            smoothed[i] = np.mean(
                padded[i:i + window_size],
                axis=0
            )

        # 构建新轨迹
        new_trajectory = Trajectory()
        new_trajectory.metadata = self.metadata.copy()
        new_trajectory.metadata["smoothed"] = True
        new_trajectory.metadata["smooth_window"] = window_size

        for state, sm_state in zip(self.states, smoothed):
            new_trajectory.states.append(
                MotorState(
                    timestamp=state.timestamp,
                    motor_0=sm_state[0],
                    motor_1=sm_state[1],
                    motor_2=sm_state[2],
                )
            )

        return new_trajectory


# ============================================================
# 数据加载器
# ============================================================

class ActionDatasetLoader:
    """
    动作录制数据加载器
    """

    def __init__(
        self,
        data_dir: str = "recorded_actions",
        file_pattern: str = "*.json",
    ):
        self.data_dir = Path(data_dir)
        self.file_pattern = file_pattern
        self.trajectories: List[Trajectory] = []
        self._stats: Optional[Dict] = None

    def load_all(self, verbose: bool = True) -> List[Trajectory]:
        """加载所有动作录制文件"""
        files = sorted(self.data_dir.glob(self.file_pattern))

        if verbose:
            files = tqdm(files, desc="Loading action recordings")

        self.trajectories = []
        for file_path in files:
            try:
                traj = self._load_single_file(file_path)
                if traj is not None and len(traj.states) > 0:
                    self.trajectories.append(traj)
            except Exception as e:
                if verbose:
                    print(f"Error loading {file_path}: {e}")
                continue

        if verbose:
            print(f"Loaded {len(self.trajectories)} trajectories")

        return self.trajectories

    def _load_single_file(self, file_path: Path) -> Optional[Trajectory]:
        """加载单个文件"""
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        traj = Trajectory()
        traj.metadata = {
            "source_file": str(file_path),
            "created_at": data.get("created_at"),
            "duration": data.get("duration", 0.0),
        }

        # 加载初始状态
        if "initial_state" in data:
            traj.initial_state = MotorState.from_dict(data["initial_state"])

        # 加载终止状态
        if "final_state" in data:
            traj.final_state = MotorState.from_dict(data["final_state"])

        # 加载动作序列
        actions = data.get("actions", [])
        for action_data in actions:
            traj.states.append(MotorState.from_dict(action_data))

        return traj

    def filter_by_duration(
        self,
        min_duration: float = 0.0,
        max_duration: float = float("inf"),
    ) -> List[Trajectory]:
        """按持续时间过滤"""
        return [
            t for t in self.trajectories
            if min_duration <= t.duration <= max_duration
        ]

    def filter_by_num_steps(
        self,
        min_steps: int = 0,
        max_steps: int = float("inf"),
    ) -> List[Trajectory]:
        """按步数过滤"""
        return [
            t for t in self.trajectories
            if min_steps <= t.num_steps <= max_steps
        ]

    def compute_statistics(self) -> Dict:
        """计算数据集统计信息"""
        if not self.trajectories:
            return {}

        all_states = np.concatenate([
            t.get_state_sequence() for t in self.trajectories
        ])

        self._stats = {
            "num_trajectories": len(self.trajectories),
            "total_steps": sum(t.num_steps for t in self.trajectories),
            "total_duration": sum(t.duration for t in self.trajectories),
            # 状态统计
            "state_mean": np.mean(all_states, axis=0),
            "state_std": np.std(all_states, axis=0),
            "state_min": np.min(all_states, axis=0),
            "state_max": np.max(all_states, axis=0),
            # 速度统计
            "velocity_mean": np.mean(np.concatenate([
                t.compute_velocities() for t in self.trajectories
            ]), axis=0),
            "velocity_std": np.std(np.concatenate([
                t.compute_velocities() for t in self.trajectories
            ]), axis=0),
        }

        return self._stats

    def get_normalization_params(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取归一化参数 (mean, std)"""
        if self._stats is None:
            self.compute_statistics()

        return self._stats["state_mean"], self._stats["state_std"]


# ============================================================
# 数据预处理
# ============================================================

class TrajectoryProcessor:
    """
    轨迹数据预处理器
    """

    def __init__(
        self,
        target_hz: float = 60.0,
        smooth_window: int = 3,
        remove_outliers: bool = True,
        outlier_threshold: float = 3.0,
        interpolation_method: str = "linear",
    ):
        self.target_hz = target_hz
        self.smooth_window = smooth_window
        self.remove_outliers = remove_outliers
        self.outlier_threshold = outlier_threshold
        self.interpolation_method = interpolation_method

    def process(self, trajectory: Trajectory) -> Trajectory:
        """处理单条轨迹"""
        # 1. 异常值移除
        if self.remove_outliers:
            trajectory = self._remove_outliers(trajectory)

        # 2. 重采样
        trajectory = trajectory.resample(self.target_hz)

        # 3. 平滑
        if self.smooth_window > 1:
            trajectory = trajectory.smooth(self.smooth_window)

        return trajectory

    def process_batch(
        self, trajectories: List[Trajectory], verbose: bool = True
    ) -> List[Trajectory]:
        """批量处理轨迹"""
        if verbose:
            trajectories = tqdm(trajectories, desc="Processing trajectories")

        return [self.process(t) for t in trajectories]

    def _remove_outliers(self, trajectory: Trajectory) -> Trajectory:
        """移除异常值 (基于速度)"""
        velocities = trajectory.compute_velocities()
        if len(velocities) == 0:
            return trajectory

        # 计算速度阈值
        v_mean = np.mean(velocities, axis=0)
        v_std = np.std(velocities, axis=0)
        threshold = self.outlier_threshold * v_std

        # 标记异常
        outlier_mask = np.any(
            np.abs(velocities - v_mean) > threshold,
            axis=1
        )

        # 过滤异常点 (保留非异常的点和端点)
        new_trajectory = Trajectory()
        new_trajectory.metadata = trajectory.metadata.copy()
        new_trajectory.initial_state = trajectory.initial_state
        new_trajectory.final_state = trajectory.final_state

        for i, state in enumerate(trajectory.states):
            if i == 0 or i == len(trajectory.states) - 1:
                new_trajectory.states.append(state)
            elif not outlier_mask[i - 1]:  # 对应 velocities[i-1]
                new_trajectory.states.append(state)

        return new_trajectory


# ============================================================
# 数据增强
# ============================================================

class TrajectoryAugmentor:
    """
    轨迹数据增强器
    """

    def __init__(
        self,
        noise_std: float = 0.1,
        time_shift_max: float = 0.1,
        mixup_alpha: float = 0.2,
        scale_range: Tuple[float, float] = (0.95, 1.05),
    ):
        self.noise_std = noise_std
        self.time_shift_max = time_shift_max
        self.mixup_alpha = mixup_alpha
        self.scale_range = scale_range

    def add_noise(
        self, trajectory: Trajectory, noise_std: Optional[float] = None
    ) -> Trajectory:
        """添加高斯噪声"""
        if noise_std is None:
            noise_std = self.noise_std

        new_trajectory = Trajectory()
        new_trajectory.metadata = trajectory.metadata.copy()
        new_trajectory.metadata["augmented"] = True
        new_trajectory.metadata["noise_std"] = noise_std

        for state in trajectory.states:
            noisy_m0 = state.motor_0 + np.random.normal(0, noise_std)
            noisy_m1 = state.motor_1 + np.random.normal(0, noise_std)
            noisy_m2 = state.motor_2 + np.random.normal(0, noise_std)

            new_trajectory.states.append(
                MotorState(
                    timestamp=state.timestamp,
                    motor_0=noisy_m0,
                    motor_1=noisy_m1,
                    motor_2=noisy_m2,
                )
            )

        return new_trajectory

    def time_shift(
        self, trajectory: Trajectory
    ) -> Trajectory:
        """时间偏移增强"""
        time_shift = np.random.uniform(
            -self.time_shift_max, self.time_shift_max
        )

        new_trajectory = Trajectory()
        new_trajectory.metadata = trajectory.metadata.copy()
        new_trajectory.metadata["augmented"] = True
        new_trajectory.metadata["time_shift"] = time_shift

        for state in trajectory.states:
            new_trajectory.states.append(
                MotorState(
                    timestamp=state.timestamp + time_shift,
                    motor_0=state.motor_0,
                    motor_1=state.motor_1,
                    motor_2=state.motor_2,
                )
            )

        return new_trajectory

    def scale(
        self, trajectory: Trajectory
    ) -> Trajectory:
        """缩放增强"""
        scale = np.random.uniform(
            self.scale_range[0], self.scale_range[1]
        )

        new_trajectory = Trajectory()
        new_trajectory.metadata = trajectory.metadata.copy()
        new_trajectory.metadata["augmented"] = True
        new_trajectory.metadata["scale"] = scale

        for state in trajectory.states:
            new_trajectory.states.append(
                MotorState(
                    timestamp=state.timestamp,
                    motor_0=state.motor_0 * scale,
                    motor_1=state.motor_1 * scale,
                    motor_2=state.motor_2 * scale,
                )
            )

        return new_trajectory

    def mixup(
        self,
        trajectory1: Trajectory,
        trajectory2: Trajectory,
        alpha: Optional[float] = None,
    ) -> Trajectory:
        """
        MixUp 增强 (混合两条轨迹)
        注意: 需要两条轨迹长度相同或进行插值对齐
        """
        if alpha is None:
            alpha = self.mixup_alpha

        # 确保时间对齐
        t1 = trajectory1.get_time_sequence()
        t2 = trajectory2.get_time_sequence()

        # 使用较长轨迹的时间轴
        if len(t1) >= len(t2):
            target_times = t1
            other_traj = trajectory2
        else:
            target_times = t2
            other_traj = trajectory1

        # 对较短的轨迹进行插值
        s1 = trajectory1.get_state_sequence()
        s2 = trajectory2.get_state_sequence()

        if len(s1) != len(target_times):
            s1_interp = np.zeros((len(target_times), 3))
            for i in range(3):
                s1_interp[:, i] = np.interp(target_times, t1, s1[:, i])
            s1 = s1_interp

        if len(s2) != len(target_times):
            s2_interp = np.zeros((len(target_times), 3))
            for i in range(3):
                s2_interp[:, i] = np.interp(target_times, t2, s2[:, i])
            s2 = s2_interp

        # MixUp 混合
        mixed = alpha * s1 + (1 - alpha) * s2

        new_trajectory = Trajectory()
        new_trajectory.metadata = {
            "augmented": True,
            "mixup_alpha": alpha,
            "mixed_from": [
                trajectory1.metadata.get("source_file"),
                trajectory2.metadata.get("source_file"),
            ]
        }

        for t, m in zip(target_times, mixed):
            new_trajectory.states.append(
                MotorState(timestamp=t, motor_0=m[0], motor_1=m[1], motor_2=m[2])
            )

        return new_trajectory

    def augment(
        self,
        trajectory: Trajectory,
        augmentations: List[str] = ["noise", "scale"],
    ) -> List[Trajectory]:
        """应用多种增强"""
        augmented = [trajectory]

        if "noise" in augmentations:
            augmented.append(self.add_noise(trajectory))

        if "scale" in augmentations:
            augmented.append(self.scale(trajectory))

        if "time_shift" in augmentations:
            augmented.append(self.time_shift(trajectory))

        return augmented


# ============================================================
# 模仿学习数据生成器
# ============================================================

class ImitationLearningDataset:
    """
    生成模仿学习数据集
    输出格式: (state, action) 对
    """

    def __init__(
        self,
        trajectories: List[Trajectory],
        normalize: bool = True,
        include_velocities: bool = True,
    ):
        self.trajectories = trajectories
        self.normalize = normalize
        self.include_velocities = include_velocities

        # 计算归一化参数
        if normalize:
            self._compute_normalization_params()

        # 生成数据集
        self.states = []
        self.actions = []
        self._generate_dataset()

    def _compute_normalization_params(self):
        """计算归一化参数"""
        all_states = []
        all_actions = []

        for traj in self.trajectories:
            states = traj.get_state_sequence()
            velocities = traj.compute_velocities()

            if len(velocities) == 0:
                continue

            states_for_norm = states[:-1] if len(states) > 1 else states
            velocities_all = np.zeros_like(states)
            velocities_all[1:] = velocities
            velocities_all = velocities_all[:-1]

            if self.include_velocities:
                combined = np.concatenate([states_for_norm, velocities_all], axis=1)
            else:
                combined = states_for_norm

            # 状态: 当前位置
            all_states.append(combined)
            # 动作: 下一个位置 - 当前位置 (即期望的电机动作)
            all_actions.append(velocities)

        self.state_mean = np.mean(np.concatenate(all_states), axis=0)
        self.state_std = np.std(np.concatenate(all_states), axis=0)
        self.action_mean = np.mean(np.concatenate(all_actions), axis=0)
        self.action_std = np.std(np.concatenate(all_actions), axis=0)

        # 避免除零
        self.state_std = np.where(self.state_std < 1e-6, 1.0, self.state_std)
        self.action_std = np.where(self.action_std < 1e-6, 1.0, self.action_std)

    def _generate_dataset(self):
        """生成状态-动作对"""
        for traj in self.trajectories:
            states = traj.get_state_sequence()
            velocities = traj.compute_velocities()

            if len(velocities) == 0:
                continue

            velocities_all = np.zeros_like(states)
            velocities_all[1:] = velocities
            velocities_all = velocities_all[:-1]

            # 去除最后一个状态 (没有对应的动作)
            if self.include_velocities:
                for s, v, a in zip(states[:-1], velocities_all, velocities):
                    state = np.concatenate([s, v])
                    self.states.append(state)
                    self.actions.append(a)
            else:
                for s, a in zip(states[:-1], velocities):
                    self.states.append(s)
                    self.actions.append(a)

        # 转换为 numpy 数组
        self.states = np.array(self.states, dtype=np.float32)
        self.actions = np.array(self.actions, dtype=np.float32)

        # 归一化
        if self.normalize:
            self.states = (self.states - self.state_mean) / self.state_std
            self.actions = (self.actions - self.action_mean) / self.action_std

    def get_batch(
        self, batch_size: int, shuffle: bool = True
    ) -> Tuple[np.ndarray, np.ndarray]:
        """获取一个批次的数据"""
        if shuffle:
            indices = np.random.permutation(len(self.states))
        else:
            indices = np.arange(len(self.states))

        for i in range(0, len(indices), batch_size):
            batch_indices = indices[i:i + batch_size]
            yield self.states[batch_indices], self.actions[batch_indices]

    def __len__(self) -> int:
        return len(self.states)

    def __getitem__(self, idx: int) -> Tuple[np.ndarray, np.ndarray]:
        return self.states[idx], self.actions[idx]


# ============================================================
# 主函数
# ============================================================

def prepare_il_dataset(
    data_dir: str = "recorded_actions",
    output_path: Optional[str] = None,
    target_hz: float = 60.0,
    augmentation: bool = True,
    augmentation_factor: int = 3,
) -> Tuple[ImitationLearningDataset, Dict]:
    """
    准备模仿学习数据集的便捷函数

    Args:
        data_dir: 动作录制数据目录
        output_path: 输出路径 (可选)
        target_hz: 目标采样频率
        augmentation: 是否使用数据增强
        augmentation_factor: 增强倍数

    Returns:
        dataset: 模仿学习数据集
        stats: 数据集统计信息
    """
    # 1. 加载数据
    print("Loading action recordings...")
    loader = ActionDatasetLoader(data_dir)
    trajectories = loader.load_all()

    if not trajectories:
        raise ValueError(f"No trajectories found in {data_dir}")

    # 2. 预处理
    print("Processing trajectories...")
    processor = TrajectoryProcessor(target_hz=target_hz)
    trajectories = processor.process_batch(trajectories)

    # 3. 数据增强
    if augmentation:
        print("Augmenting data...")
        augmentor = TrajectoryAugmentor()
        augmented = []
        for traj in tqdm(trajectories, desc="Augmenting"):
            augmented.extend(augmentor.augment(traj))
        trajectories.extend(augmented)

    # 4. 生成数据集
    print("Generating imitation learning dataset...")
    dataset = ImitationLearningDataset(trajectories)

    # 5. 统计
    stats = {
        "num_trajectories": len(trajectories),
        "num_samples": len(dataset),
        "state_shape": dataset.states.shape,
        "action_shape": dataset.actions.shape,
    }

    # 6. 保存
    if output_path:
        print(f"Saving dataset to {output_path}...")
        np.savez(
            output_path,
            states=dataset.states,
            actions=dataset.actions,
            state_mean=dataset.state_mean,
            state_std=dataset.state_std,
        )

    return dataset, stats


if __name__ == "__main__":
    # 测试
    data_dir = "../../recorded_actions"
    dataset, stats = prepare_il_dataset(data_dir)

    print("\n=== Dataset Statistics ===")
    for k, v in stats.items():
        print(f"{k}: {v}")

    # 测试数据批次
    print("\n=== Sample Batch ===")
    for batch_states, batch_actions in dataset.get_batch(batch_size=8):
        print(f"Batch states shape: {batch_states.shape}")
        print(f"Batch actions shape: {batch_actions.shape}")
        break
