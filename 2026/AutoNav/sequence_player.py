# -*- coding: utf-8 -*-
"""
SequencePlayer — 纯电机序列导航（Layer 1）
==========================================
从已采集的 BC 专家演示（BC/expert_demos/*.json）中，
Layer 1 验证时可指定一条采集轨迹并逐帧执行；后续实验也可按
目标支气管路径（path_label）加载多条演示并构建平均轨迹。

核心思路
--------
1. 对同一路径的所有演示轨迹，提取相对于起始位置的累积角度位移。
2. 将所有轨迹重采样到统一帧数（中位数长度），做逐帧平均。
3. 导航时：
   target_angles[t] = start_angles + avg_cumulative[t]
   单帧输出 delta = target[t] - target[t-1]
"""

import os
import json
import glob
import numpy as np
from typing import Optional, List, Tuple, Dict

from .config import CONTROL_LOOP_HZ, MOTOR_LIMITS, PLAYBACK_MAX_DELTA, PLAYBACK_SPEED


class SequencePlayer:
    """
    纯电机序列导航器。

    Parameters
    ----------
    demo_dir  : str   BC 专家演示目录（BC/expert_demos）
    path_label: int   目标支气管标签（与 BC/model.py 的 BRONCHUS_PATHS 一致）
    speed     : float 速度缩放（1.0 = 原速）
    """

    def __init__(
        self,
        demo_dir: str,
        path_label: int = 2,
        speed: float = PLAYBACK_SPEED,
        demo_file: Optional[str] = None,
    ):
        self.demo_dir   = demo_dir
        self.path_label = path_label
        self.speed      = speed

        # 平均轨迹：[T, 3] 相对起点的累积角度（Δm0, Δm1, Δm2）
        self._avg_cumulative: Optional[np.ndarray] = None
        self._total_steps: int = 0

        # 导航状态
        self._start_angles:  Optional[np.ndarray] = None
        self._step: int = 0
        self._prev_target: Optional[np.ndarray] = None

        self.demo_file = os.path.abspath(demo_file) if demo_file else None
        self.source_files: List[str] = []
        self._source_timestamps: Optional[np.ndarray] = None
        self._load_and_build(demo_dir, path_label)

    # ── 构建平均轨迹 ──────────────────────────────────────────────

    def _load_and_build(self, demo_dir: str, path_label: int):
        trajs = self._load_demos(demo_dir, path_label)
        if not trajs:
            raise ValueError(
                f"[SequencePlayer] 未找到 path_label={path_label} 的演示文件，"
                f"请检查目录: {demo_dir}"
            )
        # 指定文件时保持原始帧序列，不做重采样或多数据均一化。
        trajectory = trajs[0] if self.demo_file else self._build_mean_trajectory(trajs)
        self._avg_cumulative = self._resample_for_speed(trajectory)
        self._total_steps = len(self._avg_cumulative)
        mode = "单条原始轨迹" if self.demo_file else "平均轨迹"
        print(f"[SequencePlayer] path_label={path_label}  "
              f"加载 {len(trajs)} 条演示  {mode} {self._total_steps} 步")

    def _resample_for_speed(self, trajectory: np.ndarray) -> np.ndarray:
        """按时间轴调整导航速度，同时严格保留完整终点位移。"""
        if len(trajectory) <= 1:
            return trajectory.astype(np.float32)
        speed = max(float(self.speed), 0.01)
        if self.demo_file and self._source_timestamps is not None:
            source_index = self._source_timestamps
            duration = float(source_index[-1])
            target_len = max(2, int(np.ceil(duration / speed * CONTROL_LOOP_HZ)) + 1)
            target_index = np.linspace(0.0, duration, target_len)
        else:
            source_index = np.arange(len(trajectory), dtype=np.float32)
            target_len = max(2, int(np.ceil((len(trajectory) - 1) / speed)) + 1)
            target_index = np.linspace(0, len(trajectory) - 1, target_len)
        return np.stack([
            np.interp(target_index, source_index, trajectory[:, axis])
            for axis in range(3)
        ], axis=1).astype(np.float32)

    def _load_demos(self, demo_dir: str, path_label: int) -> List[np.ndarray]:
        """
        从 JSON 文件加载指定路径的所有演示，返回相对起始的累积角度序列列表。
        每条轨迹形如 [T, 3]（Δm0 cumsum, Δm1 cumsum, Δm2 cumsum）。
        """
        result = []
        files = [self.demo_file] if self.demo_file else sorted(
            glob.glob(os.path.join(demo_dir, "*.json"))
        )

        for fpath in files:
            try:
                with open(fpath, "r", encoding="utf-8") as fp:
                    data = json.load(fp)
            except Exception:
                continue

            if int(data.get("path_label", -1)) != path_label:
                continue

            frames = data.get("frames", [])
            if len(frames) < 20:
                continue

            # 优先使用采集到的绝对角度，避免 delta 量化/丢帧造成幅度损失。
            angles = np.array([
                [f.get("m0_angle", 0.0), f.get("m1_angle", 0.0), f.get("m2_angle", 0.0)]
                for f in frames
            ], dtype=np.float32)
            cum = angles - angles[0]
            if self.demo_file:
                timestamps = np.array([
                    f.get("timestamp", index / CONTROL_LOOP_HZ)
                    for index, f in enumerate(frames)
                ], dtype=np.float32)
                timestamps -= timestamps[0]
                # 防御重复或倒序时间戳，保证 np.interp 输入严格递增。
                timestamps = np.maximum.accumulate(timestamps)
                for index in range(1, len(timestamps)):
                    if timestamps[index] <= timestamps[index - 1]:
                        timestamps[index] = timestamps[index - 1] + 1e-4
                self._source_timestamps = timestamps
            result.append(cum)
            self.source_files.append(os.path.abspath(fpath))

        return result

    def _build_mean_trajectory(self, trajs: List[np.ndarray]) -> np.ndarray:
        """
        将所有轨迹重采样到中位数长度后逐帧取平均。

        Returns
        -------
        np.ndarray  [T, 3]  平均累积角度位移
        """
        lengths = [len(t) for t in trajs]
        target_len = int(np.median(lengths))

        resampled = []
        for traj in trajs:
            T = len(traj)
            # 在 [0,1] 上重采样
            x_src = np.linspace(0, 1, T)
            x_dst = np.linspace(0, 1, target_len)
            r = np.stack([
                np.interp(x_dst, x_src, traj[:, i])
                for i in range(3)
            ], axis=1)
            resampled.append(r)

        avg = np.mean(resampled, axis=0)  # [target_len, 3]
        return avg.astype(np.float32)

    # ── 导航控制 ──────────────────────────────────────────────────

    def reset(self, current_motor_angles: Tuple[float, float, float]):
        """从当前电机位置重置，准备开始导航。"""
        self._start_angles = np.array(current_motor_angles, dtype=np.float32)
        self._step = 0
        self._prev_target = self._start_angles.copy()
        print(f"[SequencePlayer] 重置  起点={current_motor_angles}  "
              f"共 {self._total_steps} 步  速度={self.speed}")

    def step(self) -> Dict[str, float]:
        """
        推进一步，返回本帧的电机目标角度增量和状态信息。

        Returns
        -------
        dict with keys: delta_m0, delta_m1, delta_m2, progress, done,
                        target_m0, target_m1, target_m2
        """
        if self._avg_cumulative is None or self._start_angles is None:
            return {"delta_m0": 0.0, "delta_m1": 0.0, "delta_m2": 0.0,
                    "progress": 0.0, "done": True,
                    "target_m0": 0.0, "target_m1": 0.0, "target_m2": 0.0}

        done = self._step >= self._total_steps

        if not done:
            # 当前目标绝对角度
            cum = self._avg_cumulative[self._step]
            target = self._start_angles + cum

            # 限位夹紧
            for i, (lo, hi) in MOTOR_LIMITS.items():
                target[i] = float(np.clip(target[i], lo, hi))

            # 本帧增量（从上一帧目标到当前目标）
            delta = target - self._prev_target

            # 单步安全限幅
            for i in range(3):
                max_d = PLAYBACK_MAX_DELTA[i]
                delta[i] = float(np.clip(delta[i], -max_d, max_d))

            self._prev_target = self._prev_target + delta
            self._step += 1
        else:
            # 若原始轨迹的瞬时变化触发了单步安全限幅，继续追赶最终目标，
            # 不能在时间轴结束时丢掉尚未执行的角度幅度。
            target = self._start_angles + self._avg_cumulative[-1]
            for i, (lo, hi) in MOTOR_LIMITS.items():
                target[i] = float(np.clip(target[i], lo, hi))
            remaining = target - self._prev_target
            if np.max(np.abs(remaining)) > 1e-3:
                delta = remaining.copy()
                for i in range(3):
                    max_d = PLAYBACK_MAX_DELTA[i]
                    delta[i] = float(np.clip(delta[i], -max_d, max_d))
                self._prev_target = self._prev_target + delta
                done = False
            else:
                delta = np.zeros(3, dtype=np.float32)
                done = True

        progress = min(self._step / max(self._total_steps, 1), 1.0)

        return {
            "delta_m0":  float(delta[0]),
            "delta_m1":  float(delta[1]),
            "delta_m2":  float(delta[2]),
            "target_m0": float(self._prev_target[0]),
            "target_m1": float(self._prev_target[1]),
            "target_m2": float(self._prev_target[2]),
            "progress":  progress,
            "done":      done,
        }

    def jump_to_step(self, step: int, current_motor_angles: Tuple[float, float, float]):
        """
        跳转到指定步骤（清除阻塞物后恢复断点时使用）。
        同时更新内部目标以当前实际电机位置重新对齐。
        """
        self._step = max(0, min(step, self._total_steps - 1))
        if self._step > 0 and self._avg_cumulative is not None:
            cum = self._avg_cumulative[self._step - 1]
            self._prev_target = self._start_angles + cum
        else:
            self._prev_target = np.array(current_motor_angles, dtype=np.float32)
        print(f"[SequencePlayer] 跳转到步骤 {self._step}/{self._total_steps}  "
              f"进度={self._step/max(self._total_steps,1)*100:.1f}%")

    @property
    def current_step(self) -> int:
        return self._step

    @property
    def total_steps(self) -> int:
        return self._total_steps

    @property
    def is_ready(self) -> bool:
        return self._avg_cumulative is not None and self._start_angles is not None

    @property
    def is_done(self) -> bool:
        return self._step >= self._total_steps

    def get_path_info(self) -> Dict[str, int]:
        return {"path_label": self.path_label, "total_steps": self._total_steps}

    def get_cumulative_trajectory(self) -> np.ndarray:
        """返回用于回放的相对起点轨迹副本，供只读的轨迹拼接使用。"""
        if self._avg_cumulative is None:
            return np.empty((0, 3), dtype=np.float32)
        return self._avg_cumulative.copy()
