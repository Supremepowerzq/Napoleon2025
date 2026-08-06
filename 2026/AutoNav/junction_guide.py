# -*- coding: utf-8 -*-
"""
JunctionGuide — 岔口引导路线纠正（Layer 2）
===========================================
在 SequencePlayer 的基础轨迹之上，叠加一个实时路线修正量，
使镜头在进入岔口时尽可能沿腔道中心线前进，减少对气管壁的碰撞。

工作原理
--------
- 视觉感知提供当前帧中“距离画面中心最近的岔口”几何中心
  (bifur_cx, bifur_cy) 及面积占比 bifur_area
- bifur_cx ∈ [-1, 1]：负值表示岔口偏左，正值表示偏右
- bifur_cy ∈ [-1, 1]：负值表示偏上，正值表示偏下
- 偏置量 = GAIN × 经过中心死区处理后的 error
- 偏置量只叠加到 Layer 1 当前绝对目标，不逐帧累积

注意事项
--------
- 仅当 bifur_area >= BIFUR_AREA_THRESHOLD 时生效（过小则岔口太远不可信）
- 总偏置有最大幅度限制，防止长时间识别导致路径持续漂移
- 方向符号约定：
    bifur_cx > 0（岔口在右）→ M1 正向旋转 → 镜头右转
    bifur_cy > 0（岔口在下）→ M2 正向旋转 → 镜头下转
  （如与实际电机方向相反，将 BIFUR_GAIN_M1/M2 改为负值即可）
"""

from typing import Dict, Iterable, Mapping, Optional, Tuple
import numpy as np

from .config import (
    BIFUR_AREA_THRESHOLD,
    BIFUR_CENTER_DEADZONE,
    BIFUR_FULL_EFFECT_DISTANCE,
    BIFUR_FULL_EFFECT_PROGRESS,
    BIFUR_GAIN_M1,
    BIFUR_GAIN_M2,
    BIFUR_MAX_OFFSET_M1,
    BIFUR_MAX_OFFSET_M2,
    BIFUR_MIN_PROGRESS_WEIGHT,
    BIFUR_RESPONSE_ALPHA,
    BIFUR_ZERO_EFFECT_DISTANCE,
)


def select_nearest_junction(
    candidates: Iterable[Mapping],
    center_x: float,
    center_y: float,
) -> Tuple[Optional[Mapping], int]:
    """返回几何中心距离画面中心最近的岔口及其原始索引。"""
    best = None
    best_index = -1
    best_distance_sq = float("inf")
    for index, candidate in enumerate(candidates):
        try:
            dx = float(candidate["cx"]) - float(center_x)
            dy = float(candidate["cy"]) - float(center_y)
        except (KeyError, TypeError, ValueError):
            continue
        distance_sq = dx * dx + dy * dy
        if distance_sq < best_distance_sq:
            best = candidate
            best_index = index
            best_distance_sq = distance_sq
    return best, best_index


def junction_progress_weight(progress: float) -> float:
    """路径前段低强度、后段高强度的连续权重。"""
    full_progress = max(float(BIFUR_FULL_EFFECT_PROGRESS), 1e-6)
    ratio = float(np.clip(float(progress) / full_progress, 0.0, 1.0))
    # 二次曲线使前段微调更温和，同时避免硬开关。
    return float(
        BIFUR_MIN_PROGRESS_WEIGHT
        + (1.0 - BIFUR_MIN_PROGRESS_WEIGHT) * ratio * ratio
    )


def junction_distance_weight(center_distance: float) -> float:
    """近中心完整响应，向外连续衰减，超过软限制后停止。"""
    distance = max(float(center_distance), 0.0)
    full_distance = float(BIFUR_FULL_EFFECT_DISTANCE)
    zero_distance = max(float(BIFUR_ZERO_EFFECT_DISTANCE), full_distance + 1e-6)
    if distance <= full_distance:
        return 1.0
    if distance >= zero_distance:
        return 0.0
    return float((zero_distance - distance) / (zero_distance - full_distance))


def pixel_axis_tracking_speed(
    center_minus_target_px: float,
    deadzone_px: float,
    full_speed_error_px: float,
    near_speed: float,
    far_speed: float,
    error_to_speed_sign: float = -1.0,
) -> float:
    """像素误差到轴速度：死区内停止，近处慢速，远处快速。"""
    error = float(center_minus_target_px)
    magnitude = abs(error)
    deadzone = max(float(deadzone_px), 0.0)
    if magnitude <= deadzone:
        return 0.0

    full_error = max(float(full_speed_error_px), deadzone + 1e-6)
    ratio = float(np.clip((magnitude - deadzone) / (full_error - deadzone), 0.0, 1.0))
    speed = abs(float(near_speed)) + (abs(float(far_speed)) - abs(float(near_speed))) * ratio

    return float(np.copysign(speed, error) * np.sign(error_to_speed_sign))


class JunctionGuide:
    """
    岔口引导修正器。

    Parameters
    ----------
    gain_m1           : float  左右修正增益（默认见 config.py）
    gain_m2           : float  上下修正增益
    area_threshold    : float  最小激活面积
    deadzone          : float  画面中心死区（归一化坐标）
    max_offset_m1/m2 : float  相对 Layer 1 的最大总偏置（°）
    response_alpha   : float  EMA 响应系数（0 = 保持旧值，1 = 立即跟随）
    """

    def __init__(
        self,
        gain_m1:        float = BIFUR_GAIN_M1,
        gain_m2:        float = BIFUR_GAIN_M2,
        area_threshold: float = BIFUR_AREA_THRESHOLD,
        deadzone:       float = BIFUR_CENTER_DEADZONE,
        max_offset_m1:  float = BIFUR_MAX_OFFSET_M1,
        max_offset_m2:  float = BIFUR_MAX_OFFSET_M2,
        response_alpha: float = BIFUR_RESPONSE_ALPHA,
    ):
        self.gain_m1        = gain_m1
        self.gain_m2        = gain_m2
        self.area_threshold = area_threshold
        self.deadzone       = float(np.clip(deadzone, 0.0, 0.95))
        self.max_offset_m1  = abs(max_offset_m1)
        self.max_offset_m2  = abs(max_offset_m2)
        self.response_alpha = float(np.clip(response_alpha, 0.0, 1.0))

        self._smooth_m1 = 0.0
        self._smooth_m2 = 0.0

    def compute_correction(
        self,
        bifur_cx:   float,
        bifur_cy:   float,
        bifur_area: float,
    ) -> Tuple[float, float]:
        """
        计算本帧相对于 Layer 1 路径的目标角度偏置。

        Returns
        -------
        (offset_m1, offset_m2)  单位：°，叠加到 Layer 1 当前绝对目标上
        """
        values_are_finite = all(np.isfinite(value) for value in (bifur_cx, bifur_cy, bifur_area))
        if not values_are_finite or bifur_area < self.area_threshold:
            # 岔口不可信，平滑衰减到 0
            self._smooth_m1 *= (1.0 - self.response_alpha)
            self._smooth_m2 *= (1.0 - self.response_alpha)
            return float(self._smooth_m1), float(self._smooth_m2)

        def _apply_deadzone(value: float) -> float:
            value = float(np.clip(value, -1.0, 1.0))
            magnitude = abs(value)
            if magnitude <= self.deadzone:
                return 0.0
            # 去掉死区后重新映射到 [0, 1]，避免死区边缘突跳。
            scaled = (magnitude - self.deadzone) / (1.0 - self.deadzone)
            return float(np.copysign(scaled, value))

        raw_m1 = self.gain_m1 * _apply_deadzone(bifur_cx)
        raw_m2 = self.gain_m2 * _apply_deadzone(bifur_cy)
        raw_m1 = float(np.clip(raw_m1, -self.max_offset_m1, self.max_offset_m1))
        raw_m2 = float(np.clip(raw_m2, -self.max_offset_m2, self.max_offset_m2))

        # EMA 平滑
        alpha = self.response_alpha
        self._smooth_m1 = alpha * raw_m1 + (1.0 - alpha) * self._smooth_m1
        self._smooth_m2 = alpha * raw_m2 + (1.0 - alpha) * self._smooth_m2

        offset_m1 = float(np.clip(self._smooth_m1, -self.max_offset_m1, self.max_offset_m1))
        offset_m2 = float(np.clip(self._smooth_m2, -self.max_offset_m2, self.max_offset_m2))

        return offset_m1, offset_m2

    def reset(self):
        """清零平滑状态（每次进入导航模式时调用）。"""
        self._smooth_m1 = 0.0
        self._smooth_m2 = 0.0

    def get_debug_info(self, bifur_cx, bifur_cy, bifur_area) -> Dict:
        in_deadzone = abs(bifur_cx) <= self.deadzone and abs(bifur_cy) <= self.deadzone
        active = bifur_area >= self.area_threshold and not in_deadzone
        return {
            "active":    active,
            "bifur_cx":  bifur_cx,
            "bifur_cy":  bifur_cy,
            "bifur_area": bifur_area,
            "in_deadzone": in_deadzone,
            "smooth_m1": self._smooth_m1,
            "smooth_m2": self._smooth_m2,
        }
