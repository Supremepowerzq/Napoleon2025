"""由 M0/M1/M2 电机序列积分得到支气管镜头端的近似三维轨迹。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence

from .geometry import Vector, add, cross, normalize, rotate, scale


@dataclass(frozen=True)
class KinematicConfig:
    """电机角度到几何运动的标定参数。"""

    m0_mm_per_degree: float = 0.10
    m1_roll_deg_per_degree: float = 1.0
    m2_tip_deg_per_degree: float = 1.0
    m0_sign: float = -1.0
    m1_sign: float = 1.0
    m2_sign: float = 1.0
    m1_zero_degree: float = 0.0
    initial_position: Vector = (0.0, 0.0, 0.0)
    initial_forward: Vector = (1.0, 0.0, 0.0)
    initial_up: Vector = (0.0, 0.0, 1.0)


@dataclass(frozen=True)
class TrajectoryPoint:
    frame_index: int
    timestamp: float
    position: Vector
    forward: Vector
    insertion_mm: float


def _number(frame: Mapping[str, Any], key: str, default: float = 0.0) -> float:
    value = frame.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"字段 {key!r} 必须是数值，实际为 {value!r}")
    return float(value)


def _delta(frame: Mapping[str, Any], previous: Mapping[str, Any], motor: str) -> float:
    delta_key = f"delta_{motor}"
    angle_key = f"{motor}_angle"
    if delta_key in frame:
        return _number(frame, delta_key)
    return _number(frame, angle_key) - _number(previous, angle_key)


def validate_frames(frames: Sequence[Mapping[str, Any]]) -> None:
    if len(frames) < 2:
        raise ValueError("至少需要 2 帧电机数据")
    required = ("m0_angle", "m1_angle", "m2_angle")
    for index, frame in enumerate(frames):
        missing = [key for key in required if key not in frame]
        if missing:
            raise ValueError(f"第 {index} 帧缺少字段：{', '.join(missing)}")
        for key in required:
            _number(frame, key)


def reconstruct(
    frames: Sequence[Mapping[str, Any]],
    config: KinematicConfig | None = None,
) -> list[TrajectoryPoint]:
    """重建每帧对应的头端位置和朝向。

    M0 控制沿当前前向的推进；M1 给出弯曲平面相对局部参考上向的
    绝对滚转角；M2 的帧间增量改变头端方向。输出单位为毫米。
    """
    validate_frames(frames)
    cfg = config or KinematicConfig()
    position = cfg.initial_position
    forward = normalize(cfg.initial_forward)
    reference_up = normalize(cfg.initial_up, fallback=(0.0, 0.0, 1.0))
    insertion_mm = 0.0
    first = frames[0]
    result = [
        TrajectoryPoint(
            frame_index=0,
            timestamp=_number(first, "timestamp"),
            position=position,
            forward=forward,
            insertion_mm=insertion_mm,
        )
    ]

    previous = first
    for index, frame in enumerate(frames[1:], start=1):
        dm0 = _delta(frame, previous, "m0")
        dm2 = _delta(frame, previous, "m2")

        roll_motor_degree = _number(frame, "m1_angle") - cfg.m1_zero_degree
        roll_radian = math.radians(
            cfg.m1_sign * roll_motor_degree * cfg.m1_roll_deg_per_degree
        )
        bend_plane_up = rotate(reference_up, forward, roll_radian)
        bend_axis = normalize(cross(forward, bend_plane_up), fallback=(0.0, -1.0, 0.0))

        pitch_radian = math.radians(cfg.m2_sign * dm2 * cfg.m2_tip_deg_per_degree)
        if abs(pitch_radian) > 1e-12:
            forward = rotate(forward, bend_axis, pitch_radian)
            reference_up = rotate(reference_up, bend_axis, pitch_radian)

        step_mm = cfg.m0_sign * dm0 * cfg.m0_mm_per_degree
        insertion_mm += step_mm
        position = add(position, scale(forward, step_mm))
        result.append(
            TrajectoryPoint(
                frame_index=index,
                timestamp=_number(frame, "timestamp", result[-1].timestamp),
                position=position,
                forward=forward,
                insertion_mm=insertion_mm,
            )
        )
        previous = frame

    return result
