"""不依赖 NumPy 的小型三维几何函数。"""

from __future__ import annotations

import math
from typing import Sequence

Vector = tuple[float, float, float]


def as_vector(value: Sequence[float]) -> Vector:
    if len(value) != 3:
        raise ValueError("三维向量必须包含 3 个分量")
    return float(value[0]), float(value[1]), float(value[2])


def add(a: Vector, b: Vector) -> Vector:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def subtract(a: Vector, b: Vector) -> Vector:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def scale(a: Vector, factor: float) -> Vector:
    return a[0] * factor, a[1] * factor, a[2] * factor


def dot(a: Vector, b: Vector) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector, b: Vector) -> Vector:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def length(a: Vector) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Vector, fallback: Vector = (1.0, 0.0, 0.0)) -> Vector:
    value = length(a)
    return scale(a, 1.0 / value) if value > 1e-12 else fallback


def distance(a: Vector, b: Vector) -> float:
    return length(subtract(a, b))


def rotate(vector: Vector, axis: Vector, radians: float) -> Vector:
    """使用 Rodrigues 公式绕 ``axis`` 旋转向量。"""
    unit_axis = normalize(axis)
    cosine = math.cos(radians)
    sine = math.sin(radians)
    return normalize(
        add(
            add(scale(vector, cosine), scale(cross(unit_axis, vector), sine)),
            scale(unit_axis, dot(unit_axis, vector) * (1.0 - cosine)),
        ),
        fallback=vector,
    )
