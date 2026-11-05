"""CT 中心线路径读取、弧长映射和相似变换配准。"""

from __future__ import annotations

import base64
import json
import math
from pathlib import Path
import re
import struct
from typing import Sequence
import zlib

from .geometry import Vector, distance


def load_markups(path: Path) -> list[Vector]:
    record = json.loads(path.read_text(encoding="utf-8"))
    try:
        points = record["markups"][0]["controlPoints"]
        return [tuple(float(v) for v in point["position"]) for point in points]
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError(f"无法从 {path} 读取 Slicer Markups 控制点") from error


def join_routes(paths: Sequence[Path]) -> list[Vector]:
    if not paths:
        raise ValueError("至少需要一个 --route 路径文件")
    route: list[Vector] = []
    for path in paths:
        part = load_markups(path)
        if len(part) < 2:
            raise ValueError(f"中心线路径至少需要两个点：{path}")
        route.extend(part if not route else part[1:])
    return route


def interpolate_polyline(points: Sequence[Vector], fraction: float) -> Vector:
    if len(points) < 2:
        raise ValueError("折线至少需要两个点")
    segments = [distance(a, b) for a, b in zip(points, points[1:])]
    total = sum(segments)
    if total <= 1e-12:
        return points[0]
    target = min(1.0, max(0.0, fraction)) * total
    walked = 0.0
    for a, b, segment in zip(points, points[1:], segments):
        if walked + segment >= target:
            ratio = (target - walked) / segment if segment else 0.0
            return tuple(a[i] + ratio * (b[i] - a[i]) for i in range(3))  # type: ignore[return-value]
        walked += segment
    return points[-1]


def map_progress_to_route(progress_mm: Sequence[float], route: Sequence[Vector]) -> list[Vector]:
    maximum = max(progress_mm)
    if maximum <= 1e-12:
        raise ValueError("M0 没有产生正向推进量，无法映射中心线弧长")
    return [interpolate_polyline(route, value / maximum) for value in progress_mm]


def _power_iteration(matrix: Sequence[Sequence[float]]) -> list[float]:
    vector = [1.0, 0.0, 0.0, 0.0]
    for _ in range(80):
        next_vector = [sum(row[j] * vector[j] for j in range(4)) for row in matrix]
        magnitude = math.sqrt(sum(value * value for value in next_vector))
        if magnitude <= 1e-12:
            return vector
        vector = [value / magnitude for value in next_vector]
    return vector


def _rotation_from_pairs(source: Sequence[Vector], target: Sequence[Vector]):
    source_center = tuple(sum(point[i] for point in source) / len(source) for i in range(3))
    target_center = tuple(sum(point[i] for point in target) / len(target) for i in range(3))
    covariance = [
        [
            sum((p[i] - source_center[i]) * (q[j] - target_center[j]) for p, q in zip(source, target))
            for j in range(3)
        ]
        for i in range(3)
    ]
    trace = covariance[0][0] + covariance[1][1] + covariance[2][2]
    h = covariance
    horn = [
        [trace, h[1][2] - h[2][1], h[2][0] - h[0][2], h[0][1] - h[1][0]],
        [h[1][2] - h[2][1], h[0][0] - h[1][1] - h[2][2], h[0][1] + h[1][0], h[0][2] + h[2][0]],
        [h[2][0] - h[0][2], h[0][1] + h[1][0], -h[0][0] + h[1][1] - h[2][2], h[1][2] + h[2][1]],
        [h[0][1] - h[1][0], h[0][2] + h[2][0], h[1][2] + h[2][1], -h[0][0] - h[1][1] + h[2][2]],
    ]
    w, x, y, z = _power_iteration(horn)
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return source_center, target_center, rotation


def _apply_rotation(rotation, point: Vector) -> Vector:
    return tuple(sum(rotation[i][j] * point[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def similarity_register(source: Sequence[Vector], target: Sequence[Vector]) -> list[Vector]:
    if len(source) != len(target) or len(source) < 2:
        raise ValueError("配准要求数量相同且至少包含两个对应点")
    source_center, target_center, rotation = _rotation_from_pairs(source, target)
    rotated = [
        _apply_rotation(rotation, tuple(point[i] - source_center[i] for i in range(3)))  # type: ignore[arg-type]
        for point in source
    ]
    denominator = sum(sum(value * value for value in point) for point in rotated)
    numerator = sum(
        sum(a * (b[i] - target_center[i]) for i, a in enumerate(point))
        for point, b in zip(rotated, target)
    )
    factor = numerator / denominator if denominator > 1e-12 else 1.0
    return [
        tuple(target_center[i] + factor * point[i] for i in range(3))  # type: ignore[misc]
        for point in rotated
    ]


def load_vtp_lines(path: Path) -> list[list[Vector]]:
    """读取 Slicer 常见的 appended/base64/zlib VTK PolyData 线段。"""
    text = path.read_text(encoding="utf-8")
    appended_match = re.search(r"<AppendedData[^>]*>\s*_([\s\S]*?)</AppendedData>", text)
    if not appended_match:
        raise ValueError(f"{path} 不是受支持的 appended VTP 文件")
    encoded_all = "".join(appended_match.group(1).split())
    arrays: list[tuple[int, str, str]] = []
    for tag in re.findall(r"<DataArray[^>]*/>", text):
        name = re.search(r'Name="([^"]+)"', tag)
        offset = re.search(r'offset="(\d+)"', tag)
        value_type = re.search(r'type="(\w+)"', tag)
        if name and offset and value_type:
            arrays.append((int(offset.group(1)), name.group(1), value_type.group(1)))
    arrays.sort()

    def unpack(name: str, occurrence: int = 0):
        matches = [index for index, item in enumerate(arrays) if item[1] == name]
        if occurrence >= len(matches):
            raise ValueError(f"VTP 中缺少数组 {name}[{occurrence}]")
        index = matches[occurrence]
        offset, _, value_type = arrays[index]
        end = arrays[index + 1][0] if index + 1 < len(arrays) else len(encoded_all)
        encoded = encoded_all[offset:end]
        blob = b"".join(
            base64.b64decode(chunk + "=" * ((-len(chunk)) % 4))
            for chunk in encoded.split("=")
            if chunk
        )
        blocks, _, _ = struct.unpack_from("<III", blob, 0)
        sizes = struct.unpack_from("<" + "I" * blocks, blob, 12)
        position = 12 + 4 * blocks
        raw = b"".join(
            zlib.decompress(blob[position + sum(sizes[:i]): position + sum(sizes[: i + 1])])
            for i in range(blocks)
        )
        formats = {"Float32": "f", "Int64": "q"}
        if value_type not in formats:
            raise ValueError(f"暂不支持 VTP 数据类型 {value_type}")
        fmt = formats[value_type]
        return struct.unpack("<" + fmt * (len(raw) // struct.calcsize(fmt)), raw)

    coordinates = unpack("Points")
    points = [tuple(coordinates[i:i + 3]) for i in range(0, len(coordinates), 3)]
    connectivity = unpack("connectivity", 1)
    offsets = unpack("offsets", 1)
    start = 0
    lines: list[list[Vector]] = []
    for end in offsets:
        lines.append([points[index] for index in connectivity[start:end]])
        start = end
    return lines
