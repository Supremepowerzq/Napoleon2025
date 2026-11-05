"""输入输出工具。"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from .kinematics import TrajectoryPoint


def load_record(path: Path) -> dict[str, Any]:
    record = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(record, dict) or not isinstance(record.get("frames"), list):
        raise ValueError("输入 JSON 必须包含数组字段 frames")
    return record


def write_trajectory_json(path: Path, points: list[TrajectoryPoint], metadata: dict[str, Any]) -> None:
    payload = {
        "metadata": metadata,
        "coordinate_system": "local: +X forward, +Z initial up",
        "unit": "mm",
        "points": [
            {
                "frame_index": point.frame_index,
                "timestamp": point.timestamp,
                "position": list(point.position),
                "forward": list(point.forward),
                "insertion_mm": point.insertion_mm,
            }
            for point in points
        ],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def write_trajectory_csv(path: Path, points: list[TrajectoryPoint]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["frame_index", "timestamp_s", "x_mm", "y_mm", "z_mm", "fx", "fy", "fz", "insertion_mm"]
        )
        for point in points:
            writer.writerow(
                [
                    point.frame_index,
                    point.timestamp,
                    *point.position,
                    *point.forward,
                    point.insertion_mm,
                ]
            )
