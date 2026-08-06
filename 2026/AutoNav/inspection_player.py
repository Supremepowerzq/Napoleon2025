# -*- coding: utf-8 -*-
"""全流程自主巡检的 Layer 1 轨迹预拼接器。"""

from __future__ import annotations

import glob
import json
import os
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .config import (
    INSPECTION_MATCH_MAX_NORMALIZED_DISTANCE,
    INSPECTION_MATCH_TOLERANCE,
    INSPECTION_MIN_FORWARD_TRAVEL_M0,
    PLAYBACK_MAX_DELTA,
    PLAYBACK_SPEED,
)
from .sequence_player import SequencePlayer


DEFAULT_INSPECTION_ROUTE: Tuple[Tuple[str, int, str], ...] = (
    ("LUB", 4, "左上"),
    ("LLB", 5, "左下"),
    ("RUL", 6, "右上"),
    ("RML", 8, "右中"),
    ("RLL", 9, "右下"),
)


def _validate_source_file(path: str, code: str, label: int) -> Tuple[bool, str]:
    """检查文件是否为指定叶段可用于拼接的“原点→叶段”单条轨迹。"""
    try:
        with open(path, "r", encoding="utf-8") as fp:
            data = json.load(fp)
        if int(data.get("path_label", -1)) != label:
            return False, f"路径标签不是 {code}"
        if str(data.get("yolo_code", code)).upper() != code:
            return False, f"路径代码不是 {code}"
        frames = data.get("frames", [])
        if len(frames) < 20:
            return False, "有效帧少于 20"
        start_m0 = float(frames[0].get("m0_angle", 0.0))
        end_m0 = float(frames[-1].get("m0_angle", 0.0))
        if start_m0 - end_m0 < INSPECTION_MIN_FORWARD_TRAVEL_M0:
            return False, "M0 前进行程不足，可能是已返回原点的闭环记录"
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        return False, f"文件读取失败: {exc}"
    return True, ""


def list_inspection_source_files(
    demo_dir: str,
    code: str,
    label: int,
) -> List[str]:
    """列出指定叶段中可供全流程规划选择的轨迹文件。"""
    result: List[str] = []
    for path in sorted(glob.glob(os.path.join(demo_dir, "*.json"))):
        valid, _reason = _validate_source_file(path, code, label)
        if valid:
            result.append(os.path.abspath(path))
    return result


@dataclass(frozen=True)
class InspectionSegment:
    name: str
    kind: str
    start_step: int
    end_step: int


@dataclass(frozen=True)
class InspectionTransition:
    from_code: str
    to_code: str
    from_step: int
    to_step: int
    normalized_distance: float


class InspectionSequencePlayer(SequencePlayer):
    """
    将五条原点到叶段的离散轨迹预先拼成一条完整 Layer 1 序列。

    叶段切换始终沿刚完成的轨迹倒序回退至近似公共点，然后从下一条
    轨迹的公共点继续前进；最后沿右下轨迹倒序返回巡检出发点。
    """

    def __init__(
        self,
        demo_dir: str,
        route: Sequence[Tuple[str, int, str]] = DEFAULT_INSPECTION_ROUTE,
        source_files: Optional[Mapping[str, str]] = None,
    ):
        self.demo_dir = demo_dir
        self.path_label = -1
        self.speed = PLAYBACK_SPEED
        self.demo_file: Optional[str] = None
        self.source_files: List[str] = []
        self._source_timestamps = None
        self.route = tuple(route)
        self.transitions: List[InspectionTransition] = []
        self.segments: List[InspectionSegment] = []

        trajectories: List[np.ndarray] = []
        for code, label, _name in self.route:
            source = self._resolve_source_file(code, label, source_files)
            player = SequencePlayer(demo_dir, path_label=label, demo_file=source)
            trajectory = player.get_cumulative_trajectory()
            if len(trajectory) < 2:
                raise ValueError(f"[Inspection] {code} 轨迹为空: {source}")
            trajectories.append(trajectory)
            self.source_files.append(source)

        self._avg_cumulative = self._compose(trajectories)
        self._total_steps = len(self._avg_cumulative)
        self._start_angles: Optional[np.ndarray] = None
        self._step = 0
        self._prev_target: Optional[np.ndarray] = None

        sources = ", ".join(os.path.basename(path) for path in self.source_files)
        print(f"[Inspection] 全流程轨迹已拼接，共 {self._total_steps} 步")
        print(f"[Inspection] 轨迹来源: {sources}")

    def _resolve_source_file(
        self,
        code: str,
        label: int,
        selected: Optional[Mapping[str, str]],
    ) -> str:
        """使用界面指定文件；未指定时保留原有的“最新有效轨迹”行为。"""
        if selected is None:
            valid = list_inspection_source_files(self.demo_dir, code, label)
            if valid:
                return valid[-1]
            raise ValueError(f"[Inspection] 未找到可用的 {code} 单条轨迹")

        if code not in selected:
            raise ValueError(f"[Inspection] 尚未选择 {code} 轨迹")
        path = os.path.abspath(str(selected[code]))
        demo_root = os.path.abspath(self.demo_dir)
        if os.path.dirname(path) != demo_root:
            raise ValueError(f"[Inspection] {code} 轨迹不在采集目录内")
        valid, reason = _validate_source_file(path, code, label)
        if not valid:
            raise ValueError(
                f"[Inspection] {code} 轨迹不可用于拼接: "
                f"{os.path.basename(path)}（{reason}）"
            )
        return path

    @staticmethod
    def _find_common_point(
        previous: np.ndarray,
        following: np.ndarray,
    ) -> Tuple[int, int, float]:
        """在三轴角度空间中寻找插入最深的近似重复点。"""
        tolerance = np.asarray(INSPECTION_MATCH_TOLERANCE, dtype=np.float32)
        difference = previous[:, None, :] - following[None, :, :]
        component_match = np.all(np.abs(difference) <= tolerance, axis=2)
        normalized = np.linalg.norm(difference / tolerance, axis=2)
        candidates = np.argwhere(
            component_match
            & (normalized <= INSPECTION_MATCH_MAX_NORMALIZED_DISTANCE)
        )
        if len(candidates) == 0:
            raise ValueError("[Inspection] 相邻轨迹未找到近似重复点")

        best_key = None
        best_pair = None
        for previous_step, following_step in candidates:
            # M0 越负表示插入越深；同深度时优先两条轨迹进度都更靠后，
            # 最后再选择三轴距离更小的点。
            insertion_depth = -0.5 * (
                float(previous[previous_step, 0])
                + float(following[following_step, 0])
            )
            shared_progress = min(
                previous_step / max(len(previous) - 1, 1),
                following_step / max(len(following) - 1, 1),
            )
            distance = float(normalized[previous_step, following_step])
            key = (insertion_depth, shared_progress, -distance)
            if best_key is None or key > best_key:
                best_key = key
                best_pair = (int(previous_step), int(following_step), distance)
        assert best_pair is not None
        return best_pair

    @staticmethod
    def _bridge(start: np.ndarray, end: np.ndarray) -> np.ndarray:
        """在两个近似公共点间生成受单步限幅约束的小范围对齐段。"""
        delta = end - start
        steps = max(
            1,
            max(
                int(np.ceil(abs(float(delta[axis])) / PLAYBACK_MAX_DELTA[axis]))
                for axis in range(3)
            ),
        )
        return np.stack(
            [start + delta * (index / steps) for index in range(1, steps + 1)],
            axis=0,
        ).astype(np.float32)

    def _compose(self, trajectories: Sequence[np.ndarray]) -> np.ndarray:
        points: List[np.ndarray] = []

        def append_segment(name: str, kind: str, values: np.ndarray) -> None:
            if len(values) == 0:
                return
            start = len(points)
            for value in values:
                if points and np.allclose(points[-1], value, atol=1e-6):
                    continue
                points.append(np.asarray(value, dtype=np.float32).copy())
            if len(points) > start:
                self.segments.append(
                    InspectionSegment(name, kind, start, len(points) - 1)
                )

        _first_code, _first_label, first_name = self.route[0]
        append_segment(f"前往{first_name}", "forward", trajectories[0])

        for index in range(1, len(trajectories)):
            previous = trajectories[index - 1]
            following = trajectories[index]
            previous_code, _previous_label, previous_name = self.route[index - 1]
            following_code, _following_label, following_name = self.route[index]
            previous_step, following_step, distance = self._find_common_point(
                previous, following
            )
            self.transitions.append(
                InspectionTransition(
                    previous_code,
                    following_code,
                    previous_step,
                    following_step,
                    distance,
                )
            )

            backtrack = (
                previous[-2::-1]
                if previous_step == 0 else
                previous[-2:previous_step - 1:-1]
            )
            append_segment(
                f"{previous_name}→{following_name}：沿原轨迹回退",
                "backtrack",
                backtrack,
            )
            append_segment(
                f"{previous_name}→{following_name}：公共点对齐",
                "bridge",
                self._bridge(previous[previous_step], following[following_step]),
            )
            append_segment(
                f"前往{following_name}",
                "forward",
                following[following_step + 1:],
            )

        _last_code, _last_label, last_name = self.route[-1]
        append_segment(
            f"{last_name}→TR：沿原轨迹返回原点",
            "return_origin",
            trajectories[-1][-2::-1],
        )
        return np.stack(points, axis=0).astype(np.float32)

    @property
    def current_segment(self) -> Optional[InspectionSegment]:
        if not self.segments:
            return None
        step = min(self._step, max(self._total_steps - 1, 0))
        for segment in self.segments:
            if segment.start_step <= step <= segment.end_step:
                return segment
        return self.segments[-1]

    @property
    def current_stage(self) -> str:
        segment = self.current_segment
        return segment.name if segment is not None else "全流程巡检"

    @property
    def current_stage_progress(self) -> float:
        segment = self.current_segment
        if segment is None:
            return 0.0
        length = max(segment.end_step - segment.start_step, 1)
        return float(np.clip(
            (self._step - segment.start_step) / length,
            0.0,
            1.0,
        ))

    def get_path_info(self) -> Dict[str, object]:
        segment = self.current_segment
        stage_kind = segment.kind if segment is not None else ""
        return {
            "path_label": -1,
            "total_steps": self._total_steps,
            "stage_name": self.current_stage,
            "stage_kind": stage_kind,
            "stage_progress": self.current_stage_progress,
            "layer2_allowed": stage_kind == "forward",
            "layer3_allowed": stage_kind == "forward",
            "source_files": tuple(self.source_files),
        }
