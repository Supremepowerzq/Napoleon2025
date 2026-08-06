"""支气管检测部署配置。所有体模调参集中在此文件。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ASSET_ROOT = PROJECT_ROOT / "model_data" / "bronchus_yolo"
DEFAULT_MODEL_PATH = ASSET_ROOT / "best.pt"
DEFAULT_UNDISTORT_MAP_PATH = ASSET_ROOT / "undistort_maps_zhiqiguan_roi.npz"


FORWARD_TREE = {
    "TR": ("RMB", "LMB"),
    "RMB": ("RULB", "BI"),
    "BI": ("RMLB", "RLLB"),
    "LMB": ("LULB", "LLLB"),
    "RULB": (),
    "RMLB": (),
    "RLLB": (),
    "LULB": (),
    "LLLB": (),
}
PARENT_MAP = {
    child: parent
    for parent, children in FORWARD_TREE.items()
    for child in children
}

LABEL_ALIASES = {
    "TR": "TR",
    "CARINA": "TR",
    "RMB": "RMB",
    "LMB": "LMB",
    "RUL": "RULB",
    "RULB": "RULB",
    "BI": "BI",
    "RML": "RMLB",
    "RMLB": "RMLB",
    "RLL": "RLLB",
    "RLLB": "RLLB",
    "LUB": "LULB",
    "LULB": "LULB",
    "LLB": "LLLB",
    "LLLB": "LLLB",
}

# 兼容现有支气管地图使用的旧短标签。
LEGACY_LABELS = {
    "TR": "TR",
    "RMB": "RMB",
    "LMB": "LMB",
    "RULB": "RUL",
    "BI": "BI",
    "RMLB": "RML",
    "RLLB": "RLL",
    "LULB": "LUB",
    "LLLB": "LLB",
}


@dataclass(frozen=True)
class StateMachineConfig:
    confidence_threshold: float = 0.30
    default_confirm_frames: int = 3
    forward_confirm_frames: Dict[Tuple[str, str], int] = field(
        default_factory=lambda: {
            ("TR", "RMB"): 4,
            ("TR", "LMB"): 4,
            ("RMB", "RULB"): 3,
            ("RMB", "BI"): 3,
            ("BI", "RMLB"): 4,
            ("BI", "RLLB"): 4,
            ("LMB", "LULB"): 4,
            ("LMB", "LLLB"): 4,
        }
    )
    backward_confirm_frames: Dict[Tuple[str, str], int] = field(
        default_factory=lambda: {
            ("RMB", "TR"): 3,
            ("LMB", "TR"): 3,
            ("RULB", "RMB"): 3,
            ("BI", "RMB"): 3,
            ("RMLB", "BI"): 3,
            ("RLLB", "BI"): 3,
            ("LULB", "LMB"): 3,
            ("LLLB", "LMB"): 3,
        }
    )

    # RMB 分叉区：RULB 稀疏窗口 + BI 延迟确认。
    rulb_window_frames: int = 6
    rulb_required_hits: int = 3
    bi_window_frames: int = 8
    bi_required_hits: int = 6
    bi_rmb_absent_frames: int = 2
    bi_commit_m0_delta_deg: float = 45.0
    rulb_max_m0_delta_deg: float = 120.0
    branch_bend_threshold: float = 0.15
    branch_bend_memory_seconds: float = 1.0
    branch_bend_m2_position_threshold_deg: float = 5.0
    branch_bend_release_position_deg: float = 2.0
    branch_bend_release_frames: int = 5
    rulb_strong_bend_m2_position_deg: float = 100.0
    rulb_idle_grace_seconds: float = 3.0
    rulb_idle_required_hits: int = 2
    rulb_held_bend_idle_required_hits: int = 1
    rulb_competition_hits: int = 2

    # M0 反馈换算；Napoleon 中正向命令下发到硬件前会乘 -1。
    m0_direction_sign: float = -1.0
    m0_feedback_deadzone: float = 3.0
    m0_command_deadzone: float = 5.0
    m2_feedback_scale: float = 225.0
    feedback_timeout_seconds: float = 0.5
    motion_resume_frames: int = 2


def normalize_label(label: object) -> str | None:
    if label is None:
        return None
    return LABEL_ALIASES.get(str(label).strip().upper())


def to_legacy_label(label: str) -> str:
    return LEGACY_LABELS.get(label, label)
