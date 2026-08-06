"""跨线程传递的数据结构；不依赖 OpenCV、Torch 或电机接口。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MotorSnapshot:
    motion_mode: str = "idle"
    m0_position: Optional[float] = None
    m2_position: Optional[float] = None
    m0_feedback_speed: float = 0.0
    logical_m0_feedback_speed: float = 0.0
    m0_command_speed: float = 0.0
    branch_bend_signal: float = 0.0
    feedback_valid: bool = False
    robot_state: str = "Unknown"
    reason: str = "not_published"
    updated_at: float = 0.0


@dataclass(frozen=True)
class BranchGateStatus:
    phase: str = "INACTIVE"
    anchor_position: Optional[float] = None
    forward_delta: float = 0.0
    rulb_hits: int = 0
    bi_hits: int = 0
    rmb_absent: bool = False
    bend_recent: bool = False
    reason: str = "inactive"


@dataclass(frozen=True)
class DetectionState:
    observed_position: Optional[str] = None
    observed_confidence: Optional[float] = None
    candidate_position: Optional[str] = None
    candidate_count: int = 0
    required_count: Optional[int] = None
    confirmed_position: str = "TR"
    confirmed_changed: bool = False
    safe_state: str = "CONFIRMED"
    reason: str = "initialized"
    motion_mode: str = "idle"
    gate: BranchGateStatus = BranchGateStatus()
    updated_at: float = 0.0
