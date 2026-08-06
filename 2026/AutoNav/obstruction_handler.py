# -*- coding: utf-8 -*-
"""
ObstructionHandler — 阻塞物检测、追踪与清除（Layer 3）
======================================================
在自主介入导航过程中，一旦检测到阻塞物，优先切换到阻塞物追踪模式，
完全清除后自动回到最近的导航断点，继续介入。

状态机
------
IDLE      → 随主控制器状态变化，本模块不输出指令
CLEARING  → 检测到阻塞物，向阻塞物方向驱动内镜进行清除
RETURNING → 阻塞物清除完毕，驱动电机回到之前保存的断点位置
（回到断点后，外层 NavController 将模式切回 NAVIGATING）

坐标约定（与视觉模块一致）
--------------------------
obstruction_cx ∈ [-1, 1]  负值=偏左  正值=偏右
obstruction_cy ∈ [-1, 1]  负值=偏上  正值=偏下
depth_mean_mm              当前腔道平均深度（用于控制前进速度）
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple, Dict
import numpy as np

from .config import (
    MOTOR_LIMITS,
    OBSTRUCTION_CLEAR_FRAMES,
    OBSTRUCTION_EXIT_AREA_THRESHOLD,
    OBSTRUCTION_RETURN_STABLE_FRAMES,
    OBSTRUCTION_TRACK_GAIN_M1,
    OBSTRUCTION_TRACK_GAIN_M2,
    OBSTRUCTION_TRACK_GAIN_M0,
    OBSTRUCTION_MIN_DEPTH_MM,
    RETURN_TOLERANCE_DEG,
    RETURN_MAX_DELTA,
)


class HandlerState(Enum):
    IDLE      = "idle"
    CLEARING  = "clearing"
    RETURNING = "returning"


@dataclass
class CheckPoint:
    """导航断点：清除完阻塞物后返回的位置。"""
    motor_angles:  np.ndarray   # [m0, m1, m2]  °
    sequence_step: int          # SequencePlayer 的步骤索引
    base_target: Optional[np.ndarray] = None
    route_stage: str = ""
    route_stage_kind: str = ""


@dataclass
class HandlerDecision:
    """ObstructionHandler 每步的输出。"""
    state:     HandlerState
    delta_m0:  float
    delta_m1:  float
    delta_m2:  float
    target_m0: float
    target_m1: float
    target_m2: float
    resumed:   bool    # True = 本帧刚完成 RETURNING，外层应切回 NAVIGATING
    info:      str     # 调试文字
    clear_count: int = 0
    return_stable_frames: int = 0


class ObstructionHandler:
    """
    阻塞物追踪与清除状态机。

    Parameters
    ----------
    track_gain_m1 : float  左右追踪增益
    track_gain_m2 : float  上下追踪增益
    track_gain_m0 : float  前进追踪增益
    clear_frames  : int    连续 N 帧未检测到阻塞物才认为清除完成
    return_tol    : float  返回断点的到位容差（°）
    """

    def __init__(
        self,
        track_gain_m1: float = OBSTRUCTION_TRACK_GAIN_M1,
        track_gain_m2: float = OBSTRUCTION_TRACK_GAIN_M2,
        track_gain_m0: float = OBSTRUCTION_TRACK_GAIN_M0,
        clear_frames:  int   = OBSTRUCTION_CLEAR_FRAMES,
        return_tol:    float = RETURN_TOLERANCE_DEG,
        exit_area_threshold: float = OBSTRUCTION_EXIT_AREA_THRESHOLD,
        return_stable_frames: int = OBSTRUCTION_RETURN_STABLE_FRAMES,
    ):
        self.gain_m1      = track_gain_m1
        self.gain_m2      = track_gain_m2
        self.gain_m0      = track_gain_m0
        self.clear_frames = clear_frames
        self.return_tol   = return_tol
        self.exit_area_threshold = max(float(exit_area_threshold), 0.0)
        self.return_stable_frames = max(int(return_stable_frames), 1)

        self._state: HandlerState = HandlerState.IDLE
        self._checkpoint: Optional[CheckPoint] = None
        self._clear_count: int = 0   # 连续未检测到阻塞物的帧数
        self._return_stable_count: int = 0

        # 内部目标累积（追踪时用增量控制，返回时用绝对目标）
        self._target: Optional[np.ndarray] = None

    # ── 状态查询 ──────────────────────────────────────────────────

    @property
    def state(self) -> HandlerState:
        return self._state

    @property
    def is_active(self) -> bool:
        return self._state != HandlerState.IDLE

    @property
    def checkpoint(self) -> Optional[CheckPoint]:
        return self._checkpoint

    # ── 外部接口 ──────────────────────────────────────────────────

    def save_checkpoint(
        self,
        motor_angles: Tuple[float, float, float],
        sequence_step: int,
        base_target: Optional[Tuple[float, float, float]] = None,
        route_stage: str = "",
        route_stage_kind: str = "",
    ):
        """导航被中断时，由 NavController 调用保存断点。"""
        self._checkpoint = CheckPoint(
            motor_angles  = np.array(motor_angles, dtype=np.float32),
            sequence_step = sequence_step,
            base_target=(
                np.array(base_target, dtype=np.float32)
                if base_target is not None else None
            ),
            route_stage=str(route_stage),
            route_stage_kind=str(route_stage_kind),
        )
        print(f"[ObstructionHandler] 断点已保存: 步骤={sequence_step}  "
              f"位置=({motor_angles[0]:.1f}, {motor_angles[1]:.1f}, {motor_angles[2]:.1f}) "
              f"阶段={route_stage or '-'}")

    def start_clearing(self, current_motor_angles: Tuple[float, float, float]):
        """切换到 CLEARING 模式（由 NavController 在检测到阻塞物时调用）。"""
        self._state = HandlerState.CLEARING
        self._clear_count = 0
        self._return_stable_count = 0
        self._target = np.array(current_motor_angles, dtype=np.float32)
        print("[ObstructionHandler] → CLEARING 追踪阻塞物")

    def start_returning(self, current_motor_angles: Tuple[float, float, float]):
        """切换到 RETURNING 模式（清除完成后调用）。"""
        if self._checkpoint is None:
            print("[ObstructionHandler] 无断点，跳过返回")
            self._state = HandlerState.IDLE
            return
        self._state = HandlerState.RETURNING
        self._return_stable_count = 0
        self._target = np.array(current_motor_angles, dtype=np.float32)
        print(f"[ObstructionHandler] → RETURNING 返回断点 步骤={self._checkpoint.sequence_step}")

    def reset(self):
        """完全重置（切换新路径或退出自主模式时调用）。"""
        self._state = HandlerState.IDLE
        self._checkpoint = None
        self._clear_count = 0
        self._return_stable_count = 0
        self._target = None

    # ── 核心更新 ──────────────────────────────────────────────────

    def update(
        self,
        obstruction_detected: bool,
        obstruction_cx:       float,
        obstruction_cy:       float,
        depth_mean_mm:        float,
        current_motor_angles: Tuple[float, float, float],
        obstruction_area:     Optional[float] = None,
        visual_fresh:         bool = True,
        visual_is_new:        bool = True,
    ) -> HandlerDecision:
        """
        每帧调用，根据当前状态输出电机指令。

        Parameters
        ----------
        obstruction_detected : 本帧是否检测到阻塞物
        obstruction_cx/cy    : 阻塞物质心（归一化 [-1,1]）
        depth_mean_mm        : 当前腔道平均深度（mm）
        current_motor_angles : 当前电机实际角度 (m0, m1, m2)

        Returns
        -------
        HandlerDecision
        """
        cur = np.array(current_motor_angles, dtype=np.float32)

        if self._state == HandlerState.CLEARING:
            return self._step_clearing(
                obstruction_detected, obstruction_cx, obstruction_cy,
                depth_mean_mm, cur, obstruction_area,
                visual_fresh, visual_is_new,
            )
        elif self._state == HandlerState.RETURNING:
            return self._step_returning(cur)
        else:
            return HandlerDecision(
                state=HandlerState.IDLE,
                delta_m0=0.0, delta_m1=0.0, delta_m2=0.0,
                target_m0=cur[0], target_m1=cur[1], target_m2=cur[2],
                resumed=False, info="idle",
            )

    def _step_clearing(
        self,
        detected: bool,
        cx: float, cy: float,
        depth_mm: float,
        cur: np.ndarray,
        area: Optional[float],
        visual_fresh: bool,
        visual_is_new: bool,
    ) -> HandlerDecision:
        """追踪并清除阻塞物。"""
        if not visual_fresh:
            return HandlerDecision(
                state=HandlerState.CLEARING,
                delta_m0=0.0, delta_m1=0.0, delta_m2=0.0,
                target_m0=self._target[0], target_m1=self._target[1],
                target_m2=self._target[2],
                resumed=False, clear_count=self._clear_count,
                info="visual stale, hold clearing target",
            )

        if not visual_is_new:
            return HandlerDecision(
                state=HandlerState.CLEARING,
                delta_m0=0.0, delta_m1=0.0, delta_m2=0.0,
                target_m0=self._target[0], target_m1=self._target[1],
                target_m2=self._target[2],
                resumed=False, clear_count=self._clear_count,
                info="waiting for new visual frame",
            )

        area_available = area is not None and np.isfinite(area)
        clear_candidate = (
            not detected
            or (
                area_available
                and float(area) <= self.exit_area_threshold
            )
        )
        if clear_candidate:
            self._clear_count += 1
            if self._clear_count >= self.clear_frames:
                # 清除完成，转 RETURNING
                self.start_returning(tuple(cur))
                return HandlerDecision(
                    state=HandlerState.RETURNING,
                    delta_m0=0.0, delta_m1=0.0, delta_m2=0.0,
                    target_m0=cur[0], target_m1=cur[1], target_m2=cur[2],
                    resumed=False,
                    clear_count=self._clear_count,
                    info=f"clearing → returning (clear_count={self._clear_count})",
                )
            # 还未确认清除，保持当前位置
            return HandlerDecision(
                state=HandlerState.CLEARING,
                delta_m0=0.0, delta_m1=0.0, delta_m2=0.0,
                target_m0=cur[0], target_m1=cur[1], target_m2=cur[2],
                resumed=False,
                clear_count=self._clear_count,
                info=(
                    f"clear candidate area="
                    f"{float(area):.3f}" if area_available else
                    "clear candidate area=n/a "
                ) + (
                    f"({self._clear_count}/{self.clear_frames})"
                ),
            )

        # 检测到阻塞物：计算追踪增量
        self._clear_count = 0

        dm1 = float(np.clip(self.gain_m1 * cx, -RETURN_MAX_DELTA[1], RETURN_MAX_DELTA[1]))
        dm2 = float(np.clip(self.gain_m2 * cy, -RETURN_MAX_DELTA[2], RETURN_MAX_DELTA[2]))

        # 前进：深度越大越快，深度过近停止
        if depth_mm > OBSTRUCTION_MIN_DEPTH_MM:
            forward_scale = float(np.clip((depth_mm - OBSTRUCTION_MIN_DEPTH_MM) / 50.0, 0.0, 1.0))
            dm0 = float(-self.gain_m0 * forward_scale)   # M0 负方向为前进
        else:
            dm0 = 0.0

        if self._target is None:
            self._target = cur.copy()

        self._target[0] = float(np.clip(self._target[0] + dm0, *MOTOR_LIMITS[0]))
        self._target[1] = float(np.clip(self._target[1] + dm1, *MOTOR_LIMITS[1]))
        self._target[2] = float(np.clip(self._target[2] + dm2, *MOTOR_LIMITS[2]))

        return HandlerDecision(
            state=HandlerState.CLEARING,
            delta_m0=dm0, delta_m1=dm1, delta_m2=dm2,
            target_m0=self._target[0], target_m1=self._target[1], target_m2=self._target[2],
            resumed=False,
            clear_count=self._clear_count,
            info=(
                f"tracking area="
                f"{float(area):.3f} " if area_available else
                "tracking area=n/a "
            ) + (
                f"cx={cx:.2f} cy={cy:.2f} depth={depth_mm:.0f}mm"
            ),
        )

    def _step_returning(self, cur: np.ndarray) -> HandlerDecision:
        """驱动电机回到断点位置。"""
        if self._checkpoint is None:
            self._state = HandlerState.IDLE
            return HandlerDecision(
                state=HandlerState.IDLE,
                delta_m0=0.0, delta_m1=0.0, delta_m2=0.0,
                target_m0=cur[0], target_m1=cur[1], target_m2=cur[2],
                resumed=True, info="no checkpoint, resumed",
            )

        goal = self._checkpoint.motor_angles
        error = goal - cur
        dist  = float(np.max(np.abs(error)))

        if dist <= self.return_tol:
            self._return_stable_count += 1
        else:
            self._return_stable_count = 0

        if self._return_stable_count >= self.return_stable_frames:
            # 连续若干帧到达断点，避免单次缓存角度抖动导致提前续航。
            self._state = HandlerState.IDLE
            print(f"[ObstructionHandler] 已返回断点 (误差={dist:.2f}°) → 恢复导航")
            return HandlerDecision(
                state=HandlerState.IDLE,
                delta_m0=0.0, delta_m1=0.0, delta_m2=0.0,
                target_m0=goal[0], target_m1=goal[1], target_m2=goal[2],
                resumed=True,
                return_stable_frames=self._return_stable_count,
                info=f"returned, error={dist:.2f}°",
            )

        # 向断点方向移动（逐步靠近）
        dm = np.clip(error * 0.3, -np.array([RETURN_MAX_DELTA[i] for i in range(3)]),
                     np.array([RETURN_MAX_DELTA[i] for i in range(3)]))

        self._target = cur + dm
        for i, (lo, hi) in MOTOR_LIMITS.items():
            self._target[i] = float(np.clip(self._target[i], lo, hi))

        return HandlerDecision(
            state=HandlerState.RETURNING,
            delta_m0=float(dm[0]), delta_m1=float(dm[1]), delta_m2=float(dm[2]),
            target_m0=self._target[0], target_m1=self._target[1], target_m2=self._target[2],
            resumed=False,
            return_stable_frames=self._return_stable_count,
            info=(
                f"returning, dist={dist:.1f}° "
                f"stable={self._return_stable_count}/{self.return_stable_frames}"
            ),
        )
