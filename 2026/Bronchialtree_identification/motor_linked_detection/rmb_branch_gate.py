"""只作用于 RMB–RULB–BI 的分叉决策区。"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional

from .config import StateMachineConfig
from .types import BranchGateStatus, MotorSnapshot


@dataclass(frozen=True)
class GateOutcome:
    handled: bool = False
    commit_position: Optional[str] = None
    observed_position: Optional[str] = None
    observed_confidence: Optional[float] = None
    candidate_position: Optional[str] = None
    candidate_count: int = 0
    required_count: Optional[int] = None
    safe_state: str = "UNCERTAIN"
    reason: str = "not_handled"
    status: BranchGateStatus = BranchGateStatus()


class RmbBranchGate:
    """BI 可见不等于已进入 BI；RULB 允许窗口内间歇出现。"""

    def __init__(self, config: StateMachineConfig) -> None:
        self.config = config
        self._rulb_seen = deque(maxlen=config.rulb_window_frames)
        self._bi_seen = deque(maxlen=config.bi_window_frames)
        self._rmb_seen = deque(maxlen=config.bi_rmb_absent_frames)
        self._rulb_confidences = deque(maxlen=config.rulb_window_frames)
        self._bi_confidences = deque(maxlen=config.bi_window_frames)
        self._active = False
        self._anchor_position: Optional[float] = None
        self._last_bend_at = float("-inf")
        self._last_forward_at = float("-inf")
        self._bend_latched = False
        self._straight_m2_frames = 0
        self._status = BranchGateStatus()

    @property
    def status(self) -> BranchGateStatus:
        return self._status

    def reset(self, reason: str = "inactive") -> None:
        self._rulb_seen.clear()
        self._bi_seen.clear()
        self._rmb_seen.clear()
        self._rulb_confidences.clear()
        self._bi_confidences.clear()
        self._active = False
        self._anchor_position = None
        self._last_bend_at = float("-inf")
        self._last_forward_at = float("-inf")
        self._bend_latched = False
        self._straight_m2_frames = 0
        self._status = BranchGateStatus(reason=reason)

    def start(self, motor_position: Optional[float]) -> None:
        self.reset("entered_rmb_gate")
        self._active = True
        self._anchor_position = motor_position
        self._status = BranchGateStatus(
            phase="RMB_GATE",
            anchor_position=motor_position,
            reason="entered_rmb_gate",
        )

    def _forward_delta(self, motor_position: Optional[float]) -> float:
        if self._anchor_position is None or motor_position is None:
            return 0.0
        return (
            motor_position - self._anchor_position
        ) * self.config.m0_direction_sign

    def _position_bent(self, motor: MotorSnapshot) -> bool:
        return (
            motor.m2_position is not None
            and abs(motor.m2_position)
            >= self.config.branch_bend_m2_position_threshold_deg
        )

    def _strong_bend(self, motor: MotorSnapshot) -> bool:
        return (
            motor.m2_position is not None
            and abs(motor.m2_position)
            >= self.config.rulb_strong_bend_m2_position_deg
        )

    def _update_bend_latch(self, motor: MotorSnapshot) -> bool:
        bend_input_active = (
            motor.branch_bend_signal >= self.config.branch_bend_threshold
        )
        if self._position_bent(motor) or (
            bend_input_active and motor.m2_position is not None
        ):
            self._bend_latched = True
            self._straight_m2_frames = 0
        elif (
            motor.m2_position is not None
            and abs(motor.m2_position)
            <= self.config.branch_bend_release_position_deg
        ):
            self._straight_m2_frames += 1
            if (
                self._straight_m2_frames
                >= self.config.branch_bend_release_frames
            ):
                self._bend_latched = False
        else:
            # M2 仍未回到直线释放区，继续保留之前的弯曲状态。
            self._straight_m2_frames = 0
        return self._bend_latched

    def _make_status(
        self,
        phase: str,
        forward_delta: float,
        rmb_absent: bool,
        bend_recent: bool,
        reason: str,
    ) -> BranchGateStatus:
        self._status = BranchGateStatus(
            phase=phase,
            anchor_position=self._anchor_position,
            forward_delta=forward_delta,
            rulb_hits=sum(self._rulb_seen),
            bi_hits=sum(self._bi_seen),
            rmb_absent=rmb_absent,
            bend_recent=bend_recent,
            reason=reason,
        )
        return self._status

    @staticmethod
    def _best_observation(confidences: Dict[str, float]) -> tuple[Optional[str], Optional[float]]:
        if not confidences:
            return None, None
        return max(confidences.items(), key=lambda item: item[1])

    def update(
        self,
        confirmed_position: str,
        confidences: Dict[str, float],
        motor: MotorSnapshot,
        transition_allowed: bool = True,
    ) -> GateOutcome:
        if confirmed_position != "RMB":
            if self._active or self._status.phase != "INACTIVE":
                self.reset("left_rmb_gate")
            return GateOutcome()

        # 黄色 RULB 候选出现后若操作者立即开始回退，先提交已到达的
        # RULB，再由普通拓扑按 RULB -> RMB -> TR 分级回退。
        if motor.motion_mode == "backward":
            if (
                self._active
                and sum(self._rulb_seen) > 0
                and motor.feedback_valid
                and self._strong_bend(motor)
                and self._forward_delta(motor.m0_position)
                <= self.config.rulb_max_m0_delta_deg
            ):
                forward_delta = self._forward_delta(motor.m0_position)
                reason = (
                    f"rulb_{sum(self._rulb_seen)}/"
                    f"{self.config.rulb_window_frames}_"
                    f"confirmed_on_reverse_start_"
                    f"delta_{forward_delta:.1f}deg"
                )
                status = self._make_status(
                    "RULB_CONFIRMED",
                    forward_delta,
                    False,
                    True,
                    reason,
                )
                self._active = False
                return GateOutcome(
                    handled=True,
                    commit_position="RULB",
                    observed_position="RULB",
                    observed_confidence=max(
                        self._rulb_confidences,
                        default=0.0,
                    ),
                    safe_state="CONFIRMED",
                    reason=reason,
                    status=status,
                )
            self.reset("backward_from_rmb")
            return GateOutcome()

        if not self._active:
            self.start(motor.m0_position)

        branch_confidences = {
            label: confidence
            for label, confidence in confidences.items()
            if label in {"RMB", "RULB", "BI"}
        }

        now = time.monotonic()
        held_bend = self._update_bend_latch(motor)
        if motor.motion_mode == "forward":
            self._last_forward_at = now
        if (
            held_bend
            or motor.branch_bend_signal >= self.config.branch_bend_threshold
        ):
            self._last_bend_at = now

        bend_recent = (
            now - self._last_bend_at
        ) <= self.config.branch_bend_memory_seconds
        forward_delta = self._forward_delta(motor.m0_position)

        # 已在前进期间出现 RULB 候选时，停车后给模型一个很短的补充确认窗口。
        # 这个窗口只累计 RULB，不累计 BI，也不允许从静止状态凭空建立候选。
        if motor.motion_mode != "forward":
            had_forward_rulb_evidence = sum(self._rulb_seen) > 0
            idle_grace_active = (
                had_forward_rulb_evidence
                and motor.feedback_valid
                and now - self._last_forward_at
                <= self.config.rulb_idle_grace_seconds
                and (
                    held_bend
                    or now - self._last_bend_at
                    <= max(
                        self.config.branch_bend_memory_seconds,
                        self.config.rulb_idle_grace_seconds,
                    )
                )
            )

            if idle_grace_active:
                # 停车时视野变化很小，空帧不应把前进阶段的稀疏命中挤出窗口。
                # 只有真正再次看到 RULB 时才追加证据。
                if "RULB" in branch_confidences:
                    self._rulb_seen.append(True)
                    self._rulb_confidences.append(
                        branch_confidences["RULB"]
                    )
                rulb_hits = sum(self._rulb_seen)
                idle_required_hits = min(
                    self.config.rulb_required_hits,
                    (
                        self.config.rulb_held_bend_idle_required_hits
                        if held_bend
                        else self.config.rulb_idle_required_hits
                    ),
                )
                rmb_absent = (
                    len(self._rmb_seen) >= self.config.bi_rmb_absent_frames
                    and not any(self._rmb_seen)
                )
                observed_position, observed_confidence = self._best_observation(
                    branch_confidences
                )

                if (
                    rulb_hits >= idle_required_hits
                    and forward_delta <= self.config.rulb_max_m0_delta_deg
                ):
                    reason = (
                        f"rulb_{rulb_hits}/{self.config.rulb_window_frames}_"
                        f"confirmed_during_idle_grace_"
                        f"delta_{forward_delta:.1f}deg"
                    )
                    status = self._make_status(
                        "RULB_CONFIRMED",
                        forward_delta,
                        rmb_absent,
                        True,
                        reason,
                    )
                    self._active = False
                    return GateOutcome(
                        handled=True,
                        commit_position="RULB",
                        observed_position="RULB",
                        observed_confidence=max(
                            self._rulb_confidences,
                            default=0.0,
                        ),
                        safe_state="CONFIRMED",
                        reason=reason,
                        status=status,
                    )

                reason = (
                    f"rulb_stop_confirm_{rulb_hits}/"
                    f"{idle_required_hits}"
                )
                status = self._make_status(
                    "RULB_STOP_CONFIRM",
                    forward_delta,
                    rmb_absent,
                    True,
                    reason,
                )
                return GateOutcome(
                    handled=True,
                    observed_position=observed_position,
                    observed_confidence=observed_confidence,
                    candidate_position="RULB",
                    candidate_count=rulb_hits,
                    required_count=idle_required_hits,
                    safe_state="CANDIDATE",
                    reason=reason,
                    status=status,
                )

            observed_position, observed_confidence = self._best_observation(
                branch_confidences
            )
            rmb_absent = (
                len(self._rmb_seen) >= self.config.bi_rmb_absent_frames
                and not any(self._rmb_seen)
            )
            status = self._make_status(
                "RMB_PAUSED",
                forward_delta,
                rmb_absent,
                bend_recent,
                "motion_idle_frozen",
            )
            return GateOutcome(
                handled=True,
                observed_position=observed_position,
                observed_confidence=observed_confidence,
                safe_state="CONFIRMED" if "RMB" in branch_confidences else "UNCERTAIN",
                reason="motion_idle_frozen",
                status=status,
            )

        self._rulb_seen.append("RULB" in branch_confidences)
        self._bi_seen.append("BI" in branch_confidences)
        self._rmb_seen.append("RMB" in branch_confidences)
        self._rulb_confidences.append(branch_confidences.get("RULB", 0.0))
        self._bi_confidences.append(branch_confidences.get("BI", 0.0))

        rmb_absent = (
            len(self._rmb_seen) >= self.config.bi_rmb_absent_frames
            and not any(self._rmb_seen)
        )
        rulb_hits = sum(self._rulb_seen)
        bi_hits = sum(self._bi_seen)
        observed_position, observed_confidence = self._best_observation(branch_confidences)

        rulb_visual_ready = (
            rulb_hits >= self.config.rulb_required_hits
            and bend_recent
        )
        rulb_strong_bend_ready = (
            rulb_hits >= 1
            and self._strong_bend(motor)
        )
        rulb_ready = (
            (rulb_visual_ready or rulb_strong_bend_ready)
            and forward_delta <= self.config.rulb_max_m0_delta_deg
        )
        bi_visual_ready = bi_hits >= self.config.bi_required_hits
        bi_ready = (
            bi_visual_ready
            and rmb_absent
            and rulb_hits < self.config.rulb_competition_hits
            and forward_delta >= self.config.bi_commit_m0_delta_deg
        )

        if motor.feedback_valid and transition_allowed and rulb_ready:
            evidence = (
                "strong_m2_bend"
                if rulb_strong_bend_ready
                else "visual_window_with_bend"
            )
            reason = (
                f"rulb_{rulb_hits}/{self.config.rulb_window_frames}_"
                f"{evidence}_"
                f"delta_{forward_delta:.1f}deg"
            )
            status = self._make_status(
                "RULB_CONFIRMED", forward_delta, rmb_absent, bend_recent, reason
            )
            self._active = False
            return GateOutcome(
                handled=True,
                commit_position="RULB",
                observed_position="RULB",
                observed_confidence=max(self._rulb_confidences, default=0.0),
                safe_state="CONFIRMED",
                reason=reason,
                status=status,
            )

        if motor.feedback_valid and transition_allowed and bi_ready:
            reason = (
                f"bi_{bi_hits}/{self.config.bi_window_frames}_rmb_absent_"
                f"delta_{forward_delta:.1f}deg"
            )
            status = self._make_status(
                "BI_CONFIRMED", forward_delta, rmb_absent, bend_recent, reason
            )
            self._active = False
            return GateOutcome(
                handled=True,
                commit_position="BI",
                observed_position="BI",
                observed_confidence=max(self._bi_confidences, default=0.0),
                safe_state="CONFIRMED",
                reason=reason,
                status=status,
            )

        if bi_visual_ready and rulb_hits < self.config.rulb_competition_hits:
            phase = "BI_SOFT"
            candidate = "BI"
            candidate_count = bi_hits
            required_count = self.config.bi_required_hits
            blocked = []
            if not rmb_absent:
                blocked.append("rmb_visible")
            if forward_delta < self.config.bi_commit_m0_delta_deg:
                blocked.append(
                    f"delta_{forward_delta:.1f}<{self.config.bi_commit_m0_delta_deg:.1f}"
                )
            if motor.motion_mode != "forward":
                blocked.append(f"motion_{motor.motion_mode}")
            if not transition_allowed:
                blocked.append("motion_arming")
            reason = "bi_soft_waiting_" + "_and_".join(blocked or ["commit"])
        elif rulb_hits > 0:
            phase = "RULB_CANDIDATE"
            candidate = "RULB"
            candidate_count = rulb_hits
            required_count = self.config.rulb_required_hits
            reason = (
                f"rulb_{rulb_hits}/{self.config.rulb_window_frames}_"
                f"bend_{'yes' if bend_recent else 'no'}"
            )
            if not transition_allowed:
                reason += "_motion_arming"
        elif bi_hits > 0:
            phase = "BI_VISIBLE"
            candidate = None
            candidate_count = 0
            required_count = None
            reason = f"bi_visible_{bi_hits}/{self.config.bi_window_frames}_not_position"
        else:
            phase = "RMB_GATE"
            candidate = None
            candidate_count = 0
            required_count = None
            reason = "waiting_branch_evidence"

        status = self._make_status(
            phase, forward_delta, rmb_absent, bend_recent, reason
        )
        safe_state = (
            "CANDIDATE"
            if candidate is not None
            else "CONFIRMED"
            if "RMB" in branch_confidences
            else "UNCERTAIN"
        )
        return GateOutcome(
            handled=True,
            observed_position=observed_position,
            observed_confidence=observed_confidence,
            candidate_position=candidate,
            candidate_count=candidate_count,
            required_count=required_count,
            safe_state=safe_state,
            reason=reason,
            status=status,
        )
