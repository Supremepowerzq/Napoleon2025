"""电机前后反馈约束的三级支气管解剖状态机。"""

from __future__ import annotations

import time
from typing import Dict, Iterable, Optional, Tuple

from .config import FORWARD_TREE, PARENT_MAP, StateMachineConfig, normalize_label
from .rmb_branch_gate import RmbBranchGate
from .types import DetectionState, MotorSnapshot


class SafeAnatomyStateMachine:
    def __init__(self, config: Optional[StateMachineConfig] = None) -> None:
        self.config = config or StateMachineConfig()
        self.branch_gate = RmbBranchGate(self.config)
        self.reset()

    def reset(self) -> None:
        self.confirmed_position = "TR"
        self.candidate_position: Optional[str] = None
        self.candidate_count = 0
        self.candidate_confidence: Optional[float] = None
        self.safe_state = "CONFIRMED"
        self.reason = "reset"
        self._last_motion_mode = "idle"
        self._motion_active_frames = 0
        self.branch_gate.reset("state_machine_reset")

    def _update_motion_arming(self, motion_mode: str) -> bool:
        if motion_mode in {"forward", "backward"}:
            if motion_mode == self._last_motion_mode:
                self._motion_active_frames += 1
            else:
                self._motion_active_frames = 1
                self.candidate_position = None
                self.candidate_count = 0
                self.candidate_confidence = None
        else:
            self._motion_active_frames = 0
            self.candidate_position = None
            self.candidate_count = 0
            self.candidate_confidence = None
        self._last_motion_mode = motion_mode
        return (
            motion_mode in {"forward", "backward"}
            and self._motion_active_frames >= self.config.motion_resume_frames
        )

    def _normalize(self, raw_detections: Iterable[Tuple[object, object]]) -> Dict[str, float]:
        result: Dict[str, float] = {}
        for item in raw_detections or []:
            if not item or len(item) < 2:
                continue
            label = normalize_label(item[0])
            if label is None:
                continue
            try:
                confidence = float(item[1])
            except (TypeError, ValueError):
                continue
            if confidence < self.config.confidence_threshold:
                continue
            result[label] = max(result.get(label, 0.0), confidence)
        return result

    @staticmethod
    def _allowed_targets(current: str, motion_mode: str) -> set[str]:
        allowed = {current}
        if motion_mode == "forward":
            allowed.update(FORWARD_TREE.get(current, ()))
        elif motion_mode == "backward":
            parent = PARENT_MAP.get(current)
            if parent:
                allowed.add(parent)
        return allowed

    def _required_frames(self, current: str, target: str, motion_mode: str) -> int:
        if current == target:
            return 1
        if motion_mode == "forward":
            return self.config.forward_confirm_frames.get(
                (current, target), self.config.default_confirm_frames
            )
        if motion_mode == "backward":
            return self.config.backward_confirm_frames.get(
                (current, target), self.config.default_confirm_frames
            )
        return self.config.default_confirm_frames

    def _state(
        self,
        motor: MotorSnapshot,
        observed_position: Optional[str] = None,
        observed_confidence: Optional[float] = None,
        confirmed_changed: bool = False,
        required_count: Optional[int] = None,
    ) -> DetectionState:
        return DetectionState(
            observed_position=observed_position,
            observed_confidence=observed_confidence,
            candidate_position=self.candidate_position,
            candidate_count=self.candidate_count,
            required_count=required_count,
            confirmed_position=self.confirmed_position,
            confirmed_changed=confirmed_changed,
            safe_state=self.safe_state,
            reason=self.reason,
            motion_mode=motor.motion_mode if motor.feedback_valid else "idle",
            gate=self.branch_gate.status,
            updated_at=time.monotonic(),
        )

    def update(
        self,
        raw_detections: Iterable[Tuple[object, object]],
        motor: MotorSnapshot,
    ) -> DetectionState:
        confidences = self._normalize(raw_detections)
        effective_motor = motor
        if not motor.feedback_valid:
            effective_motor = MotorSnapshot(
                motion_mode="idle",
                m0_position=motor.m0_position,
                feedback_valid=False,
                reason="feedback_invalid",
            )

        motion_armed = self._update_motion_arming(effective_motor.motion_mode)

        gate = self.branch_gate.update(
            self.confirmed_position,
            confidences,
            effective_motor,
            transition_allowed=motion_armed,
        )
        if gate.handled:
            self.candidate_position = gate.candidate_position
            self.candidate_count = gate.candidate_count
            self.candidate_confidence = gate.observed_confidence
            self.safe_state = gate.safe_state
            self.reason = gate.reason
            changed = False
            if gate.commit_position is not None:
                self.confirmed_position = gate.commit_position
                self.candidate_position = None
                self.candidate_count = 0
                self.safe_state = "CONFIRMED"
                changed = True
            return self._state(
                effective_motor,
                gate.observed_position,
                gate.observed_confidence,
                changed,
                gate.required_count,
            )

        if effective_motor.motion_mode == "idle":
            observed_confidence = confidences.get(self.confirmed_position)
            observed_position = (
                self.confirmed_position if observed_confidence is not None else None
            )
            self.candidate_position = None
            self.candidate_count = 0
            self.candidate_confidence = None
            self.safe_state = (
                "CONFIRMED" if observed_position is not None else "UNCERTAIN"
            )
            self.reason = "motion_idle_frozen"
            return self._state(
                effective_motor,
                observed_position,
                observed_confidence,
                False,
            )

        motion_mode = effective_motor.motion_mode
        allowed = self._allowed_targets(self.confirmed_position, motion_mode)
        legal = [
            (label, confidence)
            for label, confidence in confidences.items()
            if label in allowed
        ]
        legal.sort(key=lambda item: item[1], reverse=True)

        if not legal:
            self.candidate_position = None
            self.candidate_count = 0
            self.candidate_confidence = None
            self.safe_state = "UNCERTAIN"
            self.reason = (
                "no_detection"
                if not confidences
                else f"illegal_transition_in_{motion_mode}"
            )
            return self._state(effective_motor)

        observed_position, observed_confidence = legal[0]
        if observed_position == self.confirmed_position:
            self.candidate_position = None
            self.candidate_count = 0
            self.candidate_confidence = observed_confidence
            self.safe_state = "CONFIRMED"
            self.reason = "same_as_confirmed"
            return self._state(
                effective_motor, observed_position, observed_confidence, False, 1
            )

        if observed_position == self.candidate_position:
            self.candidate_count += 1
        else:
            self.candidate_position = observed_position
            self.candidate_count = 1
        self.candidate_confidence = observed_confidence
        required = self._required_frames(
            self.confirmed_position, observed_position, motion_mode
        )

        if self.candidate_count >= required and motion_armed:
            previous = self.confirmed_position
            self.confirmed_position = observed_position
            self.candidate_position = None
            self.candidate_count = 0
            self.safe_state = "CONFIRMED"
            self.reason = f"confirmed_{previous}_to_{observed_position}"
            if self.confirmed_position == "RMB":
                self.branch_gate.start(effective_motor.m0_position)
            return self._state(
                effective_motor,
                observed_position,
                observed_confidence,
                True,
                required,
            )

        self.safe_state = "CANDIDATE"
        if not motion_armed:
            self.reason = (
                f"motion_arming_{self._motion_active_frames}/"
                f"{self.config.motion_resume_frames}"
            )
        else:
            self.reason = f"candidate_{self.candidate_count}/{required}"
        return self._state(
            effective_motor,
            observed_position,
            observed_confidence,
            False,
            required,
        )
