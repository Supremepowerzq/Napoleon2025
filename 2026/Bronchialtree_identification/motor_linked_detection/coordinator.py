"""把视频检测、最新电机快照和状态机组合成一个轻量入口。"""

from __future__ import annotations

import time
from dataclasses import replace
from typing import Iterable, Optional, Tuple

from .config import StateMachineConfig
from .detection_bus import DETECTION_STATE_BUS
from .motor_bus import MOTOR_FEEDBACK_BUS
from .safe_state_machine import SafeAnatomyStateMachine
from .types import DetectionState


class BronchusDetectionCoordinator:
    def __init__(self, config: Optional[StateMachineConfig] = None) -> None:
        self.machine = SafeAnatomyStateMachine(config)

    def reset(self) -> None:
        self.machine.reset()
        DETECTION_STATE_BUS.publish(DetectionState(reason="reset"))

    def update(
        self,
        raw_detections: Iterable[Tuple[object, object]],
    ) -> DetectionState:
        motor = MOTOR_FEEDBACK_BUS.snapshot()
        if (
            motor.feedback_valid
            and (
                motor.updated_at <= 0.0
                or time.monotonic() - motor.updated_at
                > self.machine.config.feedback_timeout_seconds
            )
        ):
            motor = replace(
                motor,
                motion_mode="idle",
                feedback_valid=False,
                reason="motor_snapshot_stale",
            )
        result = self.machine.update(raw_detections, motor)
        DETECTION_STATE_BUS.publish(result)
        return result


def get_latest_detection() -> DetectionState:
    return DETECTION_STATE_BUS.snapshot()
