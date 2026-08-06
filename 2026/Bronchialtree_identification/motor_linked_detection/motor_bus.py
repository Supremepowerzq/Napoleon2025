"""Napoleon 控制线程与视频线程之间的无阻塞电机快照总线。"""

from __future__ import annotations

import threading
from dataclasses import replace

from .types import MotorSnapshot


class MotorFeedbackBus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot = MotorSnapshot()

    def publish(self, snapshot: MotorSnapshot) -> None:
        with self._lock:
            self._snapshot = snapshot

    def snapshot(self) -> MotorSnapshot:
        with self._lock:
            return replace(self._snapshot)


MOTOR_FEEDBACK_BUS = MotorFeedbackBus()


def get_motor_snapshot() -> MotorSnapshot:
    return MOTOR_FEEDBACK_BUS.snapshot()
