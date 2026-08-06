"""向 Napoleon UI 或其他消费者发布最新检测状态。"""

from __future__ import annotations

import threading
from dataclasses import replace

from .types import DetectionState


class DetectionStateBus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = DetectionState()

    def publish(self, state: DetectionState) -> None:
        with self._lock:
            self._state = state

    def snapshot(self) -> DetectionState:
        with self._lock:
            return replace(self._state)


DETECTION_STATE_BUS = DetectionStateBus()
