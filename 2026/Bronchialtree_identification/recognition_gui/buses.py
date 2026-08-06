"""Thread-safe state shared by the camera, Qt UI, and robot runtime."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class VideoStatus:
    camera_connected: bool = False
    model_loaded: bool = False
    detection_enabled: bool = False
    recording: bool = False
    recording_path: Optional[str] = None
    recording_started_at: float = 0.0
    fps: float = 0.0
    message: str = "等待视频线程"


class LatestFrameBus:
    """Keep only the newest frame so the GUI can never build up latency."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sequence = 0
        self._frame: Optional[np.ndarray] = None

    def publish(self, frame: np.ndarray) -> None:
        with self._lock:
            self._frame = np.ascontiguousarray(frame).copy()
            self._sequence += 1

    def snapshot(self) -> tuple[int, Optional[np.ndarray]]:
        with self._lock:
            return self._sequence, self._frame


class RecognitionControlBus:
    """Commands consumed by the video worker; it never controls the robot."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._detection_enabled = False
        self._recording_path: Optional[Path] = None
        self._shutdown = False

    def set_detection_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._detection_enabled = bool(enabled)

    def detection_enabled(self) -> bool:
        with self._lock:
            return self._detection_enabled

    def start_recording(self, path: Path) -> None:
        with self._lock:
            self._recording_path = Path(path)

    def stop_recording(self) -> None:
        with self._lock:
            self._recording_path = None

    def recording_path(self) -> Optional[Path]:
        with self._lock:
            return self._recording_path

    def request_shutdown(self) -> None:
        with self._lock:
            self._shutdown = True

    def shutdown_requested(self) -> bool:
        with self._lock:
            return self._shutdown


class VideoStatusBus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status = VideoStatus()

    def update(self, **changes: object) -> VideoStatus:
        with self._lock:
            self._status = replace(self._status, **changes)
            return self._status

    def snapshot(self) -> VideoStatus:
        with self._lock:
            return replace(self._status)


class GuiMessageBus:
    """A small status stream; the main window deliberately has no long log panel."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sequence = 0
        self._message = "系统初始化中"
        self._level = "info"
        self._updated_at = time.monotonic()

    def publish(self, message: str, level: str = "info") -> None:
        compact = " ".join(str(message).strip().splitlines())
        with self._lock:
            self._sequence += 1
            self._message = compact
            self._level = level
            self._updated_at = time.monotonic()

    def snapshot(self) -> tuple[int, str, str, float]:
        with self._lock:
            return self._sequence, self._message, self._level, self._updated_at


FRAME_BUS = LatestFrameBus()
CONTROL_BUS = RecognitionControlBus()
VIDEO_STATUS_BUS = VideoStatusBus()
GUI_MESSAGE_BUS = GuiMessageBus()
