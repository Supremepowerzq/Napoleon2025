"""Adapter that runs the existing Napoleon controller without changing it."""

from __future__ import annotations

import importlib.util
import re
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from types import ModuleType
from typing import Any, Callable, Optional

from Bronchialtree_identification.motor_linked_detection.napoleon_adapter import (
    NapoleonMotorFeedbackAdapter,
)

from .buses import GUI_MESSAGE_BUS


RUNTIME_ROOT = Path(__file__).resolve().parents[2]
ROBOT_MAIN = RUNTIME_ROOT / "main2026-Xhandwriting-AC-AutoNav(0710)-Bronchus.py"
AUTONAV_DATASET_DIR = RUNTIME_ROOT / "BC" / "expert_demos" / "AutoNavdatasets"

DISPLAY_NAMES = {
    "TR": "Fixed Tracheal Origin",
    "RMB": "Right Main Bronchus",
    "RUL": "Right Upper Lobe",
    "BI": "Bronchus Intermedius",
    "RML": "Right Middle Lobe",
    "RLL": "Right Lower Lobe",
    "LMB": "Left Main Bronchus",
    "LUB": "Left Upper Lobe",
    "LLB": "Left Lower Lobe",
}

_HAN_RE = re.compile(r"[\u3400-\u9fff]")
_LEGACY_STATE_TRANSLATIONS = {
    "\u624b\u52a8\u63a7\u5236": "Manual Control",
    "\u89c6\u89c9\u81ea\u4e3b\u63a7\u5236": "Vision Control",
    "\u7a7a\u95f2\u4fdd\u6301": "Idle",
    "\u6267\u884c\u89d2\u5ea6\u5f52\u96f6": "Returning to Zero",
    "\u5199\u5165\u5f53\u524d\u4f4d\u7f6e\u4e3a\u96f6\u70b9": "Setting Current Position as Zero",
    "\u65ad\u7535\u7ef4\u62a4": "Powered-Off Maintenance",
    "AI\u81ea\u4e3b\u5de1\u68c0": "Autonomous Inspection",
}
_LEGACY_MESSAGE_CATEGORIES = (
    ("\u81ea\u4e3b\u5de1\u68c0", "Autonomous inspection status updated"),
    ("\u5bfc\u822a", "Navigation status updated"),
    ("\u91c7\u96c6", "Path collection status updated"),
    ("\u8fd4\u56de\u539f\u70b9", "Return-to-origin status updated"),
)


def _english_runtime_text(text: object) -> str:
    """Keep legacy-controller messages from leaking Chinese into this UI."""
    message = str(text).replace("\uff5c", " | ")
    if not _HAN_RE.search(message):
        return message
    if message in _LEGACY_STATE_TRANSLATIONS:
        return _LEGACY_STATE_TRANSLATIONS[message]
    for marker, replacement in _LEGACY_MESSAGE_CATEGORIES:
        if marker in message:
            return replacement
    return "Robot controller status updated"


@dataclass(frozen=True)
class RobotStatus:
    ready: bool = False
    state: str = "Disconnected"
    controller_connected: bool = False
    error: Optional[str] = None


class _UiStatusAdapter:
    """Only mirrors controller status; it never issues hardware commands."""

    bc_path_var = None
    nav_target_var = None
    action_listbox = None

    def push_log(self, text: str) -> None:
        GUI_MESSAGE_BUS.publish(_english_runtime_text(text))

    def update_state(self, current: str, next_state: Optional[str] = None) -> None:
        message = _english_runtime_text(current)
        if next_state:
            message += f" | {_english_runtime_text(next_state)}"
        GUI_MESSAGE_BUS.publish(message)

    def __getattr__(self, name: str) -> Callable[..., None]:
        # The legacy UI exposes several progress/collection methods. The new
        # compact console intentionally has no long action panel, so these are
        # safe status-only no-ops.
        if name.startswith(("update_", "set_", "refresh_")):
            return lambda *_args, **_kwargs: None
        raise AttributeError(name)


class RobotRuntime(threading.Thread):
    def __init__(self) -> None:
        super().__init__(name="napoleon_robot_runtime", daemon=False)
        self._lock = threading.Lock()
        self._robot: Optional[Any] = None
        self._status = RobotStatus()
        self._stop_requested = threading.Event()

    def run(self) -> None:
        feedback = NapoleonMotorFeedbackAdapter()
        robot = None
        try:
            module = self._load_controller()
            if self._stop_requested.is_set():
                return
            robot = module.VisionRobotStateMachine()
            robot.voice_window = _UiStatusAdapter()
            with self._lock:
                self._robot = robot
                self._status = RobotStatus(
                    ready=True,
                    state=str(robot.state),
                    controller_connected=self._controller_connected(robot),
                )
            GUI_MESSAGE_BUS.publish("Robot controller ready")
            robot.start()

            while robot.state_thread.is_alive() and not self._stop_requested.is_set():
                feedback.publish_robot(robot)
                with self._lock:
                    self._status = replace(
                        self._status,
                        state=str(robot.state),
                        controller_connected=self._controller_connected(robot),
                    )
                time.sleep(1.0 / 60.0)

            if self._stop_requested.is_set() and robot.state_thread.is_alive():
                robot.request_shutdown("Safe exit requested by the PyQt interface")
            robot.join()
        except Exception as exc:
            message = f"Robot controller failed to start: {type(exc).__name__}"
            with self._lock:
                self._status = RobotStatus(
                    controller_connected=self._status.controller_connected,
                    error=message,
                )
            GUI_MESSAGE_BUS.publish(message, "error")
        finally:
            if robot is not None:
                controller = getattr(robot, "xbox", None)
                stop = getattr(controller, "stop", None)
                if callable(stop):
                    try:
                        stop()
                    except Exception:
                        pass
            feedback.close()
            with self._lock:
                self._robot = None
                if self._status.error is None:
                    self._status = RobotStatus(state="Stopped")

    def _load_controller(self) -> ModuleType:
        module_name = "_napoleon_autonav_recognition_runtime"
        spec = importlib.util.spec_from_file_location(module_name, ROBOT_MAIN)
        if spec is None or spec.loader is None:
            raise ImportError(f"Unable to load robot controller: {ROBOT_MAIN}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)

        # Qt owns the only GUI. Camera and voice are supplied by this compact
        # runtime, while all robot, motor, controller and AutoNav code remains
        # the original implementation.
        module.VOICE_SWITCH = False
        module.CAMERA_SWITCH = False
        original_controller = module.XboxController

        def controller_factory():
            try:
                from Bronchialtree_identification.motor_linked_detection.xinput_controller import (
                    XInputController,
                )

                controller = XInputController()
                with self._lock:
                    self._status = replace(
                        self._status,
                        controller_connected=True,
                    )
                GUI_MESSAGE_BUS.publish("Controller connected through Windows XInput")
                return controller
            except Exception as exc:
                GUI_MESSAGE_BUS.publish(
                    f"XInput unavailable; using the legacy controller interface ({type(exc).__name__})",
                    "warning",
                )
                controller = original_controller()
                with self._lock:
                    self._status = replace(
                        self._status,
                        controller_connected=True,
                    )
                return controller

        module.XboxController = controller_factory
        return module

    @staticmethod
    def _controller_connected(robot: Any) -> bool:
        controller = getattr(robot, "xbox", None)
        if controller is None:
            return False
        snapshot = getattr(controller, "snapshot", None)
        if callable(snapshot):
            try:
                return bool(snapshot().get("connected", False))
            except Exception:
                return False
        return bool(getattr(robot, "controller_available", False))

    def status(self) -> RobotStatus:
        with self._lock:
            return replace(self._status)

    def submit_command(self, command: str) -> bool:
        with self._lock:
            robot = self._robot
        if robot is None:
            GUI_MESSAGE_BUS.publish("Robot is not ready; command not executed", "warning")
            return False
        robot.ui_command_queue.put((command, f"PyQt button - {command}"))
        return True

    def start_navigation(
        self,
        target_text: str,
        enable_layer2: bool = False,
        enable_layer3: bool = False,
    ) -> bool:
        with self._lock:
            robot = self._robot
        if robot is None:
            GUI_MESSAGE_BUS.publish("Robot is not ready; navigation cannot start", "warning")
            return False
        robot._on_nav_ui_start(target_text, enable_layer2, enable_layer3)  # noqa: SLF001
        return True

    def start_inspection(self) -> bool:
        """Start with the newest single RMB collection, never an averaged path."""
        for _label, target in self.navigation_targets():
            if target.startswith("RMB - ") and "|" in target:
                GUI_MESSAGE_BUS.publish("Autonomous inspection is using the latest RMB path")
                return self.start_navigation(
                    target,
                    enable_layer2=False,
                    enable_layer3=False,
                )
        GUI_MESSAGE_BUS.publish("No RMB path was found; inspection cannot start", "warning")
        return False

    def pause_navigation(self) -> bool:
        return self._call_robot("_on_nav_ui_pause")

    def stop_navigation(self) -> bool:
        return self._call_robot("_on_nav_ui_stop")

    def _call_robot(self, method_name: str) -> bool:
        with self._lock:
            robot = self._robot
        if robot is None:
            GUI_MESSAGE_BUS.publish("Robot is not ready", "warning")
            return False
        getattr(robot, method_name)()
        return True

    def request_shutdown(self) -> None:
        self._stop_requested.set()
        with self._lock:
            robot = self._robot
        if robot is not None:
            robot.request_shutdown("Safe exit requested by the PyQt interface")

    @staticmethod
    def navigation_targets() -> list[tuple[str, str]]:
        """Return display text and a target string accepted by the old callback."""
        targets: list[tuple[str, str]] = [
            ("Return to Fixed Tracheal Origin (TR)", "TR - Fixed Tracheal Origin"),
        ]
        if not AUTONAV_DATASET_DIR.is_dir():
            return targets
        pattern = re.compile(r"_(RMB|RUL|BI|RML|RLL|LMB|LUB|LLB)_", re.IGNORECASE)
        for path in sorted(AUTONAV_DATASET_DIR.glob("*.json"), key=lambda item: item.stat().st_mtime, reverse=True):
            match = pattern.search(path.name)
            if not match:
                continue
            code = match.group(1).upper()
            name = DISPLAY_NAMES.get(code, code)
            label = f"{name} | {path.stem}"
            target = f"{code} - {name} | {path.name}"
            targets.append((label, target))
        return targets
