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
    "TR": "气管固定原点",
    "RMB": "右主支气管",
    "RUL": "右上叶",
    "BI": "中间支气管",
    "RML": "右中叶",
    "RLL": "右下叶",
    "LMB": "左主支气管",
    "LUB": "左上叶",
    "LLB": "左下叶",
}


@dataclass(frozen=True)
class RobotStatus:
    ready: bool = False
    state: str = "未连接"
    controller_connected: bool = False
    error: Optional[str] = None


class _UiStatusAdapter:
    """Only mirrors controller status; it never issues hardware commands."""

    bc_path_var = None
    nav_target_var = None
    action_listbox = None

    def push_log(self, text: str) -> None:
        GUI_MESSAGE_BUS.publish(text)

    def update_state(self, current: str, next_state: Optional[str] = None) -> None:
        message = str(current)
        if next_state:
            message += f"｜{next_state}"
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
            GUI_MESSAGE_BUS.publish("机器人控制器已就绪")
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
                robot.request_shutdown("PyQt界面请求安全退出")
            robot.join()
        except Exception as exc:
            message = f"机器人控制器启动失败：{type(exc).__name__}: {exc}"
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
                    self._status = RobotStatus(state="已停止")

    def _load_controller(self) -> ModuleType:
        module_name = "_napoleon_autonav_recognition_runtime"
        spec = importlib.util.spec_from_file_location(module_name, ROBOT_MAIN)
        if spec is None or spec.loader is None:
            raise ImportError(f"无法加载机器人主程序：{ROBOT_MAIN}")
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
                GUI_MESSAGE_BUS.publish("手柄使用 Windows XInput 轮询")
                return controller
            except Exception as exc:
                GUI_MESSAGE_BUS.publish(f"XInput不可用，回退原手柄接口：{exc}", "warning")
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
            GUI_MESSAGE_BUS.publish("机器人尚未就绪，命令未执行", "warning")
            return False
        robot.ui_command_queue.put((command, f"PyQt按钮-{command}"))
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
            GUI_MESSAGE_BUS.publish("机器人尚未就绪，无法开始导航", "warning")
            return False
        robot._on_nav_ui_start(target_text, enable_layer2, enable_layer3)  # noqa: SLF001
        return True

    def start_inspection(self) -> bool:
        """Start with the newest single RMB collection, never an averaged path."""
        for _label, target in self.navigation_targets():
            if target.startswith("RMB - ") and "|" in target:
                GUI_MESSAGE_BUS.publish("自主巡检使用最近一次RMB现场采集路径")
                return self.start_navigation(
                    target,
                    enable_layer2=False,
                    enable_layer3=False,
                )
        GUI_MESSAGE_BUS.publish("未找到RMB现场采集路径，无法开始巡检", "warning")
        return False

    def pause_navigation(self) -> bool:
        return self._call_robot("_on_nav_ui_pause")

    def stop_navigation(self) -> bool:
        return self._call_robot("_on_nav_ui_stop")

    def _call_robot(self, method_name: str) -> bool:
        with self._lock:
            robot = self._robot
        if robot is None:
            GUI_MESSAGE_BUS.publish("机器人尚未就绪", "warning")
            return False
        getattr(robot, method_name)()
        return True

    def request_shutdown(self) -> None:
        self._stop_requested.set()
        with self._lock:
            robot = self._robot
        if robot is not None:
            robot.request_shutdown("PyQt界面请求安全退出")

    @staticmethod
    def navigation_targets() -> list[tuple[str, str]]:
        """Return display text and a target string accepted by the old callback."""
        targets: list[tuple[str, str]] = [
            ("返回气管固定原点（TR）", "TR - 气管固定原点"),
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
            label = f"{name}｜{path.stem}"
            target = f"{code} - {name} | {path.name}"
            targets.append((label, target))
        return targets
