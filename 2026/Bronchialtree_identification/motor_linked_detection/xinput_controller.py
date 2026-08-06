"""轻量版专用 Windows XInput 手柄轮询，不修改原 JoystickInterfaceV2。"""

from __future__ import annotations

import ctypes
import threading
from ctypes import wintypes
from typing import Optional


ERROR_SUCCESS = 0


class _XInputGamepad(ctypes.Structure):
    _fields_ = (
        ("buttons", wintypes.WORD),
        ("left_trigger", wintypes.BYTE),
        ("right_trigger", wintypes.BYTE),
        ("left_x", wintypes.SHORT),
        ("left_y", wintypes.SHORT),
        ("right_x", wintypes.SHORT),
        ("right_y", wintypes.SHORT),
    )


class _XInputState(ctypes.Structure):
    _fields_ = (
        ("packet_number", wintypes.DWORD),
        ("gamepad", _XInputGamepad),
    )


_BUTTON_MASKS = {
    "A": 0x1000,
    "B": 0x2000,
    "X": 0x4000,
    "Y": 0x8000,
    "LB": 0x0100,
    "RB": 0x0200,
    "BACK": 0x0020,
    "START": 0x0010,
    "TL": 0x0040,
    "TR": 0x0080,
}


def _load_xinput():
    last_error: Optional[Exception] = None
    for library_name in ("xinput1_4.dll", "xinput9_1_0.dll", "xinput1_3.dll"):
        try:
            return ctypes.WinDLL(library_name)
        except OSError as exc:
            last_error = exc
    raise OSError(f"无法加载 Windows XInput: {last_error}")


class XInputController:
    """以固定频率直接轮询 Xbox 状态，避开事件轴不更新问题。"""

    def __init__(self, controller_index: int = 0, poll_hz: float = 120.0) -> None:
        self.controller_index = int(controller_index)
        self.poll_interval = 1.0 / max(20.0, float(poll_hz))
        self._dll = _load_xinput()
        self._get_state = self._dll.XInputGetState
        self._get_state.argtypes = (wintypes.DWORD, ctypes.POINTER(_XInputState))
        self._get_state.restype = wintypes.DWORD
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._pressed_latches: set[str] = set()
        self._buttons = 0
        self._connected = False
        self._lt = self._rt = 0.0
        self._lx = self._ly = self._rx = self._ry = 0.0

        first = self._query()
        if first is None:
            raise ValueError(f"XInput 控制器 {self.controller_index} 未连接")
        self._apply_state(first, latch_buttons=False)
        self.thread = threading.Thread(
            target=self._poll_loop,
            name="light_xinput_poll",
            daemon=True,
        )
        self.thread.start()

    @staticmethod
    def _normalize_axis(raw: int) -> float:
        denominator = 32767.0 if raw >= 0 else 32768.0
        return max(-1.0, min(1.0, float(raw) / denominator))

    def _query(self) -> Optional[_XInputState]:
        state = _XInputState()
        result = int(self._get_state(self.controller_index, ctypes.byref(state)))
        return state if result == ERROR_SUCCESS else None

    def _apply_state(self, state: _XInputState, latch_buttons: bool = True) -> None:
        gamepad = state.gamepad
        new_buttons = int(gamepad.buttons)
        with self._lock:
            if latch_buttons:
                newly_pressed = new_buttons & ~self._buttons
                for name, mask in _BUTTON_MASKS.items():
                    if newly_pressed & mask:
                        self._pressed_latches.add(name)
            self._buttons = new_buttons
            self._lt = float(gamepad.left_trigger) / 255.0
            self._rt = float(gamepad.right_trigger) / 255.0
            self._lx = self._normalize_axis(int(gamepad.left_x))
            self._ly = self._normalize_axis(int(gamepad.left_y))
            self._rx = self._normalize_axis(int(gamepad.right_x))
            self._ry = self._normalize_axis(int(gamepad.right_y))
            self._connected = True

    def _poll_loop(self) -> None:
        while not self._shutdown.is_set():
            state = self._query()
            if state is None:
                with self._lock:
                    self._connected = False
                    self._lt = self._rt = 0.0
                    self._lx = self._ly = self._rx = self._ry = 0.0
            else:
                self._apply_state(state)
            self._shutdown.wait(self.poll_interval)

    def is_button_pressed(self, button: str) -> bool:
        with self._lock:
            if button in self._pressed_latches:
                self._pressed_latches.remove(button)
                return True
        return False

    def get_trigger_value(self, trigger: str) -> float:
        with self._lock:
            if trigger == "LT":
                return self._lt
            if trigger == "RT":
                return self._rt
        raise ValueError(f"未知扳机: {trigger}")

    def get_joystick_value(self, axis: str) -> float:
        with self._lock:
            values = {
                "LX": self._lx,
                "LY": self._ly,
                "RX": self._rx,
                "RY": self._ry,
            }
            if axis in values:
                return values[axis]
        raise ValueError(f"未知摇杆轴: {axis}")

    def snapshot(self) -> dict[str, float | bool]:
        with self._lock:
            return {
                "connected": self._connected,
                "LT": self._lt,
                "RT": self._rt,
                "LX": self._lx,
                "LY": self._ly,
                "RX": self._rx,
                "RY": self._ry,
            }

    def stop(self) -> None:
        self._shutdown.set()
        if self.thread.is_alive():
            self.thread.join(timeout=0.5)
