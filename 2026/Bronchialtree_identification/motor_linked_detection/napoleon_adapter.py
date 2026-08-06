"""从 Napoleon 主状态机发布缓存遥测；本适配器绝不访问串口。"""

from __future__ import annotations

import time
from typing import Any, Optional

from .config import StateMachineConfig
from .motor_bus import MOTOR_FEEDBACK_BUS
from .types import MotorSnapshot


class NapoleonMotorFeedbackAdapter:
    """将已有电机缓存和手柄量转换为检测状态机需要的只读快照。"""

    def __init__(self, config: Optional[StateMachineConfig] = None) -> None:
        self.config = config or StateMachineConfig()
        self._closed = False

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _manual_forward_command(self, robot: Any) -> float:
        lock = getattr(robot, "manual_input_lock", None)
        if lock is None:
            return 0.0
        with lock:
            ui = self._safe_float(
                getattr(robot, "ui_manual_input", {}).get("forward", 0.0)
            )
            controller = self._safe_float(
                getattr(robot, "controller_manual_input", {}).get("forward", 0.0)
            )
        return ui if abs(ui) > abs(controller) else controller

    def _bend_intent(self, robot: Any) -> float:
        values = [0.0]
        xbox = getattr(robot, "xbox", None)
        if xbox is not None:
            try:
                values.append(abs(float(xbox.get_joystick_value("LY"))))
            except Exception:
                pass

        lock = getattr(robot, "manual_input_lock", None)
        if lock is not None:
            with lock:
                ui_vertical = abs(
                    self._safe_float(
                        getattr(robot, "ui_manual_input", {}).get("vertical", 0.0)
                    )
                )
            # UI vertical 是每周期角度增量，满量程约 8.33。
            values.append(min(1.0, ui_vertical / (50.0 * 10.0 / 60.0)))
        return max(values)

    def publish_robot(self, robot: Any) -> MotorSnapshot:
        if self._closed:
            return MOTOR_FEEDBACK_BUS.snapshot()

        motors = getattr(robot, "motors", None)
        if motors is None:
            return self.publish_invalid("robot_has_no_motors")

        try:
            angles = motors.get_cached_angles()
            m0_position = self._safe_float(angles[0])
            m2_position = self._safe_float(angles[2])
            m0_feedback = self._safe_float(getattr(motors.m0, "speed", 0.0))
            m2_feedback = self._safe_float(getattr(motors.m2, "speed", 0.0))
            logical_feedback = m0_feedback * self.config.m0_direction_sign
            robot_state = str(getattr(robot, "state", "Unknown"))

            command = 0.0
            reason = "motor_feedback"
            if robot_state in {"Idle", "PowerOff"}:
                motion_mode = "idle"
                reason = f"robot_state_{robot_state}"
            elif robot_state == "ManualControl":
                command = self._manual_forward_command(robot)
                if abs(command) <= self.config.m0_command_deadzone:
                    motion_mode = "idle"
                    reason = "manual_command_neutral"
                elif abs(logical_feedback) > self.config.m0_feedback_deadzone:
                    command_mode = "forward" if command > 0 else "backward"
                    feedback_mode = (
                        "forward" if logical_feedback > 0 else "backward"
                    )
                    if command_mode == feedback_mode:
                        motion_mode = feedback_mode
                        reason = "manual_motor_feedback"
                    else:
                        motion_mode = "idle"
                        reason = "manual_command_feedback_mismatch"
                else:
                    # 解剖状态只跟随真实 M0 运动；命令刚下发但电机尚未运动时
                    # 保持 idle，避免扳机噪声或启动延迟提前触发拓扑跳转。
                    motion_mode = "idle"
                    reason = "manual_command_waiting_feedback"
            else:
                if logical_feedback > self.config.m0_feedback_deadzone:
                    motion_mode = "forward"
                elif logical_feedback < -self.config.m0_feedback_deadzone:
                    motion_mode = "backward"
                else:
                    motion_mode = "idle"

            bend_feedback = min(
                1.0,
                abs(m2_feedback) / max(1.0, self.config.m2_feedback_scale),
            )
            held_bend = (
                abs(m2_position)
                >= self.config.branch_bend_m2_position_threshold_deg
            )
            bend_signal = max(
                self._bend_intent(robot),
                bend_feedback,
                1.0 if held_bend else 0.0,
            )
            snapshot = MotorSnapshot(
                motion_mode=motion_mode,
                m0_position=m0_position,
                m2_position=m2_position,
                m0_feedback_speed=m0_feedback,
                logical_m0_feedback_speed=logical_feedback,
                m0_command_speed=command,
                branch_bend_signal=bend_signal,
                feedback_valid=True,
                robot_state=robot_state,
                reason=reason,
                updated_at=time.monotonic(),
            )
            MOTOR_FEEDBACK_BUS.publish(snapshot)
            return snapshot
        except Exception as exc:
            return self.publish_invalid(f"adapter_error:{exc}")

    def publish_invalid(self, reason: str) -> MotorSnapshot:
        snapshot = MotorSnapshot(
            motion_mode="idle",
            feedback_valid=False,
            reason=reason,
            updated_at=time.monotonic(),
        )
        MOTOR_FEEDBACK_BUS.publish(snapshot)
        return snapshot

    def close(self) -> None:
        self.publish_invalid("adapter_closed")
        self._closed = True
