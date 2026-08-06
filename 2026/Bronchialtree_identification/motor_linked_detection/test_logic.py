"""无需摄像头/电机的纯逻辑回归测试。"""

from __future__ import annotations

import threading
import time
import unittest

from .coordinator import BronchusDetectionCoordinator
from .motor_bus import MOTOR_FEEDBACK_BUS
from .napoleon_adapter import NapoleonMotorFeedbackAdapter
from .safe_state_machine import SafeAnatomyStateMachine
from .types import MotorSnapshot


def motor(
    position: float,
    mode: str = "forward",
    bend: float = 0.0,
    m2_position: float | None = None,
) -> MotorSnapshot:
    return MotorSnapshot(
        motion_mode=mode,
        m0_position=position,
        m2_position=m2_position,
        branch_bend_signal=bend,
        feedback_valid=True,
    )


class StateMachineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.machine = SafeAnatomyStateMachine()

    def enter_rmb(self) -> None:
        for _ in range(4):
            result = self.machine.update([("RMB", 0.9)], motor(0.0))
        self.assertEqual(result.confirmed_position, "RMB")

    def test_forward_and_backward_use_motor_direction(self) -> None:
        self.enter_rmb()
        for _ in range(3):
            result = self.machine.update([("TR", 0.9)], motor(0.0, "backward"))
        self.assertEqual(result.confirmed_position, "TR")


class _FakeMotor:
    def __init__(self, speed: float = 0.0) -> None:
        self.speed = speed


class _FakeMotors:
    def __init__(
        self,
        m0_speed: float,
        m2_speed: float = 0.0,
        m2_position: float = 0.0,
    ) -> None:
        self.m0 = _FakeMotor(m0_speed)
        self.m2 = _FakeMotor(m2_speed)
        self.m2_position = m2_position

    def get_cached_angles(self) -> tuple[float, float, float]:
        return (-25.0, 0.0, self.m2_position)


class _FakeXbox:
    def __init__(self, ly: float) -> None:
        self.ly = ly

    def get_joystick_value(self, name: str) -> float:
        if name != "LY":
            raise KeyError(name)
        return self.ly


class _FakeRobot:
    def __init__(
        self,
        m0_speed: float,
        ly: float,
        command: float,
        m2_position: float = 0.0,
    ) -> None:
        self.state = "ManualControl"
        self.motors = _FakeMotors(m0_speed, m2_position=m2_position)
        self.xbox = _FakeXbox(ly)
        self.manual_input_lock = threading.Lock()
        self.ui_manual_input = {
            "forward": command,
            "horizontal": 0.0,
            "vertical": 0.0,
        }
        self.controller_manual_input = {
            "forward": 0.0,
            "horizontal": 0.0,
            "vertical": 0.0,
        }


class NapoleonAdapterTests(unittest.TestCase):
    def test_reads_cached_feedback_without_motor_control_calls(self) -> None:
        adapter = NapoleonMotorFeedbackAdapter()
        snapshot = adapter.publish_robot(_FakeRobot(-80.0, -0.7, 80.0))
        self.assertTrue(snapshot.feedback_valid)
        self.assertEqual(snapshot.motion_mode, "forward")
        self.assertEqual(snapshot.m0_position, -25.0)
        self.assertAlmostEqual(snapshot.branch_bend_signal, 0.7)

    def test_bend_direction_is_ignored(self) -> None:
        adapter = NapoleonMotorFeedbackAdapter()
        upward = adapter.publish_robot(_FakeRobot(-80.0, 0.6, 80.0))
        downward = adapter.publish_robot(_FakeRobot(-80.0, -0.6, 80.0))
        self.assertAlmostEqual(upward.branch_bend_signal, 0.6)
        self.assertAlmostEqual(downward.branch_bend_signal, 0.6)

    def test_held_m2_angle_remains_bent_after_joystick_centers(self) -> None:
        adapter = NapoleonMotorFeedbackAdapter()
        snapshot = adapter.publish_robot(
            _FakeRobot(
                m0_speed=0.0,
                ly=0.0,
                command=0.0,
                m2_position=80.0,
            )
        )
        self.assertEqual(snapshot.motion_mode, "idle")
        self.assertEqual(snapshot.m2_position, 80.0)
        self.assertEqual(snapshot.branch_bend_signal, 1.0)

    def test_command_without_real_m0_feedback_remains_idle(self) -> None:
        adapter = NapoleonMotorFeedbackAdapter()
        snapshot = adapter.publish_robot(
            _FakeRobot(
                m0_speed=0.0,
                ly=0.0,
                command=-80.0,
                m2_position=0.0,
            )
        )
        self.assertEqual(snapshot.motion_mode, "idle")
        self.assertEqual(snapshot.reason, "manual_command_waiting_feedback")

    def test_mismatched_command_and_cached_m0_feedback_remains_idle(self) -> None:
        adapter = NapoleonMotorFeedbackAdapter()
        snapshot = adapter.publish_robot(
            _FakeRobot(
                m0_speed=-80.0,
                ly=0.0,
                command=-80.0,
                m2_position=0.0,
            )
        )
        self.assertEqual(snapshot.motion_mode, "idle")
        self.assertEqual(snapshot.reason, "manual_command_feedback_mismatch")

    def test_rotation_command_does_not_count_as_bending(self) -> None:
        adapter = NapoleonMotorFeedbackAdapter()
        robot = _FakeRobot(
            m0_speed=0.0,
            ly=0.0,
            command=0.0,
            m2_position=0.0,
        )
        robot.controller_manual_input["horizontal"] = 50.0
        snapshot = adapter.publish_robot(robot)
        self.assertEqual(snapshot.branch_bend_signal, 0.0)

    def test_stale_feedback_cannot_advance_detection(self) -> None:
        MOTOR_FEEDBACK_BUS.publish(
            MotorSnapshot(
                motion_mode="forward",
                m0_position=-50.0,
                feedback_valid=True,
                updated_at=time.monotonic() - 2.0,
            )
        )
        coordinator = BronchusDetectionCoordinator()
        for _ in range(6):
            state = coordinator.update([("RMB", 0.99)])
        self.assertEqual(state.motion_mode, "idle")
        self.assertEqual(state.confirmed_position, "TR")


class StateMachineBranchGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.machine = SafeAnatomyStateMachine()

    def enter_rmb(self) -> None:
        for _ in range(4):
            result = self.machine.update([("RMB", 0.9)], motor(0.0))
        self.assertEqual(result.confirmed_position, "RMB")

    def test_bi_remains_soft_while_rmb_visible(self) -> None:
        self.enter_rmb()
        for _ in range(8):
            result = self.machine.update(
                [("RMB", 0.7), ("BI", 0.95)], motor(-60.0)
            )
        self.assertEqual(result.confirmed_position, "RMB")
        self.assertEqual(result.gate.phase, "BI_SOFT")

    def test_bi_commits_after_rmb_disappears_and_progress(self) -> None:
        self.enter_rmb()
        for _ in range(6):
            result = self.machine.update([("BI", 0.9)], motor(-10.0))
        self.assertEqual(result.gate.phase, "BI_SOFT")
        for _ in range(2):
            result = self.machine.update([("BI", 0.9)], motor(-60.0))
        self.assertEqual(result.confirmed_position, "BI")

    def test_idle_does_not_accumulate_bi_or_rulb_evidence(self) -> None:
        self.enter_rmb()
        for _ in range(12):
            result = self.machine.update(
                [("BI", 0.95), ("RULB", 0.90)],
                motor(-60.0, mode="idle", bend=0.8),
            )
        self.assertEqual(result.confirmed_position, "RMB")
        self.assertEqual(result.gate.phase, "RMB_PAUSED")
        self.assertEqual(result.gate.bi_hits, 0)
        self.assertEqual(result.gate.rulb_hits, 0)

    def test_resume_requires_two_active_detection_cycles(self) -> None:
        self.enter_rmb()
        for _ in range(6):
            result = self.machine.update(
                [("RMB", 0.75), ("BI", 0.95)], motor(-60.0)
            )
        self.assertEqual(result.gate.phase, "BI_SOFT")

        for _ in range(8):
            result = self.machine.update([("BI", 0.95)], motor(-60.0, "idle"))
        self.assertEqual(result.confirmed_position, "RMB")

        first_active = self.machine.update([("BI", 0.95)], motor(-60.0))
        self.assertEqual(first_active.confirmed_position, "RMB")
        self.assertFalse(first_active.confirmed_changed)
        self.assertIn("motion_arming", first_active.reason)

        second_active = self.machine.update([("BI", 0.95)], motor(-60.0))
        self.assertEqual(second_active.confirmed_position, "BI")
        self.assertTrue(second_active.confirmed_changed)

    def test_rulb_accepts_sparse_hits_with_either_bend_direction(self) -> None:
        self.enter_rmb()
        sequence = [[("RUL", 0.82)], [], [("RULB", 0.85)], [], [("RUL", 0.88)]]
        for detections in sequence:
            result = self.machine.update(detections, motor(-12.0, bend=0.6))
        self.assertEqual(result.confirmed_position, "RULB")

    def test_rulb_candidate_survives_misses_and_confirms_after_stop(self) -> None:
        self.enter_rmb()
        moving = self.machine.update(
            [("RULB", 0.82)],
            motor(-12.0, bend=0.6),
        )
        self.assertEqual(moving.confirmed_position, "RMB")
        self.assertEqual(moving.candidate_position, "RULB")
        self.assertEqual(moving.candidate_count, 1)

        for _ in range(8):
            missed_idle = self.machine.update(
                [],
                motor(-12.0, mode="idle"),
            )
            self.assertEqual(missed_idle.confirmed_position, "RMB")
            self.assertEqual(missed_idle.gate.phase, "RULB_STOP_CONFIRM")
            self.assertEqual(missed_idle.candidate_count, 1)

        confirmed_idle = self.machine.update(
            [("RULB", 0.88)],
            motor(-12.0, mode="idle"),
        )
        self.assertEqual(confirmed_idle.confirmed_position, "RULB")
        self.assertTrue(confirmed_idle.confirmed_changed)
        self.assertEqual(confirmed_idle.gate.phase, "RULB_CONFIRMED")

    def test_rulb_idle_confirmation_still_requires_prior_bend(self) -> None:
        self.enter_rmb()
        for _ in range(2):
            moving = self.machine.update(
                [("RULB", 0.90)],
                motor(-12.0, bend=0.0),
            )
        self.assertEqual(moving.candidate_position, "RULB")

        for _ in range(4):
            stopped = self.machine.update(
                [("RULB", 0.95)],
                motor(-12.0, mode="idle"),
            )
        self.assertEqual(stopped.confirmed_position, "RMB")
        self.assertEqual(stopped.gate.phase, "RMB_PAUSED")

    def test_held_m2_bend_confirms_existing_rulb_candidate_on_stop(self) -> None:
        self.enter_rmb()
        moving = self.machine.update(
            [("RULB", 0.82)],
            motor(-12.0, m2_position=80.0),
        )
        self.assertEqual(moving.confirmed_position, "RMB")
        self.assertEqual(moving.candidate_position, "RULB")

        stopped = self.machine.update(
            [],
            motor(-12.0, mode="idle", m2_position=80.0),
        )
        self.assertEqual(stopped.confirmed_position, "RULB")
        self.assertTrue(stopped.confirmed_changed)
        self.assertEqual(stopped.gate.phase, "RULB_CONFIRMED")

    def test_strong_m2_bend_confirms_single_rulb_hit_while_forward(self) -> None:
        self.enter_rmb()
        result = self.machine.update(
            [("RULB", 0.82)],
            motor(-12.0, m2_position=500.0),
        )
        self.assertEqual(result.confirmed_position, "RULB")
        self.assertTrue(result.confirmed_changed)
        self.assertEqual(result.gate.phase, "RULB_CONFIRMED")
        self.assertIn("strong_m2_bend", result.reason)

    def test_reverse_start_commits_existing_strong_bend_rulb_candidate(self) -> None:
        self.enter_rmb()
        candidate = self.machine.update(
            [("RULB", 0.82)],
            motor(-12.0, bend=0.6, m2_position=4.0),
        )
        self.assertEqual(candidate.confirmed_position, "RMB")
        self.assertEqual(candidate.candidate_position, "RULB")

        reverse_start = self.machine.update(
            [],
            motor(-12.0, mode="backward", m2_position=500.0),
        )
        self.assertEqual(reverse_start.confirmed_position, "RULB")
        self.assertTrue(reverse_start.confirmed_changed)
        self.assertIn("confirmed_on_reverse_start", reverse_start.reason)

        for _ in range(3):
            reverse_result = self.machine.update(
                [("RMB", 0.90)],
                motor(-8.0, mode="backward", m2_position=500.0),
            )
        self.assertEqual(reverse_result.confirmed_position, "RMB")
        self.assertNotEqual(reverse_result.confirmed_position, "TR")

    def test_bend_latch_survives_centered_stick_until_m2_returns_straight(self) -> None:
        self.enter_rmb()
        self.machine.update(
            [],
            motor(-4.0, bend=0.6, m2_position=4.0),
        )
        for position in (-6.0, -8.0):
            self.machine.update(
                [],
                motor(position, bend=0.0, m2_position=4.0),
            )

        moving = self.machine.update(
            [("RULB", 0.82)],
            motor(-12.0, bend=0.0, m2_position=4.0),
        )
        self.assertEqual(moving.candidate_position, "RULB")

        stopped = self.machine.update(
            [],
            motor(-12.0, mode="idle", bend=0.0, m2_position=4.0),
        )
        self.assertEqual(stopped.confirmed_position, "RULB")
        self.assertTrue(stopped.confirmed_changed)

    def test_bend_does_not_change_other_subtrees(self) -> None:
        idle_bending = MotorSnapshot(
            motion_mode="idle",
            m0_position=0.0,
            branch_bend_signal=1.0,
            feedback_valid=True,
        )
        for _ in range(5):
            result = self.machine.update([("LMB", 0.99)], idle_bending)
        self.assertEqual(result.confirmed_position, "TR")


if __name__ == "__main__":
    unittest.main()
