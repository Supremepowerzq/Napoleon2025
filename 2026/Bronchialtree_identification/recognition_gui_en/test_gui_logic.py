"""Hardware-free tests for the new PyQt recognition console helpers."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from .bronchus_map import BronchusMapWidget, EDGE_WAYPOINTS, NODE_POSITIONS
from .buses import LatestFrameBus, RecognitionControlBus
from .display_gate import InitialDisplayGate
from .robot_runtime import RobotRuntime


class FrameBusTests(unittest.TestCase):
    def test_only_latest_frame_is_retained(self) -> None:
        bus = LatestFrameBus()
        first = np.zeros((4, 4, 3), dtype=np.uint8)
        second = np.full((4, 4, 3), 7, dtype=np.uint8)
        bus.publish(first)
        bus.publish(second)
        sequence, snapshot = bus.snapshot()
        self.assertEqual(sequence, 2)
        self.assertIsNotNone(snapshot)
        self.assertTrue(np.all(snapshot == 7))


class ControlBusTests(unittest.TestCase):
    def test_detection_and_recording_are_independent(self) -> None:
        bus = RecognitionControlBus()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "clip.avi"
            bus.start_recording(output)
            self.assertEqual(bus.recording_path(), output)
            self.assertFalse(bus.detection_enabled())
            bus.set_detection_enabled(True)
            self.assertTrue(bus.detection_enabled())
            self.assertEqual(bus.recording_path(), output)
            bus.stop_recording()
            self.assertIsNone(bus.recording_path())


class MapTopologyTests(unittest.TestCase):
    def test_confirmed_path_uses_existing_topology(self) -> None:
        self.assertEqual(BronchusMapWidget._path_to("RLLB"), {"TR", "RMB", "BI", "RLLB"})
        self.assertEqual(BronchusMapWidget._path_to("LULB"), {"TR", "LMB", "LULB"})

    def test_no_confirmed_position_has_no_active_path(self) -> None:
        self.assertEqual(BronchusMapWidget._path_to(None), set())

    def test_registered_map_anchors_land_on_the_airway_asset(self) -> None:
        asset = Path(__file__).resolve().parent / "assets" / "bronchi.png"
        image = cv2.imread(str(asset))
        self.assertIsNotNone(image)
        source_height, source_width = image.shape[:2]
        crop_top = int(source_height * 0.20)
        crop_height = int(source_height * 0.56)
        crop_width = int(source_width * 0.97)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        anchors = list(NODE_POSITIONS.items())
        anchors.extend(
            (f"{parent}-{child}-{index}", point)
            for (parent, child), route in EDGE_WAYPOINTS.items()
            for index, point in enumerate(route)
        )
        for name, (rx, ry) in anchors:
            x = round(rx * crop_width)
            y = crop_top + round(ry * crop_height)
            patch = gray[y - 10 : y + 11, x - 10 : x + 11]
            self.assertGreater(float(patch.mean()), 80.0, name)


class DetectionDisplayTests(unittest.TestCase):
    @staticmethod
    def _state(
        updated_at: float,
        observed: str | None = "TR",
        confirmed: str = "TR",
        confidence: float | None = 0.80,
        safe_state: str = "CONFIRMED",
    ) -> SimpleNamespace:
        return SimpleNamespace(
            updated_at=updated_at,
            observed_position=observed,
            observed_confidence=confidence,
            confirmed_position=confirmed,
            safe_state=safe_state,
        )

    def test_default_internal_tr_does_not_arm_display(self) -> None:
        gate = InitialDisplayGate(required_updates=4, confidence_threshold=0.60)
        gate.reset(1.0)
        self.assertFalse(gate.update(self._state(2.0, observed=None)))
        self.assertEqual(gate.consecutive_updates, 0)

    def test_single_false_positive_tr_does_not_arm_display(self) -> None:
        gate = InitialDisplayGate(required_updates=4, confidence_threshold=0.60)
        gate.reset(1.0)
        self.assertFalse(gate.update(self._state(2.0)))
        self.assertEqual(gate.consecutive_updates, 1)

    def test_repeated_ui_refresh_does_not_count_same_inference_twice(self) -> None:
        gate = InitialDisplayGate(required_updates=2, confidence_threshold=0.60)
        gate.reset(1.0)
        state = self._state(2.0)
        self.assertFalse(gate.update(state))
        self.assertFalse(gate.update(state))
        self.assertEqual(gate.consecutive_updates, 1)

    def test_four_consecutive_high_confidence_updates_arm_display(self) -> None:
        gate = InitialDisplayGate(required_updates=4, confidence_threshold=0.60)
        gate.reset(1.0)
        for updated_at in (2.0, 3.0, 4.0):
            self.assertFalse(gate.update(self._state(updated_at)))
        self.assertTrue(gate.update(self._state(5.0)))
        self.assertTrue(gate.armed)

    def test_low_confidence_or_missing_frame_resets_streak(self) -> None:
        gate = InitialDisplayGate(required_updates=3, confidence_threshold=0.60)
        gate.reset(1.0)
        self.assertFalse(gate.update(self._state(2.0)))
        self.assertFalse(gate.update(self._state(3.0, confidence=0.40)))
        self.assertEqual(gate.consecutive_updates, 0)
        self.assertFalse(gate.update(self._state(4.0)))
        self.assertFalse(gate.update(self._state(5.0, observed=None)))
        self.assertEqual(gate.consecutive_updates, 0)

    def test_different_confirmed_label_restarts_streak(self) -> None:
        gate = InitialDisplayGate(required_updates=3, confidence_threshold=0.60)
        gate.reset(1.0)
        self.assertFalse(gate.update(self._state(2.0)))
        self.assertFalse(
            gate.update(self._state(3.0, observed="RMB", confirmed="RMB"))
        )
        self.assertEqual(gate.candidate, "RMB")
        self.assertEqual(gate.consecutive_updates, 1)


class NavigationMenuTests(unittest.TestCase):
    def test_tr_is_always_separate_return_origin_target(self) -> None:
        targets = RobotRuntime.navigation_targets()
        self.assertTrue(targets)
        self.assertEqual(targets[0][1], "TR - Fixed Tracheal Origin")
        self.assertNotIn("|", targets[0][1])

    def test_available_navigation_entries_are_single_files(self) -> None:
        targets = RobotRuntime.navigation_targets()[1:]
        self.assertTrue(targets)
        self.assertTrue(all("|" in target for _label, target in targets))


class ControllerStatusTests(unittest.TestCase):
    def test_live_snapshot_controls_connection_badge(self) -> None:
        class FakeController:
            @staticmethod
            def snapshot():
                return {"connected": True}

        class FakeRobot:
            xbox = FakeController()
            controller_available = False

        self.assertTrue(RobotRuntime._controller_connected(FakeRobot()))


if __name__ == "__main__":
    unittest.main()
