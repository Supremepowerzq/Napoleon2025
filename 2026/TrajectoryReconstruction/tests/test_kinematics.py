import math
import unittest

from broncho_trajectory.kinematics import KinematicConfig, reconstruct


class KinematicsTest(unittest.TestCase):
    def test_straight_insertion(self):
        frames = [
            {"timestamp": 0.0, "m0_angle": 0.0, "m1_angle": 0.0, "m2_angle": 0.0},
            {"timestamp": 0.1, "m0_angle": -10.0, "m1_angle": 0.0, "m2_angle": 0.0},
        ]
        points = reconstruct(frames, KinematicConfig(m0_mm_per_degree=0.1))
        self.assertAlmostEqual(points[-1].position[0], 1.0)
        self.assertAlmostEqual(points[-1].position[1], 0.0)
        self.assertAlmostEqual(points[-1].position[2], 0.0)

    def test_bend_changes_direction(self):
        frames = [
            {"timestamp": 0.0, "m0_angle": 0.0, "m1_angle": 0.0, "m2_angle": 0.0},
            {"timestamp": 0.1, "m0_angle": -1.0, "m1_angle": 0.0, "m2_angle": 90.0},
        ]
        config = KinematicConfig(m0_mm_per_degree=1.0, m2_tip_deg_per_degree=1.0)
        point = reconstruct(frames, config)[-1]
        self.assertTrue(math.isclose(point.position[0], 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(point.position[2], 1.0, abs_tol=1e-9))

    def test_explicit_delta_has_priority(self):
        frames = [
            {"timestamp": 0.0, "m0_angle": 0.0, "m1_angle": 0.0, "m2_angle": 0.0},
            {"timestamp": 0.1, "m0_angle": -100.0, "m1_angle": 0.0, "m2_angle": 0.0,
             "delta_m0": -2.0, "delta_m2": 0.0},
        ]
        point = reconstruct(frames, KinematicConfig(m0_mm_per_degree=0.5))[-1]
        self.assertAlmostEqual(point.position[0], 1.0)

    def test_rejects_too_few_frames(self):
        with self.assertRaises(ValueError):
            reconstruct([{"m0_angle": 0.0, "m1_angle": 0.0, "m2_angle": 0.0}])


if __name__ == "__main__":
    unittest.main()
