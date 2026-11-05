import unittest

from broncho_trajectory.centerline import interpolate_polyline, map_progress_to_route, similarity_register


class CenterlineTest(unittest.TestCase):
    def test_interpolate_polyline_by_arc_length(self):
        route = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 2.0, 0.0)]
        self.assertEqual(interpolate_polyline(route, 0.75), (2.0, 1.0, 0.0))

    def test_progress_preserves_retraction(self):
        route = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
        mapped = map_progress_to_route([0.0, 5.0, 10.0, 8.0], route)
        self.assertEqual(mapped[-1], (8.0, 0.0, 0.0))

    def test_similarity_registration(self):
        source = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
        target = [(5.0, 2.0, 0.0), (7.0, 2.0, 0.0), (9.0, 2.0, 0.0)]
        registered = similarity_register(source, target)
        for actual, expected in zip(registered, target):
            for a, b in zip(actual, expected):
                self.assertAlmostEqual(a, b, places=7)


if __name__ == "__main__":
    unittest.main()
