import json
import tempfile
import unittest
from pathlib import Path

from trajectory_reconstruction_integration import reconstruct_file


class TrajectoryIntegrationTest(unittest.TestCase):
    def test_reconstructs_bc_recording(self):
        fixture = Path(__file__).parent / "TrajectoryReconstruction" / "examples" / "minimal_session.json"
        with tempfile.TemporaryDirectory() as directory:
            result = reconstruct_file(fixture, directory)

            self.assertEqual(result["frames"], 5)
            for kind in ("json", "csv", "html"):
                self.assertTrue(Path(result[kind]).is_file(), kind)

            payload = json.loads(Path(result["json"]).read_text(encoding="utf-8"))
            self.assertEqual(len(payload["points"]), 5)
            self.assertEqual(payload["metadata"]["source"], str(fixture.resolve()))


if __name__ == "__main__":
    unittest.main()
