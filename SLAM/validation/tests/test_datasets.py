"""Dataset adapters: calibration, frame counts and pose conventions.

    python -m unittest SLAM/validation/tests/test_datasets.py

Each test is skipped when its dataset is not on this machine.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import sequences  # noqa: E402


def have(name):
    try:
        return len(sequences.get(name)) > 0
    except Exception:  # noqa: BLE001
        return False


@unittest.skipUnless(have("kitti_04"), "KITTI odometry not present (data_external/kitti_odometry or KITTI_DIR)")
class TestKitti(unittest.TestCase):
    def test_calibration_counts_and_image_size(self):
        seq = sequences.get("kitti_04")
        root = Path(seq.paths[0]).parents[1]
        P0 = np.array(next(line for line in (root / "calib.txt").read_text().splitlines()
                           if line.startswith("P0:")).split()[1:], float).reshape(3, 4)
        np.testing.assert_allclose(seq.K, P0[:, :3])
        self.assertEqual(seq.K[0, 0], seq.K[1, 1])
        self.assertEqual(len(seq), 271)
        self.assertEqual(len(seq.timestamps), len(seq))
        gray, color = seq.load(0)
        self.assertEqual(gray.shape[::-1], tuple(seq.size))
        self.assertIsNone(color)

    def test_ground_truth_is_camera_to_world(self):
        seq = sequences.get("kitti_04")
        stamps, R_wc, t_wc = seq.ground_truth
        self.assertEqual(len(stamps), len(seq))
        np.testing.assert_allclose(R_wc[0], np.eye(3), atol=1e-6)
        np.testing.assert_allclose(t_wc[0], np.zeros(3), atol=1e-6)
        # Sequence 04 is a straight drive: the camera centre moves along the first camera's optical axis (+z).
        # With the inverse (world to camera) convention this z would come out negative.
        self.assertGreater(t_wc[100, 2], 50.0)
        self.assertLess(abs(t_wc[100, 0]), 0.2 * t_wc[100, 2])
        for R in R_wc[::50]:
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-5)


if __name__ == "__main__":
    unittest.main()
