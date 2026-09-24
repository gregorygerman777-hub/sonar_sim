"""Pool configuration: the primary intrinsics are the raw K Dr. Negahdaripour confirmed on 2026-09-23.

    python -m unittest SLAM/validation/tests/test_pool.py

The frames and OSCalibration.mat are not public; the tests that need them are skipped when they are absent.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import sequences  # noqa: E402

# As written in his reply: K = [1403.5 0 476.5; 0 1403.5 392.9; 0 0 1] (4 significant figures).
K_CONFIRMED = np.array([[1403.5, 0, 476.5], [0, 1403.5, 392.9], [0, 0, 1.0]])
POOL_DIR = Path(os.environ.get("POOL_DIR", Path.home() / "Downloads/Archive 2"))
POOL_CALIB = Path(os.environ.get("POOL_CALIB", Path.home() / "Downloads/OSCalibration.mat"))


class TestPoolIntrinsics(unittest.TestCase):
    def test_stored_constant_matches_confirmed_k(self):
        np.testing.assert_allclose(sequences.POOL_K_RAW, K_CONFIRMED, atol=0.06)

    @unittest.skipUnless(POOL_CALIB.exists(), "OSCalibration.mat not present")
    def test_calibration_file_matches_confirmed_k(self):
        import scipy.io
        np.testing.assert_allclose(scipy.io.loadmat(str(POOL_CALIB))["K"].astype(float), K_CONFIRMED, atol=0.06)

    @unittest.skipUnless((POOL_DIR / "opt1.bmp").exists(), "pool frames not present")
    def test_default_pool_uses_confirmed_k(self):
        seq = sequences.get("pool")
        self.assertEqual(seq.name, "pool_raw")
        np.testing.assert_allclose(seq.K, K_CONFIRMED, atol=0.06)


class TestRunNames(unittest.TestCase):
    """A --stride run without --tag once overwrote results/pool_raw_orb; subsampled runs must get their own name."""

    def test_stride_run_does_not_reuse_the_full_rate_name(self):
        import run_slam
        self.assertEqual(run_slam.output_name("pool_raw", "orb"), "pool_raw_orb")
        self.assertEqual(run_slam.output_name("pool_raw", "orb", stride=2), "pool_raw_orb_stride2")
        self.assertEqual(run_slam.output_name("pool_raw", "sift", stride=3, tag="_x"), "pool_raw_sift_x")


if __name__ == "__main__":
    unittest.main()
