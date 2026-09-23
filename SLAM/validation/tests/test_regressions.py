"""Regression tests for bugs found during validation (see CHANGELOG.md). Each needs a downloaded dataset
and is skipped when it is missing. Each was confirmed to FAIL on the code before its fix.

    python -m unittest SLAM/validation/tests/test_regressions.py -v
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE.parent / "datasets")]

from monoslam import ba  # noqa: E402
from monoslam.system import MonoSLAM, Settings  # noqa: E402
import sequences  # noqa: E402

OLD = os.environ.get("MONOSLAM_OLD_BEHAVIOUR") == "1"   # set to reproduce the pre-fix behaviour


def have(name):
    try:
        return len(sequences.get(name)) > 0
    except Exception:  # noqa: BLE001
        return False


@unittest.skipUnless(have("tum_freiburg1_xyz"), "TUM fr1/xyz not downloaded")
class TestScaleGauge(unittest.TestCase):
    """CHANGELOG 1: with a single fixed keyframe in local BA the monocular scale blew up by about 2500x
    within 50 frames of TUM fr1/xyz (median scene depth 1 at initialization, 2.47e3 at frame 50)."""

    def test_scene_depth_stays_bounded(self):
        seq = sequences.get("tum_freiburg1_xyz")
        settings = Settings(local_ba_min_fixed=1) if OLD else Settings()
        saved = ba.LAMBDA_MIN
        ba.LAMBDA_MIN = 1e-12 if OLD else saved
        try:
            slam = MonoSLAM(seq.K, seq.size, settings, log=lambda *_: None)
            for i in range(60):
                gray, color = seq.load(i)
                slam.process(i, float(seq.timestamps[i]), gray, color)
        finally:
            ba.LAMBDA_MIN = saved
        depth = slam.median_depth()
        self.assertTrue(0.2 < depth < 5.0, f"median scene depth {depth:.3g} (started at 1)")


if __name__ == "__main__":
    unittest.main()
