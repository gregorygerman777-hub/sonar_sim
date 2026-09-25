"""make_figures.py: the interactive overlay stays a usable size.

    python -m unittest SLAM/validation/tests/test_figures.py
"""

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import make_figures  # noqa: E402


class TestInteractiveOverlay(unittest.TestCase):
    """A KITTI map has up to 413,000 points, which made 3_overlay.html files of 20 MB (too slow to open, and too big
    for the repository). The interactive view shows at most MAX_HTML_POINTS of them and says so."""

    def test_large_maps_are_subsampled_and_labelled(self):
        rng = np.random.default_rng(0)
        n = make_figures.MAX_HTML_POINTS + 5000
        cloud, rgb = rng.normal(size=(n, 3)), rng.integers(0, 255, (n, 3))
        traj = np.cumsum(rng.normal(size=(50, 3)), axis=0)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "3_overlay.html"
            make_figures._interactive(path, cloud, rgb, traj, None, "test run", "m")
            html = path.read_text()
        self.assertIn(f"{make_figures.MAX_HTML_POINTS:,} of {n:,} map points shown", html)
        self.assertLess(len(html), 60 * make_figures.MAX_HTML_POINTS)   # about 40 bytes a point

    def test_small_maps_are_complete(self):
        rng = np.random.default_rng(1)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "3_overlay.html"
            make_figures._interactive(path, rng.normal(size=(300, 3)), rng.integers(0, 255, (300, 3)),
                                      rng.normal(size=(10, 3)), None, "small run", "m")
            self.assertNotIn("map points shown", path.read_text())


if __name__ == "__main__":
    unittest.main()
