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


class TestRunWithoutPoses(unittest.TestCase):
    """A run that never initialized a map writes an empty trajectory and an empty PLY. export.read_ply could not read
    that PLY (it crashed run_all.sh once the figure step covered every run), and there is nothing to draw."""

    def test_empty_ply_reads_as_no_points(self):
        from monoslam import export
        with tempfile.TemporaryDirectory() as d:
            export.write_ply(Path(d) / "a.ply", np.empty((0, 3)))
            export.write_ply(Path(d) / "b.ply", np.empty((0, 3)), np.empty((0, 3), np.uint8))
            xyz, rgb = export.read_ply(Path(d) / "a.ply")
            self.assertEqual((xyz.shape, rgb), ((0, 3), None))
            xyz, rgb = export.read_ply(Path(d) / "b.ply")
            self.assertEqual((xyz.shape, rgb.shape), ((0, 3), (0, 3)))

    def test_no_figures_for_a_run_without_poses(self):
        import json
        from unittest import mock
        from monoslam import export
        with tempfile.TemporaryDirectory() as d:
            run, out = Path(d) / "fake_orb_stride4", Path(d) / "figs"
            run.mkdir()
            (run / "run_meta.json").write_text(json.dumps(dict(dataset="fake", frames_total=60, frames_posed=0,
                                                               K=[[100, 0, 32], [0, 100, 24], [0, 0, 1]])))
            (run / "metrics.json").write_text(json.dumps(dict(status="TOO FEW POSED", frames_posed=0)))
            export.write_tum(run / "trajectory_tum.txt", [], [], [])
            export.write_ply(run / "points.ply", np.empty((0, 3)))
            with mock.patch.object(sys, "argv", ["make_figures.py", str(run), "--out", str(out)]), \
                    mock.patch("builtins.print"):
                make_figures.main()
            self.assertFalse(out.exists() and any(out.iterdir()))


class TestViewPlane(unittest.TestCase):
    def test_the_same_map_gives_the_same_view_on_every_call(self):
        """The RANSAC generator was a default argument, created once: a second figure drawn in the same process
        continued its random stream, so the view depended on what had been drawn before."""
        rng = np.random.default_rng(3)
        floor = np.column_stack((rng.uniform(-5, 5, (2000, 2)), rng.normal(0, 0.05, 2000)))
        clutter = rng.uniform(-5, 5, (800, 3))
        xyz, centres = np.vstack((floor, clutter)), np.array([[0.0, 0.0, 2.0]])
        first = make_figures.dominant_plane_normal(xyz, centres)
        np.testing.assert_array_equal(make_figures.dominant_plane_normal(xyz, centres), first)


class TestStatusLine(unittest.TestCase):
    def test_too_few_posed_is_not_called_no_ground_truth(self):
        meta = dict(frames_posed=2, frames_total=60)
        line = make_figures.status_line(meta, dict(status="TOO FEW POSED", frames_posed=2))
        self.assertNotIn("no ground truth", line)
        self.assertIn("too few frames posed", line)
        self.assertEqual(make_figures.status_line(dict(frames_posed=12, frames_total=117), None),
                         "no ground truth; posed 12/117 frames")


class TestKittiOverview(unittest.TestCase):
    def test_draws_the_scored_sequences(self):
        import json
        from unittest import mock
        import kitti_overview
        n = 30
        gt = np.column_stack((np.linspace(0, 20, n), np.zeros(n), np.linspace(0, 40, n)))
        fake = type("Seq", (), {"ground_truth": (np.arange(n) * 0.1, np.repeat(np.eye(3)[None], n, 0), gt)})()
        with tempfile.TemporaryDirectory() as d:
            run = Path(d) / "results" / "kitti_03_sift"
            run.mkdir(parents=True)
            (run / "run_meta.json").write_text(json.dumps(dict(frames_posed=n, frames_total=n, maps=[{"map": 0}])))
            (run / "metrics.json").write_text(json.dumps(dict(ate_m=dict(rmse=0.5), ate_rmse_percent_of_path=1.1)))
            rows = np.column_stack((np.arange(n) * 0.1, gt + 0.3, np.tile([0, 0, 0, 1.0], (n, 1))))
            np.savetxt(run / "aligned_estimate_tum.txt", rows, header="timestamp tx ty tz qx qy qz qw")
            with mock.patch.object(kitti_overview, "HERE", Path(d)), mock.patch.object(sys, "argv", ["x"]), \
                    mock.patch("sequences.get", lambda name: fake), mock.patch("builtins.print"):
                kitti_overview.main()
            self.assertTrue((Path(d) / "figures" / "kitti" / "overview_sift.png").stat().st_size > 10000)
        self.assertEqual(kitti_overview.panel_title("03", dict(frames_posed=396, frames_total=801, maps=[1, 2]),
                                                    dict(ate_m=dict(rmse=1.21), ate_rmse_percent_of_path=0.43)),
                         "03: 1.2 m (0.4 %), main map of 2")


if __name__ == "__main__":
    unittest.main()
