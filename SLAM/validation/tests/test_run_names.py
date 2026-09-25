"""Run directories: a run with another frame stride must never silently replace an existing one.

    python -m unittest SLAM/validation/tests/test_run_names.py
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]


class TestColmapOverwriteGuard(unittest.TestCase):
    """run_colmap.py keeps the stride out of the directory name (the reported fr3 and AQUALOC baselines are
    stride 3 and 2 runs under the plain name), so it must refuse to replace a run of another stride."""

    def test_refuses_a_different_stride_unless_told(self):
        import run_colmap
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "run_meta.json").write_text(json.dumps(dict(stride=3)))
            with self.assertRaises(SystemExit):
                run_colmap.check_overwrite(d, stride=1)
            run_colmap.check_overwrite(d, stride=3)                    # same stride: a rerun, allowed
            run_colmap.check_overwrite(d, stride=1, overwrite=True)    # explicit
        with tempfile.TemporaryDirectory() as d:
            run_colmap.check_overwrite(Path(d) / "new_run", stride=2)  # nothing there yet


class TestColmapRerun(unittest.TestCase):
    """run_colmap.py overwrote only what a run with a model writes. A rerun that built no model would have kept the
    earlier run's frames.csv, scores and pool export next to its own empty trajectory (as entry 21 for run_slam.py)."""

    def test_rerun_removes_the_earlier_runs_outputs(self):
        import run_colmap
        with tempfile.TemporaryDirectory() as d:
            run = Path(d)
            for name in ("trajectory_tum.txt", "points.ply", "frames.csv", "run_meta.json", "metrics.json",
                         "aligned_estimate_tum.txt", "associated_gt_tum.txt", "notes.txt"):
                (run / name).write_text("earlier run")
            (run / "export").mkdir()
            for name in ("pool_reconstruction.mat", "camera_trajectory.csv", "model.ply", "README.txt"):
                (run / "export" / name).write_text("earlier model")
            run_colmap.clear_previous_run(run)
            self.assertEqual(sorted(p.name for p in run.iterdir()), ["notes.txt"])   # only a run's own outputs go


class _FakeSequence:
    name, K, size, notes = "fake", [[100.0, 0, 16], [0, 100.0, 12], [0, 0, 1]], (32, 24), ""

    def __init__(self, n=9):
        self.timestamps = [0.1 * i for i in range(n)]

    def __len__(self):
        return len(self.timestamps)

    def subsample(self, stride):
        return self

    def load(self, i):
        return None, None


class _FakeMap:
    """What run_slam.main needs from an initialized map: frames first..last posed, the rest untracked."""

    def __init__(self, first, last, n):
        import numpy as np
        self.initialized, self.kfs = True, [type("KF", (), {"frame": first})()]
        self._traj = [dict(index=i, timestamp=0.1 * i, status="tracked", R_wc=np.eye(3), t_wc=np.array([i, 0.0, 0.0]),
                           inliers=50) if first <= i <= last else dict(index=i, timestamp=0.1 * i, status="untracked")
                      for i in range(n)]

    def finish(self):
        pass

    def trajectory(self):
        return self._traj

    def map_points(self):
        import numpy as np
        return np.zeros((2, 3)), np.zeros((2, 3), np.uint8), np.array([2, 2])

    def summary(self):
        return dict(status_counts={}, n_keyframes=1, init=None, stats={}, reprojection={})


class TestRerunReplacesPreviousRun(unittest.TestCase):
    """A rerun into an existing run directory must leave only its own outputs. It once left maps/map_k directories
    of the earlier run (which had more maps), and evaluate.py and the figures counted them as maps of the new run
    (aqualoc_harbor_07_sift, kitti_00_orb)."""

    def run_main(self, out, maps=None, track=None, git_commit=None):
        import gc
        import warnings
        from unittest import mock
        import run_slam
        from monoslam import export
        argv = ["run_slam.py", "--dataset", "fake", "--out", str(out)]
        with mock.patch.object(sys, "argv", argv), mock.patch("sequences.get", lambda name: _FakeSequence()), \
                mock.patch.object(run_slam, "track", track or (lambda *a, **k: maps)), \
                mock.patch.object(export, "git_commit", git_commit or (lambda root: ("test", False))), \
                mock.patch("builtins.print"), warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", ResourceWarning)
            run_slam.main()
            gc.collect()
        self.assertEqual([str(w.message) for w in caught if issubclass(w.category, ResourceWarning)], [],
                         "run_slam.main left a file open")

    def test_commit_is_read_when_the_run_starts(self):
        """The code a run executes is the code it imported when it started. The commit was read when the run ended,
        so kitti_08_sift (17:44 to 18:04) recorded a commit made at 18:02, and an unrelated uncommitted edit of 18:04."""
        state = dict(tracking_done=False)

        def track(*args, **kwargs):
            state["tracking_done"] = True
            return [(_FakeMap(0, 4, 9), 0)]

        def git_commit(root):
            return ("committed during the run", True) if state["tracking_done"] else ("at the start", False)

        with tempfile.TemporaryDirectory() as d:
            self.run_main(d, track=track, git_commit=git_commit)
            meta = json.loads((Path(d) / "fake_orb" / "run_meta.json").read_text())
        self.assertEqual((meta["git_commit"], meta["git_dirty"]), ("at the start", False))

    def test_fewer_maps_on_the_rerun(self):
        with tempfile.TemporaryDirectory() as d:
            run = Path(d) / "fake_orb"
            self.run_main(d, [(_FakeMap(0, 1, 9), 0), (_FakeMap(3, 4, 9), 3), (_FakeMap(5, 8, 9), 5)])
            self.assertEqual(sorted(p.name for p in (run / "maps").iterdir()), ["map_0", "map_1"])
            for name in ("metrics.json", "aligned_estimate_tum.txt", "associated_gt_tum.txt"):   # as evaluate.py does
                (run / name).write_text("scored earlier run")
            (run / "maps" / "map_0" / "aligned_estimate_tum.txt").write_text("scored earlier run")
            (run / "notes.txt").write_text("not written by a run")
            self.run_main(d, [(_FakeMap(0, 1, 9), 0), (_FakeMap(3, 8, 9), 3)])
            meta = json.loads((run / "run_meta.json").read_text())
            self.assertEqual(meta["primary_map"], 1)
            self.assertEqual(sorted(p.name for p in (run / "maps").iterdir()), ["map_0"])
            self.assertEqual(sorted(p.name for p in (run / "maps" / "map_0").iterdir()), ["points.ply", "trajectory_tum.txt"])
            for name in ("metrics.json", "aligned_estimate_tum.txt", "associated_gt_tum.txt"):
                self.assertFalse((run / name).exists(), f"{name} of the earlier run survived the rerun")
            self.assertTrue((run / "notes.txt").exists())      # only a run's own outputs are removed

    def test_no_map_on_the_rerun(self):
        with tempfile.TemporaryDirectory() as d:
            run = Path(d) / "fake_orb"
            self.run_main(d, [(_FakeMap(0, 1, 9), 0), (_FakeMap(3, 8, 9), 3)])
            self.run_main(d, [])
            self.assertEqual(json.loads((run / "run_meta.json").read_text())["frames_posed"], 0)
            self.assertFalse((run / "maps").exists())
            self.assertFalse((run / "keyframes.txt").exists())


class TestGitDirty(unittest.TestCase):
    """A rerun rewrites tracked results; that alone must not mark the run as made with modified code."""

    def test_only_source_changes_count(self):
        import subprocess
        from monoslam import export
        with tempfile.TemporaryDirectory() as d:
            run = lambda *a: subprocess.run(["git", *a], cwd=d, check=True, capture_output=True)
            run("init", "-q")
            run("config", "user.email", "t@example.com")
            run("config", "user.name", "t")
            for rel in ("SLAM/validation/run_slam.py", "SLAM/validation/results/x/run_meta.json",
                        "SLAM/validation/figures/x/1.png"):
                (Path(d) / rel).parent.mkdir(parents=True, exist_ok=True)
                (Path(d) / rel).write_text("a")
            run("add", "-A")
            run("commit", "-q", "-m", "c")
            (Path(d) / "SLAM/validation/results/x/run_meta.json").write_text("b")
            (Path(d) / "SLAM/validation/figures/x/1.png").write_text("b")
            self.assertFalse(export.git_commit(d)[1])
            (Path(d) / "SLAM/validation/run_slam.py").write_text("b")
            self.assertTrue(export.git_commit(d)[1])


if __name__ == "__main__":
    unittest.main()
