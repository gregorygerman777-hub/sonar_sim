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
