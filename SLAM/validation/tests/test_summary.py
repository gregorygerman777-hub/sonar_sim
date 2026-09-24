"""summarize.py: the summary table reports what each run recorded.

    python -m unittest SLAM/validation/tests/test_summary.py
"""

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import summarize  # noqa: E402


def write_run(results, name, **meta):
    (results / name).mkdir()
    (results / name / "run_meta.json").write_text(json.dumps(dict(dataset="fake", frames_total=10, **meta)))


class TestMapCount(unittest.TestCase):
    """A SLAM run that never initialized a map was listed with 1 map (the synthetic caustic scene at 1 frame in 4
    and in 8, results/SUMMARY.md)."""

    def test_maps_and_models_are_counted_as_recorded(self):
        with tempfile.TemporaryDirectory() as d:
            results = Path(d)
            write_run(results, "fake_orb", method="monoslam", frontend="orb", frames_posed=0, fraction_posed=0.0,
                      maps=[])
            write_run(results, "fake_sift", method="monoslam", frontend="sift", frames_posed=8, fraction_posed=0.8,
                      maps=[{"map": 0}, {"map": 1}])
            write_run(results, "fake_colmap_sequential", method="colmap_sequential", frontend="sift", frames_posed=9,
                      fraction_posed=0.9, models={"0": 9, "1": 1})
            write_run(results, "fake_colmap_exhaustive", method="colmap_exhaustive", frontend="sift", frames_posed=0,
                      fraction_posed=0.0, models={})
            with mock.patch.object(summarize, "RESULTS", results), mock.patch("builtins.print"):
                summarize.main()
            maps = {r["run"]: r["maps"] for r in csv.DictReader(open(results / "summary.csv"))}
        self.assertEqual(maps, {"fake_orb": "0", "fake_sift": "2", "fake_colmap_sequential": "2",
                                "fake_colmap_exhaustive": "0"})


if __name__ == "__main__":
    unittest.main()
