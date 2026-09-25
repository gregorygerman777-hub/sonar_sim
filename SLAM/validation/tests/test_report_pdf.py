"""build_report_pdf.py: the tables in the summary PDF say what each run's metrics say.

    python -m unittest SLAM/validation/tests/test_report_pdf.py
"""

import csv
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import build_report_pdf  # noqa: E402

FIELDS = ["dataset", "method", "run", "frames", "posed", "status", "ate_rmse_m", "ate_pct", "path_m", "run_path_m",
          "map_surface_median_m", "maps"]
ROWS = [
    # A partial run: its ATE and % cover the 574 posed frames only; the sequence is 2453 m long.
    dict(dataset="kitti_01", method="ours (ORB)", run="kitti_01_orb", frames=1101, posed=574, status="PARTIAL",
         ate_rmse_m=173.5, ate_pct=12.37, path_m=1403.0, run_path_m=2453.0, map_surface_median_m="", maps=8),
    dict(dataset="kitti_01", method="ours (SIFT)", run="kitti_01_sift", frames=1101, posed=478, status="PARTIAL",
         ate_rmse_m=7.07, ate_pct=0.63, path_m=1116.0, run_path_m=2453.0, map_surface_median_m="", maps=4),
    # The method posed too few frames to score: a result, not a failure.
    dict(dataset="synthetic_pool_caustics", method="COLMAP (exhaustive), 1 frame in 4",
         run="synthetic_pool_caustics_colmap_exhaustive", frames=60, posed=2, status="TOO FEW POSED", ate_rmse_m="",
         ate_pct="", path_m="", run_path_m=9.9, map_surface_median_m="", maps=1),
    dict(dataset="pool_raw", method="ours (ORB)", run="pool_raw_orb", frames=117, posed=12, status="no ground truth",
         ate_rmse_m="", ate_pct="", path_m="", run_path_m="", map_surface_median_m="", maps=4),
]


def cells(table):
    return [list(map(str, row)) for row in table._cellvalues]


class TestTables(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        (Path(self.tmp.name) / "results").mkdir()
        with open(Path(self.tmp.name) / "results" / "summary.csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(ROWS)
        self.patch = mock.patch.object(build_report_pdf, "HERE", Path(self.tmp.name))
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        self.tmp.cleanup()

    def test_compact_table_shows_the_sequence_length_and_does_not_call_a_result_a_failure(self):
        spec = dict(datasets=["kitti_01", "synthetic_pool_caustics"], labels={},
                    methods=[["ours (ORB)", "ORB"], ["ours (SIFT)", "SIFT"], ["COLMAP (exhaustive), 1 frame in 4", "COLMAP"]])
        rows = cells(build_report_pdf.compact_table(spec))
        self.assertEqual(rows[1][1], "2453 m")                   # not ORB's posed 1403 m
        self.assertIn("574/1101, 8 maps", rows[1][2])
        self.assertEqual(rows[1][4], "not run")
        self.assertNotIn("failed", rows[2][4].lower())
        self.assertIn("too few posed", rows[2][4].lower())

    def test_results_table_separates_no_ground_truth_from_too_few_posed(self):
        cfg = dict(table_datasets=["synthetic_pool_caustics", "pool_raw"], dataset_labels={})
        rows = {r[1]: r for r in cells(build_report_pdf.results_table(cfg))[1:]}
        self.assertIn("too few posed", rows["COLMAP (exh.), 1 in 4"][3].lower())
        self.assertEqual(rows["ours (ORB)"][3], "no ground truth")


if __name__ == "__main__":
    unittest.main()
