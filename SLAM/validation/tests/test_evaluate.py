"""evaluate.py on synthetic trajectories with a known answer.

    python -m unittest SLAM/validation/tests/test_evaluate.py
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import evaluate  # noqa: E402
from monoslam import export, geometry as geo  # noqa: E402


class FakeSequence:
    def __init__(self, stamps, R_wc, t_wc, name="fake_sequence"):
        self.name = name
        self.ground_truth = (stamps, R_wc, t_wc)


def run_with(gt_R, gt_p, seed=0, name="fake_sequence", rpe_delta_m=1.0):
    """Score an estimate that is ground truth under a similarity transform plus 1 cm of position noise."""
    rng = np.random.default_rng(seed)
    n = len(gt_p)
    stamps = np.arange(n) * 0.1
    Rs = geo.rotvec_to_matrix(np.array([0.3, -1.1, 0.7]))[0]
    s, ts = 0.25, np.array([4.0, -2.0, 1.0])
    est_p = s * (gt_p @ Rs.T) + ts + s * rng.normal(0, 0.01, (n, 3))
    est_R = np.einsum("ij,njk->nik", Rs, gt_R)
    original = evaluate.sequences.get
    evaluate.sequences.get = lambda key: FakeSequence(stamps, gt_R, gt_p, name)
    try:
        with tempfile.TemporaryDirectory() as d:
            run = Path(d)
            (run / "run_meta.json").write_text(json.dumps(dict(dataset="fake_sequence", frames_total=n,
                                                               frame_timestamps=stamps.tolist())))
            export.write_tum(run / "trajectory_tum.txt", stamps, est_R, est_p)
            return evaluate.evaluate(run, rpe_delta_m=rpe_delta_m)
    finally:
        evaluate.sequences.get = original


class TestOrientationError(unittest.TestCase):
    def test_straight_path_orientation_is_reported_as_undetermined(self):
        # A 100 m straight drive with 2 mm of lateral wobble; the camera looks along the path.
        n = 100
        rng = np.random.default_rng(1)
        gt_p = np.column_stack((rng.normal(0, 0.002, n), rng.normal(0, 0.002, n), np.linspace(0, 100, n)))
        gt_R = np.repeat(np.eye(3)[None], n, axis=0)
        r = run_with(gt_R, gt_p)
        self.assertLess(r["ate_m"]["rmse"], 0.05)            # positions are right...
        self.assertIsNone(r["orientation_error_deg"])       # ...but a roll about the path cannot be scored
        self.assertIn("nearly straight", r["orientation_error_note"])
        self.assertLess(r["rpe_rotation_deg"]["rmse"], 1e-6)  # the alignment free rotation check still works

    def test_curved_path_orientation_is_scored(self):
        n = 120
        a = np.linspace(0, 1.5 * np.pi, n)
        gt_p = np.column_stack((10 * np.cos(a), 10 * np.sin(a), 0.5 * np.sin(3 * a)))
        gt_R = geo.rotvec_to_matrix(np.column_stack((np.zeros(n), np.zeros(n), a)))
        r = run_with(gt_R, gt_p)
        self.assertIsNotNone(r["orientation_error_deg"])
        self.assertLess(r["orientation_error_deg"]["rmse"], 0.5)


class TestRelativeError(unittest.TestCase):
    def test_rpe_is_computed_on_kitti_like_frame_spacing(self):
        """KITTI frames are about 1.4 m apart, so an RPE over 1 m of travel has no frame pairs; the KITTI default
        must still produce one (it silently came out empty on KITTI 04)."""
        n = 300
        a = np.linspace(0, np.pi, n)
        gt_p = np.column_stack((140 * np.sin(a), np.zeros(n), 140 * (1 - np.cos(a))))   # 1.47 m per frame
        gt_R = geo.rotvec_to_matrix(np.column_stack((np.zeros(n), -a, np.zeros(n))))
        r = run_with(gt_R, gt_p, name="kitti_99", rpe_delta_m=None)
        self.assertEqual(r["rpe_delta_m"], 100.0)
        self.assertIsNotNone(r["rpe_translation_m"])
        self.assertLess(r["rpe_rotation_deg"]["rmse"], 1e-6)
        self.assertEqual(evaluate.default_rpe_delta_m("tum_freiburg1_xyz"), 1.0)

    def test_segments_are_measured_along_the_ground_truth(self):
        """RPE over d metres compares pose pairs d metres apart along the ground truth, as the KITTI benchmark does.
        evo picks the pairs along the estimate unless told otherwise, so where the monocular scale had drifted a
        '100 m' segment could span 200 m of road (KITTI 00 SIFT: 122 m instead of 51 m)."""
        from scipy.spatial.transform import Rotation
        n = 61
        wobble = 0.05 * np.sin(np.arange(n))                                     # keeps the alignment well posed
        gt_p = np.column_stack((np.arange(n, dtype=float), wobble, np.zeros(n)))  # about 1 m per frame
        est_x = np.concatenate(([0.0], np.cumsum(np.where(np.arange(1, n) <= 30, 1.0, 0.5))))   # half scale later
        est_p = np.column_stack((est_x, wobble, np.zeros(n)))
        gt_R = np.repeat(np.eye(3)[None], n, axis=0)
        stamps = np.arange(n) * 0.1
        original = evaluate.sequences.get
        evaluate.sequences.get = lambda key: FakeSequence(stamps, gt_R, gt_p)
        try:
            with tempfile.TemporaryDirectory() as d:
                run = Path(d)
                (run / "run_meta.json").write_text(json.dumps(dict(dataset="fake_sequence", frames_total=n,
                                                                   frame_timestamps=stamps.tolist())))
                export.write_tum(run / "trajectory_tum.txt", stamps, gt_R, est_p)
                r = evaluate.evaluate(run, rpe_delta_m=10.0)
                est = np.loadtxt(run / "aligned_estimate_tum.txt", ndmin=2)
                ref = np.loadtxt(run / "associated_gt_tum.txt", ndmin=2)
        finally:
            evaluate.sequences.get = original

        def pose(row):
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = Rotation.from_quat(row[4:8]).as_matrix(), row[1:4]
            return T

        # Pairs 10 m apart along the ground truth (evo's rule: the nearest distance, within 10 % of 10 m), scored on
        # the aligned estimate that evaluate.py saved.
        dist = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(ref[:, 1:4], axis=0), axis=1))))
        errors = []
        for i in range(len(ref) - 1):
            j = i + 1 + int(np.argmin(np.abs(dist[i + 1:] - dist[i] - 10.0)))
            if abs(dist[j] - dist[i] - 10.0) <= 1.0:
                g = np.linalg.inv(pose(ref[i])) @ pose(ref[j])
                e = np.linalg.inv(pose(est[i])) @ pose(est[j])
                errors.append(np.linalg.norm((np.linalg.inv(g) @ e)[:3, 3]))
        self.assertGreater(len(errors), 40)
        self.assertAlmostEqual(r["rpe_translation_m"]["rmse"], float(np.sqrt(np.mean(np.square(errors)))), places=6)


class TestRunLength(unittest.TestCase):
    def test_run_length_counts_frames_that_were_not_posed(self):
        """gt_path_length_m covers the posed frames only; the length of the run (every frame the method was given)
        is recorded separately, so a table does not show a partial run's posed length as the sequence length."""
        n = 50
        gt_p = np.column_stack((np.arange(n) * 0.1, 0.01 * np.sin(np.arange(n)), np.zeros(n)))
        gt_R = np.repeat(np.eye(3)[None], n, axis=0)
        stamps = np.arange(n) * 0.1
        original = evaluate.sequences.get
        evaluate.sequences.get = lambda key: FakeSequence(stamps, gt_R, gt_p)
        try:
            with tempfile.TemporaryDirectory() as d:
                run = Path(d)
                (run / "run_meta.json").write_text(json.dumps(dict(dataset="fake_sequence", frames_total=n,
                                                                   frame_timestamps=stamps.tolist())))
                export.write_tum(run / "trajectory_tum.txt", stamps[:20], gt_R[:20], 2.0 * gt_p[:20])
                r = evaluate.evaluate(run)
        finally:
            evaluate.sequences.get = original
        self.assertAlmostEqual(r["gt_path_length_m"], evaluate.path_length(gt_p[:20]), places=9)
        self.assertAlmostEqual(r["run_path_length_m"], evaluate.path_length(gt_p), places=9)


class TestTooFewFrames(unittest.TestCase):
    def test_a_run_with_two_posed_frames_is_labelled_not_failed(self):
        n = 50
        gt_p = np.column_stack((np.arange(n) * 0.1, np.zeros(n), np.zeros(n)))
        gt_R = np.repeat(np.eye(3)[None], n, axis=0)
        original = evaluate.sequences.get
        evaluate.sequences.get = lambda key: FakeSequence(np.arange(n) * 0.1, gt_R, gt_p)
        try:
            with tempfile.TemporaryDirectory() as d:
                run = Path(d)
                (run / "run_meta.json").write_text(json.dumps(dict(dataset="fake_sequence", frames_total=n)))
                export.write_tum(run / "trajectory_tum.txt", [0.0, 0.1], gt_R[:2], gt_p[:2])
                r = evaluate.evaluate(run)
        finally:
            evaluate.sequences.get = original
        self.assertEqual(r["status"], "TOO FEW POSED")
        self.assertEqual(r["frames_posed"], 2)


if __name__ == "__main__":
    unittest.main()
