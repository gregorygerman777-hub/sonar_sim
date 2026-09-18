"""Checks for the KITTI odometry port against analytic cases and the C++ SE(2) helpers."""
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT), str(ROOT / "python")]

import numpy as np

import kitti_odometry as kitti
import slam
import sonar


def straight_line(spacing_m=1.0, count=41):
    poses = np.zeros((count, 3))
    poses[:, 0] = spacing_m * np.arange(count)
    return kitti.se2_trajectory_to_matrices(poses)


class FormatChecks(unittest.TestCase):
    def test_embedding_matches_planar_pose_axes_and_point_transform(self):
        pose = np.array([1.5, -2.0, 0.8])
        matrix = kitti.se2_to_matrix(pose)
        np.testing.assert_allclose(matrix[:3, :3], slam.pose_axes(pose))
        local = np.array([[0.7, 2.1], [-1.2, 3.3]])
        expected = slam.points_in_world(local, pose)
        homogeneous = np.c_[local, np.zeros(2), np.ones(2)]
        np.testing.assert_allclose((matrix @ homogeneous.T).T[:, :2], expected)
        np.testing.assert_allclose(kitti.matrix_to_se2(matrix), pose)

    def test_relative_motion_matches_cpp_relative_pose_2d(self):
        rng = np.random.default_rng(1)
        for _ in range(20):
            a, b = rng.uniform(-3, 3, 3), rng.uniform(-3, 3, 3)
            delta = np.linalg.inv(kitti.se2_to_matrix(a)) @ kitti.se2_to_matrix(b)
            expected = np.array(sonar.relative_pose_2d(a, b))
            got = kitti.matrix_to_se2(delta)
            np.testing.assert_allclose(got[:2], expected[:2], atol=1e-12)
            self.assertAlmostEqual(np.angle(np.exp(1j * (got[2] - expected[2]))), 0.0, places=12)

    def test_pose_file_round_trip_and_first_frame_identity(self):
        rng = np.random.default_rng(2)
        matrices = kitti.se2_trajectory_to_matrices(rng.uniform(-5, 5, (7, 3)))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "00.txt"
            kitti.write_poses(path, matrices)
            lines = path.read_text().strip().splitlines()
            self.assertEqual(len(lines), 7)
            self.assertTrue(all(len(line.split()) == 12 for line in lines))
            np.testing.assert_allclose(kitti.read_poses(path), matrices, atol=1e-8)
        relative = kitti.relative_to_first(matrices)
        np.testing.assert_allclose(relative[0], np.eye(4), atol=1e-12)
        # Relative motions are unchanged by the gauge change.
        for i in range(1, 7):
            np.testing.assert_allclose(np.linalg.inv(relative[0]) @ relative[i],
                                       np.linalg.inv(matrices[0]) @ matrices[i], atol=1e-12)


class MetricChecks(unittest.TestCase):
    def test_identical_trajectories_have_zero_error(self):
        gt = straight_line()
        errors = kitti.sequence_errors(gt, gt.copy(), lengths=(2.5, 4.5, 8.5), step=1)
        self.assertGreater(len(errors), 0)
        self.assertEqual(max(e["t_err"] for e in errors), 0.0)
        self.assertEqual(max(e["r_err"] for e in errors), 0.0)

    def test_segment_end_is_first_frame_strictly_beyond_length(self):
        distances = kitti.trajectory_distances(straight_line(spacing_m=1.0, count=6))
        self.assertEqual(kitti.last_frame_from_segment_length(distances, 0, 2.5), 3)
        self.assertEqual(kitti.last_frame_from_segment_length(distances, 0, 3.0), 4)
        self.assertEqual(kitti.last_frame_from_segment_length(distances, 2, 2.5), 5)
        self.assertEqual(kitti.last_frame_from_segment_length(distances, 3, 2.5), -1)

    def test_translation_error_for_known_scale_drift(self):
        # Frames every 1 m; lengths 2.5 and 4.5 m end 3 and 5 frames later.
        gt = straight_line(spacing_m=1.0, count=21)
        est = straight_line(spacing_m=1.1, count=21)
        errors = kitti.sequence_errors(gt, est, lengths=(2.5, 4.5), step=1)
        for e in errors:
            actual = e["last_frame"] - e["first_frame"]
            self.assertAlmostEqual(e["t_err"], 0.1 * actual / e["length"], places=12)
            self.assertAlmostEqual(e["r_err"], 0.0, places=12)
        summary = kitti.average_errors(errors)
        expected = 100 * np.mean([0.1 * (e["last_frame"] - e["first_frame"]) / e["length"] for e in errors])
        self.assertAlmostEqual(summary["t_err_percent"], expected, places=9)

    def test_rotation_error_for_known_yaw_rate_bias(self):
        gt = straight_line(spacing_m=1.0, count=21)
        bias = np.radians(2.0)  # radians of heading error per metre travelled
        poses = np.zeros((21, 3))
        poses[:, 0] = np.arange(21)
        poses[:, 2] = bias * poses[:, 0]
        est = kitti.se2_trajectory_to_matrices(poses)
        errors = kitti.sequence_errors(gt, est, lengths=(2.5, 4.5), step=1)
        for e in errors:
            actual = e["last_frame"] - e["first_frame"]
            self.assertAlmostEqual(e["r_err"], bias * actual / e["length"], places=12)
        summary = kitti.average_errors(errors)
        self.assertAlmostEqual(
            summary["r_err_deg_per_m"],
            np.degrees(np.mean([bias * (e["last_frame"] - e["first_frame"]) / e["length"] for e in errors])),
            places=9)

    def test_speed_uses_timestamps(self):
        gt = straight_line(spacing_m=0.5, count=11)
        times = 0.4 * np.arange(11)
        errors = kitti.sequence_errors(gt, gt, lengths=(2.0,), step=1, times=times)
        for e in errors:
            self.assertAlmostEqual(e["speed"], 2.0 / (0.4 * (e["last_frame"] - e["first_frame"])))
        binned = kitti.errors_by_speed(errors, bin_width=0.25, minimum_segments=1)
        self.assertTrue(all(row["t_err_percent"] == 0.0 for row in binned))

    def test_aligned_ate_removes_only_rigid_motion(self):
        rng = np.random.default_rng(3)
        truth = rng.uniform(-4, 4, (30, 2))
        c, s = np.cos(0.6), np.sin(0.6)
        moved = truth @ np.array([[c, -s], [s, c]]).T + [3.0, -1.0]
        _, ate = kitti.aligned_ate(moved, truth)
        self.assertLess(ate, 1e-10)
        _, scaled = kitti.aligned_ate(1.3 * moved, truth)
        self.assertGreater(scaled, 0.3)

    def test_aligned_ate_se3_removes_only_rigid_motion(self):
        rng = np.random.default_rng(4)
        truth = np.tile(np.eye(4), (25, 1, 1))
        truth[:, :3, 3] = rng.uniform(-5, 5, (25, 3))
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = 0.9
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rotation = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k
        moved = truth.copy()
        moved[:, :3, 3] = truth[:, :3, 3] @ rotation.T + [2.0, -1.0, 0.5]
        _, ate = kitti.aligned_ate_se3(moved, truth)
        self.assertLess(ate, 1e-10)
        scaled = moved.copy()
        scaled[:, :3, 3] *= 1.2
        _, scaled_ate = kitti.aligned_ate_se3(scaled, truth)
        self.assertGreater(scaled_ate, 0.3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
