"""Checks for the visual odometry front end on synthetic data and the real KITTI calibration."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT / "SLAM/kitti_benchmark_20260916")]

import cv2
import numpy as np

import kitti_dataset
import kitti_odometry as kitti
import stereo_vo

K = np.array([[718.856, 0.0, 607.1928], [0.0, 718.856, 185.2157], [0.0, 0.0, 1.0]])
BASELINE = 0.5371657


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def project(points_cam):
    uv = points_cam @ K.T
    return uv[:, :2] / uv[:, 2:3]


def synthetic_pair(rng, motion, count=400, noise_px=0.3, outlier_fraction=0.1):
    """Previous-frame 3D points with descriptors, and their current-frame pixels under `motion`."""
    points = np.column_stack((rng.uniform(-15, 15, count), rng.uniform(-3, 3, count), rng.uniform(5, 45, count)))
    descriptors = rng.integers(0, 256, (count, 32), dtype=np.uint8)
    moved = points @ motion[:3, :3].T + motion[:3, 3]
    pixels = project(moved) + rng.normal(0.0, noise_px, (count, 2))
    outliers = rng.random(count) < outlier_fraction
    pixels[outliers] += rng.uniform(-60, 60, (int(outliers.sum()), 2))
    prev = stereo_vo.FrameFeatures(project(points), descriptors, points)
    cur = stereo_vo.FrameFeatures(pixels, descriptors.copy())
    return prev, cur


class MotionChecks(unittest.TestCase):
    def setUp(self):
        self.front = stereo_vo.StereoFrontEnd(K, BASELINE)
        self.motion = np.eye(4)
        self.motion[:3, :3] = rotation([0.1, 1.0, 0.05], np.radians(2.5))
        self.motion[:3, 3] = [0.03, -0.01, -1.2]  # forward motion appears as negative z in T_cur_prev

    def test_pnp_recovers_known_motion_with_outliers(self):
        prev, cur = synthetic_pair(np.random.default_rng(1), self.motion)
        estimate, info = self.front.motion_pnp(prev, cur)
        self.assertIsNotNone(estimate)
        self.assertGreater(info["inliers"], 300)
        np.testing.assert_allclose(estimate[:3, 3], self.motion[:3, 3], atol=0.01)
        angle = np.degrees(np.arccos(np.clip((np.trace(estimate[:3, :3].T @ self.motion[:3, :3]) - 1) / 2, -1, 1)))
        self.assertLess(angle, 0.05)

    def test_essential_recovers_rotation_and_direction(self):
        prev, cur = synthetic_pair(np.random.default_rng(2), self.motion, outlier_fraction=0.05)
        estimate, info = self.front.motion_essential(prev, cur)
        self.assertIsNotNone(estimate)
        angle = np.degrees(np.arccos(np.clip((np.trace(estimate[:3, :3].T @ self.motion[:3, :3]) - 1) / 2, -1, 1)))
        self.assertLess(angle, 0.3)
        direction = self.motion[:3, 3] / np.linalg.norm(self.motion[:3, 3])
        self.assertGreater(float(estimate[:3, 3] @ direction), 0.999)

    def test_chain_inverts_relative_motion(self):
        poses = stereo_vo.chain([self.motion, self.motion])
        np.testing.assert_allclose(poses[1], np.linalg.inv(self.motion), atol=1e-12)
        np.testing.assert_allclose(np.linalg.inv(poses[1]) @ poses[2], np.linalg.inv(self.motion), atol=1e-12)
        # Chained VO positions written in KITTI form must reproduce the motion through the port.
        matrices = kitti.relative_to_first(poses)
        delta = np.linalg.inv(matrices[0]) @ matrices[1]
        np.testing.assert_allclose(delta, np.linalg.inv(self.motion), atol=1e-12)


class StereoChecks(unittest.TestCase):
    def test_stereo_depth_from_shifted_texture(self):
        rng = np.random.default_rng(3)
        texture = cv2.GaussianBlur(rng.integers(0, 256, (376, 1400), dtype=np.uint8), (0, 0), 1.2)
        shift = 12
        left = np.ascontiguousarray(texture[:, :1241])
        right = np.ascontiguousarray(texture[:, shift:shift + 1241])   # right camera sees the scene shifted left
        front = stereo_vo.StereoFrontEnd(K, BASELINE)
        frame = front.extract(left, right)
        depths = frame.points3d[np.isfinite(frame.points3d[:, 2]), 2]
        self.assertGreater(len(depths), 200)
        expected = K[0, 0] * BASELINE / shift
        self.assertLess(abs(np.median(depths) - expected) / expected, 0.05)

    def test_bucketing_caps_features_per_cell(self):
        settings = stereo_vo.Settings(per_cell=5, grid=(4, 2))
        keypoints = [cv2.KeyPoint(float(x), float(y), 7.0, -1, float(x + y)) for x in range(0, 100, 5) for y in range(0, 50, 5)]
        descriptors = np.zeros((len(keypoints), 32), dtype=np.uint8)
        points, _ = stereo_vo.bucket(keypoints, descriptors, (100, 400), settings)
        self.assertLessEqual(len(points), 5 * 8)
        self.assertGreater(len(points), 0)


class DatasetChecks(unittest.TestCase):
    def test_real_sequence_00_calibration(self):
        sequence_dir = ROOT / "data_external/kitti_odometry/dataset/sequences/00"
        if not (sequence_dir / "calib.txt").exists():
            self.skipTest("KITTI calibration not downloaded")
        calibration = kitti_dataset.load_calibration(sequence_dir)
        self.assertAlmostEqual(calibration["K"][0, 0], 718.856, places=3)
        self.assertAlmostEqual(calibration["baseline_m"], 0.5371657, places=5)
        truth = kitti_dataset.load_ground_truth(ROOT / "data_external/kitti_odometry/dataset/poses", "00")
        self.assertEqual(truth.shape, (4541, 4, 4))
        np.testing.assert_allclose(truth[0], np.eye(4), atol=1e-6)
        self.assertEqual(len(kitti_dataset.load_times(sequence_dir)), 4541)


if __name__ == "__main__":
    unittest.main(verbosity=2)
