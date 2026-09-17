"""Regression checks for the benchmark: imaging gain, catalogue determinism, prior results."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT), str(ROOT / "python"), str(ROOT / "SLAM/research")]

import numpy as np

import imaging
import kitti_odometry as kitti
import sequences
import slam
import slam_experiment
import sonar
from evaluate import estimate


class ImagingChecks(unittest.TestCase):
    def test_range_compensation_brings_far_targets_above_threshold(self):
        # Equal spheres on boresight: raw peaks differ by > 100x between 2 and 8 m, so the
        # 2.5 percent relative threshold drops the far one; compensated peaks stay within 4x.
        simulator = slam_experiment.build_simulator()
        gain = imaging.range_compensation(simulator, 1.2e6)
        raw, compensated = [], []
        for range_m in (2.0, 4.0, 8.0):
            image = simulator.render([sonar.make_sphere((0.0, range_m, 0.0), 0.12, 0.8)],
                                     position=(0.0, 0.0, 0.0), axes=slam.pose_axes((0.0, 0.0, 0.0)))
            raw.append(float(np.max(image)))
            compensated.append(float(np.max(image * gain[None, :])))
        self.assertGreater(raw[0] / raw[2], 100.0)
        self.assertLess(compensated[0] / compensated[2], 4.0, compensated)
        self.assertGreater(min(compensated) / max(compensated), 0.025)
        self.assertLess(min(raw) / max(raw), 0.025)


class CatalogueChecks(unittest.TestCase):
    def test_catalogue_is_deterministic_and_forward_looking(self):
        for spec in sequences.catalogue():
            truth_a, objects_a = spec["build"]()
            truth_b, objects_b = spec["build"]()
            np.testing.assert_array_equal(truth_a, truth_b)
            self.assertEqual(len(objects_a), len(objects_b))
            self.assertGreater(len(objects_a), 0)
            steps = np.linalg.norm(np.diff(truth_a[:, :2], axis=0), axis=1)
            self.assertLess(steps.max(), 1.0)
            if int(spec["name"]) >= 4:
                travel = np.diff(truth_a[:, :2], axis=0)
                travel /= np.linalg.norm(travel, axis=1)[:, None]
                forward = np.array([slam.pose_axes(p)[:2, 1] for p in truth_a[:-1]])
                angle = np.degrees(np.arccos(np.clip(np.sum(travel * forward, axis=1), -1, 1)))
                self.assertLess(angle.max(), 15.0, spec["name"])


class PriorResultChecks(unittest.TestCase):
    def test_raw_arc_reproduces_geometry_validation_ate(self):
        # Sequence 01 with raw imagery is exactly the 2026-09-15 seed-91501 run.
        spec = sequences.catalogue()[1]
        truth, objects = spec["build"]()
        simulator = slam_experiment.build_simulator()
        _, features, descriptors = slam_experiment.render_survey(
            simulator, objects, truth, sequences.positions_for(truth))
        result = estimate(dict(ids=np.arange(81), times=sequences.times_for(truth),
                               points=features, desc=descriptors), 1)
        _, ate = kitti.aligned_ate(result["poses"][:, :2], truth[:, :2])
        self.assertAlmostEqual(ate, 0.25769132573237985, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
