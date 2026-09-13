import math
import sys
import unittest

import numpy as np

sys.path[:0] = [".", "python"]
import slam
import sonar


class SlamChecks(unittest.TestCase):
    def test_pose_composition_and_relative_inverse(self):
        base = (1.0, 2.0, math.pi / 2)
        moved = sonar.compose_pose_2d(base, (1.0, 0.0, 0.1))
        np.testing.assert_allclose(moved, (1.0, 3.0, math.pi / 2 + 0.1), atol=1e-12)
        np.testing.assert_allclose(sonar.relative_pose_2d(base, moved), (1.0, 0.0, 0.1),
                                   atol=1e-12)

    def test_planar_imu_constant_velocity(self):
        imu = sonar.PlanarImu(velocity=(1.0, -0.5))
        state, velocity = imu.step((0.0, 0.0), 0.0, 2.0)
        np.testing.assert_allclose(state, (2.0, -1.0, 0.0), atol=1e-12)
        np.testing.assert_allclose(velocity, (1.0, -0.5), atol=1e-12)

    def test_icp_recovers_known_relative_pose(self):
        reference = np.array([[-1.2, 2.0], [-0.2, 3.1], [0.8, 2.3], [1.5, 4.2],
                              [-1.7, 4.5], [0.4, 5.1]])
        truth = np.array([0.35, -0.18, math.radians(6.0)])
        c, s = math.cos(truth[2]), math.sin(truth[2])
        rotation = np.array([[c, -s], [s, c]])
        current = (reference - truth[:2]) @ rotation
        recovered, rmse, inliers = slam.icp_relative(reference, current,
                                                      initial=(0.3, -0.1, 0.05),
                                                      max_correspondence_m=0.8)
        np.testing.assert_allclose(recovered, truth, atol=1e-10)
        self.assertLess(rmse, 1e-10)
        self.assertEqual(inliers, len(reference))

    def test_loop_closure_reduces_drift_and_covariance_is_finite(self):
        graph = sonar.PlanarSlam()
        for _ in range(4):
            graph.add_odometry((1.02, 0.0, math.pi / 2 + 0.01), 0.08, 0.03)
        before = np.linalg.norm(graph.poses()[-1, :2])
        graph.add_loop_closure(0, 4, (0.0, 0.0, 0.0), 0.01, 0.005)
        graph.optimize()
        after = np.linalg.norm(graph.poses()[-1, :2])
        self.assertLess(after, before / 100.0)
        uncertainty = graph.uncertainties()
        self.assertTrue(np.all(np.isfinite(uncertainty)))
        self.assertTrue(np.all(uncertainty[1:] > 0.0))

    def test_descriptor_loop_candidate_uses_only_old_scans(self):
        first = np.array([1.0, 0.0, 0.0])
        descriptors = [first]
        descriptors.extend(np.roll(first, 1) for _ in range(12))
        descriptors.append(first.copy())
        index, similarity = slam.candidate_loop(descriptors, 13, minimum_separation=12)
        self.assertEqual(index, 0)
        self.assertAlmostEqual(similarity, 1.0)

    def test_rendered_sonar_loop_closes_without_truth_measurements(self):
        import slam_experiment

        result = slam_experiment.run_experiment()
        metrics = result["metrics"]
        self.assertGreaterEqual(metrics["loop_closure_count"], 1)
        self.assertLess(metrics["closure_after_m"], 0.01)
        self.assertLess(metrics["optimized_rmse_m"], metrics["graph_before_rmse_m"] / 4.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
