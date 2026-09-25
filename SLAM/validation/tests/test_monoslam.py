"""Unit and end to end tests for SLAM/validation/monoslam.

    python -m unittest discover -s SLAM/validation/tests -v
"""

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from monoslam import ba, export, features as feat, geometry as geo  # noqa: E402
from monoslam.system import MonoSLAM, Settings  # noqa: E402

K = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]])


def random_rotation(rng, scale=1.0):
    return geo.rotvec_to_matrix(rng.normal(size=3) * scale)[0]


def look_at(position, target):
    z = target - position
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))   # R_wc


class TestGeometry(unittest.TestCase):
    def test_left_jacobian_matches_finite_differences(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            w = rng.normal(size=3) * rng.uniform(0, 2.5)
            p = rng.normal(size=3)
            J = -geo.skew(geo.rotvec_to_matrix(w)[0] @ p) @ geo.left_jacobian_batch(w)[0]
            num = np.zeros((3, 3))
            for k in range(3):
                d = np.zeros(3)
                d[k] = 1e-6
                num[:, k] = (geo.rotvec_to_matrix(w + d)[0] @ p - geo.rotvec_to_matrix(w - d)[0] @ p) / 2e-6
            np.testing.assert_allclose(J, num, atol=1e-6)

    def test_triangulation_is_exact_without_noise(self):
        rng = np.random.default_rng(1)
        X = rng.uniform([-1, -1, 4], [1, 1, 8], (50, 3))
        R1, t1 = np.eye(3), np.zeros(3)
        R2, t2 = random_rotation(rng, 0.1), np.array([-0.5, 0.05, 0.1])
        uv1, _ = geo.project(K, R1, t1, X)
        uv2, _ = geo.project(K, R2, t2, X)
        np.testing.assert_allclose(geo.triangulate(K, R1, t1, R2, t2, uv1, uv2), X, atol=1e-8)

    def test_umeyama_recovers_similarity(self):
        rng = np.random.default_rng(2)
        src = rng.normal(size=(100, 3))
        R, s, t = random_rotation(rng), 3.7, np.array([1.0, -2.0, 0.5])
        dst = s * src @ R.T + t
        s2, R2, t2 = geo.umeyama(src, dst)
        self.assertAlmostEqual(s2, s, places=9)
        np.testing.assert_allclose(R2, R, atol=1e-9)
        np.testing.assert_allclose(t2, t, atol=1e-9)

    def test_epipolar_distance_zero_for_true_matches(self):
        rng = np.random.default_rng(3)
        X = rng.uniform([-1, -1, 4], [1, 1, 8], (30, 3))
        R2, t2 = random_rotation(rng, 0.1), np.array([0.4, 0.0, 0.1])
        uv1, _ = geo.project(K, np.eye(3), np.zeros(3), X)
        uv2, _ = geo.project(K, R2, t2, X)
        E = geo.essential_from_poses(np.eye(3), np.zeros(3), R2, t2)
        self.assertLess(geo.epipolar_sq_dist_px(K, E, uv1, uv2).max(), 1e-12)


class TestBundleAdjustment(unittest.TestCase):
    def test_recovers_perturbed_scene(self):
        rng = np.random.default_rng(4)
        X = rng.uniform([-2, -2, 5], [2, 2, 9], (150, 3))
        Rs = [np.eye(3)] + [random_rotation(rng, 0.05) for _ in range(4)]
        ts = [np.zeros(3)] + [np.array([0.3 * i, 0.05 * i, 0.0]) for i in range(1, 5)]
        oc, op, uv = [], [], []
        for c in range(5):
            p, _ = geo.project(K, Rs[c], ts[c], X)
            oc += [c] * len(X)
            op += list(range(len(X)))
            uv.append(p)
        oc, op, uv = np.array(oc), np.array(op), np.vstack(uv)
        w0 = geo.matrix_to_rotvec(np.array(Rs)) + rng.normal(0, 0.01, (5, 3))
        t0 = np.array(ts) + rng.normal(0, 0.02, (5, 3))
        X0 = X + rng.normal(0, 0.05, X.shape)
        fixed = np.array([True, True, False, False, False])   # two fixed cameras fix the gauge including scale
        w0[:2], t0[:2] = geo.matrix_to_rotvec(np.array(Rs[:2])), np.array(ts[:2])
        w, t, X1, chi2 = ba.bundle_adjust(K, w0, t0, X0, oc, op, uv, np.ones(len(oc)), fixed_cams=fixed,
                                          robust=False, max_nfev=100)
        self.assertLess(chi2.max(), 1e-8)
        np.testing.assert_allclose(X1, X, atol=1e-5)
        np.testing.assert_allclose(t, np.array(ts), atol=1e-6)

    def test_motion_only_rejects_nothing_on_clean_data(self):
        rng = np.random.default_rng(5)
        X = rng.uniform([-2, -2, 5], [2, 2, 9], (80, 3))
        R, t = random_rotation(rng, 0.1), np.array([0.2, -0.1, 0.3])
        uv, _ = geo.project(K, R, t, X)
        R1, t1, chi2 = ba.optimize_pose(K, R @ random_rotation(rng, 0.02), t + 0.05, X, uv, np.ones(len(X)))
        np.testing.assert_allclose(R1, R, atol=1e-7)
        np.testing.assert_allclose(t1, t, atol=1e-7)


class TestExportConvention(unittest.TestCase):
    """The exported trajectory is camera to world: inverting it must reproject known points correctly."""

    def test_tum_round_trip_reprojects(self):
        rng = np.random.default_rng(6)
        X = rng.uniform([-1, -1, -1], [1, 1, 1], (40, 3))
        stamps, R_wc, t_wc, pixels = [], [], [], []
        for i in range(10):
            c = np.array([4 * np.cos(0.3 * i), 4 * np.sin(0.3 * i), 1.0])
            Rwc = look_at(c, np.zeros(3))
            Rcw, tcw = geo.invert(Rwc, c)
            pixels.append(geo.project(K, Rcw, tcw, X)[0])
            stamps.append(i * 0.1)
            R_wc.append(Rwc)
            t_wc.append(c)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "traj.txt"
            export.write_tum(path, stamps, R_wc, t_wc)
            ts, R_read, t_read = export.read_tum(path)
        np.testing.assert_allclose(ts, stamps)
        for i in range(10):
            Rcw, tcw = geo.invert(R_read[i], t_read[i])
            uv, z = geo.project(K, Rcw, tcw, X)
            self.assertTrue((z > 0).all())
            np.testing.assert_allclose(uv, pixels[i], atol=1e-5)

    def test_quaternion_order_is_xyzw(self):
        R = geo.rotvec_to_matrix([0, 0, np.pi / 2])[0]   # 90 degrees about z
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "traj.txt"
            export.write_tum(path, [0.0], [R], [np.array([1.0, 2.0, 3.0])])
            row = np.loadtxt(path, comments="#")
        np.testing.assert_allclose(row[1:4], [1, 2, 3])
        np.testing.assert_allclose(row[4:8], [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)], atol=1e-9)

    def test_empty_trajectory_reads_quietly(self):
        """A run that posed no frame writes a header only trajectory; reading it gave a numpy 'input contained no
        data' warning in every scoring log, which reads like an error. The result was right and stays right."""
        import warnings
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "traj.txt"
            export.write_tum(path, [], [], [])
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                ts, R, t = export.read_tum(path)
        self.assertEqual([str(w.message) for w in caught], [])
        self.assertEqual((ts.shape, R.shape, t.shape), ((0,), (0, 3, 3), (0, 3)))

    def test_ply_round_trip(self):
        xyz = np.random.default_rng(7).normal(size=(20, 3))
        rgb = np.random.default_rng(8).integers(0, 255, (20, 3)).astype(np.uint8)
        with tempfile.TemporaryDirectory() as d:
            export.write_ply(Path(d) / "p.ply", xyz, rgb)
            x2, c2 = export.read_ply(Path(d) / "p.ply")
        np.testing.assert_allclose(x2, xyz, atol=1e-6)
        np.testing.assert_array_equal(c2, rgb)


class SyntheticFeatureWorld:
    """Point features with ORB-like descriptors seen by a moving camera; bypasses images entirely."""

    def __init__(self, n_frames=40, n_points=1500, seed=10, outlier_fraction=0.05, noise_px=0.5):
        rng = np.random.default_rng(seed)
        self.rng = rng
        # Points on a bumpy ground patch plus a mound: not planar.
        xy = rng.uniform(-3, 3, (n_points, 2))
        z = 0.5 * np.exp(-np.sum(xy ** 2, 1) / 1.5) + 0.1 * rng.normal(size=n_points)
        self.X = np.column_stack((xy, z))
        self.desc = rng.integers(0, 256, (n_points, 32), dtype=np.uint8)
        self.poses = []
        for i in range(n_frames):
            a = np.radians(-40 + 100 * i / (n_frames - 1))
            c = np.array([3.5 * np.cos(a), 3.5 * np.sin(a), 2.5])
            Rwc = look_at(c, np.zeros(3))
            self.poses.append((Rwc, c))
        self.noise_px, self.outlier_fraction = noise_px, outlier_fraction

    def features(self, i):
        Rwc, c = self.poses[i]
        Rcw, tcw = geo.invert(Rwc, c)
        uv, z = geo.project(K, Rcw, tcw, self.X)
        vis = (z > 0) & (uv[:, 0] > 0) & (uv[:, 0] < 640) & (uv[:, 1] > 0) & (uv[:, 1] < 480)
        idx = np.flatnonzero(vis)
        uv = uv[idx] + self.rng.normal(0, self.noise_px, (len(idx), 2))
        desc = self.desc[idx].copy()
        flip = self.rng.random(desc.shape) < 0.02          # a few bit flips per observation
        desc[flip] ^= np.uint8(1)
        n_out = int(self.outlier_fraction * len(idx))
        uv = np.vstack((uv, self.rng.uniform([0, 0], [640, 480], (n_out, 2))))
        desc = np.vstack((desc, self.rng.integers(0, 256, (n_out, 32), dtype=np.uint8)))
        return feat.Features(uv, desc, np.ones(len(uv)), "orb")


class TestEndToEnd(unittest.TestCase):
    def test_trajectory_and_map_recovered_up_to_similarity(self):
        world = SyntheticFeatureWorld()
        slam = MonoSLAM(K, (640, 480), Settings(), log=lambda *_: None)
        slam.extract = world.features   # feed features directly; the argument is the frame index
        for i in range(len(world.poses)):
            slam.process(i, float(i), i)
        slam.finish()
        traj = [p for p in slam.trajectory() if p["status"] != "untracked"]
        self.assertGreaterEqual(len(traj), len(world.poses) - 2)
        est = np.array([p["t_wc"] for p in traj])
        gt = np.array([world.poses[p["index"]][1] for p in traj])
        s, R, t = geo.umeyama(est, gt)
        aligned = s * est @ R.T + t
        ate = np.sqrt(np.mean(np.sum((aligned - gt) ** 2, 1)))
        self.assertLess(ate, 0.02, f"ATE {ate:.4f} m on a {np.ptp(gt, 0).max():.1f} m path")
        # Orientation: the aligned estimate must match ground truth rotation too.
        worst = 0.0
        for p in traj:
            dR = world.poses[p["index"]][0].T @ (R @ p["R_wc"])
            worst = max(worst, np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
        self.assertLess(worst, 1.0)   # sanity bound: a pose convention error gives tens of degrees
        # Map: every reconstructed point should lie near a true point after the same alignment.
        xyz, _, _ = slam.map_points()
        self.assertGreater(len(xyz), 300)
        mapped = s * xyz @ R.T + t
        from scipy.spatial import cKDTree
        d, _ = cKDTree(world.X).query(mapped)
        self.assertLess(np.median(d), 0.02)


if __name__ == "__main__":
    unittest.main()
