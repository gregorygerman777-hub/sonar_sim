"""Relocalization after tracking loss (CHANGELOG entry 15), on a synthetic corridor with a known answer.

    python -m unittest SLAM/validation/tests/test_relocalization.py
    MONOSLAM_OLD_BEHAVIOUR=1 python -m unittest ...     (the behaviour before entry 15: both tests fail)

The camera flies 2 m above a 44 m strip of ground points, looking forward and down, so keyframes far apart see
disjoint ground. It is then blinded (frames without features) and brought back over ground it mapped before.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import run_slam  # noqa: E402
from monoslam import features as feat, geometry as geo  # noqa: E402
from monoslam.system import MonoSLAM, Settings  # noqa: E402

OLD = os.environ.get("MONOSLAM_OLD_BEHAVIOUR") == "1"
K = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]])


def look_at(position, target):
    z = target - position
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))   # R_wc


class CorridorWorld:
    def __init__(self, n_points=5000, seed=21):
        rng = np.random.default_rng(seed)
        self.rng = np.random.default_rng(seed + 1)
        self.X = np.column_stack((rng.uniform(-2, 42, n_points), rng.uniform(-3, 3, n_points),
                                  0.3 * rng.random(n_points) ** 3))          # a floor with some low relief
        self.desc = rng.integers(0, 256, (n_points, 32), dtype=np.uint8)

    @staticmethod
    def pose(x):
        c = np.array([x, 0.0, 2.0])
        return look_at(c, np.array([x + 3.0, 0.0, 0.0])), c

    def features(self, x):
        Rwc, c = self.pose(x)
        Rcw, tcw = geo.invert(Rwc, c)
        uv, z = geo.project(K, Rcw, tcw, self.X)
        idx = np.flatnonzero((z > 0) & (uv[:, 0] > 0) & (uv[:, 0] < 640) & (uv[:, 1] > 0) & (uv[:, 1] < 480))
        uv = uv[idx] + self.rng.normal(0, 0.5, (len(idx), 2))
        desc = self.desc[idx].copy()
        desc[self.rng.random(desc.shape) < 0.02] ^= np.uint8(1)
        return feat.Features(uv, desc, np.ones(len(uv)), "orb")

    @staticmethod
    def blind():
        return feat.Features(np.empty((0, 2)), np.empty((0, 32), np.uint8), np.empty(0), "orb")


def settings(**kw):
    if OLD:
        kw.update(reloc_place_candidates=0, reopen_maps=False)
    return Settings(**kw)


def make_slam(s):
    slam = MonoSLAM(K, (640, 480), s, log=lambda *_: None)
    slam.extract = lambda frame: frame          # frames are handed over as features
    return slam


class TestPlaceRecognition(unittest.TestCase):
    def test_relocalizes_over_ground_mapped_long_before(self):
        world = CorridorWorld()
        slam = make_slam(settings(reloc_candidates=5))
        xs = np.arange(0.0, 30.0, 0.4)
        for i, x in enumerate(xs):
            slam.process(i, float(i), world.features(x))
        n = len(xs)
        for j in range(3):
            slam.process(n + j, float(n + j), world.blind())
        # Back over the start of the corridor, which only keyframes much older than the 5 most recent have seen.
        rec = slam.process(n + 3, float(n + 3), world.features(1.0))
        self.assertNotEqual(rec.status, "untracked")
        # The relocalized pose must agree with the truth once the run is aligned by a similarity.
        slam.finish()
        traj = [p for p in slam.trajectory() if p["status"] != "untracked"]
        truth = {i: CorridorWorld.pose(x)[1] for i, x in enumerate(xs)}
        truth[n + 3] = CorridorWorld.pose(1.0)[1]
        est = np.array([p["t_wc"] for p in traj])
        gt = np.array([truth[p["index"]] for p in traj])
        s, R, t = geo.umeyama(est, gt)
        err = np.linalg.norm(s * est @ R.T + t - gt, axis=1)
        self.assertLess(err[[p["index"] for p in traj].index(n + 3)], 0.1)


class TestReopenMap(unittest.TestCase):
    def test_camera_back_in_a_closed_map_reopens_it(self):
        world = CorridorWorld()
        s = settings(new_map_after_lost=10)
        frames = [world.features(x) for x in np.arange(0.0, 20.0, 0.4)]
        frames += [world.blind()] * 12                                        # the map is closed after 10
        frames += [world.features(x) for x in np.arange(17.0, 22.0, 0.4)]    # back over the last mapped ground
        maps = run_slam.track(len(frames), lambda i: (frames[i], None), np.arange(len(frames), dtype=float),
                              lambda: make_slam(s), s, log=lambda *_: None)
        live = [m for m, _ in maps if m.initialized]
        self.assertEqual(len(live), 1, "the camera came back to the first map, so no second map should exist")
        posed = {r.index for r in live[0].frames if r.status != "untracked"}
        self.assertTrue(set(range(62, len(frames))) <= posed, sorted(set(range(62, len(frames))) - posed))


if __name__ == "__main__":
    unittest.main()
