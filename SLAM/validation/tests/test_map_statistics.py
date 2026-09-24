"""Map point statistics used for culling (CHANGELOG entry 16), on the synthetic corridor.

    python -m unittest SLAM/validation/tests/test_map_statistics.py

* visibility is counted once per tracked frame (ORB-SLAM IncreaseVisible), not once per search attempt;
* the found ratio test culls points only while they are recent (ORB-SLAM MapPointCulling), not for ever.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE.parent / "datasets"), str(HERE)]

from monoslam import features as feat, geometry as geo  # noqa: E402
from monoslam.system import Settings  # noqa: E402
from test_relocalization import K, CorridorWorld, make_slam  # noqa: E402


def features_at(world, R_wc, c):
    Rcw, tcw = geo.invert(R_wc, c)
    uv, z = geo.project(K, Rcw, tcw, world.X)
    idx = np.flatnonzero((z > 0) & (uv[:, 0] > 0) & (uv[:, 0] < 640) & (uv[:, 1] > 0) & (uv[:, 1] < 480))
    uv = uv[idx] + world.rng.normal(0, 0.5, (len(idx), 2))
    desc = world.desc[idx].copy()
    desc[world.rng.random(desc.shape) < 0.02] ^= np.uint8(1)
    return feat.Features(uv, desc, np.ones(len(uv)), "orb")


def mapped_corridor(n=25):
    world = CorridorWorld()
    slam = make_slam(Settings())
    for i, x in enumerate(np.arange(n) * 0.4):
        slam.process(i, float(i), world.features(x))
    return world, slam


class TestVisibility(unittest.TestCase):
    def test_counted_once_per_frame_even_when_the_search_is_retried(self):
        world, slam = mapped_corridor()
        n = len(slam.frames)
        # A sudden 3 degree yaw the constant velocity model does not predict: about 26 px of image motion, so the
        # 15 px search fails and the 40 px search is needed.
        R_wc, c = CorridorWorld.pose(n * 0.4)
        yaw = geo.rotvec_to_matrix([0.0, np.radians(3.0), 0.0])[0]
        before = {pid: p.visible for pid, p in slam.points.items()}
        calls = []
        search = slam._search_by_projection
        slam._search_by_projection = lambda *a, **k: calls.append(1) or search(*a, **k)
        rec = slam.process(n, float(n), features_at(world, R_wc @ yaw, c))
        self.assertNotEqual(rec.status, "untracked")
        self.assertGreaterEqual(len(calls), 2, "the scenario must make tracking retry the search")
        increase = max(p.visible - before.get(pid, p.visible) for pid, p in slam.points.items())
        self.assertLessEqual(increase, 1)


class TestCulling(unittest.TestCase):
    def test_established_points_leave_the_found_ratio_test(self):
        world, slam = mapped_corridor()
        last = slam.kfs[-1]
        established = [p for p in slam.points.values()
                       if not p.bad and last.id - p.first_kf > slam.s.cull_after_kfs + 1 and len(p.obs) >= 4]
        recent = [p for p in slam.points.values() if not p.bad and last.id - p.first_kf == 1]
        self.assertTrue(established and recent)
        old, young = established[0], recent[0]
        for p in (old, young):
            p.visible, p.found = 1000, 1          # found in 0.1 % of the frames that should have seen it
        slam._local_mapping(last)
        self.assertFalse(old.bad, "a point past its probation was culled by the found ratio test")
        self.assertTrue(young.bad, "a recent point with a found ratio of 0.001 must still be culled")


if __name__ == "__main__":
    unittest.main()
