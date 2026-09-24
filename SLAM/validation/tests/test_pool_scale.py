"""pool_scale.py: stripe width and spacing recovered from rendered floor views with known geometry.

    python -m unittest SLAM/validation/tests/test_pool_scale.py
"""

import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import pool_scale  # noqa: E402

K = np.array([[1403.5, 0, 476.5], [0, 1403.5, 392.9], [0, 0, 1.0]])
WIDTH, CENTRES, HEIGHT = 0.30, (-0.7, 0.8), 1.0


def floor(x, y):
    """Grey floor: bright tiles, two dark stripes along y, and a dark round 'rock' of diameter 0.4 at (0.05, 0.3)."""
    g = np.full(x.shape, 200.0)
    for c in CENTRES:
        g[np.abs(x - c) <= WIDTH / 2] = 50.0
    g[(x - 0.05) ** 2 + (y - 0.3) ** 2 <= 0.2 ** 2] = 40.0
    return g


def render(R_wc, c):
    """Pinhole view of the plane z = 0 from a camera at c (world z up)."""
    v, u = np.mgrid[0:768, 0:1024].astype(float)
    rays = np.stack([u, v, np.ones_like(u)], -1) @ np.linalg.inv(K).T @ R_wc.T
    s = -c[2] / rays[..., 2]
    img = floor(c[0] + s * rays[..., 0], c[1] + s * rays[..., 1])
    img[~(s > 0)] = 0
    return np.clip(img, 0, 255).astype(np.uint8)


def look_at(c, target):
    z = target - c
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1), c


class TestStripeMeasurement(unittest.TestCase):
    def test_width_and_spacing_recovered_rock_rejected(self):
        poses = [look_at(np.array([2.2 * np.cos(a), 2.2 * np.sin(a), HEIGHT]), np.array([0.0, 0, 0]))
                 for a in np.linspace(0, 2 * np.pi, 12, endpoint=False)]
        frames = [render(R, c) for R, c in poses]
        dips, across = pool_scale.measure(frames, K, poses, np.zeros(3), np.array([0, 0, 1.0]), np.zeros(3), HEIGHT)
        stripes = [g for g in pool_scale.cluster(dips, gap=pool_scale.MIN_STRIPE_WIDTH * HEIGHT)
                   if len(set(g[:, 0].astype(int))) >= 3]
        self.assertEqual(len(stripes), 2, [np.median(g[:, 1]) for g in stripes])
        for g in stripes:
            self.assertAlmostEqual(np.median(g[:, 2]), WIDTH, delta=0.03 * WIDTH)
        spacing = abs(np.median(stripes[1][:, 1]) - np.median(stripes[0][:, 1]))
        self.assertAlmostEqual(spacing, CENTRES[1] - CENTRES[0], delta=0.02)


class TestElongation(unittest.TestCase):
    """The pool run reported two rocks on the mat as stripes: a rock covering most of the rows a frame sees in its
    columns pulls the median profile down just like a stripe. The elongation check must reject it."""

    def setUp(self):
        self.coords = np.arange(-1, 1, 0.01)
        self.rect = np.full((100, len(self.coords)), 200.0)
        self.rect[:, 150:] = np.nan                          # part of the floor not seen in this frame

    def test_short_blob_that_makes_the_profile_dip_is_rejected(self):
        self.rect[40:100, 90:120] = 40.0                     # 60 % of the seen rows: the column median dips
        prof = np.nanmedian(self.rect, 0)
        dips = pool_scale.stripe_dips(prof, self.coords, min_width=0.2)
        self.assertEqual(len(dips), 1)
        c, w, depth, dark = dips[0]
        self.assertFalse(pool_scale.is_elongated(self.rect, self.coords, c, w, dark + depth / 2))

    def test_full_length_stripe_is_kept(self):
        self.rect[:, 90:120] = 40.0
        c, w, depth, dark = pool_scale.stripe_dips(np.nanmedian(self.rect, 0), self.coords, min_width=0.2)[0]
        self.assertTrue(pool_scale.is_elongated(self.rect, self.coords, c, w, dark + depth / 2))


if __name__ == "__main__":
    unittest.main()
