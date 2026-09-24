"""Synthetic renderer: the K written to K.txt must describe the rendered pixels in OpenCV's convention.

    python -m unittest SLAM/validation/tests/test_synthetic.py
"""

import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import synthetic  # noqa: E402
from monoslam import geometry as geo  # noqa: E402


class TestSyntheticIntrinsics(unittest.TestCase):
    def test_saved_K_projects_each_ray_to_its_own_pixel(self):
        w, h = 64, 48
        K = np.array([[50.0, 0, w / 2], [0, 50.0, h / 2], [0, 0, 1.0]])      # as main() builds it
        rays = synthetic.pixel_rays(K, (w, h))                               # what render() casts, row by row
        uv, _ = geo.project(synthetic.opencv_K(K), np.eye(3), np.zeros(3), rays)
        j, i = np.divmod(np.arange(w * h), w)                                # row, column of each ray
        np.testing.assert_allclose(uv, np.column_stack((i, j)), atol=1e-9)  # OpenCV: pixel (i, j) is at (i, j)


if __name__ == "__main__":
    unittest.main()
