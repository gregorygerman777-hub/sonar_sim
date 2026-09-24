"""export_pool_deliverable.py: the change to the floor frame must not change any projection.

    python -m unittest SLAM/validation/tests/test_export.py
"""

import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import export_pool_deliverable as deliver  # noqa: E402
from monoslam import geometry as geo  # noqa: E402

K = np.array([[1403.5, 0, 476.5], [0, 1403.5, 392.9], [0, 0, 1.0]])


class TestFloorFrame(unittest.TestCase):
    def test_similarity_change_keeps_every_projection(self):
        rng = np.random.default_rng(3)
        n_cams, n_pts = 7, 200
        R_cw = geo.rotvec_to_matrix(rng.normal(0, 1.0, (n_cams, 3)))
        t_cw = rng.normal(0, 2.0, (n_cams, 3))
        X = rng.normal(0, 3.0, (n_pts, 3))
        B = geo.rotvec_to_matrix(rng.normal(0, 1.0, 3))[0]          # rows of a right handed frame
        origin, scale = rng.normal(0, 5.0, 3), 0.37
        R2, t2, X2 = deliver.to_frame(R_cw, t_cw, X, B, origin, scale)
        for i in range(n_cams):
            uv1, z1 = geo.project(K, R_cw[i], t_cw[i], X)
            uv2, z2 = geo.project(K, R2[i], t2[i], X2)
            np.testing.assert_allclose(uv2, uv1, rtol=1e-9, atol=1e-6)
            np.testing.assert_allclose(z2, scale * z1, rtol=1e-9, atol=1e-9)
            np.testing.assert_allclose(R2[i] @ R2[i].T, np.eye(3), atol=1e-12)
            C1 = -R_cw[i].T @ t_cw[i]
            C2 = -R2[i].T @ t2[i]
            np.testing.assert_allclose(C2, scale * B @ (C1 - origin), atol=1e-9)
        np.testing.assert_allclose(X2, scale * (X - origin) @ B.T)


if __name__ == "__main__":
    unittest.main()
