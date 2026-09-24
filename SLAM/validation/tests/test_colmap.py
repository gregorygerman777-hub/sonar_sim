"""COLMAP's pixel convention, measured, and the conversion run_colmap.py applies (CHANGELOG entry 17).

    python -m unittest SLAM/validation/tests/test_colmap.py
"""

import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]


def colmap_keypoint_near(image, x0, y0):
    import pycolmap
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        (d / "images").mkdir()
        cv2.imwrite(str(d / "images" / "a.png"), image)
        options = pycolmap.FeatureExtractionOptions()
        options.num_threads = 1
        pycolmap.extract_features(d / "db.db", d / "images", extraction_options=options, device=pycolmap.Device.cpu)
        con = sqlite3.connect(d / "db.db")
        rows, cols, data = con.execute("select rows, cols, data from keypoints").fetchone()
        con.close()
    kp = np.frombuffer(data, np.float32).reshape(rows, cols)[:, :2]
    return kp[np.argmin(np.linalg.norm(kp - [x0, y0], axis=1))]


class TestColmapConvention(unittest.TestCase):
    def test_colmap_keypoints_are_half_a_pixel_from_opencv(self):
        """A symmetric blob centred on pixel (100, 60) in OpenCV's convention (pixel centres at integers) is found by
        COLMAP at (100.5, 60.5): COLMAP puts pixel centres at half integers."""
        y, x = np.mgrid[0:160, 0:240].astype(float)
        image = np.clip(40 + 180 * np.exp(-((x - 100) ** 2 + (y - 60) ** 2) / (2 * 3.0 ** 2)), 0, 255).astype(np.uint8)
        np.testing.assert_allclose(colmap_keypoint_near(image, 100, 60), [100.5, 60.5], atol=0.05)

    def test_principal_point_is_converted(self):
        import run_colmap
        K = np.array([[1403.5, 0, 476.5], [0, 1403.5, 392.9], [0, 0, 1.0]])
        np.testing.assert_allclose(run_colmap.colmap_pinhole_params(K), [1403.5, 1403.5, 477.0, 393.4])


if __name__ == "__main__":
    unittest.main()
