"""The front end returns keypoints in OpenCV's pixel convention: the centre of the top left pixel at (0, 0), as K is.

    python -m unittest SLAM/validation/tests/test_features.py
"""

import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import features  # noqa: E402


def blob_offsets(extractor, n=30, seed=0):
    """Detected minus true position of Gaussian blobs at random sub pixel centres."""
    rng = np.random.default_rng(seed)
    h, w = 240, 320
    yy, xx = np.mgrid[0:h, 0:w].astype(float)
    out = []
    for _ in range(n):
        cx, cy, s = rng.uniform(60, 260), rng.uniform(60, 180), rng.uniform(2.5, 5.0)
        img = np.clip(40 + 180 * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * s * s)), 0, 255).astype(np.uint8)
        f = extractor(img)
        if len(f.uv):
            k = int(np.argmin(np.linalg.norm(f.uv - [cx, cy], axis=1)))
            out.append(f.uv[k] - [cx, cy])
    return np.array(out)


class TestSiftKeypointPosition(unittest.TestCase):
    """OpenCV's SIFT reports each keypoint 0.25 px right of and below its position (it detects on the image upsampled
    by 2 and halves the coordinates without the half pixel shift of the pixel centre convention). Measured here on
    blobs at known centres; the SIFT front end must return the true position (CHANGELOG entry 26)."""

    def test_blob_centres_are_found_where_they_are(self):
        e = blob_offsets(features.Extractor("sift", n_features=50))
        self.assertGreater(len(e), 20)
        np.testing.assert_allclose(e.mean(0), [0.0, 0.0], atol=0.05)
        self.assertLess(e.std(0).max(), 0.05)


if __name__ == "__main__":
    unittest.main()
