"""Feature extraction and descriptor matching for the ORB and SIFT front ends.

ORB uses the same detector parameters as SLAM/optical_benchmark_20260918/stereo_vo.py
(scale 1.2, 8 levels, FAST threshold 12) and the same strongest per cell bucketing, but
keeps each keypoint's pyramid level so measurement noise can scale with it.
"""

from dataclasses import dataclass

import cv2
import numpy as np

_POPCOUNT = np.array([bin(i).count("1") for i in range(256)], dtype=np.uint8)


@dataclass
class Features:
    uv: np.ndarray          # (N,2) float pixels
    desc: np.ndarray        # (N,32) uint8 for ORB, (N,128) float32 for SIFT
    sigma: np.ndarray       # (N,) pixel standard deviation of the keypoint position
    kind: str


class Extractor:
    def __init__(self, kind="orb", n_features=2000, grid=(8, 6)):
        self.kind = kind
        self.n_features = n_features
        self.grid = grid
        if kind == "orb":
            self.detector = cv2.ORB_create(nfeatures=2 * n_features, scaleFactor=1.2, nlevels=8, fastThreshold=12)
        elif kind == "sift":
            self.detector = cv2.SIFT_create(nfeatures=2 * n_features)
        else:
            raise ValueError(kind)

    def __call__(self, gray, mask=None):
        kps, desc = self.detector.detectAndCompute(gray, mask)
        if desc is None or len(kps) == 0:
            width = 32 if self.kind == "orb" else 128
            dtype = np.uint8 if self.kind == "orb" else np.float32
            return Features(np.empty((0, 2)), np.empty((0, width), dtype), np.empty(0), self.kind)
        keep = self._bucket(kps, gray.shape)
        uv = np.array([kps[i].pt for i in keep], dtype=float)
        if self.kind == "orb":
            sigma = 1.2 ** np.array([kps[i].octave for i in keep], dtype=float)
        else:
            octave = np.array([kps[i].octave & 255 for i in keep])
            octave = np.where(octave >= 128, octave - 256, octave)
            sigma = 2.0 ** np.maximum(octave, 0).astype(float)
        return Features(uv, desc[keep], sigma, self.kind)

    def _bucket(self, kps, shape):
        """Strongest keypoints per grid cell so that texture rich regions do not take every slot."""
        cols, rows = self.grid
        per_cell = max(1, int(np.ceil(self.n_features / (cols * rows))))
        cell_w, cell_h = shape[1] / cols, shape[0] / rows
        order = np.argsort([-k.response for k in kps])
        counts = {}
        keep = []
        for i in order:
            x, y = kps[i].pt
            cell = (int(x // cell_w), int(y // cell_h))
            if counts.get(cell, 0) >= per_cell:
                continue
            counts[cell] = counts.get(cell, 0) + 1
            keep.append(i)
        return np.array(sorted(keep), dtype=int)


def distance_matrix(da, db, kind):
    """Pairwise descriptor distances (Hamming for ORB, L2 for SIFT), shape (len(da), len(db))."""
    if len(da) == 0 or len(db) == 0:
        return np.zeros((len(da), len(db)))
    if kind == "orb":
        x = np.bitwise_xor(da[:, None, :], db[None, :, :])
        return _POPCOUNT[x].sum(axis=2).astype(float)
    a2 = np.sum(da.astype(np.float64) ** 2, axis=1)[:, None]
    b2 = np.sum(db.astype(np.float64) ** 2, axis=1)[None, :]
    return np.sqrt(np.maximum(a2 + b2 - 2.0 * da.astype(np.float64) @ db.astype(np.float64).T, 0.0))


def paired_distance(da, db, kind):
    """Distances between corresponding rows."""
    if kind == "orb":
        return _POPCOUNT[np.bitwise_xor(da, db)].sum(axis=1).astype(float)
    return np.linalg.norm(da.astype(np.float64) - db.astype(np.float64), axis=1)


# Absolute acceptance thresholds on descriptor distance (ORB-SLAM uses 50 and 100 for ORB).
STRICT = {"orb": 50.0, "sift": 200.0}
LOOSE = {"orb": 100.0, "sift": 300.0}


def ratio_match(fa, fb, ratio=0.8, mutual=True, max_dist=None):
    """Brute force knn matching with Lowe's ratio test and optional mutual consistency. Returns (K,2) indices."""
    if len(fa.desc) < 2 or len(fb.desc) < 2:
        return np.empty((0, 2), int)
    norm = cv2.NORM_HAMMING if fa.kind == "orb" else cv2.NORM_L2
    matcher = cv2.BFMatcher(norm)
    pairs = matcher.knnMatch(fa.desc, fb.desc, k=2)
    max_dist = LOOSE[fa.kind] if max_dist is None else max_dist
    ab = {}
    for p in pairs:
        if len(p) == 2 and p[0].distance < ratio * p[1].distance and p[0].distance < max_dist:
            ab[p[0].queryIdx] = p[0].trainIdx
    if mutual and ab:
        back = matcher.match(fb.desc, fa.desc)
        ba = {m.queryIdx: m.trainIdx for m in back}
        ab = {a: b for a, b in ab.items() if ba.get(b) == a}
    return np.array(sorted(ab.items()), dtype=int).reshape(-1, 2)
