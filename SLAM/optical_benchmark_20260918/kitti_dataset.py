"""KITTI odometry dataset access: calibration, timestamps, image paths, ground truth."""

from pathlib import Path

import cv2
import numpy as np

GROUND_TRUTH_SEQUENCES = [f"{i:02d}" for i in range(11)]


def load_calibration(sequence_dir):
    """Projection matrices P0..P3 (3x4), the left intrinsics K and the stereo baseline in metres."""
    matrices = {}
    for line in (Path(sequence_dir) / "calib.txt").read_text().splitlines():
        if ":" not in line:
            continue
        name, values = line.split(":", 1)
        matrices[name.strip()] = np.array(values.split(), dtype=float).reshape(3, 4)
    intrinsics = matrices["P0"][:, :3]
    # Camera 1 is displaced along x by -P1[0,3] / fx relative to camera 0.
    baseline_m = float(-matrices["P1"][0, 3] / matrices["P1"][0, 0])
    return dict(P=matrices, K=intrinsics, baseline_m=baseline_m)


def load_times(sequence_dir):
    return np.loadtxt(Path(sequence_dir) / "times.txt", dtype=float)


def load_ground_truth(poses_dir, sequence):
    rows = np.loadtxt(Path(poses_dir) / f"{sequence}.txt", dtype=float)
    poses = np.tile(np.eye(4), (len(rows), 1, 1))
    poses[:, :3, :] = rows.reshape(len(rows), 3, 4)
    return poses


def frame_paths(sequence_dir, camera):
    return sorted((Path(sequence_dir) / f"image_{camera}").glob("*.png"))


def read_gray(path):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    return image
