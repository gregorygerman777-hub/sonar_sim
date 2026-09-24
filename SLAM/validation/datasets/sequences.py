"""Dataset adapters. Every adapter returns a Sequence with undistorted pinhole images.

A Sequence exposes
    name, K (3x3 of the undistorted images), size (width, height),
    timestamps (N,), load(i) -> (gray uint8, color BGR uint8 or None),
    ground_truth: (timestamps, R_wc, t_wc) camera to world in the dataset's world frame, or None.
Ground truth is only read by the evaluation code, never by the SLAM run.
"""

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DATA = ROOT / "data_external"


@dataclass
class Sequence:
    name: str
    K: np.ndarray
    size: tuple
    timestamps: np.ndarray
    paths: list
    ground_truth: Optional[tuple] = None
    remap: Optional[tuple] = None          # (map1, map2) for undistortion, or None
    notes: dict = field(default_factory=dict)

    def __len__(self):
        return len(self.paths)

    def load(self, i):
        image = cv2.imread(str(self.paths[i]), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise FileNotFoundError(self.paths[i])
        if image.dtype != np.uint8:
            image = cv2.convertScaleAbs(image, alpha=255.0 / max(int(image.max()), 1))
        if self.remap is not None:
            image = cv2.remap(image, self.remap[0], self.remap[1], cv2.INTER_LINEAR)
        if image.ndim == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), image
        return image, None

    def subsample(self, stride):
        if stride <= 1:
            return self
        return Sequence(self.name, self.K, self.size, self.timestamps[::stride], self.paths[::stride],
                        self.ground_truth, self.remap, dict(self.notes, stride=stride))


# ---------------------------------------------------------------------- TUM RGB-D (monocular use of the RGB stream)
TUM_INTRINSICS = {
    # fx fy cx cy, then k1 k2 p1 p2 k3 (OpenCV order). Published on the TUM RGB-D "file formats" page.
    "freiburg1": ([517.3, 516.5, 318.6, 255.3], [0.2624, -0.9531, -0.0054, 0.0026, 1.1633]),
    "freiburg2": ([520.9, 521.0, 325.1, 249.7], [0.2312, -0.7849, -0.0033, -0.0001, 0.9172]),
    "freiburg3": ([535.4, 539.2, 320.1, 247.6], [0.0, 0.0, 0.0, 0.0, 0.0]),
}


def tum_rgbd(name, root=DATA / "tum"):
    folder = Path(root) / name
    camera = next(k for k in TUM_INTRINSICS if k in name)
    (fx, fy, cx, cy), dist = TUM_INTRINSICS[camera]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    stamps, paths = [], []
    for line in (folder / "rgb.txt").read_text().splitlines():
        if line.startswith("#") or not line.strip():
            continue
        ts, rel = line.split()[:2]
        stamps.append(float(ts))
        paths.append(folder / rel)
    width, height = 640, 480
    remap = None
    if any(dist):
        remap = cv2.initUndistortRectifyMap(K, np.array(dist), None, K, (width, height), cv2.CV_32FC1)
    gt = np.loadtxt(folder / "groundtruth.txt", comments="#")
    ground_truth = (gt[:, 0], Rotation.from_quat(gt[:, 4:8]).as_matrix(), gt[:, 1:4])
    return Sequence(f"tum_{name.replace('rgbd_dataset_', '')}", K, (width, height), np.array(stamps), paths,
                    ground_truth, remap, dict(source="TUM RGB-D, RGB stream only", undistorted=bool(remap)))


# ---------------------------------------------------------------------- EuRoC MAV (ASL folder layout)
def euroc(name, root=DATA / "euroc"):
    import yaml
    cam = Path(root) / name / "mav0" / "cam0"
    sensor = yaml.safe_load((cam / "sensor.yaml").read_text())
    fx, fy, cx, cy = sensor["intrinsics"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    width, height = sensor["resolution"]
    dist = np.array(sensor["distortion_coefficients"])
    remap = cv2.initUndistortRectifyMap(K, dist, None, K, (width, height), cv2.CV_32FC1)
    T_BS = np.array(sensor["T_BS"]["data"]).reshape(4, 4)
    rows = list(csv.reader(open(cam / "data.csv")))[1:]
    stamps = np.array([int(r[0]) * 1e-9 for r in rows])
    paths = [cam / "data" / r[1].strip() for r in rows]
    gt = np.loadtxt(Path(root) / name / "mav0/state_groundtruth_estimate0/data.csv", delimiter=",", comments="#")
    R_WB = Rotation.from_quat(gt[:, [5, 6, 7, 4]]).as_matrix()   # file stores qw qx qy qz
    p_WB = gt[:, 1:4]
    R_WC = R_WB @ T_BS[:3, :3]
    t_WC = p_WB + R_WB @ T_BS[:3, 3]
    return Sequence(f"euroc_{name}", K, (width, height), stamps, paths, (gt[:, 0] * 1e-9, R_WC, t_WC), remap,
                    dict(source="EuRoC MAV cam0", undistorted=True))


# ---------------------------------------------------------------------- AQUALOC harbor sequences
def aqualoc_harbor(seq=7, root=DATA / "aqualoc"):
    import yaml
    root = Path(root)
    calib = yaml.safe_load((root / "harbor_camera_calib.yaml").read_text())["cam0"]
    fx, fy, cx, cy = calib["intrinsics"]
    K_raw = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    D = np.array(calib["distortion_coeffs"]).reshape(4, 1)
    width, height = calib["resolution"]
    assert calib["distortion_model"] == "equidistant"
    # Keep the focal length and principal point: a pinhole view with the same K, fisheye distortion removed.
    K = K_raw.copy()
    remap = cv2.fisheye.initUndistortRectifyMap(K_raw, D, np.eye(3), K, (width, height), cv2.CV_32FC1)
    folder = root / "raw_data" / f"harbor_images_sequence_{seq:02d}"
    paths = sorted(folder.glob("frame*.png"))
    index = np.array([int(p.stem[5:]) for p in paths], float)
    gt = np.loadtxt(root / f"new_harbor_colmap_traj_sequence_{seq:02d}.txt", comments="#")
    ground_truth = (gt[:, 0], Rotation.from_quat(gt[:, 4:8]).as_matrix(), gt[:, 1:4])
    return Sequence(f"aqualoc_harbor_{seq:02d}", K, (width, height), index, paths, ground_truth, remap,
                    dict(source="AQUALOC harbor, COLMAP ground truth (new_ files, 2021 rescale)",
                         timestamp="image number", undistorted=True))


# ---------------------------------------------------------------------- KITTI odometry (left grayscale camera only)
def kitti(seq=0, root=None):
    """KITTI odometry sequence <seq>, used monocularly: camera 0 (left, grayscale, already rectified).
    Ground truth (sequences 00 to 10) is poses/<seq>.txt, one row major 3 x 4 T_w_cam0 per frame, the world being
    the first camera frame. Dr. Negahdaripour named KITTI as the reference dataset with ground truth."""
    import os
    root = Path(root or os.environ.get("KITTI_DIR", DATA / "kitti_odometry" / "dataset"))
    folder = root / "sequences" / f"{seq:02d}"
    calib = {}
    for line in (folder / "calib.txt").read_text().splitlines():
        if ":" in line:
            key, values = line.split(":", 1)
            calib[key.strip()] = np.array(values.split(), float).reshape(3, 4)
    K = calib["P0"][:, :3].copy()
    stamps = np.loadtxt(folder / "times.txt", ndmin=1)
    paths = sorted((folder / "image_0").glob("*.png"))
    image = cv2.imread(str(paths[0]), cv2.IMREAD_GRAYSCALE)
    ground_truth = None
    poses = root / "poses" / f"{seq:02d}.txt"
    if poses.exists():
        T = np.loadtxt(poses, ndmin=2).reshape(-1, 3, 4)
        ground_truth = (stamps[:len(T)], T[:, :, :3], T[:, :, 3])
    return Sequence(f"kitti_{seq:02d}", K, (image.shape[1], image.shape[0]), stamps, paths, ground_truth, None,
                    dict(source="KITTI odometry, camera 0 (left grayscale), rectified; GPS/INS ground truth",
                         undistorted=False))


# ---------------------------------------------------------------------- synthetic (rendered by synthetic.py)
def synthetic(name="synthetic_pool", root=DATA / "synthetic"):
    folder = Path(root) / name
    K = np.loadtxt(folder / "K.txt")
    paths = sorted((folder / "images").glob("*.png"))
    gt = np.loadtxt(folder / "groundtruth_tum.txt", comments="#")
    image = cv2.imread(str(paths[0]))
    return Sequence(name, K, (image.shape[1], image.shape[0]), gt[:, 0], paths,
                    (gt[:, 0], Rotation.from_quat(gt[:, 4:8]).as_matrix(), gt[:, 1:4]), None,
                    dict(source="rendered by datasets/synthetic.py", undistorted=False))


# ---------------------------------------------------------------------- Dr. Negahdaripour's pool frames (no ground truth)
POOL_K_RAW = np.array([[1403.46109, 0, 476.5168], [0, 1403.46109, 392.915659], [0, 0, 1.0]])
# Dr. Negahdaripour confirmed on 2026-09-23 that K = [1403.5 0 476.5; 0 1403.5 392.9; 0 0 1], the raw K, is the
# camera matrix for these frames, so "raw" is the primary pool configuration. "width_scaled" stays as a comparison.
POOL_PRIMARY = "raw"


def pool(root=None, calibration=None, k_hypothesis=POOL_PRIMARY):
    import os
    import scipy.io
    root = Path(root or os.environ.get("POOL_DIR", Path.home() / "Downloads/Archive 2"))
    calibration = Path(calibration or os.environ.get("POOL_CALIB", Path.home() / "Downloads/OSCalibration.mat"))
    if calibration.exists():
        K_raw = scipy.io.loadmat(str(calibration))["K"].astype(float)
        k_source = str(calibration.name)
    else:  # the same matrix, as quoted in SLAM/pool_consistency_20260919 (for machines without the .mat)
        K_raw = POOL_K_RAW.copy()
        k_source = "POOL_K_RAW constant (OSCalibration.mat not found)"
    paths = [Path(root) / f"opt{i}.bmp" for i in range(1, 118)]
    image = cv2.imread(str(paths[0]))
    height, width = image.shape[:2]
    # The calibration implies a (2 cx, 2 cy) = (953, 786) image; the frames are 1024 x 768.
    if k_hypothesis == "width_scaled":
        K = K_raw.copy()
        K[0, 0] *= width / (2 * K_raw[0, 2])
        K[0, 2] = width / 2.0
    elif k_hypothesis == "raw":
        K = K_raw
    else:
        raise ValueError(k_hypothesis)
    return Sequence(f"pool_{k_hypothesis}", K, (width, height), np.arange(1, 118, dtype=float), paths, None, None,
                    dict(source="opt1.bmp..opt117.bmp and OSCalibration.mat from Dr. Negahdaripour",
                         K_raw=K_raw.tolist(), K_source=k_source, k_hypothesis=k_hypothesis,
                         timestamp="frame number"))


def get(name, **kw):
    if name.startswith("tum_"):
        return tum_rgbd("rgbd_dataset_" + name[4:], **kw)
    if name.startswith("euroc_"):
        return euroc(name[6:], **kw)
    if name.startswith("aqualoc_harbor_"):
        return aqualoc_harbor(int(name.rsplit("_", 1)[1]), **kw)
    if name.startswith("kitti_"):
        return kitti(int(name[6:]), **kw)
    if name.startswith("synthetic"):
        return synthetic(name, **kw)
    if name.startswith("pool"):
        return pool(k_hypothesis=name[5:] or POOL_PRIMARY, **kw)
    raise KeyError(name)
