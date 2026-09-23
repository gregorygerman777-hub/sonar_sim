"""Standard outputs: TUM trajectory (camera to world), PLY point cloud, run metadata."""

import json
import subprocess
from pathlib import Path

import numpy as np

from . import geometry as geo


def write_tum(path, timestamps, R_wc, t_wc):
    """One line per pose: timestamp tx ty tz qx qy qz qw, camera to world, quaternion scalar last."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        fh.write("# timestamp tx ty tz qx qy qz qw  (camera to world, OpenCV camera axes)\n")
        for ts, R, t in zip(timestamps, R_wc, t_wc):
            q = geo.quaternion_xyzw(R)
            fh.write(f"{ts:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


def read_tum(path):
    """Returns timestamps (N,), R_wc (N,3,3), t_wc (N,3)."""
    rows = np.loadtxt(path, comments="#", ndmin=2)
    if rows.size == 0:
        return np.empty(0), np.empty((0, 3, 3)), np.empty((0, 3))
    from scipy.spatial.transform import Rotation
    return rows[:, 0], Rotation.from_quat(rows[:, 4:8]).as_matrix(), rows[:, 1:4]


def write_ply(path, xyz, rgb=None):
    """ASCII PLY with float xyz and optional uchar rgb."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(xyz, float)
    with open(path, "w") as fh:
        fh.write("ply\nformat ascii 1.0\n")
        fh.write(f"element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\n")
        if rgb is not None:
            fh.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        fh.write("end_header\n")
        for i, p in enumerate(xyz):
            if rgb is not None:
                c = rgb[i]
                fh.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
            else:
                fh.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def read_ply(path):
    lines = Path(path).read_text().splitlines()
    n = int(next(l for l in lines if l.startswith("element vertex")).split()[-1])
    start = lines.index("end_header") + 1
    data = np.array([l.split() for l in lines[start:start + n]], dtype=float).reshape(n, -1)
    xyz = data[:, :3]
    rgb = data[:, 3:6].astype(np.uint8) if data.shape[1] >= 6 else None
    return xyz, rgb


def git_commit(root):
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip())
        return commit, dirty
    except Exception:  # noqa: BLE001
        return None, None


def write_meta(path, **meta):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(meta, indent=2, default=_json_default))


def _json_default(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(type(o))
