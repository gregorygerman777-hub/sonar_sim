"""Standard outputs: TUM trajectory (camera to world), PLY point cloud, run metadata."""

import json
import subprocess
import warnings
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
    """Returns timestamps (N,), R_wc (N,3,3), t_wc (N,3); empty arrays for a run that posed no frame."""
    with warnings.catch_warnings():     # a header only file is a valid empty trajectory, not a problem to report
        warnings.filterwarnings("ignore", message="loadtxt: input contained no data")
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
    """xyz (N, 3) and rgb (N, 3) uint8, or None when the file has no colour; N is 0 for a map with no point."""
    lines = Path(path).read_text().splitlines()
    n = int(next(l for l in lines if l.startswith("element vertex")).split()[-1])
    start = lines.index("end_header") + 1
    if n == 0:     # what write_ply writes for a run that never initialized a map
        has_rgb = "property uchar red" in lines[:start]
        return np.empty((0, 3)), (np.empty((0, 3), np.uint8) if has_rgb else None)
    data = np.array([l.split() for l in lines[start:start + n]], dtype=float).reshape(n, -1)
    xyz = data[:, :3]
    rgb = data[:, 3:6].astype(np.uint8) if data.shape[1] >= 6 else None
    return xyz, rgb


GENERATED = ("/results/", "/figures/")   # outputs a run rewrites; not evidence of modified code


def _is_generated(path):
    path = "/" + path
    return any(g in path for g in GENERATED) or path.endswith("REPORT_summary.pdf")


def git_commit(root):
    """HEAD and whether any tracked source file differs from it. Generated outputs are not counted: results are
    tracked, so a rerun rewrites them, and counting them marked every rerun as dirty (CHANGELOG entry 18)."""
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        status = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root, text=True)
        changed = [line[3:].split(" -> ")[-1] for line in status.splitlines() if line.strip()]
        return commit, any(not _is_generated(path) for path in changed)
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
