"""Rigid body helpers, projection, triangulation.

Pose convention used everywhere inside monoslam: a pose is stored as (R, t) with
    x_cam = R @ x_world + t        (world to camera, "T_cw")
OpenCV camera axes: x right, y down, z forward. Exporters convert to camera to world.
"""

import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    v = np.asarray(v, dtype=float)
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def skew_batch(v):
    """(N,3) -> (N,3,3) skew matrices."""
    out = np.zeros((len(v), 3, 3))
    out[:, 0, 1], out[:, 0, 2] = -v[:, 2], v[:, 1]
    out[:, 1, 0], out[:, 1, 2] = v[:, 2], -v[:, 0]
    out[:, 2, 0], out[:, 2, 1] = -v[:, 1], v[:, 0]
    return out


def rotvec_to_matrix(w):
    return Rotation.from_rotvec(np.asarray(w, dtype=float).reshape(-1, 3)).as_matrix()


def matrix_to_rotvec(R):
    return Rotation.from_matrix(np.asarray(R, dtype=float).reshape(-1, 3, 3)).as_rotvec()


def left_jacobian_batch(w):
    """SO(3) left Jacobian J_l(w) for (N,3) rotation vectors.

    R(w + d) ~= exp([J_l(w) d]x) R(w), hence d(R(w) p)/dw = -[R(w) p]x J_l(w).
    """
    w = np.asarray(w, dtype=float).reshape(-1, 3)
    theta = np.linalg.norm(w, axis=1)
    W = skew_batch(w)
    W2 = W @ W
    small = theta < 1e-6
    th = np.where(small, 1.0, theta)
    a = np.where(small, 0.5 - theta ** 2 / 24.0, (1.0 - np.cos(th)) / th ** 2)
    b = np.where(small, 1.0 / 6.0 - theta ** 2 / 120.0, (th - np.sin(th)) / th ** 3)
    return np.eye(3)[None] + a[:, None, None] * W + b[:, None, None] * W2


def invert(R, t):
    return R.T, -R.T @ t


def compose(Ra, ta, Rb, tb):
    """(Ra,ta) o (Rb,tb): x -> Ra (Rb x + tb) + ta."""
    return Ra @ Rb, Ra @ tb + ta


def to_matrix(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def camera_center(R, t):
    return -R.T @ t


def project(K, R, t, X):
    """Pixels (N,2) and depths (N,) of world points X (N,3)."""
    Xc = X @ R.T + t
    z = Xc[:, 2]
    zs = np.where(np.abs(z) < 1e-12, 1e-12, z)
    u = K[0, 0] * Xc[:, 0] / zs + K[0, 2]
    v = K[1, 1] * Xc[:, 1] / zs + K[1, 2]
    return np.column_stack((u, v)), z


def normalize(K, uv):
    """Pixels to normalized image coordinates."""
    return np.column_stack(((uv[:, 0] - K[0, 2]) / K[0, 0], (uv[:, 1] - K[1, 2]) / K[1, 1]))


def triangulate(K, R1, t1, R2, t2, uv1, uv2):
    """Linear (DLT) triangulation of pixel correspondences between two posed views. Returns (N,3)."""
    x1, x2 = normalize(K, uv1), normalize(K, uv2)
    P1 = np.hstack((R1, t1.reshape(3, 1)))
    P2 = np.hstack((R2, t2.reshape(3, 1)))
    A = np.stack((x1[:, 0:1] * P1[2] - P1[0], x1[:, 1:2] * P1[2] - P1[1],
                  x2[:, 0:1] * P2[2] - P2[0], x2[:, 1:2] * P2[2] - P2[1]), axis=1)  # (N,4,4)
    _, _, vt = np.linalg.svd(A)
    Xh = vt[:, -1, :]
    w = Xh[:, 3:4]
    w = np.where(np.abs(w) < 1e-12, 1e-12, w)
    return Xh[:, :3] / w


def parallax_cos(R1, t1, R2, t2, X):
    """Cosine of the angle at X between the rays to the two camera centres."""
    c1, c2 = camera_center(R1, t1), camera_center(R2, t2)
    a, b = X - c1, X - c2
    return np.sum(a * b, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) + 1e-12)


def essential_from_poses(R1, t1, R2, t2):
    """E with x2^T E x1 = 0 for normalized coords, from two world to camera poses."""
    R, t = compose(R2, t2, *invert(R1, t1))
    return skew(t) @ R


def epipolar_sq_dist_px(K, E, uv1, uv2):
    """Squared distance (pixels, in image 2) of uv2 from the epipolar line of uv1."""
    Kinv = np.linalg.inv(K)
    F = Kinv.T @ E @ Kinv
    p1 = np.column_stack((uv1, np.ones(len(uv1))))
    p2 = np.column_stack((uv2, np.ones(len(uv2))))
    lines = p1 @ F.T
    num = np.sum(lines * p2, axis=1) ** 2
    return num / (lines[:, 0] ** 2 + lines[:, 1] ** 2 + 1e-18)


def quaternion_xyzw(R):
    return Rotation.from_matrix(R).as_quat()  # scipy order is (x, y, z, w)


def umeyama(src, dst, with_scale=True):
    """s, R, t minimising |dst - (s R src + t)|^2 (Umeyama 1991). src, dst are (N,3)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = np.mean(np.sum(xs ** 2, axis=1))
    s = np.trace(np.diag(D) @ S) / var_s if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t
