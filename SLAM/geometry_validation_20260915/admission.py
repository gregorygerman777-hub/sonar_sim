"""Validate independently supplied mounting calibration without repairing it."""
import numpy as np

def require_rigid_mount(transform):
    t = np.asarray(transform, dtype=float)
    if t.shape != (4, 4) or not np.all(np.isfinite(t)):
        raise ValueError('Mount must be a finite 4x4 matrix')
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError('Invalid homogeneous bottom row')
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0):
        raise ValueError('Mount rotation is not orthonormal; obtain verified calibration')
    if not np.isclose(np.linalg.det(r), 1, atol=1e-6, rtol=0):
        raise ValueError('Mount rotation must have determinant +1')
    return t.copy()
