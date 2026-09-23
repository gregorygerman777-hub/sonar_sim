"""Bundle adjustment: Levenberg-Marquardt with the Schur complement and a Huber kernel (IRLS).

Residual for observation k of point j in camera i (world to camera pose R_i, t_i):
    e_k = (pi(K, R_i X_j + t_i) - z_k) / sigma_k               (2-vector)
Robust cost: sum_k rho(|e_k|^2) with the Huber kernel rho(s) = s for s <= d^2, 2 d sqrt(s) - d^2 above,
d^2 = 5.991 (chi-square, 2 dof, 95 %), as in ORB-SLAM / g2o.

Camera updates are left perturbations: R <- exp([dw]x) R, t <- t + dt, so
    d(R X + t)/d dw = -[R X]x,   d(R X + t)/d dt = I,   d(R X + t)/dX = R.
The camera block is solved densely after eliminating points (Schur complement).
"""

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.sparse import bsr_matrix, csr_matrix
from scipy.sparse.linalg import splu

from . import geometry as geo

CHI2_2DOF_95 = 5.991
HUBER_DELTA = np.sqrt(CHI2_2DOF_95)
REL_TOL = 1e-6         # stop when an accepted step lowers the robust cost by less than this fraction
LAMBDA_MIN = 1e-7      # floor on the Marquardt damping; keeps steps along the flat monocular gauge small


def block_diag_dense(blocks):
    n = len(blocks)
    out = np.zeros((6 * n, 6 * n))
    for i in range(n):
        out[6 * i:6 * i + 6, 6 * i:6 * i + 6] = blocks[i]
    return out


def block_diag_sparse(blocks):
    n = len(blocks)
    return bsr_matrix((blocks, np.arange(n), np.arange(n + 1)), shape=(6 * n, 6 * n)).tocsr()


def _huber(s, robust):
    """Robust cost and IRLS weight for squared norms s."""
    if not robust:
        return s, np.ones_like(s)
    d = HUBER_DELTA
    big = s > d * d
    cost = np.where(big, 2 * d * np.sqrt(np.maximum(s, 1e-300)) - d * d, s)
    w = np.where(big, d / np.sqrt(np.maximum(s, 1e-300)), 1.0)
    return cost, w


def _residuals(K, Rs, ts, X, oc, op, uv, inv_sigma):
    RX = np.einsum("mij,mj->mi", Rs[oc], X[op])
    Xc = RX + ts[oc]
    z = Xc[:, 2]
    zs = np.where(z > 1e-9, z, 1e-9)
    u = K[0, 0] * Xc[:, 0] / zs + K[0, 2]
    v = K[1, 1] * Xc[:, 1] / zs + K[1, 2]
    e = np.column_stack((u - uv[:, 0], v - uv[:, 1])) * inv_sigma[:, None]
    return e, RX, Xc, zs


def bundle_adjust(K, rotvecs, trans, points, obs_cam, obs_pt, obs_uv, obs_sigma,
                  fixed_cams=None, fixed_points=None, robust=True, max_nfev=30):
    """Optimise camera poses (world to camera, rotation vectors) and points jointly.

    Returns (rotvecs, trans, points, chi2) with chi2 the per observation squared normalised error.
    `max_nfev` bounds the number of LM iterations.
    """
    Rs = geo.rotvec_to_matrix(np.asarray(rotvecs, float))
    ts = np.array(trans, dtype=float, copy=True).reshape(-1, 3)
    X = np.array(points, dtype=float, copy=True).reshape(-1, 3)
    oc, op = np.asarray(obs_cam, int), np.asarray(obs_pt, int)
    uv = np.asarray(obs_uv, float).reshape(-1, 2)
    inv_sigma = 1.0 / np.asarray(obs_sigma, float)
    C, P, M = len(Rs), len(X), len(oc)
    fixed_cams = np.zeros(C, bool) if fixed_cams is None else np.asarray(fixed_cams, bool)
    fixed_points = np.zeros(P, bool) if fixed_points is None else np.asarray(fixed_points, bool)
    if M == 0:
        return geo.matrix_to_rotvec(Rs), ts, X, np.zeros(0)

    cam_slot = np.full(C, -1)
    free_cam = np.flatnonzero(~fixed_cams)
    cam_slot[free_cam] = np.arange(len(free_cam))
    pt_slot = np.full(P, -1)
    free_pt = np.flatnonzero(~fixed_points)
    pt_slot[free_pt] = np.arange(len(free_pt))
    nc, npt = len(free_cam), len(free_pt)
    if nc == 0 and npt == 0:
        e, *_ = _residuals(K, Rs, ts, X, oc, op, uv, inv_sigma)
        return geo.matrix_to_rotvec(Rs), ts, X, np.sum(e ** 2, 1)

    cs, ps = cam_slot[oc], pt_slot[op]
    has_c, has_p = cs >= 0, ps >= 0
    both = np.flatnonzero(has_c & has_p)
    # Sparsity pattern of W (camera-point coupling), one 6x3 block per observation with both free.
    w_rows = (6 * cs[both])[:, None, None] + np.arange(6)[None, :, None] + np.zeros((1, 1, 3), int)
    w_cols = (3 * ps[both])[:, None, None] + np.arange(3)[None, None, :] + np.zeros((1, 6, 1), int)
    v_rows = (3 * np.arange(npt))[:, None, None] + np.arange(3)[None, :, None] + np.zeros((1, 1, 3), int)
    v_cols = (3 * np.arange(npt))[:, None, None] + np.arange(3)[None, None, :] + np.zeros((1, 3, 1), int)

    def cost_of(Rs_, ts_, X_):
        e, *_ = _residuals(K, Rs_, ts_, X_, oc, op, uv, inv_sigma)
        s = np.sum(e ** 2, 1)
        c, _ = _huber(s, robust)
        return float(np.sum(c)), s

    lam = 1e-4
    cost, _ = cost_of(Rs, ts, X)
    for _ in range(max_nfev):
        e, RX, Xc, z = _residuals(K, Rs, ts, X, oc, op, uv, inv_sigma)
        s = np.sum(e ** 2, 1)
        _, w = _huber(s, robust)
        fx, fy = K[0, 0], K[1, 1]
        dproj = np.zeros((M, 2, 3))
        dproj[:, 0, 0] = fx / z
        dproj[:, 0, 2] = -fx * Xc[:, 0] / z ** 2
        dproj[:, 1, 1] = fy / z
        dproj[:, 1, 2] = -fy * Xc[:, 1] / z ** 2
        dproj *= inv_sigma[:, None, None]
        Jc = np.concatenate((dproj @ -geo.skew_batch(RX), dproj), axis=2)   # (M,2,6)
        Jp = dproj @ Rs[oc]                                                 # (M,2,3)
        wJc = Jc * w[:, None, None]
        wJp = Jp * w[:, None, None]
        Hcc_obs = np.einsum("mki,mkj->mij", wJc, Jc)
        Hpp_obs = np.einsum("mki,mkj->mij", wJp, Jp)
        Hcp_obs = np.einsum("mki,mkj->mij", wJc, Jp)
        gc_obs = np.einsum("mki,mk->mi", wJc, e)
        gp_obs = np.einsum("mki,mk->mi", wJp, e)

        Hcc = np.zeros((nc, 6, 6))
        gc = np.zeros((nc, 6))
        np.add.at(Hcc, cs[has_c], Hcc_obs[has_c])
        np.add.at(gc, cs[has_c], gc_obs[has_c])
        Hpp = np.zeros((npt, 3, 3))
        gp = np.zeros((npt, 3))
        np.add.at(Hpp, ps[has_p], Hpp_obs[has_p])
        np.add.at(gp, ps[has_p], gp_obs[has_p])

        improved = False
        W = csr_matrix((Hcp_obs[both].ravel(), (w_rows.ravel(), w_cols.ravel())), shape=(6 * nc, 3 * npt)) \
            if (nc and npt) else None
        for _attempt in range(10):
            Hpp_d = Hpp + lam * np.einsum("nii->ni", Hpp)[:, :, None] * np.eye(3) + 1e-9 * np.eye(3)
            Hpp_inv = np.linalg.inv(Hpp_d) if npt else Hpp_d
            if nc:
                Hcc_d = Hcc + lam * np.einsum("nii->ni", Hcc)[:, :, None] * np.eye(6) + 1e-9 * np.eye(6)
                b = -gc.ravel()
                S = block_diag_sparse(Hcc_d)
                if W is not None:
                    Vinv = csr_matrix((Hpp_inv.ravel(), (v_rows.ravel(), v_cols.ravel())), shape=(3 * npt, 3 * npt))
                    WV = W @ Vinv
                    S = (S - WV @ W.T).tocsc()
                    b = b + WV @ gp.ravel()
                try:
                    if nc <= 150:
                        dc = cho_solve(cho_factor(S.toarray()), b).reshape(nc, 6)
                    else:
                        dc = splu(S).solve(b).reshape(nc, 6)   # the reduced camera system is sparse (covisibility)
                    if not np.all(np.isfinite(dc)):
                        raise np.linalg.LinAlgError
                except (np.linalg.LinAlgError, RuntimeError):
                    lam *= 10
                    continue
            else:
                dc = np.zeros((0, 6))
            if npt:
                rhs = -gp.ravel()
                if W is not None:
                    rhs = rhs - W.T @ dc.ravel()
                dp = np.einsum("nij,nj->ni", Hpp_inv, rhs.reshape(npt, 3))
            else:
                dp = np.zeros((0, 3))
            Rs_new, ts_new, X_new = Rs.copy(), ts.copy(), X.copy()
            if nc:
                Rs_new[free_cam] = geo.rotvec_to_matrix(dc[:, :3]) @ Rs[free_cam]
                ts_new[free_cam] = ts[free_cam] + dc[:, 3:]
            if npt:
                X_new[free_pt] = X[free_pt] + dp
            new_cost, _ = cost_of(Rs_new, ts_new, X_new)
            if np.isfinite(new_cost) and new_cost < cost:
                rel = (cost - new_cost) / max(cost, 1e-300)
                Rs, ts, X, cost = Rs_new, ts_new, X_new, new_cost
                lam = max(lam / 10, LAMBDA_MIN)
                improved = True
                break
            lam *= 10
        if not improved or rel < REL_TOL:
            break
    e, _, Xc, _ = _residuals(K, Rs, ts, X, oc, op, uv, inv_sigma)
    chi2 = np.sum(e ** 2, 1)
    chi2[Xc[:, 2] <= 1e-9] = np.inf    # behind the camera is an outlier regardless of pixel error
    return geo.matrix_to_rotvec(Rs), ts, X, chi2


def optimize_pose(K, R, t, X, uv, sigma, robust=True):
    """Motion only BA: refine one world to camera pose against fixed 3D points. Returns R, t, chi2."""
    w, t2, _, chi2 = bundle_adjust(K, geo.matrix_to_rotvec(R), t.reshape(1, 3), X, np.zeros(len(X), int),
                                   np.arange(len(X)), uv, sigma, fixed_points=np.ones(len(X), bool),
                                   robust=robust, max_nfev=20)
    return geo.rotvec_to_matrix(w)[0], t2[0], chi2
