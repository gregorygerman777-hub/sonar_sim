"""Stage B: full pairwise geometric verification, every one of C(117,2)=6786 frame pairs.

For each pair (i, j), i < j: ORB descriptor matching (ratio test), essential-matrix
RANSAC, cheirality-checked pose recovery (cv2.recoverPose), PLUS a homography fit on
the same correspondences and the ORB-SLAM-style planarity ratio R_H = S_H/(S_H+S_F)
(Mur-Artal, Montiel & Tardos, "ORB-SLAM", IEEE T-RO 2015, Sec. IV; originally Torr &
Zisserman's model-selection scoring for E/F vs H). This flags pairs where the scene
subtended by the correspondences is (near-)planar, a well known degeneracy of the
5-point/8-point essential-matrix solver: many equally-consistent essential matrices
fit a planar point set, so recovered rotation/translation can be spurious even with
a healthy inlier count. This is essential here because the dominant scene content is
a flat tiled pool floor.
"""
import argparse
import itertools
import pickle
import time
from multiprocessing import Pool

import cv2
import numpy as np

OUT_DIR = "/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd"
TH_SCORE = 5.99   # chi-square 95% for 2 dof (symmetric transfer error), as in ORB-SLAM
GAMMA = 5.99       # same threshold used for both H and F scoring for comparability

_FRAMES = None
_K = None


def _init(features_path, K):
    global _FRAMES, _K
    with open(features_path, "rb") as fh:
        _FRAMES = pickle.load(fh)["frames"]
    _K = K


def _symmetric_transfer_score_H(H, pa, pb, sigma2=1.0):
    if H is None or abs(np.linalg.det(H)) < 1e-12:
        return 0.0, 0
    Hinv = np.linalg.inv(H)
    pa_h = np.hstack([pa, np.ones((len(pa), 1))])
    pb_h = np.hstack([pb, np.ones((len(pb), 1))])
    proj_b = (H @ pa_h.T).T
    proj_b = proj_b[:, :2] / proj_b[:, 2:3]
    proj_a = (Hinv @ pb_h.T).T
    proj_a = proj_a[:, :2] / proj_a[:, 2:3]
    d2_fwd = np.sum((proj_b - pb) ** 2, axis=1) / sigma2
    d2_bwd = np.sum((proj_a - pa) ** 2, axis=1) / sigma2
    score = np.sum(np.where(d2_fwd < TH_SCORE, GAMMA - d2_fwd, 0)) + \
            np.sum(np.where(d2_bwd < TH_SCORE, GAMMA - d2_bwd, 0))
    inliers = int(np.sum((d2_fwd < TH_SCORE) & (d2_bwd < TH_SCORE)))
    return float(score), inliers


def _symmetric_transfer_score_F(F, pa, pb, sigma2=1.0):
    pa_h = np.hstack([pa, np.ones((len(pa), 1))])
    pb_h = np.hstack([pb, np.ones((len(pb), 1))])
    Fx1 = (F @ pa_h.T).T
    Ftx2 = (F.T @ pb_h.T).T
    num = np.sum(pb_h * Fx1, axis=1) ** 2
    d2_to_b = num / (Fx1[:, 0] ** 2 + Fx1[:, 1] ** 2 + 1e-12) / sigma2
    d2_to_a = num / (Ftx2[:, 0] ** 2 + Ftx2[:, 1] ** 2 + 1e-12) / sigma2
    score = np.sum(np.where(d2_to_b < TH_SCORE, GAMMA - d2_to_b, 0)) + \
            np.sum(np.where(d2_to_a < TH_SCORE, GAMMA - d2_to_a, 0))
    inliers = int(np.sum((d2_to_b < TH_SCORE) & (d2_to_a < TH_SCORE)))
    return float(score), inliers


def verify_pair(args):
    i, j = args
    a, b = _FRAMES[i], _FRAMES[j]
    da, db = a["descriptors"], b["descriptors"]
    result = dict(i=i + 1, j=j + 1, matches=0, inliers=0, mean_epi_err=np.nan,
                  rotation_deg=np.nan, R=None, t_dir=None, r_h=np.nan,
                  h_inliers=0, f_inliers=0)
    if da is None or db is None or len(da) < 8 or len(db) < 8:
        return result
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    knn = matcher.knnMatch(da, db, k=2)
    matches = np.array([(m.queryIdx, m.trainIdx) for m, n in
                         (p for p in knn if len(p) == 2) if m.distance < 0.8 * n.distance])
    result["matches"] = int(len(matches))
    if len(matches) < 8:
        return result
    pa = a["keypoints"][matches[:, 0]]
    pb = b["keypoints"][matches[:, 1]]

    E, mask = cv2.findEssentialMat(pa, pb, _K, method=cv2.RANSAC, prob=0.999,
                                    threshold=1.0, maxIters=300)
    if E is not None and E.shape == (3, 3):
        n_in, R, t, mask2 = cv2.recoverPose(E, pa, pb, _K, mask=mask)
        result["inliers"] = int(n_in)
        if n_in >= 8:
            inlier_mask = mask2.ravel().astype(bool)
            Kinv = np.linalg.inv(_K)
            x1 = (np.hstack([pa[inlier_mask], np.ones((inlier_mask.sum(), 1))]) @ Kinv.T)[:, :2]
            x2 = (np.hstack([pb[inlier_mask], np.ones((inlier_mask.sum(), 1))]) @ Kinv.T)[:, :2]
            x1h = np.hstack([x1, np.ones((len(x1), 1))])
            x2h = np.hstack([x2, np.ones((len(x2), 1))])
            Ex1 = (E @ x1h.T).T
            Etx2 = (E.T @ x2h.T).T
            numer = np.sum(x2h * Ex1, axis=1) ** 2
            denom = Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2
            sampson_norm = np.sqrt(numer / np.maximum(denom, 1e-12))
            result["mean_epi_err"] = float(np.mean(sampson_norm) * _K[0, 0])
            result["rotation_deg"] = float(np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0])))
            result["R"] = R
            result["t_dir"] = (t.ravel() / max(np.linalg.norm(t), 1e-12))

            # planarity / degeneracy test on the SAME correspondence set (all ratio-test
            # matches, not just E-inliers, so H and F compete fairly)
            F_px = Kinv.T @ E @ Kinv
            score_f, n_f = _symmetric_transfer_score_F(F_px, pa, pb)
            H, hmask = cv2.findHomography(pa, pb, cv2.RANSAC, 1.0, maxIters=300)
            if H is not None:
                score_h, n_h = _symmetric_transfer_score_H(H, pa, pb)
                denom_rh = score_h + score_f
                result["r_h"] = float(score_h / denom_rh) if denom_rh > 0 else np.nan
                result["h_inliers"] = n_h
                result["f_inliers"] = n_f
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--k-name", required=True, choices=["scaled", "raw"])
    parser.add_argument("--pairs", choices=["all", "consecutive"], default="all")
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    K_scaled = np.array([[1507.968, 0.0, 512.0], [0.0, 1403.46109, 392.915659], [0.0, 0.0, 1.0]])
    K_raw = np.array([[1403.46109, 0.0, 476.5168], [0.0, 1403.46109, 392.915659], [0.0, 0.0, 1.0]])
    K = K_scaled if args.k_name == "scaled" else K_raw

    features_path = f"{OUT_DIR}/features.pkl"
    with open(features_path, "rb") as fh:
        n = len(pickle.load(fh)["frames"])

    if args.pairs == "all":
        job_list = list(itertools.combinations(range(n), 2))
    else:
        job_list = [(i, i + 1) for i in range(n - 1)]

    started = time.perf_counter()
    with Pool(args.workers, initializer=_init, initargs=(features_path, K)) as pool:
        results = pool.map(verify_pair, job_list, chunksize=32)
    print(f"{len(results)} pairs verified in {time.perf_counter() - started:.1f}s")

    out_path = f"{OUT_DIR}/pairwise_{args.k_name}_{args.pairs}.pkl"
    with open(out_path, "wb") as fh:
        pickle.dump(dict(K=K, results=results, n_frames=n), fh)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
