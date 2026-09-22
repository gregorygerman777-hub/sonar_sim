# Same protocol as pairwise_verify.py, all 117 frames and all C(117,2)=6786
# pairs, but SIFT instead of ORB. Prompted by what showed up on the 80-frame
# subset: edge (60,82), the one ORB cross-block measurement everything hinged
# on, only gets 5 inliers under SIFT (vs ORB's 25), while 154 other
# cross-block pairs clear the bar that ORB found nothing for. Worth redoing
# properly instead of just patching in a few pairs.
import pickle
import time
from itertools import combinations
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

DATA = Path("/Users/gregsobe/Downloads/Archive")
OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
K = np.array([[1507.968, 0.0, 512.0], [0.0, 1403.46109, 392.915659], [0.0, 0.0, 1.0]])
N_FRAMES = 117


def extract_sift(frame_idx):
    im = cv2.imread(str(DATA / f"opt{frame_idx}.bmp"), cv2.IMREAD_GRAYSCALE)
    sift = cv2.SIFT_create(nfeatures=4000, contrastThreshold=0.02)
    kp, desc = sift.detectAndCompute(im, None)
    pts = np.array([k.pt for k in kp], dtype=float) if kp else np.empty((0, 2))
    return frame_idx, pts, desc


_FEATS = None


def _init(features):
    global _FEATS
    _FEATS = features


def verify_pair(args):
    i, j = args
    pa_pts, da = _FEATS[i]
    pb_pts, db = _FEATS[j]
    result = dict(i=i, j=j, matches=0, inliers=0, rotation_deg=np.nan, r_h=np.nan, epi_err=np.nan)
    if da is None or db is None or len(da) < 8 or len(db) < 8:
        return result
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    knn = matcher.knnMatch(da, db, k=2)
    matches = np.array([(m.queryIdx, m.trainIdx) for m, n in
                         (p for p in knn if len(p) == 2) if m.distance < 0.75 * n.distance])
    result["matches"] = int(len(matches))
    if len(matches) < 8:
        return result
    pa = pa_pts[matches[:, 0]]
    pb = pb_pts[matches[:, 1]]
    E, mask = cv2.findEssentialMat(pa, pb, K, method=cv2.RANSAC, prob=0.999,
                                    threshold=1.0, maxIters=500)
    if E is None or E.shape != (3, 3):
        return result
    n_in, R, t, mask2 = cv2.recoverPose(E, pa, pb, K, mask=mask)
    result["inliers"] = int(n_in)
    if n_in < 8:
        return result
    result["rotation_deg"] = float(np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0])))
    result["R"] = R
    inlier_mask = mask2.ravel().astype(bool)
    Kinv = np.linalg.inv(K)
    x1 = (np.hstack([pa[inlier_mask], np.ones((inlier_mask.sum(), 1))]) @ Kinv.T)[:, :2]
    x2 = (np.hstack([pb[inlier_mask], np.ones((inlier_mask.sum(), 1))]) @ Kinv.T)[:, :2]
    x1h = np.hstack([x1, np.ones((len(x1), 1))])
    x2h = np.hstack([x2, np.ones((len(x2), 1))])
    Ex1 = (E @ x1h.T).T
    Etx2 = (E.T @ x2h.T).T
    numer = np.sum(x2h * Ex1, axis=1) ** 2
    denom = Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2
    result["epi_err"] = float(np.mean(np.sqrt(numer / np.maximum(denom, 1e-12))) * K[0, 0])
    H, hmask = cv2.findHomography(pa, pb, cv2.RANSAC, 1.0, maxIters=500)
    if H is not None and np.isfinite(H).all() and abs(np.linalg.det(H)) > 1e-9:
        try:
            Hinv = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            Hinv = None
        if Hinv is not None:
            TH, GAMMA = 5.99, 5.99
            pah = np.hstack([pa, np.ones((len(pa), 1))])
            pbh = np.hstack([pb, np.ones((len(pb), 1))])
            proj_b = (H @ pah.T).T; proj_b = proj_b[:, :2] / proj_b[:, 2:3]
            proj_a = (Hinv @ pbh.T).T; proj_a = proj_a[:, :2] / proj_a[:, 2:3]
            d2f = np.sum((proj_b - pb) ** 2, axis=1)
            d2b = np.sum((proj_a - pa) ** 2, axis=1)
            score_h = np.sum(np.where(d2f < TH, GAMMA - d2f, 0)) + np.sum(np.where(d2b < TH, GAMMA - d2b, 0))
            Fpx = Kinv.T @ E @ Kinv
            Fx1 = (Fpx @ pah.T).T; Ftx2 = (Fpx.T @ pbh.T).T
            num = np.sum(pbh * Fx1, axis=1) ** 2
            d2tb = num / (Fx1[:, 0] ** 2 + Fx1[:, 1] ** 2 + 1e-12)
            d2ta = num / (Ftx2[:, 0] ** 2 + Ftx2[:, 1] ** 2 + 1e-12)
            score_f = np.sum(np.where(d2tb < TH, GAMMA - d2tb, 0)) + np.sum(np.where(d2ta < TH, GAMMA - d2ta, 0))
            denom_rh = score_h + score_f
            result["r_h"] = float(score_h / denom_rh) if denom_rh > 0 else np.nan
    return result


def main():
    started = time.perf_counter()
    with Pool(8) as pool:
        feats = pool.map(extract_sift, range(1, N_FRAMES + 1))
    features = {f: (pts, desc) for f, pts, desc in feats}
    counts = [len(pts) for f, (pts, desc) in features.items()]
    print(f"SIFT extraction: {len(features)} frames in {time.perf_counter()-started:.1f}s, "
          f"median features/frame {int(np.median(counts))}, min {min(counts)}, max {max(counts)}")

    all_pairs = list(combinations(range(1, N_FRAMES + 1), 2))
    print(f"total pairs: {len(all_pairs)}")

    started = time.perf_counter()
    with Pool(8, initializer=_init, initargs=(features,)) as pool:
        results = pool.map(verify_pair, all_pairs, chunksize=16)
    print(f"verified {len(results)} pairs in {time.perf_counter()-started:.1f}s")

    with open(OUT / "sift_pairwise_all.pkl", "wb") as fh:
        pickle.dump(dict(results=results, n_frames=N_FRAMES, K=K), fh)
    print("wrote sift_pairwise_all.pkl")

    inliers = np.array([r["inliers"] for r in results])
    for th in (12, 15, 20, 25, 30):
        print(f"  inliers>={th}: {(inliers>=th).sum()} edges")


if __name__ == "__main__":
    main()
