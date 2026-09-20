# Try a second, independent feature type on the trusted-core frames: does
# anything corroborate edge (60,82), and can a stronger descriptor pull any
# near-miss cross-block pair over the 20-inlier bar the way ORB couldn't?
# ORB (binary, 256 bit) is fast but not very discriminative on repetitive
# texture; SIFT (float, 128-dim, real scale-space extrema) usually holds up
# better on exactly this kind of self-similar rock/pebble/tile scene. This is
# a real test of whether (60,82) is genuine or an ORB artifact.
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

BLOCK_A = sorted({6,7,8,9,10,11,12,13,14,15,16,17,18,61,62,63,64,65,66,67,
                   69,70,71,72,73,74,75,76,77,78,79,80,81,82})
BLOCK_B = sorted({19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,
                   39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,60,84,86,
                   90,91,92,97,98,115,116})
FRAMES = sorted(BLOCK_A + BLOCK_B)  # the 80-frame trusted core


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
    if H is not None and abs(np.linalg.det(H)) > 1e-12:
        Kinv2 = Kinv
        Fpx = Kinv2.T @ E @ Kinv2
        # symmetric transfer scores, same construction as pairwise_verify.py
        TH, GAMMA = 5.99, 5.99
        pah = np.hstack([pa, np.ones((len(pa), 1))])
        pbh = np.hstack([pb, np.ones((len(pb), 1))])
        proj_b = (H @ pah.T).T; proj_b = proj_b[:, :2] / proj_b[:, 2:3]
        Hinv = np.linalg.inv(H)
        proj_a = (Hinv @ pbh.T).T; proj_a = proj_a[:, :2] / proj_a[:, 2:3]
        d2f = np.sum((proj_b - pb) ** 2, axis=1)
        d2b = np.sum((proj_a - pa) ** 2, axis=1)
        score_h = np.sum(np.where(d2f < TH, GAMMA - d2f, 0)) + np.sum(np.where(d2b < TH, GAMMA - d2b, 0))
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
        feats = pool.map(extract_sift, FRAMES)
    features = {f: (pts, desc) for f, pts, desc in feats}
    counts = {f: len(pts) for f, (pts, desc) in features.items()}
    print(f"SIFT extraction: {len(features)} frames in {time.perf_counter()-started:.1f}s, "
          f"median features/frame {int(np.median(list(counts.values())))}")

    cross_pairs = [(min(i, j), max(i, j)) for i in BLOCK_A for j in BLOCK_B]
    all_pairs = list(combinations(FRAMES, 2))
    print(f"cross-block pairs: {len(cross_pairs)}, all trusted-core pairs: {len(all_pairs)}")

    started = time.perf_counter()
    with Pool(8, initializer=_init, initargs=(features,)) as pool:
        results = pool.map(verify_pair, all_pairs, chunksize=16)
    print(f"verified {len(results)} pairs in {time.perf_counter()-started:.1f}s")

    # save immediately -- this took over 3 minutes, don't lose it to a bug below
    with open(OUT / "sift_crossblock_result.pkl", "wb") as fh:
        pickle.dump(dict(results=results, block_a=BLOCK_A, block_b=BLOCK_B), fh)
    print(f"wrote sift_crossblock_result.pkl")

    by_pair = {(r["i"], r["j"]): r for r in results}
    r_6082 = by_pair.get((60, 82))
    print(f"\n(60,82) under SIFT: {r_6082}")

    cross_results = [by_pair[(i, j)] for (i, j) in cross_pairs if (i, j) in by_pair]
    print(f"cross-block pairs resolved: {len(cross_results)} of {len(cross_pairs)}")
    cleared = [r for r in cross_results if r["inliers"] >= 20]
    cleared.sort(key=lambda r: -r["inliers"])
    print(f"\ncross-block pairs clearing inliers>=20 under SIFT: {len(cleared)} of {len(cross_results)}")
    for r in cleared[:20]:
        print(f"  {r['i']:3d}-{r['j']:3d} inliers={r['inliers']:3d} matches={r['matches']:4d} "
              f"rot={r['rotation_deg']:6.1f}deg r_h={r['r_h']:.2f} epi={r['epi_err']:.3f}")


if __name__ == "__main__":
    main()
