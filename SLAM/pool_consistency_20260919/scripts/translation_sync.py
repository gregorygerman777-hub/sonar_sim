# Rotation was checked for multi-view consistency, translation wasn't -- this
# closes that gap. Only uses block A (the 34 frames with a certified rotation)
# since we need R_i to express directions in a common frame in the first
# place. Given R_i and a measured unit direction t_ij (camera i's frame),
# two-view geometry says t_ij is parallel to R_i (c_j - c_i) for camera
# centres c_i, c_j. That's linear and homogeneous in the unknown centres once
# R_i is known, so one global SVD (Govindu CVPR 2001; closed form in Jiang,
# Cui & Tan ICCV 2013) recovers every centre up to a global scale. Then check
# consistency the same way as rotation: angle between measured and predicted
# direction.
import pickle
from pathlib import Path

import numpy as np

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")


def main():
    with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
        d = pickle.load(fh)
    with open(OUT / "spectral_sync_result.pkl", "rb") as fh:
        rot_result = pickle.load(fh)

    # block A: the exact 34-frame component on the stable side of the (60,82)
    # graph cut (Section 4.8 / Fig. 8), used verbatim for consistency with the
    # rest of the report rather than recomputed from a different criterion.
    block_a = sorted({6,7,8,9,10,11,12,13,14,15,16,17,18,61,62,63,64,65,66,67,
                       69,70,71,72,73,74,75,76,77,78,79,80,81,82})
    print(f"block A (stable side of the single-edge cut): {len(block_a)} frames")

    R_final = rot_result["R_nls"]  # 0-indexed keys, from the standard (not pruned) NLS fit
    R = {f: R_final[f - 1] for f in block_a}

    # the usual inliers>=20 threshold only gives 49 edges among 34 nodes here,
    # which is rank-19 deficient -- checked this against generic synthetic
    # motion on the same graph and got the same deficiency, so it's just too
    # few edges, not a bad camera configuration. Makes sense: a bearing-only
    # edge is rank 2, not rank 3 like a full rotation, so you need more of
    # them. Swept the threshold on the synthetic case and it's fully rank
    # (down to the one unavoidable scale gauge) from inliers>=12 down to >=4,
    # so using 12, the loosest one that still works.
    TRANSLATION_THRESHOLD = 12
    edges = []
    for r in d["results"]:
        i, j = r["i"], r["j"]
        if i in block_a and j in block_a and r["t_dir"] is not None and r["inliers"] >= TRANSLATION_THRESHOLD:
            edges.append((i, j, r["t_dir"], r["inliers"]))
    print(f"block-A internal edges with a valid translation direction, inliers>={TRANSLATION_THRESHOLD}: {len(edges)}")

    order = block_a
    pos = {f: k for k, f in enumerate(order)}
    m = len(order)
    # homogeneous linear system: for each edge, two independent rows from
    # skew(t_ij) @ R_i @ (c_j - c_i) = 0  (skew(t) has rank 2, so 2 of its 3
    # rows are independent constraints per edge)
    rows_list = []
    for (i, j, t_ij, w) in edges:
        Ri = R[i]
        tx, ty, tz = t_ij
        skew = np.array([[0, -tz, ty], [tz, 0, -tx], [-ty, tx, 0]])
        A_block = skew @ Ri  # 3x3, multiplies (c_j - c_i)
        ci, cj = pos[i], pos[j]
        for row in range(2):  # rank-2: use first two rows of the skew-based constraint
            full_row = np.zeros(3 * m)
            full_row[3*cj:3*cj+3] = A_block[row]
            full_row[3*ci:3*ci+3] = -A_block[row]
            rows_list.append(full_row * np.sqrt(w))
    M = np.array(rows_list)
    print(f"linear system: {M.shape[0]} rows, {M.shape[1]} unknowns (3 x {m} centres)")

    # fix the gauge (global translation + scale + rotation of the WHOLE point
    # set are all unobservable) by pinning the first camera to the origin and
    # solving the reduced system for the rest, then taking the min singular
    # vector for scale (homogeneous least squares).
    ref = order[0]
    ref_col = pos[ref]
    keep_cols = [c for c in range(m) if c != ref_col]
    col_idx = np.array([3*c + k for c in keep_cols for k in range(3)])
    M_reduced = M[:, col_idx]
    U, S, Vt = np.linalg.svd(M_reduced, full_matrices=False)
    print(f"smallest 5 singular values: {S[-5:].round(4)}")
    x = Vt[-1]
    centres = {ref: np.zeros(3)}
    for idx, c in enumerate(keep_cols):
        centres[order[c]] = x[3*idx:3*idx+3]

    # residuals: angle between measured t_ij and predicted direction from solved centres
    residuals = []
    for (i, j, t_ij, w) in edges:
        pred = R[i] @ (centres[j] - centres[i])
        norm = np.linalg.norm(pred)
        if norm < 1e-9:
            continue
        pred_dir = pred / norm
        cos_angle = np.clip(np.dot(pred_dir, t_ij), -1, 1)
        # direction has a sign ambiguity from the homogeneous solve; take the
        # smaller of the two possible angles
        angle = np.degrees(np.arccos(abs(cos_angle)))
        residuals.append(angle)
    residuals = np.array(residuals)
    print(f"\ntranslation-direction residual (angle between measured and solved-geometry "
          f"predicted direction), n={len(residuals)}:")
    print(f"  median {np.median(residuals):.2f} deg, mean {residuals.mean():.2f} deg, "
          f"90th pct {np.percentile(residuals,90):.2f} deg, fraction>10deg {(residuals>10).mean():.3f}")

    import json as jsonlib
    (OUT / "translation_sync_result.json").write_text(jsonlib.dumps(dict(
        n_edges=len(edges), n_frames=m, singular_values=S.tolist(),
        residual_median=float(np.median(residuals)), residual_mean=float(residuals.mean()),
        residual_frac_over_10=float((residuals>10).mean()),
        residuals=residuals.tolist(), block_a=block_a), indent=2))
    print("\nwrote translation_sync_result.json")


if __name__ == "__main__":
    main()
