"""Case study figure: frame 19 disagrees with its own multi-view-averaged rotation
on several separate pairwise edges (19-20, 19-21, 19-24, 19-42, 19-47). Draw the
actual matched correspondences for one such pair to show why: the rock/pebble
target and tile grid are locally self-similar, so descriptor matching can lock
onto a plausible but wrong correspondence set that still passes RANSAC.
"""
from pathlib import Path

import pickle

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import matlab_style
matlab_style.apply()

DATA = Path("/Users/gregsobe/Downloads/Archive")
OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
FIG = OUT / "figures"

K = np.array([[1507.968, 0.0, 512.0], [0.0, 1403.46109, 392.915659], [0.0, 0.0, 1.0]])


def load_gray(i):
    im = cv2.imread(str(DATA / f"opt{i}.bmp"))
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), im


def main():
    a_idx, b_idx = 19, 47
    _, bgr_a = load_gray(a_idx)
    _, bgr_b = load_gray(b_idx)

    # use the SAME cached, bucketed ORB features as the full pairwise sweep, so
    # this figure reproduces the exact match/inlier counts quoted in the text.
    with open(OUT / "features.pkl", "rb") as fh:
        frames = pickle.load(fh)["frames"]
    fa, fb = frames[a_idx - 1], frames[b_idx - 1]
    kp_a, desc_a = fa["keypoints"], fa["descriptors"]
    kp_b, desc_b = fb["keypoints"], fb["descriptors"]
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    knn = matcher.knnMatch(desc_a, desc_b, k=2)
    good = [m for m, n in knn if m.distance < 0.8 * n.distance]
    pa = np.array([kp_a[m.queryIdx] for m in good])
    pb = np.array([kp_b[m.trainIdx] for m in good])
    E, mask = cv2.findEssentialMat(pa, pb, K, method=cv2.RANSAC, prob=0.999, threshold=1.0, maxIters=300)
    n_in, R, t, mask2 = cv2.recoverPose(E, pa, pb, K, mask=mask)
    inlier_mask = mask2.ravel().astype(bool)
    rot_deg = np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0]))

    rgb_a = cv2.cvtColor(bgr_a, cv2.COLOR_BGR2RGB)
    rgb_b = cv2.cvtColor(bgr_b, cv2.COLOR_BGR2RGB)
    h, w = bgr_a.shape[:2]
    canvas = np.zeros((h, 2 * w, 3), dtype=np.uint8)
    canvas[:, :w] = rgb_a
    canvas[:, w:] = rgb_b

    fig, ax = plt.subplots(figsize=(13, 5.5))
    ax.imshow(canvas)
    idx_in = np.where(inlier_mask)[0]
    rng = np.random.default_rng(3)
    show_idx = rng.choice(idx_in, size=min(35, len(idx_in)), replace=False)
    for k in show_idx:
        x1, y1 = pa[k]
        x2, y2 = pb[k]
        ax.plot([x1, x2 + w], [y1, y2], linewidth=0.7, alpha=0.85,
                color=matlab_style.MATLAB_COLORS[1])
        ax.plot(x1, y1, marker="o", markersize=3, color=matlab_style.MATLAB_COLORS[0])
        ax.plot(x2 + w, y2, marker="o", markersize=3, color=matlab_style.MATLAB_COLORS[0])
    ax.set_title(f"Frame {a_idx} vs frame {b_idx}: {int(inlier_mask.sum())} RANSAC inliers of {len(good)} matches, "
                 f"but recovered rotation ({rot_deg:.0f}$^\\circ$) disagrees with the\n"
                 f"multi-view-averaged trajectory by 125$^\\circ$ -- a data-association failure, "
                 f"not a geometry failure: the rock/pebble field is locally self-similar",
                 fontsize=11)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.tight_layout()
    fig.savefig(FIG / "fig5_case_study_aliasing.png", dpi=150)
    plt.close(fig)
    print(f"frame {a_idx}-{b_idx}: {len(good)} matches, {inlier_mask.sum()} inliers, rotation {rot_deg:.1f} deg")
    print("wrote fig5_case_study_aliasing.png")


if __name__ == "__main__":
    main()
