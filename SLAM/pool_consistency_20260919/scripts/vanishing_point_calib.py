# Try to get an independent focal length estimate from the tiled floor's
# vanishing points, just as a sanity check on the supplied K. Two line
# families: the near-parallel grid lines (angle clustering finds these fine),
# and the orthogonal receding ones, which don't share an image-space angle so
# need actual VP-RANSAC (sample line pairs, intersect, count how many other
# lines pass near that point). Given both VPs, f = sqrt(-(v1-pp).(v2-pp))
# for zero skew / known principal point (Caprile & Torre 1990). Reporting
# whatever this gives, including if it doesn't work.
import json
from pathlib import Path

import cv2
import numpy as np

DATA = Path("/Users/gregsobe/Downloads/Archive")
OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
CANDIDATE_FRAMES = [50, 55, 58, 60, 62, 65, 70, 90, 110, 116]
RNG = np.random.default_rng(0)


def load_gray(i):
    im = cv2.imread(str(DATA / f"opt{i}.bmp"))
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)


def detect_lines(gray, min_len=35):
    lsd = cv2.createLineSegmentDetector(0)
    lines = lsd.detect(gray)[0]
    if lines is None:
        return np.empty((0, 4))
    lines = lines.reshape(-1, 4)
    lengths = np.hypot(lines[:, 2] - lines[:, 0], lines[:, 3] - lines[:, 1])
    return lines[lengths > min_len]


def homogeneous(l):
    return np.cross([l[0], l[1], 1.0], [l[2], l[3], 1.0])


def point_line_distance(point, line_h):
    a, b, c = line_h
    return abs(a * point[0] + b * point[1] + c) / max(np.hypot(a, b), 1e-9)


def dominant_family(lines, tol_deg=6):
    angles = np.degrees(np.arctan2(lines[:, 3] - lines[:, 1], lines[:, 2] - lines[:, 0])) % 180
    best_mask, best_count = None, -1
    for center in range(0, 180, 2):
        diff = np.minimum(np.abs(angles - center), 180 - np.abs(angles - center))
        mask = diff < tol_deg
        if mask.sum() > best_count:
            best_count, best_mask = mask.sum(), mask
    return best_mask


def vp_ransac(lines, n_iters=3000, inlier_px=6.0):
    if len(lines) < 8:
        return None, 0
    coeffs = np.array([homogeneous(l) for l in lines])
    best_vp, best_inliers = None, -1
    idx = np.arange(len(lines))
    for _ in range(n_iters):
        i, j = RNG.choice(idx, size=2, replace=False)
        cross = np.cross(coeffs[i], coeffs[j])
        if abs(cross[2]) < 1e-9:
            continue
        vp = cross[:2] / cross[2]
        dists = np.array([point_line_distance(vp, c) for c in coeffs])
        n_in = int((dists < inlier_px).sum())
        if n_in > best_inliers:
            best_inliers, best_vp = n_in, vp
    if best_vp is None:
        return None, 0
    dists = np.array([point_line_distance(best_vp, c) for c in coeffs])
    inlier_lines = lines[dists < inlier_px]
    if len(inlier_lines) >= 2:
        A = np.array([homogeneous(l)[:2] for l in inlier_lines])
        b = -np.array([homogeneous(l)[2] for l in inlier_lines])
        refined, *_ = np.linalg.lstsq(A, b, rcond=None)
        return refined, len(inlier_lines)
    return best_vp, best_inliers


def main():
    estimates = []
    for idx in CANDIDATE_FRAMES:
        gray = load_gray(idx)
        lines = detect_lines(gray)
        if len(lines) < 30:
            continue
        fam1_mask = dominant_family(lines)
        fam1 = lines[fam1_mask]
        rest = lines[~fam1_mask]
        if len(fam1) < 10:
            continue
        v1, n1 = vp_ransac(fam1, inlier_px=4.0)
        v2, n2 = vp_ransac(rest, inlier_px=4.0)
        if v1 is None or v2 is None or n1 < 8 or n2 < 15:
            print(f"frame {idx}: insufficient VP support (fam1 n={n1}, fam2 n={n2}) -- skipped")
            continue
        cx, cy = 512.0, 392.915659
        d1, d2 = np.array(v1) - [cx, cy], np.array(v2) - [cx, cy]
        dot = float(np.dot(d1, d2))
        if dot >= 0:
            print(f"frame {idx}: v1={v1.round(0)} v2={v2.round(0)} dot={dot:.0f} (>=0, orthogonality "
                  f"constraint violated) -- skipped")
            continue
        f = float(np.sqrt(-dot))
        estimates.append(dict(frame=idx, n1=n1, n2=n2, v1=list(map(float, v1)), v2=list(map(float, v2)), f=f))
        print(f"frame {idx}: fam1 n={n1} v1={np.round(v1,0)}  fam2 n={n2} v2={np.round(v2,0)}  f_estimate={f:.0f}")

    if estimates:
        f_values = np.array([e["f"] for e in estimates])
        print(f"\nusable frames: {len(f_values)}/{len(CANDIDATE_FRAMES)}")
        print(f"f estimate: median {np.median(f_values):.0f}px, mean {f_values.mean():.0f}px, "
              f"std {f_values.std():.0f}px, range [{f_values.min():.0f}, {f_values.max():.0f}]")
        print("compare: OSCalibration.mat fy=1403.5px, width-scaled fx=1508.0px")
    else:
        print("\nNo frame produced two well-supported, sufficiently orthogonal vanishing points.")
    (OUT / "vanishing_point_estimates.json").write_text(json.dumps(estimates, indent=2))


if __name__ == "__main__":
    main()
