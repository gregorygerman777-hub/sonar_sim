"""Pool scale: measure the floor lane stripes in the COLMAP model's units, so one known length fixes the scale.

    python SLAM/validation/pool_scale.py [--run results/pool_raw_colmap_exhaustive]

The pool reconstruction is monocular, so its unit is arbitrary. No sonar data exists for the same instants, so the
sonar extrinsic in OSCalibration.mat cannot fix it. What the frames do contain is the pool floor's dark lane stripes.
This script
1. fits the floor plane to the map points (RANSAC, then least squares on the inliers),
2. finds the stripe direction from the gradient structure tensor of a median floor mosaic,
3. rectifies each frame onto the floor plane (a top view in model units, using the frame's COLMAP pose and K) and
   takes the median intensity profile across the stripes,
4. detects dark dips, keeps those wider than MIN_STRIPE_WIDTH (pebbles on the mat are narrower), clusters them by
   position and reports each stripe's width (full width at half depth) and the centre spacing, with the spread
   over frames.
The measured widths are in model units. The metric scale is length_m / width_units once the real stripe width (or
spacing) is known; results/pool_scale.json lists what the common competition values would imply, as conditional
numbers only. No ground truth exists here and nothing was tuned to a target value.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import maximum_filter1d, uniform_filter1d

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import export  # noqa: E402
import sequences  # noqa: E402

MIN_STRIPE_WIDTH = 0.25    # as a fraction of the camera height above the floor (pebbles are about 0.05 to 0.15)
MIN_DEPTH_GREY = 30        # grey levels between the floor and the dip bottom


def fit_plane(xyz, cameras, rng=None, iters=2000, rel_thresh=0.01):
    """Floor plane (point, unit normal towards the cameras) and inlier mask."""
    rng = rng or np.random.default_rng(0)
    scale = np.median(np.linalg.norm(xyz - np.median(xyz, 0), axis=1))
    thresh = rel_thresh * scale
    best, best_n, best_p = 0, None, None
    for _ in range(iters):
        p = xyz[rng.choice(len(xyz), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-12:
            continue
        n /= np.linalg.norm(n)
        count = np.sum(np.abs((xyz - p[0]) @ n) < thresh)
        if count > best:
            best, best_n, best_p = count, n, p[0]
    inl = np.abs((xyz - best_p) @ best_n) < thresh
    c0 = xyz[inl].mean(0)
    n = np.linalg.svd(xyz[inl] - c0, full_matrices=False)[2][-1]
    if np.mean((cameras - c0) @ n) < 0:
        n = -n
    return c0, n, inl


def rectify(image, K, R_wc, t_wc, grid_world, zmax):
    """Sample a grey image at world points on the floor (grid_world: H x W x 3). Unseen cells are NaN."""
    Xc = (grid_world - t_wc) @ R_wc
    z = Xc[..., 2]
    uv = Xc @ K.T
    with np.errstate(divide="ignore", invalid="ignore"):
        mx = (uv[..., 0] / z).astype(np.float32)
        my = (uv[..., 1] / z).astype(np.float32)
    h, w = image.shape[:2]
    ok = (z > 1e-6) & (z < zmax) & (mx >= 0) & (mx < w - 1) & (my >= 0) & (my < h - 1)
    out = cv2.remap(image.astype(np.float32), np.where(ok, mx, -1), np.where(ok, my, -1), cv2.INTER_LINEAR)
    out[~ok] = np.nan
    return out


def stripe_dips(profile, coords, min_width, min_depth=MIN_DEPTH_GREY, side=20):
    """Dark dips in a 1D profile: list of (centre, width at half depth, depth, dark level). NaN = unseen."""
    res = coords[1] - coords[0]
    good = ~np.isnan(profile)
    if good.sum() < 2 * side:
        return []
    p = uniform_filter1d(np.where(good, profile, np.nanmax(profile)), 3)
    base = maximum_filter1d(p, max(int(3 * min_width / res), 3))
    dip = ((base - p) > min_depth) & good
    starts = np.flatnonzero(np.diff(np.r_[0, dip.astype(int)]) == 1)
    ends = np.flatnonzero(np.diff(np.r_[dip.astype(int), 0]) == -1)
    out = []
    for i0, i1 in zip(starts, ends):
        if i0 < side or i1 > len(p) - side - 1 or not good[i0 - side:i0].all() or not good[i1 + 1:i1 + 1 + side].all():
            continue
        k = i0 + int(np.argmin(p[i0:i1 + 1]))
        dark = p[k]
        bl, br = np.median(p[i0 - side:i0]), np.median(p[i1 + 1:i1 + 1 + side])
        hl, hr = (bl + dark) / 2, (br + dark) / 2
        i = k
        while i > i0 - side and p[i] < hl:
            i -= 1
        j = k
        while j < i1 + side and p[j] < hr:
            j += 1
        xl = coords[i] + (hl - p[i]) / (p[i + 1] - p[i]) * res
        xr = coords[j - 1] + (hr - p[j - 1]) / (p[j] - p[j - 1]) * res
        if xr - xl >= min_width:
            out.append((float((xl + xr) / 2), float(xr - xl), float(min(bl, br) - dark), float(dark)))
    return out


def is_elongated(rect, coords, centre, width, level, parts=3):
    """A stripe is long and straight: the dip must be darker than `level` in each of `parts` consecutive slices of
    the seen rows along the stripe. A rock or pebble on the floor is dark only over a short stretch."""
    cols = np.abs(coords - centre) <= width / 4
    rows = np.flatnonzero(np.sum(~np.isnan(rect[:, cols]), 1) > 0)
    if len(rows) < 3 * parts:
        return False
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return all(np.nanmedian(rect[chunk][:, cols]) < level for chunk in np.array_split(rows, parts))


def stripe_direction(mosaic):
    """Unit vector (in mosaic pixel axes u, v) across the dominant straight edges (structure tensor)."""
    valid = ~np.isnan(mosaic)
    g = cv2.GaussianBlur(np.nan_to_num(mosaic).astype(np.float32), (0, 0), 2)
    gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1)
    inner = cv2.erode(valid.astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
    J = np.array([[np.sum(gx[inner] ** 2), np.sum(gx[inner] * gy[inner])],
                  [np.sum(gx[inner] * gy[inner]), np.sum(gy[inner] ** 2)]])
    return np.linalg.eigh(J)[1][:, 1]


def measure(frames, K, poses, c0, n, centre, height, res_frac=0.005, half_frac=3.0, zmax_frac=3.0):
    """frames: list of grey images; poses: list of (R_wc, t_wc). Lengths are relative to the camera height."""
    e1 = np.cross(n, [1.0, 0, 0]) if abs(n[0]) < 0.9 else np.cross(n, [0, 1.0, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    res, half, zmax = res_frac * height * 2, half_frac * height, zmax_frac * height
    g = np.arange(-half, half, res)
    u, v = np.meshgrid(g, g)
    grid = centre + u[..., None] * e1 + v[..., None] * e2
    stack = np.array([rectify(f, K, R, t, grid, zmax) for f, (R, t) in zip(frames, poses)])
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            mosaic = np.nanmedian(stack, 0)
    d = stripe_direction(mosaic)
    across = d[0] * e1 + d[1] * e2
    along = np.cross(n, across)
    res = res_frac * height
    A = np.arange(-half, half, res)
    B = np.arange(-half, half, res)
    aa, bb = np.meshgrid(A, B)
    grid = centre + aa[..., None] * across + bb[..., None] * along
    dips = []
    for k, (f, (R, t)) in enumerate(zip(frames, poses)):
        r = rectify(f, K, R, t, grid, zmax)
        cnt = np.sum(~np.isnan(r), 0)
        with np.errstate(all="ignore"):
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                prof = np.where(cnt > 40, np.nanmedian(r, 0), np.nan)
        for c, w, depth, dark in stripe_dips(prof, A, MIN_STRIPE_WIDTH * height):
            if is_elongated(r, A, c, w, dark + depth / 2):
                dips.append((k, c, w, depth))
    return np.array(dips).reshape(-1, 4), across


def cluster(dips, gap):
    """Group dips by centre position (sorted, split where consecutive centres differ by more than gap)."""
    if len(dips) == 0:
        return []
    order = np.argsort(dips[:, 1])
    groups, cur = [], [order[0]]
    for a, b in zip(order[:-1], order[1:]):
        if dips[b, 1] - dips[a, 1] > gap:
            groups.append(dips[cur])
            cur = []
        cur.append(b)
    groups.append(dips[cur])
    return groups


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, default=HERE / "results" / f"pool_{sequences.POOL_PRIMARY}_colmap_exhaustive")
    args = parser.parse_args()
    meta = json.loads((args.run / "run_meta.json").read_text())
    ts, R, t = export.read_tum(args.run / "trajectory_tum.txt")
    xyz, _ = export.read_ply(args.run / "points.ply")
    seq = sequences.get(meta["dataset_key"])
    K = np.asarray(meta["K"], float)
    c0, n, inl = fit_plane(xyz, t)
    heights = (t - c0) @ n
    height = float(np.median(heights))
    centre = t.mean(0) - ((t.mean(0) - c0) @ n) * n
    index = [int(round(x)) - 1 for x in ts]
    frames = [seq.load(i)[0] for i in index]
    dips, _ = measure(frames, K, list(zip(R, t)), c0, n, centre, height)
    stripes = []
    for g in cluster(dips, gap=MIN_STRIPE_WIDTH * height):
        if len(set(g[:, 0].astype(int))) < 3:     # seen in fewer than 3 frames: not reported
            continue
        stripes.append(dict(centre_units=float(np.median(g[:, 1])), width_units_median=float(np.median(g[:, 2])),
                            width_units_iqr=[float(x) for x in np.percentile(g[:, 2], [25, 75])],
                            width_units_range=[float(g[:, 2].min()), float(g[:, 2].max())],
                            detections=int(len(g)), frames=sorted({int(ts[int(k)]) for k in g[:, 0]})))
    out = dict(run=args.run.name, floor_plane_inliers=int(inl.sum()), map_points=int(len(xyz)),
               camera_height_units=dict(median=height, iqr=[float(x) for x in np.percentile(heights, [25, 75])]),
               stripes=stripes, min_stripe_width_units=MIN_STRIPE_WIDTH * height)
    if len(stripes) >= 2:
        out["stripe_spacing_units"] = float(abs(stripes[-1]["centre_units"] - stripes[0]["centre_units"]))
        out["spacing_over_width"] = out["stripe_spacing_units"] / float(np.median([s["width_units_median"] for s in stripes]))
    if stripes:
        w = float(np.median([s["width_units_median"] for s in stripes]))
        out["conditional_scale"] = {
            "note": "metres per model unit IF the stripe width is the stated value; not a measurement of the pool",
            **{f"stripe_width_{m:.3f}_m": dict(metres_per_unit=m / w, camera_height_m=height * m / w)
               for m in (0.20, 0.254, 0.25, 0.30)}}
    (HERE / "results" / "pool_scale.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
