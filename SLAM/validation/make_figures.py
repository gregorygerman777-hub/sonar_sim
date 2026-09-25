"""Figures for a run: trajectory, 3D model, and trajectory over the model (PNG at 200 dpi, plus interactive HTML).

    python SLAM/validation/make_figures.py SLAM/validation/results/<run> [--baseline <colmap run>]

With ground truth (metrics.json present) everything is drawn in the ground truth frame after the
Sim(3) alignment from evaluate.py, in metres. Without ground truth it is drawn in the run's own frame
in arbitrary units (monocular scale is unknown), which the axis labels say.

View frame: "up" is the normal of the dominant plane of the map (floor, seabed or desk), found by RANSAC
and oriented towards the cameras; the top view looks down that normal. Map outliers are removed for
display only by a statistical filter (mean distance to the 8 nearest neighbours above the global mean
plus 2 standard deviations); the PLY file on disk keeps every point.
Colours: estimate blue, ground truth black, COLMAP baseline orange, in every figure.
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import export, geometry as geo  # noqa: E402

C_EST, C_GT, C_BASE = "#2563eb", "#111827", "#f59e0b"
DPI = 200
MAX_HTML_POINTS = 100_000   # interactive overlay only (a KITTI map has up to 413,000 points); the PNGs show every point


def statistical_filter(xyz, k=8, n_std=2.0):
    if len(xyz) < k + 2:
        return np.ones(len(xyz), bool)
    d, _ = cKDTree(xyz).query(xyz, k=k + 1)
    m = d[:, 1:].mean(1)
    return m < m.mean() + n_std * m.std()


def dominant_plane_normal(xyz, centers, rng=None, iters=500):
    if len(xyz) < 50:
        return None
    rng = np.random.default_rng(0) if rng is None else rng   # a fresh generator per call: the same map, the same view
    scale = np.median(np.linalg.norm(xyz - np.median(xyz, 0), axis=1))
    thresh = 0.02 * scale
    best, best_n = 0, None
    for _ in range(iters):
        p = xyz[rng.choice(len(xyz), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-12:
            continue
        n /= np.linalg.norm(n)
        count = np.sum(np.abs((xyz - p[0]) @ n) < thresh)
        if count > best:
            best, best_n, best_p = count, n, p[0]
    if best_n is None:
        return None
    inl = np.abs((xyz - best_p) @ best_n) < thresh
    q = xyz[inl] - xyz[inl].mean(0)
    n = np.linalg.svd(q, full_matrices=False)[2][-1]
    if np.mean((centers - xyz[inl].mean(0)) @ n) < 0:
        n = -n
    return n


def view_basis(up, traj):
    """Orthonormal (e1, e2, up): e1 along the trajectory's main direction within the plane."""
    q = traj - traj.mean(0)
    q = q - np.outer(q @ up, up)
    e1 = np.linalg.svd(q, full_matrices=False)[2][0] if len(q) > 2 else np.cross(up, [1, 0, 0])
    e1 -= (e1 @ up) * up
    if np.linalg.norm(e1) < 1e-9:
        e1 = np.cross(up, [0, 1, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    return np.vstack((e1, e2, up))


def load_run(run_dir, aligned):
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "run_meta.json").read_text())
    metrics = json.loads((run_dir / "metrics.json").read_text()) if (run_dir / "metrics.json").exists() else None
    ts, R, t = export.read_tum(run_dir / "trajectory_tum.txt")
    xyz, rgb = export.read_ply(run_dir / "points.ply")
    if aligned and metrics and "sim3_scale" in metrics:
        s = metrics["sim3_scale"]
        Ra, ta = np.array(metrics["sim3_rotation"]), np.array(metrics["sim3_translation"])
        t = s * t @ Ra.T + ta
        R = np.einsum("ij,njk->nik", Ra, R)
        xyz = s * xyz @ Ra.T + ta if len(xyz) else xyz
    return dict(meta=meta, metrics=metrics, ts=ts, R=R, t=t, xyz=xyz, rgb=rgb, dir=run_dir)


def status_line(meta, m):
    """The line under a figure's title: the scores, or why there are none (no ground truth, or too few frames posed
    to align with the ground truth, which is not the same thing)."""
    posed = f"posed {meta['frames_posed']}/{meta['frames_total']} frames"
    if m and "sim3_scale" in m:
        stat = (f"ATE RMSE {m['ate_m']['rmse']:.3f} m ({m['ate_rmse_percent_of_path']:.2f} % of "
                f"{m['gt_path_length_m']:.1f} m), posed {m['fraction_posed']:.0%}" +
                ("  PARTIAL" if m["status"] == "PARTIAL" else ""))
        if m.get("all_maps"):
            am = m["all_maps"]
            stat += (f"\nall {am['maps']} maps: {am['frames_posed']}/{m['frames_total']} frames posed, "
                     f"ATE {am['ate_rmse_m_each_map_aligned_separately']:.3f} m (each map aligned separately)")
        return stat
    if m and m.get("status") == "TOO FEW POSED":
        return f"too few frames posed to align with the ground truth; {posed}"
    return f"no ground truth; {posed}"


def frustum_lines(R_wc, c, K, size, depth):
    w, h = size
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], float)
    rays = np.column_stack(((corners[:, 0] - K[0, 2]) / K[0, 0], (corners[:, 1] - K[1, 2]) / K[1, 1], np.ones(4)))
    pts = c + depth * rays @ R_wc.T
    lines = [(c, p) for p in pts] + [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--baseline", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--frustum-every", type=int, default=0, help="0 chooses about 25 frustums")
    args = parser.parse_args()

    run = load_run(args.run, aligned=True)
    if not len(run["t"]):
        print(f"{args.run.name}: no frame was posed, nothing to draw")
        return
    has_gt = run["metrics"] is not None and "sim3_scale" in run["metrics"]
    base = None
    if args.baseline and (args.baseline / "trajectory_tum.txt").exists():
        base = load_run(args.baseline, aligned=True)
        if base["metrics"] is None or "sim3_scale" not in base["metrics"]:
            base = None if has_gt else base
    units = "m" if has_gt else "arbitrary units"
    out = args.out or (HERE / "figures" / run["meta"]["dataset"] / run["dir"].name)
    out.mkdir(parents=True, exist_ok=True)
    gt = None
    if has_gt:
        g = np.loadtxt(run["dir"] / "associated_gt_tum.txt", comments="#", ndmin=2)
        gt = g[:, 1:4]
        full = _full_gt(run)
    K = np.array(run["meta"]["K"])
    size = tuple(run["meta"].get("image_size", [int(2 * K[0, 2]), int(2 * K[1, 2])]))
    keep = statistical_filter(run["xyz"]) if len(run["xyz"]) else np.zeros(0, bool)
    xyz, rgb = run["xyz"][keep], (run["rgb"][keep] if run["rgb"] is not None else None)
    up = dominant_plane_normal(xyz, run["t"])
    if up is None:
        up = -np.mean(run["R"][:, :, 1], axis=0)
        up /= np.linalg.norm(up)
    B = view_basis(up, run["t"])
    P = lambda a: a @ B.T  # noqa: E731
    traj, cloud = P(run["t"]), P(xyz) if len(xyz) else np.zeros((0, 3))
    title = run["meta"]["dataset"] + "  |  " + run["dir"].name.replace(run["meta"]["dataset"] + "_", "")
    stat = status_line(run["meta"], run["metrics"])

    # ---------------- 1: trajectory
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
    for ax, (a, b), name in ((axes[0], (0, 1), "top view"), (axes[1], (0, 2), "side view")):
        if has_gt:
            fg = P(full)
            ax.plot(fg[:, a], fg[:, b], color=C_GT, lw=1.0, alpha=0.35, label="ground truth, whole run")
            pg = P(gt)
            ax.plot(pg[:, a], pg[:, b], color=C_GT, lw=1.6, label="ground truth")
        if base is not None:
            pb = P(base["t"])
            ax.plot(pb[:, a], pb[:, b], ".", color=C_BASE, ms=2.5, label="COLMAP baseline")
        others = sorted(run["dir"].glob("maps/map_*/aligned_estimate_tum.txt")) if has_gt else []
        for k, other in enumerate(others):
            q = np.loadtxt(other, comments="#", ndmin=2)
            if len(q):
                po = P(q[:, 1:4])
                ax.plot(po[:, a], po[:, b], color="#93c5fd", lw=1.2,
                        label="other maps (after tracking loss, each aligned separately)" if k == 0 else None)
        ax.plot(traj[:, a], traj[:, b], color=C_EST, lw=1.4, label="estimate (Sim(3) aligned)" if has_gt else "estimate")
        ax.plot(traj[0, a], traj[0, b], "o", color="#16a34a", ms=7, label="start")
        ax.plot(traj[-1, a], traj[-1, b], "s", color="#dc2626", ms=7, label="end")
        ax.set_xlabel(f"along track [{units}]")
        ax.set_ylabel(f"{'across track' if b == 1 else 'up'} [{units}]")
        ax.set_title(name)
        ax.axis("equal")
        ax.grid(alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle(f"{title}\n{stat}", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "1_trajectory.png", dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)

    # ---------------- 2: model and 3: overlay
    colors = rgb / 255.0 if rgb is not None and len(rgb) else None
    lim = _limits(np.vstack((cloud, traj)) if len(cloud) else traj)
    for fname, with_traj in (("2_model.png", False), ("3_overlay.png", True)):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(projection="3d")
        ax.computed_zorder = False      # draw in the order below, so the trajectory stays on top of the map points
        if len(cloud):
            ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], c=colors, s=1.2, depthshade=False, linewidths=0,
                       zorder=1)
        if with_traj:
            ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color=C_EST, lw=2.0, label="estimated trajectory", zorder=3)
            if has_gt:
                pg = P(gt)
                ax.plot(pg[:, 0], pg[:, 1], pg[:, 2], color=C_GT, lw=1.0, alpha=0.7, label="ground truth", zorder=2)
            every = args.frustum_every or max(1, len(traj) // 25)
            depth = 0.04 * np.ptp(lim, axis=1).max()
            for i in range(0, len(traj), every):
                for p0, p1 in frustum_lines(B @ run["R"][i], traj[i], K, size, depth):
                    ax.plot(*np.column_stack((p0, p1)), color=C_EST, lw=0.6, zorder=3)
            ax.legend(loc="upper left", fontsize=8)
        ax.set_xlim(*lim[0])
        ax.set_ylim(*lim[1])
        ax.set_zlim(*lim[2])
        ax.set_box_aspect(np.ptp(lim, axis=1))
        ax.zaxis.set_major_locator(MaxNLocator(3))   # a flat box (a street, a pool floor) had overlapping labels
        ax.view_init(elev=28, azim=-60)
        ax.set_xlabel(f"along [{units}]")
        ax.set_ylabel(f"across [{units}]")
        ax.set_zlabel(f"up [{units}]")
        n_pts = len(cloud)
        what = "3D map points" if not with_traj else "trajectory over the 3D map"
        ax.set_title(f"{title}\n{what}: {n_pts} points shown ({len(run['xyz'])} in map), {stat}", fontsize=9)
        fig.tight_layout()
        fig.savefig(out / fname, dpi=DPI, bbox_inches="tight", pad_inches=0.05)
        plt.close(fig)
    # ---------------- 4: overlay seen from above (map points coloured by height, cameras with viewing direction)
    fig, ax = plt.subplots(figsize=(8, 7))
    if len(cloud):
        sc = ax.scatter(cloud[:, 0], cloud[:, 1], c=cloud[:, 2], cmap="cividis", s=1.5, linewidths=0, zorder=1)
        plt.colorbar(sc, ax=ax, shrink=0.7, label=f"map point height [{units}]")
    if has_gt:
        pg = P(gt)
        ax.plot(pg[:, 0], pg[:, 1], color=C_GT, lw=1.0, alpha=0.7, label="ground truth", zorder=2)
    ax.plot(traj[:, 0], traj[:, 1], color=C_EST, lw=1.8, label="estimated trajectory", zorder=3)
    every = args.frustum_every or max(1, len(traj) // 25)
    look = np.einsum("ij,njk->nik", B, run["R"])[:, :, 2]   # optical axis in the view frame
    arrow = 0.05 * np.ptp(lim[:2], axis=1).max()
    for i in range(0, len(traj), every):
        d = look[i, :2] / max(np.linalg.norm(look[i, :2]), 1e-9)
        ax.annotate("", traj[i, :2] + arrow * d, traj[i, :2], zorder=4,
                    arrowprops=dict(arrowstyle="->", color=C_EST, lw=1.0))
    ax.plot(traj[0, 0], traj[0, 1], "o", color="#16a34a", ms=7, label="start", zorder=5)
    ax.plot(traj[-1, 0], traj[-1, 1], "s", color="#dc2626", ms=7, label="end", zorder=5)
    pad = 0.05 * np.ptp(traj[:, :2], axis=0).max()   # the whole trajectory stays inside the axes
    ax.set_xlim(min(lim[0][0], traj[:, 0].min() - pad), max(lim[0][1], traj[:, 0].max() + pad))
    ax.set_ylim(min(lim[1][0], traj[:, 1].min() - pad), max(lim[1][1], traj[:, 1].max() + pad))
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel(f"along [{units}]")
    ax.set_ylabel(f"across [{units}]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"{title}\ntrajectory over the 3D map, seen from above (arrows: viewing direction); {stat}",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(out / "4_overlay_top.png", dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    _interactive(out / "3_overlay.html", cloud, rgb, traj, P(gt) if has_gt else None, title, units)
    print(f"figures written to {out}")


def _full_gt(run):
    import sequences
    seq = sequences.get(run["meta"].get("dataset_key") or run["meta"]["dataset"])
    gt_t, _, gt_p = seq.ground_truth
    stamps = run["meta"].get("frame_timestamps") or list(run["ts"])
    lo, hi = min(stamps), max(stamps)   # the whole run, so frames that were never posed show up too
    sel = (gt_t >= lo) & (gt_t <= hi)
    return gt_p[sel]


def _limits(points):
    lo, hi = np.percentile(points, 1, axis=0), np.percentile(points, 99, axis=0)
    centre, half = (lo + hi) / 2, (hi - lo) / 2
    half = np.maximum(half, 0.05 * half.max()) * 1.1
    return np.column_stack((centre - half, centre + half))


def _interactive(path, cloud, rgb, traj, gt, title, units):
    try:
        import plotly.graph_objects as go
    except ImportError:
        return
    data = []
    if len(cloud) > MAX_HTML_POINTS:          # a random subset with a fixed seed, said in the title
        keep = np.sort(np.random.default_rng(0).choice(len(cloud), MAX_HTML_POINTS, replace=False))
        title = f"{title} ({MAX_HTML_POINTS:,} of {len(cloud):,} map points shown)"
        cloud, rgb = cloud[keep], (rgb[keep] if rgb is not None else None)
    if len(cloud):
        col = [f"rgb({r},{g},{b})" for r, g, b in rgb] if rgb is not None else "gray"
        data.append(go.Scatter3d(x=cloud[:, 0], y=cloud[:, 1], z=cloud[:, 2], mode="markers",
                                 marker=dict(size=1.5, color=col), name="map points"))
    data.append(go.Scatter3d(x=traj[:, 0], y=traj[:, 1], z=traj[:, 2], mode="lines",
                             line=dict(color=C_EST, width=5), name="estimated trajectory"))
    if gt is not None:
        data.append(go.Scatter3d(x=gt[:, 0], y=gt[:, 1], z=gt[:, 2], mode="lines",
                                 line=dict(color=C_GT, width=3), name="ground truth"))
    fig = go.Figure(data=data)
    fig.update_layout(title=title, scene=dict(aspectmode="data", xaxis_title=f"along [{units}]",
                                              yaxis_title=f"across [{units}]", zaxis_title=f"up [{units}]"))
    fig.write_html(str(path), include_plotlyjs="cdn")


if __name__ == "__main__":
    main()
