"""Pool sequence (no ground truth): cross-method agreement, coverage, and capture timing.

    python SLAM/validation/pool_analysis.py

Without ground truth, the evidence is:
1. Coverage: how many of the 117 frames each method could pose.
2. Agreement: two pipelines (ours, COLMAP) are aligned by Sim(3) on their common frames and
   the RMS position disagreement is reported as a percentage of the reference trajectory's extent. Rotation
   disagreement after alignment is reported in degrees. Agreement does not prove correctness, but large
   disagreement shows at least one of them is wrong. The rotation after a position-only alignment is poorly
   determined when the common frames are few and nearly collinear, so an alignment free measure is also given:
   the relative rotation between consecutive common frames, compared between the two methods.
3. Reprojection error of each final map.
4. Capture timing from the files' modification times (the only timing information delivered with the frames).
5. Intrinsics comparison: the primary K model (raw, confirmed by Dr. Negahdaripour) against the width scaled
   hypothesis, by registered frames, reprojection error and map size for each COLMAP matcher.
Writes results/pool_analysis.json and figures/pool/{agreement.png, coverage.png}.
"""

import itertools
import json
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import export, geometry as geo  # noqa: E402
from sequences import POOL_PRIMARY  # noqa: E402

RESULTS = HERE / "results"
FIG = HERE / "figures" / "pool"
POOL_DIR = Path(os.environ.get("POOL_DIR", Path.home() / "Downloads/Archive 2"))
PRIMARY = f"pool_{POOL_PRIMARY}"   # the raw K, confirmed by Dr. Negahdaripour on 2026-09-23
COLORS = {"ours (ORB)": "#2563eb", "ours (SIFT)": "#7c3aed", "COLMAP (sequential)": "#f59e0b",
          "COLMAP (exhaustive)": "#b45309"}


def label(meta):
    if meta.get("method", "monoslam") == "monoslam":
        return f"ours ({meta['frontend'].upper()})"
    return f"COLMAP ({meta['method'].split('_', 1)[1]})"


def load(run):
    meta = json.loads((run / "run_meta.json").read_text())
    ts, R, t = export.read_tum(run / "trajectory_tum.txt")
    return dict(name=run.name, label=label(meta), meta=meta, ts=np.round(ts).astype(int), R=R, t=t)


def agreement(a, b):
    common, ia, ib = np.intersect1d(a["ts"], b["ts"], return_indices=True)
    if len(common) < 4:
        return dict(common_frames=int(len(common)))
    s, R, t = geo.umeyama(a["t"][ia], b["t"][ib])
    aligned = s * a["t"][ia] @ R.T + t
    err = np.linalg.norm(aligned - b["t"][ib], axis=1)
    extent = float(np.linalg.norm(np.ptp(b["t"][ib], axis=0)))
    rot = []
    for i, j in zip(ia, ib):
        dR = b["R"][j].T @ (R @ a["R"][i])
        rot.append(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
    # Alignment free check: relative rotation between consecutive common frames, compared across the two methods.
    rel = []
    for k in range(len(ia) - 1):
        ra = a["R"][ia[k]].T @ a["R"][ia[k + 1]]
        rb = b["R"][ib[k]].T @ b["R"][ib[k + 1]]
        d = ra.T @ rb
        rel.append(np.degrees(np.arccos(np.clip((np.trace(d) - 1) / 2, -1, 1))))
    return dict(relative_rotation_median_deg=float(np.median(rel)), relative_rotation_max_deg=float(np.max(rel)),
                common_frames=int(len(common)), rms_percent_of_extent=float(100 * np.sqrt(np.mean(err ** 2)) / extent),
                max_percent_of_extent=float(100 * err.max() / extent), rotation_median_deg=float(np.median(rot)),
                rotation_max_deg=float(np.max(rot)))


def file_times():
    times = {}
    for i in range(1, 118):
        p = POOL_DIR / f"opt{i}.bmp"
        if p.exists():
            times[i] = p.stat().st_mtime
    if len(times) < 2:
        return None
    idx = np.array(sorted(times))
    t = np.array([times[i] for i in idx])
    gaps = np.diff(t)
    big = [dict(after_frame=int(idx[k]), before_frame=int(idx[k + 1]), seconds=float(gaps[k]))
           for k in np.flatnonzero(gaps > 60)]
    return dict(first_mtime=float(t[0]), last_mtime=float(t[-1]), median_gap_s=float(np.median(gaps)),
                max_gap_s=float(gaps.max()), gaps_over_60s=big, monotonic=bool(np.all(gaps >= 0)))


def caustic_index():
    """Crude per frame index of thin bright structure on the floor (lower image half): fraction of pixels whose
    white top-hat (15 px disc) exceeds 25 grey levels. Moving caustics raise it; tile grout and pebbles give a
    baseline of about 0.06. It is a descriptive proxy, not a calibrated caustic measure."""
    import cv2
    out = {}
    for i in range(1, 118):
        p = POOL_DIR / f"opt{i}.bmp"
        im = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if im is None:
            continue
        floor = im[im.shape[0] // 2:, :].astype(np.float32)
        th = cv2.morphologyEx(floor, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
        out[i] = float(np.mean(th > 25))
    return out


def k_model_comparison(out):
    """Primary (raw) K against width scaled K, per COLMAP matcher: frames, reprojection error, points, agreement."""
    rows = {}
    for matcher in ("exhaustive", "sequential"):
        a, b = f"{PRIMARY}_colmap_{matcher}", f"pool_width_scaled_colmap_{matcher}"
        if a == b or a not in out["runs"] or b not in out["runs"]:
            continue
        pick = lambda r: dict(frames_posed=r["frames_posed"], models=r["models"], map_points=r["map_points"],
                              reprojection_mean_px=(r["reprojection"] or {}).get("mean_px"))
        rows[matcher] = dict(primary=pick(out["runs"][a]), width_scaled=pick(out["runs"][b]),
                             agreement=out["agreement"].get(f"{a} -> {b}"))
    return rows


def main():
    runs = [load(r) for r in sorted(RESULTS.glob("pool_*")) if (r / "trajectory_tum.txt").exists()]
    out = dict(runs={}, agreement={}, file_times=file_times(), caustic_index=caustic_index())
    for r in runs:
        m = r["meta"]
        out["runs"][r["name"]] = dict(label=r["label"], k_hypothesis=m.get("dataset"), frames_posed=m.get("frames_posed"),
                                      frames_total=m.get("frames_total"), reprojection=m.get("reprojection"),
                                      keyframes=m.get("keyframes"), map_points=m.get("map_points"),
                                      status_counts=m.get("status_counts"), models=m.get("models"),
                                      initialization=m.get("initialization"))
    for a, b in itertools.permutations(runs, 2):
        out["agreement"][f"{a['name']} -> {b['name']}"] = agreement(a, b)
    out["primary"] = PRIMARY
    out["k_model_comparison"] = k_model_comparison(out)
    (RESULTS / "pool_analysis.json").write_text(json.dumps(out, indent=2))
    FIG.mkdir(parents=True, exist_ok=True)

    # Coverage: which frames each run posed.
    fig, ax = plt.subplots(figsize=(12, 0.6 + 0.45 * len(runs)))
    for k, r in enumerate(runs):
        ax.scatter(r["ts"], np.full(len(r["ts"]), k), s=9, marker="s",
                   color=COLORS.get(r["label"], "#6b7280"))
    ax.set_yticks(range(len(runs)))
    ax.set_yticklabels([f"{r['label']}  [{r['meta']['dataset'].replace('pool_', 'K ')}]  "
                        f"{len(r['ts'])}/117" for r in runs], fontsize=8)
    ft = out["file_times"]
    if ft:
        for g in ft["gaps_over_60s"]:
            ax.axvline(g["after_frame"] + 0.5, color="#dc2626", lw=1)
            ax.text(g["after_frame"] + 0.7, len(runs) - 0.6, f"{g['seconds'] / 60:.1f} min capture gap",
                    color="#dc2626", fontsize=8)
    ci = out["caustic_index"]
    if ci:
        ax2 = ax.twinx()
        ax2.plot(list(ci.keys()), list(ci.values()), color="#0891b2", lw=1.2, alpha=0.8)
        ax2.set_ylabel("floor caustic index", color="#0891b2", fontsize=8)
        ax2.tick_params(axis="y", labelcolor="#0891b2", labelsize=7)
    ax.set_xlim(0, 118)
    ax.set_xlabel("frame number (opt<n>.bmp)")
    ax.set_title("Pool sequence: frames given a pose by each method", fontsize=10)
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG / "coverage.png", dpi=200)
    plt.close(fig)

    # Agreement: every run aligned onto COLMAP exhaustive (or the run with most frames), top and side views.
    ref = next((r for r in runs if r["label"] == "COLMAP (exhaustive)" and r["meta"]["dataset"] == PRIMARY),
               max(runs, key=lambda r: len(r["ts"])) if runs else None)
    if ref is not None:
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        centred = ref["t"] - ref["t"].mean(0)
        basis = np.linalg.svd(centred, full_matrices=False)[2]
        for r in runs:
            if r["meta"]["dataset"] != ref["meta"]["dataset"]:
                continue
            if r is ref:
                p = r["t"]
            else:
                common, ia, ib = np.intersect1d(r["ts"], ref["ts"], return_indices=True)
                if len(common) < 4:
                    continue
                s, R, t = geo.umeyama(r["t"][ia], ref["t"][ib])
                p = s * r["t"] @ R.T + t
            q = (p - ref["t"].mean(0)) @ basis.T
            for ax, (i, j) in zip(axes, ((0, 1), (0, 2))):
                ax.plot(q[:, i], q[:, j], ".-", ms=3, lw=0.8, color=COLORS.get(r["label"], "#6b7280"),
                        label=f"{r['label']} ({len(r['ts'])} frames)")
        for ax, name in zip(axes, ("main plane", "side")):
            ax.set_title(name)
            ax.axis("equal")
            ax.grid(alpha=0.3)
            ax.set_xlabel("principal axis 1 [COLMAP units]")
        axes[0].set_ylabel("principal axis 2")
        axes[1].set_ylabel("principal axis 3")
        axes[0].legend(fontsize=8)
        fig.suptitle(f"Pool: trajectories Sim(3) aligned onto {ref['label']} on common frames (no ground truth)",
                     fontsize=10)
        fig.tight_layout()
        fig.savefig(FIG / "agreement.png", dpi=200)
        plt.close(fig)
    # Frame order along each COLMAP trajectory: which frames form which part of the path (two capture passes?).
    for r in runs:
        if not r["label"].startswith("COLMAP") or r["meta"]["dataset"] != PRIMARY:
            continue
        c = r["t"] - r["t"].mean(0)
        basis = np.linalg.svd(c, full_matrices=False)[2]
        q = c @ basis.T
        fig, ax = plt.subplots(figsize=(6.5, 4.6))
        sc = ax.scatter(q[:, 0], q[:, 1], c=r["ts"], cmap="viridis", s=18, zorder=3)
        ax.plot(q[:, 0], q[:, 1], color="#9ca3af", lw=0.6, zorder=2)
        for k in range(len(r["ts"])):
            if r["ts"][k] % 10 == 0 or r["ts"][k] in (1, 105, 106, 117):
                ax.annotate(str(r["ts"][k]), q[k, :2], fontsize=7, xytext=(3, 3), textcoords="offset points")
        plt.colorbar(sc, label="frame number")
        ax.axis("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("principal axis 1 [arbitrary units]")
        ax.set_ylabel("principal axis 2 [arbitrary units]")
        ax.set_title(f"Pool, {r['label']}: camera positions by frame number", fontsize=9)
        fig.tight_layout()
        fig.savefig(FIG / f"frame_order_{r['name'].split('colmap_')[1]}.png", dpi=200)
        plt.close(fig)
    print(json.dumps(out["agreement"], indent=1)[:3000])
    print(json.dumps(out["file_times"], indent=1))


if __name__ == "__main__":
    main()
