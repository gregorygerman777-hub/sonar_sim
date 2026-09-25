"""KITTI 00 to 10 on one page: each sequence seen from above, the estimate over the ground truth.

    python SLAM/validation/kitti_overview.py [--frontend sift]

Reads results/kitti_<nn>_<frontend>/ (the Sim(3) aligned estimate of the primary map and metrics.json, both written by
evaluate.py) and the ground truth of the whole sequence. KITTI's world frame is the first left camera: x right,
y down, z forward, so the view from above is x against z. Writes figures/kitti/overview_<frontend>.png.
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

import sequences  # noqa: E402
from make_figures import C_EST, C_GT, DPI  # noqa: E402


def panel_title(seq, meta, m):
    maps = len(meta.get("maps") or [])
    return f"{seq}: {m['ate_m']['rmse']:.1f} m ({m['ate_rmse_percent_of_path']:.1f} %)" + (
        f", main map of {maps}" if maps > 1 else "")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frontend", default="sift", choices=("orb", "sift"))
    args = parser.parse_args()
    runs = [(f"{i:02d}", HERE / "results" / f"kitti_{i:02d}_{args.frontend}") for i in range(11)]
    runs = [(s, r) for s, r in runs if (r / "metrics.json").exists()]
    if not runs:
        print("no scored KITTI runs")
        return
    fig, axes = plt.subplots(3, 4, figsize=(10, 7.4))
    for ax, (seq, run) in zip(axes.flat, runs):
        meta = json.loads((run / "run_meta.json").read_text())
        m = json.loads((run / "metrics.json").read_text())
        gt = sequences.get(f"kitti_{seq}").ground_truth[2]
        est = np.loadtxt(run / "aligned_estimate_tum.txt", comments="#", ndmin=2)[:, 1:4]
        ax.plot(gt[:, 0], gt[:, 2], color=C_GT, lw=1.1, label="ground truth")
        ax.plot(est[:, 0], est[:, 2], color=C_EST, lw=1.0, label=f"ours ({args.frontend.upper()})")
        ax.plot(gt[0, 0], gt[0, 2], "o", color="#16a34a", ms=4, label="start of the sequence")
        ax.set_title(panel_title(seq, meta, m), fontsize=10)
        ax.set_aspect("equal", adjustable="datalim")
        ax.tick_params(labelsize=8)
        ax.grid(alpha=0.3)
    for ax in axes.flat[len(runs):]:
        ax.axis("off")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    axes.flat[-1].legend(handles, labels, loc="center", fontsize=10.5, frameon=False)
    fig.suptitle(f"KITTI 00 to 10 from above [m], left camera only. ATE after one Sim(3) alignment (% of the posed "
                 f"path)", fontsize=11)
    fig.tight_layout()
    out = HERE / "figures" / "kitti"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"overview_{args.frontend}.png", dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print("wrote", out / f"overview_{args.frontend}.png")


if __name__ == "__main__":
    main()
