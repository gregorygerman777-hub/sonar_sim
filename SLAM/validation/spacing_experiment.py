"""Frame spacing experiment on the synthetic pool: which condition breaks tracking?

The synthetic arc moves 1.25 degrees around the rock per frame, and the caustics evolve 0.1 s per frame.
Running on 1 frame in n multiplies both: n x 1.25 degrees of viewpoint change and n x 0.1 s of caustic
motion between consecutive frames. Comparing the static scene with the caustic scene at the same n
separates the two causes. Reads results/synthetic_pool{,_caustics}_{orb,sift}[_strideN]/ and writes
figures/pool/spacing_experiment.png and results/spacing_experiment.json.
"""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
DEG_PER_FRAME = 300.0 / 239


def collect():
    rows = []
    for run in sorted(RESULTS.glob("synthetic_pool*")):
        meta_p = run / "run_meta.json"
        if not meta_p.exists() or meta_p.read_text() == "":
            continue
        meta = json.loads(meta_p.read_text())
        if meta.get("method", "monoslam") != "monoslam":
            continue
        m = json.loads((run / "metrics.json").read_text()) if (run / "metrics.json").exists() else {}
        stride = int(meta.get("stride", 1))
        rows.append(dict(run=run.name, scene="caustics" if "caustics" in meta["dataset"] else "static",
                         frontend=meta["frontend"], stride=stride, deg_per_frame=stride * DEG_PER_FRAME,
                         caustic_seconds_per_frame=0.1 * stride if "caustics" in meta["dataset"] else 0.0,
                         fraction_posed=meta.get("fraction_posed", 0.0),
                         ate_rmse_m=(m.get("ate_m") or {}).get("rmse"),
                         ate_percent=m.get("ate_rmse_percent_of_path")))
    return rows


def main():
    rows = collect()
    (RESULTS / "spacing_experiment.json").write_text(json.dumps(rows, indent=2))
    out = HERE / "figures" / "pool"
    out.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    styles = {("static", "orb"): ("#2563eb", "o-"), ("caustics", "orb"): ("#dc2626", "s-"),
              ("static", "sift"): ("#2563eb", "o--"), ("caustics", "sift"): ("#dc2626", "s--")}
    for key, (color, style) in styles.items():
        sel = sorted((r for r in rows if (r["scene"], r["frontend"]) == key), key=lambda r: r["stride"])
        if not sel:
            continue
        x = [r["deg_per_frame"] for r in sel]
        label = f"{key[0]} scene, {key[1].upper()}"
        axes[0].plot(x, [100 * r["fraction_posed"] for r in sel], style, color=color, label=label)
        pts = [(r["deg_per_frame"], r["ate_percent"]) for r in sel if r["ate_percent"] is not None]
        if pts:
            axes[1].plot(*zip(*pts), style, color=color, label=label)
    axes[0].set_ylabel("frames posed [%]")
    axes[0].set_ylim(-5, 105)
    axes[1].set_ylabel("ATE RMSE [% of path length]")
    axes[1].set_yscale("log")
    for ax in axes:
        ax.set_xlabel("viewpoint change between consecutive frames [deg around the rock]")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle("Synthetic pool with exact ground truth: frame spacing vs moving caustics "
                 "(caustics advance 0.1 s per original frame)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "spacing_experiment.png", dpi=200)
    print(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
