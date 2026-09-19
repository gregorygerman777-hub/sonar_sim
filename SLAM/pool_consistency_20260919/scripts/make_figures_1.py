# Figures that don't need the threshold sweep: matchability decay, failure
# correlation scatter, R_H planarity histogram, calibration sensitivity.
import json
import pickle
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import matlab_style
matlab_style.apply()

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
FIG = OUT / "figures"
FIG.mkdir(exist_ok=True)

with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
    all_scaled = pickle.load(fh)
with open(OUT / "pairwise_scaled_consecutive.pkl", "rb") as fh:
    consec_scaled = pickle.load(fh)
with open(OUT / "pairwise_raw_consecutive.pkl", "rb") as fh:
    consec_raw = pickle.load(fh)
with open(OUT / "features.pkl", "rb") as fh:
    frames = pickle.load(fh)["frames"]
graph_summary = json.loads((OUT / "graph_summary.json").read_text())

results = all_scaled["results"]
sep = np.array([abs(r["j"] - r["i"]) for r in results])
inliers = np.array([r["inliers"] for r in results])

# --- Figure 1: matchability decay vs frame separation --------------------
fig, ax = plt.subplots(figsize=(6.2, 4.4))
bins = np.arange(1, sep.max() + 1)
mean_inl = [inliers[sep == s].mean() for s in bins]
frac_verified = [(inliers[sep == s] >= 20).mean() * 100 for s in bins]
ax.plot(bins, mean_inl, color=matlab_style.MATLAB_COLORS[0], linewidth=1.4, label="mean essential-matrix inliers")
ax2 = ax.twinx()
ax2.plot(bins, frac_verified, color=matlab_style.MATLAB_COLORS[1], linewidth=1.4, linestyle="--",
         label=r"fraction with $\geq$20 inliers (%)")
ax2.set_ylabel("pairs with $\\geq$20 inliers (%)")
ax2.grid(False)
ax.set_xlabel("frame separation $|i-j|$")
ax.set_ylabel("mean inlier count")
ax.set_title("Matchability decay with frame separation, all 6786 pairs")
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8.5)
fig.tight_layout()
fig.savefig(FIG / "fig1_matchability_decay.png")
plt.close(fig)

# --- Figure 2: failure correlation scatter (contrast, Laplacian variance) --
consec = {r["i"]: r for r in consec_scaled["results"]}
step_inl, lap, contrast = [], [], []
for k in range(1, len(frames)):
    r = consec.get(k)
    if r is None:
        continue
    step_inl.append(r["inliers"])
    lap.append(frames[k]["lap_var"])
    contrast.append(frames[k - 1]["contrast"])
step_inl, lap, contrast = map(np.array, (step_inl, lap, contrast))

fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.4))
for ax, x, xlabel, r in zip(axes, [lap, contrast], ["Laplacian variance (sharpness)", "intensity std (contrast)"],
                            [graph_summary["correlations"]["lap_var"]["r"], graph_summary["correlations"]["contrast"]["r"]]):
    ax.scatter(x, step_inl, s=22, alpha=0.75, edgecolor="black", linewidth=0.4, color=matlab_style.MATLAB_COLORS[0])
    z = np.polyfit(x, step_inl, 1)
    xs = np.linspace(x.min(), x.max(), 50)
    ax.plot(xs, np.polyval(z, xs), color=matlab_style.MATLAB_COLORS[1], linewidth=1.5)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("essential-matrix inliers (consecutive step)")
    ax.set_title(f"r = {r:.2f}")
fig.suptitle("Tracking quality vs. per-frame image statistics (n=116 consecutive steps)", fontsize=12)
fig.tight_layout()
fig.savefig(FIG / "fig2_failure_correlation.png")
plt.close(fig)

# --- Figure 3: R_H planarity-ratio histogram ------------------------------
rh = np.array([r["r_h"] for r in results if np.isfinite(r["r_h"])])
fig, ax = plt.subplots(figsize=(6.2, 4.4))
ax.hist(rh, bins=40, color=matlab_style.MATLAB_COLORS[0], edgecolor="black", linewidth=0.4)
ax.axvline(0.45, color=matlab_style.MATLAB_COLORS[1], linestyle="--", linewidth=1.5,
           label=r"$R_H=0.45$ planarity threshold" "\n(Mur-Artal et al. 2015)")
ax.set_xlabel(r"$R_H = S_H / (S_H + S_F)$")
ax.set_ylabel("count of verified pairs")
ax.set_title(f"Homography-vs-essential model-selection ratio, n={len(rh)} pairs")
ax.legend(fontsize=8.5)
fig.tight_layout()
fig.savefig(FIG / "fig3_planarity_ratio.png")
plt.close(fig)

# --- Figure 4: calibration sensitivity (raw vs scaled K), consecutive chain
rot_scaled = np.array([r["rotation_deg"] for r in consec_scaled["results"]])
rot_raw = np.array([r["rotation_deg"] for r in consec_raw["results"]])
steps = np.arange(1, len(rot_scaled) + 1)
fig, ax = plt.subplots(figsize=(9.5, 4.2))
ax.plot(steps, rot_scaled, color=matlab_style.MATLAB_COLORS[0], linewidth=1.3, label="width-scaled K (primary)")
ax.plot(steps, rot_raw, color=matlab_style.MATLAB_COLORS[1], linewidth=1.1, linestyle="--", label="raw OSCalibration.mat K")
ax.set_xlabel("step (consecutive frame pair)")
ax.set_ylabel("recovered rotation magnitude (deg)")
ax.set_title("Calibration sensitivity: per-step rotation under two K hypotheses")
ax.legend(fontsize=9)
fig.tight_layout()
fig.savefig(FIG / "fig4_calibration_sensitivity.png")
plt.close(fig)

print("wrote figures 1-4 to", FIG)
