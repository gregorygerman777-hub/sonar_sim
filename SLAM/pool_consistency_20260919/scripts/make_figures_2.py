"""Figure 6: threshold sweep (self-consistency vs. graph coverage trade-off).
Figure 7: naive sequential chain vs. multi-view-averaged global orientation.
"""
import json
import pickle
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

import matlab_style
matlab_style.apply()

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
FIG = OUT / "figures"

# --- Figure 6: threshold sweep --------------------------------------------
rows = []
for th in (12, 15, 17, 20, 25, 30, 40):
    p = OUT / f"sweep_{th}.json"
    if p.exists():
        rows.append(json.loads(p.read_text()))
rows.sort(key=lambda r: r["threshold"])
th = np.array([r["threshold"] for r in rows])
med = np.array([r["median_residual_deg"] for r in rows])
frac = np.array([r["frac_over_10deg"] * 100 for r in rows])
nodes = np.array([r["n_nodes_connected"] for r in rows])

fig, ax = plt.subplots(figsize=(7.0, 4.8))
l1, = ax.plot(th, med, marker="o", color=matlab_style.MATLAB_COLORS[0], linewidth=1.6, label="median disagreement (deg)")
ax.set_yscale("log")
ax.set_xlabel("inlier threshold admitted into the pose graph")
ax.set_ylabel("median rotation disagreement after averaging (deg, log scale)")
ax2 = ax.twinx()
l2, = ax2.plot(th, nodes, marker="s", color=matlab_style.MATLAB_COLORS[2], linewidth=1.6, linestyle="--",
               label="frames in largest connected component")
ax2.set_ylabel("frames connected (of 117)")
ax2.grid(False)
ax.axvline(20, color="gray", linewidth=1.0, linestyle=":")
ax.text(20.3, med.max() * 0.6, "adopted\nthreshold", fontsize=8.5, color="gray")
ax.set_title("Self-consistency vs. graph coverage trade-off\n(rotation averaging over the verified pairwise graph)")
ax.legend([l1, l2], [l1.get_label(), l2.get_label()], loc="center right", fontsize=8.5)
fig.tight_layout()
fig.savefig(FIG / "fig6_threshold_sweep.png")
plt.close(fig)
print("wrote fig6_threshold_sweep.png")

# --- Figure 7: sequential vs averaged, cumulative heading -----------------
with open(OUT / "rotation_averaging_result.pkl", "rb") as fh:
    rot_result = pickle.load(fh)  # currently holds the threshold=20 run
with open(OUT / "pairwise_scaled_consecutive.pkl", "rb") as fh:
    consec = pickle.load(fh)["results"]

keep = rot_result["keep"]
R_final = rot_result["R_final"]

# naive sequential cumulative rotation angle (signed, about the dominant axis)
seq_cum = [0.0]
for r in consec:
    if r["R"] is None:
        seq_cum.append(seq_cum[-1])
        continue
    rotvec = cv2.Rodrigues(r["R"])[0].ravel()
    seq_cum.append(seq_cum[-1] + np.degrees(rotvec[1]))  # y-axis ~ yaw for this camera orientation

order = sorted(keep)
ref = order[0]
avg_cum = {}
for node in order:
    rel = Rotation.from_matrix(R_final[node] @ R_final[ref].T).as_rotvec()
    avg_cum[node] = np.degrees(rel[1])

fig, ax = plt.subplots(figsize=(10, 4.6))
ax.plot(range(1, 118), seq_cum, color=matlab_style.MATLAB_COLORS[1], linewidth=1.1, linestyle="--",
        label="naive sequential chain (frame-to-frame only)")
xs = [node + 1 for node in order]
ys = [avg_cum[node] for node in order]
ax.plot(xs, ys, color=matlab_style.MATLAB_COLORS[0], linewidth=1.8, marker=".", markersize=4,
        label=f"multi-view rotation averaging (trusted core, {len(order)}/117 frames)")
excluded = [e + 1 for e in rot_result["excluded"]]
for e in excluded:
    ax.axvspan(e - 0.5, e + 0.5, color="gray", alpha=0.12, linewidth=0)
ax.set_xlabel("frame index")
ax.set_ylabel("cumulative yaw-axis rotation (deg, arbitrary reference)")
ax.set_title("Naive frame-to-frame chaining vs. graph-verified multi-view rotation averaging\n"
             "(shaded columns: frames excluded from the trusted core at inlier"+r"$\geq$20)")
ax.legend(loc="upper left", fontsize=8.5)
fig.tight_layout()
fig.savefig(FIG / "fig7_sequential_vs_averaged.png")
plt.close(fig)
print("wrote fig7_sequential_vs_averaged.png")
