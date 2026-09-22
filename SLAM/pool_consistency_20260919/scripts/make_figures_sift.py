# Figures for the SIFT cross-check section: connectivity/residual comparison
# and the NLS-vs-spectral disagreement histogram.
import pickle
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

import matlab_style
matlab_style.apply()

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
FIG = OUT / "figures"

with open(OUT / "sift_pipeline_result.pkl", "rb") as fh:
    sift = pickle.load(fh)
diffs = sift["spectral_nls_diffs"]

# Fig 11: NLS-vs-spectral disagreement, ORB (Sec 4.7 style, from ORB ambiguity) vs SIFT
fig, ax = plt.subplots(figsize=(7.0, 4.6))
ax.hist(diffs, bins=24, range=(0, 180), color=matlab_style.MATLAB_COLORS[0],
        edgecolor="black", linewidth=0.4, label=f"SIFT graph, all 117 frames (n={len(diffs)})")
ax.axvline(np.median(diffs), color=matlab_style.MATLAB_COLORS[1], linewidth=2, linestyle="--",
           label=f"median = {np.median(diffs):.0f} deg")
ax.set_xlabel("disagreement between NLS and spectral estimates (deg)")
ax.set_ylabel("count (frames)")
ax.set_title("Two independent rotation estimators, same SIFT graph: do they agree?")
ax.legend(fontsize=9)
fig.tight_layout()
fig.savefig(FIG / "fig11_sift_nls_spectral_disagreement.png")
print("wrote fig11")

# Fig 12: side-by-side bar comparison, ORB vs SIFT, on 4 metrics
fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
labels = ["ORB", "SIFT"]
conn = [80, 117]
axes[0].bar(labels, conn, color=[matlab_style.MATLAB_COLORS[1], matlab_style.MATLAB_COLORS[0]])
axes[0].set_ylim(0, 132)
axes[0].set_ylabel("frames connected (of 117)")
axes[0].set_title("Connectivity after pruning")
for i, v in enumerate(conn):
    axes[0].text(i, v + 2, str(v), ha="center")

resid = [0.90, 5.47]
axes[1].bar(labels, resid, color=[matlab_style.MATLAB_COLORS[1], matlab_style.MATLAB_COLORS[0]])
axes[1].set_ylabel("NLS median residual (deg)")
axes[1].set_title("Local fit quality\n(looks like ORB wins)")
for i, v in enumerate(resid):
    axes[1].text(i, v + 0.1, f"{v:.2f}", ha="center")

# ORB: fraction of frames confirmed unstable via 6-init test (40/80 in the diagnosed
# subgraph); SIFT: fraction disagreeing >10deg between NLS and spectral, all 117 frames
frac_bad = [40/80*100, (diffs > 10).mean()*100]
axes[2].bar(labels, frac_bad, color=[matlab_style.MATLAB_COLORS[1], matlab_style.MATLAB_COLORS[0]])
axes[2].set_ylim(0, 100)
axes[2].set_ylabel("% of frames with no certified answer")
axes[2].set_title("What independent cross-checking\nactually finds")
for i, v in enumerate(frac_bad):
    axes[2].text(i, v + 2, f"{v:.0f}%", ha="center")

fig.suptitle("Local residual looks better under SIFT; independent cross-checking says otherwise", fontsize=12)
fig.tight_layout()
fig.savefig(FIG / "fig12_orb_vs_sift_comparison.png")
print("wrote fig12")
