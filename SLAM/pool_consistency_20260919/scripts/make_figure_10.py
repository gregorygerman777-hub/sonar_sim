import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import matlab_style
matlab_style.apply()

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
d = json.loads((OUT / "translation_sync_result.json").read_text())
resid = np.array(d["residuals"])

fig, ax = plt.subplots(figsize=(6.6, 4.4))
ax.hist(resid, bins=24, range=(0, 180), color=matlab_style.MATLAB_COLORS[0], edgecolor="black", linewidth=0.4)
ax.axvline(np.median(resid), color=matlab_style.MATLAB_COLORS[1], linewidth=2, linestyle="--",
           label=f"median = {np.median(resid):.0f} deg")
ax.set_xlabel("angle between measured and reconstruction-predicted translation direction (deg)")
ax.set_ylabel("count")
ax.set_title(f"Translation-direction residual, block A (n={len(resid)} edges, inliers$\\geq$12)")
ax.legend(fontsize=9)
fig.tight_layout()
fig.savefig(OUT / "figures" / "fig10_translation_residual.png")
print("wrote fig10_translation_residual.png")
