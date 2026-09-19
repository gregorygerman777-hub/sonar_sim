"""Figure 8: graph structure showing the single bridge edge (60,82) that gates
the entire unstable block from the rest of the trusted core.
Figure 9: cross-block candidate rotation histogram (sub-threshold population).
"""
import json
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

import sys
sys.path.insert(0, str(OUT))
from validate_spectral_sync import load_real_topology

nodes, edge_list = load_real_topology()
G = nx.Graph()
G.add_nodes_from(nodes)
G.add_edges_from(edge_list)

rows = json.loads((OUT / "multi_init_scan.json").read_text())
unstable = {r["frame"] for r in rows if r["max_disagreement"] > 10}
block_a = {6,7,8,9,10,11,12,13,14,15,16,17,18,61,62,63,64,65,66,67,69,70,71,72,73,74,75,76,77,78,79,80,81,82}
block_b = set(nodes) - block_a

pos = nx.spring_layout(G, seed=7, k=0.6, iterations=200)
fig, ax = plt.subplots(figsize=(8, 6.5))
nx.draw_networkx_edges(G, pos, ax=ax, edge_color="#B0B0B0", width=0.8, alpha=0.7)
nx.draw_networkx_edges(G, pos, edgelist=[(60, 82)], ax=ax, edge_color=matlab_style.MATLAB_COLORS[1], width=3.0)
nx.draw_networkx_nodes(G, pos, nodelist=sorted(block_a), ax=ax, node_color=matlab_style.MATLAB_COLORS[0],
                        node_size=90, label=f"block A, {len(block_a)} frames (frame-to-frame stable)")
nx.draw_networkx_nodes(G, pos, nodelist=sorted(block_b), ax=ax, node_color=matlab_style.MATLAB_COLORS[2],
                        node_size=90, label=f"block B, {len(block_b)} frames (unstable under re-initialisation)")
nx.draw_networkx_nodes(G, pos, nodelist=[60, 82], ax=ax, node_color=matlab_style.MATLAB_COLORS[1],
                        node_size=160, edgecolors="black", linewidths=1.2, label="bridge endpoints (60, 82)")
ax.set_title("Trusted-core graph (80 nodes, 146 edges): a single edge gates 46 of 80 frames",
              fontsize=11.5)
ax.legend(fontsize=8, loc="lower left")
ax.axis("off")
fig.tight_layout()
fig.savefig(FIG / "fig8_bridge_graph.png", dpi=150)
plt.close(fig)
print("wrote fig8_bridge_graph.png")

with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
    d = pickle.load(fh)
cross = [r for r in d["results"] if ((r["i"] in block_a and r["j"] in block_b) or
                                       (r["i"] in block_b and r["j"] in block_a)) and r["inliers"] >= 12]
rots = np.array([r["rotation_deg"] for r in cross])
fig, ax = plt.subplots(figsize=(7.2, 4.4))
ax.hist(rots, bins=18, range=(0, 180), color=matlab_style.MATLAB_COLORS[0], edgecolor="black", linewidth=0.4,
        label=f"sub-threshold candidates, 12$\\leq$inliers$<$20 (n={len(rots)})")
ax.axvline(112.1, color=matlab_style.MATLAB_COLORS[1], linewidth=2.2, linestyle="--",
           label="edge (60,82): 112.1 deg, 25 inliers\n(sole edge above the 20-inlier bar)")
ax.set_xlabel("recovered rotation magnitude (deg)")
ax.set_ylabel("count")
ax.set_title("Rotation estimates for all A-B block pairs at inliers $\\geq$12 (n=82 of 1564 candidates)")
ax.legend(fontsize=8.5)
fig.tight_layout()
fig.savefig(FIG / "fig9_crossblock_histogram.png")
plt.close(fig)
print("wrote fig9_crossblock_histogram.png")
