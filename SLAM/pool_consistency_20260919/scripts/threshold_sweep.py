# How does rotation-averaging self-consistency change with the inlier
# threshold used to admit an edge into the graph? Runs each threshold in its
# own subprocess with a timeout so one bad case can't stall the whole sweep.
import pickle
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from rotation_averaging import largest_component, spanning_tree_init, refine

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")


def load_edges(threshold):
    with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
        d = pickle.load(fh)
    n = d["n_frames"]
    edges = [(r["i"] - 1, r["j"] - 1, r["R"], r["inliers"], r["mean_epi_err"], r["r_h"])
             for r in d["results"] if r["inliers"] >= threshold and r["R"] is not None]
    return n, edges


def run_one(threshold):
    n, edges = load_edges(threshold)
    if len(edges) < 20:
        return None
    keep, labels, A = largest_component(n, edges)
    R_init = spanning_tree_init(n, edges, keep)
    R_final, result = refine(n, edges, keep, R_init)
    rel_edges = [(i, j, R, w) for (i, j, R, w, *_) in edges if i in keep and j in keep]
    idx_i = np.array([i for i, j, R, w in rel_edges])
    idx_j = np.array([j for i, j, R, w in rel_edges])
    order = sorted(keep)
    pos = {node: k for k, node in enumerate(order)}
    R_stack = np.stack([R_final[node] for node in order])
    R_meas_stack = np.stack([R for i, j, R, w in rel_edges])
    ii = np.array([pos[i] for i in idx_i])
    jj = np.array([pos[j] for j in idx_j])
    R_pred = np.einsum('eij,ekj->eik', R_stack[jj], R_stack[ii])
    M = np.einsum('eij,ekj->eik', R_meas_stack, R_pred)
    resids = np.degrees(np.linalg.norm(Rotation.from_matrix(M).as_rotvec(), axis=1))
    return dict(threshold=threshold, n_edges=len(edges), n_nodes_connected=len(keep),
                median_residual_deg=float(np.median(resids)), mean_residual_deg=float(resids.mean()),
                frac_over_10deg=float((resids > 10).mean()), nfev=result.nfev, cost=float(result.cost))


if __name__ == "__main__":
    th = int(sys.argv[1])
    row = run_one(th)
    print(row)
    if row:
        (OUT / f"sweep_{th}.json").write_text(__import__("json").dumps(row))
