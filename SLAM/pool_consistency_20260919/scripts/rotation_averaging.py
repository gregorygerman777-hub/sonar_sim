"""Stage D: robust multi-view rotation averaging over the verified pairwise graph.

Sequential (frame i -> i+1 only) chaining uses one noisy measurement per step and
has no way to detect or down-weight a bad one; once one step fails the whole downstream
chain is corrupted. With the full pairwise graph, every frame's global orientation is
over-determined by many independent pairwise measurements, so outliers can be
outvoted. This is standard multi-view geometry practice (see Hartley, Trumpy, Chin
and Kahl, "Rotation Averaging," IJCV 2013): build the graph of pairwise relative
rotations, initialise by a spanning tree, then jointly refine all absolute rotations
by nonlinear least squares with a robust loss.
"""
import pickle
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import breadth_first_order, connected_components
from scipy.spatial.transform import Rotation

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
INLIER_THRESHOLD = 12


def load_edges():
    with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
        d = pickle.load(fh)
    n = d["n_frames"]
    edges = []
    for r in d["results"]:
        if r["inliers"] >= INLIER_THRESHOLD and r["R"] is not None:
            edges.append((r["i"] - 1, r["j"] - 1, r["R"], r["inliers"], r["mean_epi_err"], r["r_h"]))
    return n, edges


def largest_component(n, edges):
    rows = [e[0] for e in edges] + [e[1] for e in edges]
    cols = [e[1] for e in edges] + [e[0] for e in edges]
    A = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(n, n))
    n_comp, labels = connected_components(A, directed=False)
    biggest = np.argmax(np.bincount(labels))
    keep = set(np.where(labels == biggest)[0].tolist())
    return keep, labels, A


def spanning_tree_init(n, edges, keep):
    adjacency = {i: [] for i in keep}
    for (i, j, R, w, *_ ) in edges:
        if i in keep and j in keep:
            adjacency[i].append((j, R))
            adjacency[j].append((i, R.T))
    root = min(keep)
    R_global = {root: np.eye(3)}
    frontier = [root]
    visited = {root}
    while frontier:
        node = frontier.pop()
        for neighbor, R_node_to_neighbor in adjacency[node]:
            if neighbor in visited:
                continue
            # measured R maps node-frame -> neighbor-frame: R_neighbor = R_node_to_neighbor @ R_node
            R_global[neighbor] = R_node_to_neighbor @ R_global[node]
            visited.add(neighbor)
            frontier.append(neighbor)
    return R_global


def refine(n, edges, keep, R_init):
    order = sorted(keep)
    index_of = {node: k for k, node in enumerate(order)}
    ref = order[0]  # fixed to identity
    x0 = np.zeros(3 * (len(order) - 1))
    for node in order[1:]:
        k = index_of[node] - 1
        x0[3 * k:3 * k + 3] = Rotation.from_matrix(R_init[node]).as_rotvec()

    rel_edges = [(i, j, R, w) for (i, j, R, w, *_) in edges if i in keep and j in keep]
    idx_i = np.array([index_of[i] for i, j, R, w in rel_edges])
    idx_j = np.array([index_of[j] for i, j, R, w in rel_edges])
    R_meas_stack = np.stack([R for i, j, R, w in rel_edges])
    weight_stack = np.sqrt(np.array([w for i, j, R, w in rel_edges], dtype=float))

    def unpack_array(x):
        """(len(order), 3, 3) rotation matrices; ref node fixed to identity."""
        rest = Rotation.from_rotvec(x.reshape(-1, 3)).as_matrix()
        out = np.empty((len(order), 3, 3))
        out[0] = np.eye(3)
        out[1:] = rest
        return out

    def residuals(x):
        R_all = unpack_array(x)
        R_pred = np.einsum('eij,ekj->eik', R_all[idx_j], R_all[idx_i])  # R_j @ R_i^T
        M = np.einsum('eij,ekj->eik', R_meas_stack, R_pred)             # R_meas @ R_pred^T
        err = Rotation.from_matrix(M).as_rotvec()
        return (err * weight_stack[:, None]).ravel()

    result = least_squares(residuals, x0, loss="huber", f_scale=np.radians(2.0),
                            max_nfev=200, verbose=0)
    R_all = unpack_array(result.x)
    return {order[k]: R_all[k] for k in range(len(order))}, result


def main():
    n, edges = load_edges()
    keep, labels, A = largest_component(n, edges)
    print(f"largest connected component at inliers>={INLIER_THRESHOLD}: {len(keep)}/{n} frames")
    excluded = sorted(set(range(n)) - keep)
    print(f"excluded frames (1-indexed): {[e+1 for e in excluded]}")

    R_init = spanning_tree_init(n, edges, keep)
    R_final, result = refine(n, edges, keep, R_init)
    print(f"refinement cost: init -> final, nfev={result.nfev}, cost={result.cost:.4f}")

    # per-edge residual after refinement, for outlier / consistency reporting
    edge_report = []
    for i, j, R_meas, w, epi, r_h in edges:
        if i not in keep or j not in keep:
            continue
        R_pred = R_final[j] @ R_final[i].T
        resid_deg = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_meas @ R_pred.T).as_rotvec())))
        edge_report.append(dict(i=i + 1, j=j + 1, inliers=int(w), measured_rot_deg=float(np.degrees(
            np.linalg.norm(cv2.Rodrigues(R_meas)[0]))), residual_after_averaging_deg=resid_deg,
            epi_err=epi, r_h=r_h, sep=abs(j - i)))
    edge_report.sort(key=lambda r: -r["residual_after_averaging_deg"])
    print("\n--- worst-agreeing edges after global rotation averaging (likely mismatches) ---")
    for r in edge_report[:15]:
        print(f"  {r['i']:3d}-{r['j']:3d} sep{r['sep']:3d} inl{r['inliers']:3d} "
              f"measured_rot={r['measured_rot_deg']:6.1f}deg  disagreement_with_global={r['residual_after_averaging_deg']:6.1f}deg "
              f"R_H={r['r_h']:.2f}")

    resid = np.array([r["residual_after_averaging_deg"] for r in edge_report])
    print(f"\nedge residual after averaging: median {np.median(resid):.2f} deg, "
          f"mean {resid.mean():.2f} deg, 90th pct {np.percentile(resid, 90):.2f} deg, "
          f"fraction > 10 deg: {(resid > 10).mean():.3f}")

    # sequential-chain vs rotation-averaged per-step angle, for frames present in `keep`
    with open(OUT / "pairwise_scaled_consecutive.pkl", "rb") as fh:
        consec = pickle.load(fh)["results"]
    comparison = []
    for r in consec:
        i, j = r["i"] - 1, r["j"] - 1
        if i not in keep or j not in keep or r["R"] is None:
            continue
        seq_deg = np.degrees(np.linalg.norm(cv2.Rodrigues(r["R"])[0]))
        avg_pred = R_final[j] @ R_final[i].T
        avg_deg = np.degrees(np.linalg.norm(Rotation.from_matrix(avg_pred).as_rotvec()))
        comparison.append(dict(step=i + 1, sequential_deg=float(seq_deg), averaged_deg=float(avg_deg),
                                inliers=r["inliers"]))
    print(f"\nconsecutive steps comparable (both endpoints in largest component): {len(comparison)}/116")

    import json
    (OUT / "rotation_averaging_result.pkl").write_bytes(pickle.dumps(dict(
        R_final=R_final, keep=keep, excluded=excluded, edge_report=edge_report,
        comparison=comparison, n=n)))
    print(f"\nwrote {OUT / 'rotation_averaging_result.pkl'}")


if __name__ == "__main__":
    main()
