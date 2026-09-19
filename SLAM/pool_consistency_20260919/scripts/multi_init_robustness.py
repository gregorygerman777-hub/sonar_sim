"""Falsification test: is the NLS (Huber) rotation-averaging solution on the
real pruned graph unique, or merely one local optimum among several with
comparably low residual?

Motivation: the spectral gap on this exact graph is 0.0031 in the noiseless
case (matching the scalar normalized-Laplacian Fiedler value to 3 significant
figures, an exact identity for consistent group synchronization data) and
collapses to 0.000088 on the real measured rotations. A small spectral gap is
the standard warning sign in synchronization theory that the semidefinite
relaxation is not tight and the maximum-likelihood estimate is not certified
unique (Bandeira, Boumal and Singer, Math. Programming 2017). NLS with a
robust loss approximates the MLE via local optimization from one spanning
tree initialization; a small gap means a DIFFERENT initialization could
converge to a meaningfully different answer with similarly low residual.

Test: refit from 6 different spanning-tree roots (which changes the initial
guess, not the data or the objective) and measure the SPREAD of the
resulting relative rotations across a fixed set of frame pairs, after gauge
alignment. Large spread would refute "the 0.90 deg residual solution is the
answer"; small spread would refute the practical relevance of the small gap
for this particular objective and data.
"""
import pickle
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from rotation_averaging import refine

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")


def spanning_tree_init_from_root(n, edges, keep, root):
    adjacency = {i: [] for i in keep}
    for (i, j, R, w, *_ ) in edges:
        if i in keep and j in keep:
            adjacency[i].append((j, R))
            adjacency[j].append((i, R.T))
    R_global = {root: np.eye(3)}
    frontier = [root]
    visited = {root}
    while frontier:
        node = frontier.pop()
        for neighbor, R_node_to_neighbor in adjacency[node]:
            if neighbor in visited:
                continue
            R_global[neighbor] = R_node_to_neighbor @ R_global[node]
            visited.add(neighbor)
            frontier.append(neighbor)
    return R_global


def main():
    with open(OUT / "spectral_sync_result.pkl", "rb") as fh:
        d = pickle.load(fh)
    cleaned = d["cleaned_edges"]
    keep = set(d["keep"])  # 0-indexed
    edges = [(i - 1, j - 1, R, 1.0, None, None) for (i, j), R in cleaned.items()
             if (i - 1) in keep and (j - 1) in keep]
    n = d["n"]

    roots = sorted(keep)[:6:1][:6]  # first 6 distinct kept nodes as different roots
    # spread these out rather than taking consecutive ones
    ordered_keep = sorted(keep)
    roots = [ordered_keep[k] for k in np.linspace(0, len(ordered_keep) - 1, 6).astype(int)]
    print(f"refitting from {len(roots)} different spanning-tree roots (frames, 1-indexed): "
          f"{[r+1 for r in roots]}")

    solutions = []
    for root in roots:
        R_init = spanning_tree_init_from_root(n, edges, keep, root)
        R_final, result = refine(n, edges, keep, R_init)
        rel_edges = [(i, j, R, w) for (i, j, R, w, *_ ) in edges]
        resids = []
        for i, j, R_meas, w in rel_edges:
            R_pred = R_final[j] @ R_final[i].T
            resids.append(np.degrees(np.linalg.norm(Rotation.from_matrix(R_meas @ R_pred.T).as_rotvec())))
        resids = np.array(resids)
        solutions.append(dict(root=root, R_final=R_final, median_resid=float(np.median(resids)),
                               mean_resid=float(resids.mean()), cost=float(result.cost)))
        print(f"  root {root+1}: median residual {np.median(resids):.4f} deg, cost {result.cost:.4f}")

    # compare solutions pairwise on a fixed set of RELATIVE rotations (gauge invariant),
    # not on absolute R_i (which differ trivially by the arbitrary reference choice)
    ref_solution = solutions[0]
    test_nodes = sorted(keep)[::10]  # a spread-out sample of frames
    print(f"\ncomparing relative rotation R(node0 -> node_k) across the {len(solutions)} solutions, "
          f"for {len(test_nodes)} sampled frame pairs from a common base node {test_nodes[0]+1}")
    base = test_nodes[0]
    max_spreads = []
    for k in test_nodes[1:]:
        angles = []
        for sol in solutions:
            R_rel = sol["R_final"][k] @ sol["R_final"][base].T
            angles.append(sol)
        # relative rotation between solution 0's R_rel and each other solution's R_rel
        R_rel_ref = ref_solution["R_final"][k] @ ref_solution["R_final"][base].T
        diffs = []
        for sol in solutions[1:]:
            R_rel_other = sol["R_final"][k] @ sol["R_final"][base].T
            diff = np.degrees(np.linalg.norm(Rotation.from_matrix(R_rel_ref @ R_rel_other.T).as_rotvec()))
            diffs.append(diff)
        max_spreads.append(max(diffs))
        print(f"  frame {base+1} -> {k+1}: max disagreement across the {len(solutions)-1} "
              f"non-reference solutions = {max(diffs):.4f} deg")

    print(f"\noverall max cross-initialization disagreement over all sampled pairs: "
          f"{max(max_spreads):.4f} deg")
    print(f"residual costs across the {len(solutions)} inits: "
          f"{[round(s['cost'], 4) for s in solutions]}")


if __name__ == "__main__":
    main()


def full_scan():
    with open(OUT / "spectral_sync_result.pkl", "rb") as fh:
        d = pickle.load(fh)
    cleaned = d["cleaned_edges"]
    keep = set(d["keep"])
    edges = [(i - 1, j - 1, R, 1.0, None, None) for (i, j), R in cleaned.items()
             if (i - 1) in keep and (j - 1) in keep]
    n = d["n"]
    ordered_keep = sorted(keep)
    roots = [ordered_keep[k] for k in np.linspace(0, len(ordered_keep) - 1, 6).astype(int)]

    solutions = []
    for root in roots:
        R_init = spanning_tree_init_from_root(n, edges, keep, root)
        R_final, _ = refine(n, edges, keep, R_init)
        solutions.append(R_final)

    base = ordered_keep[0]
    rows = []
    for k in ordered_keep[1:]:
        R_rel_ref = solutions[0][k] @ solutions[0][base].T
        diffs = []
        for R_final in solutions[1:]:
            R_rel_other = R_final[k] @ R_final[base].T
            diffs.append(np.degrees(np.linalg.norm(
                Rotation.from_matrix(R_rel_ref @ R_rel_other.T).as_rotvec())))
        rows.append(dict(frame=k + 1, max_disagreement=max(diffs)))

    rows.sort(key=lambda r: r["frame"])
    unstable = [r["frame"] for r in rows if r["max_disagreement"] > 10]
    stable = [r["frame"] for r in rows if r["max_disagreement"] <= 10]
    print(f"\n=== full scan over all {len(rows)} kept frames (relative to base frame {base+1}) ===")
    print(f"UNSTABLE frames (>10 deg cross-init disagreement), n={len(unstable)}: {unstable}")
    print(f"stable frames (<=10 deg), n={len(stable)}")
    import json
    (OUT / "multi_init_scan.json").write_text(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    full_scan()
