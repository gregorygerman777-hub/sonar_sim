"""Stage G: use triplet cycle-consistency to prune specific bad edges out of the
threshold-20 trusted-core graph, then redo rotation averaging on the cleaned
graph. This is the full N>2 loop: pairwise verification (Stage B) finds
candidate measurements; rotation averaging (Stage D) fits a global rotation
to all of them; triplet cycle-consistency (this stage) finds specific edges
that fail EVERY three-view loop they participate in (an edge-local test that
does not depend on the global fit at all) and removes them before refitting.
"""
import itertools
import pickle
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from rotation_averaging import largest_component, spanning_tree_init, refine

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
THRESHOLD = 20
MIN_TRIANGLES = 2  # only judge an edge "always bad" if it appears in >=2 loops


def find_bad_edges(edges_dict):
    adj = defaultdict(set)
    for (i, j) in edges_dict:
        adj[i].add(j)
        adj[j].add(i)
    triangles = []
    for i in sorted(adj):
        neighbors = sorted(x for x in adj[i] if x > i)
        for a, b in itertools.combinations(neighbors, 2):
            lo, hi = min(a, b), max(a, b)
            if lo in adj and hi in adj[lo]:
                triangles.append((i, a, b) if a < b else (i, b, a))

    edge_bad = defaultdict(int)
    edge_total = defaultdict(int)
    all_errors = []
    for (i, j, k) in triangles:
        R_ij, R_jk, R_ik = edges_dict[(i, j)], edges_dict[(j, k)], edges_dict[(i, k)]
        loop = R_ik.T @ (R_jk @ R_ij)
        err = float(np.degrees(np.linalg.norm(Rotation.from_matrix(loop).as_rotvec())))
        all_errors.append(err)
        for e in [(i, j), (j, k), (i, k)]:
            edge_total[e] += 1
            if err > 10:
                edge_bad[e] += 1
    bad_edges = {e for e in edge_total if edge_total[e] >= MIN_TRIANGLES and edge_bad[e] == edge_total[e]}
    return bad_edges, len(triangles), np.array(all_errors)


def main():
    with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
        d = pickle.load(fh)
    n = d["n_frames"]
    full_edges = {(r["i"], r["j"]): (r["R"], r["inliers"]) for r in d["results"]
                  if r["inliers"] >= THRESHOLD and r["R"] is not None}
    rot_only = {k: v[0] for k, v in full_edges.items()}

    bad_edges, n_tri_before, errors_before = find_bad_edges(rot_only)
    print(f"threshold {THRESHOLD}: {len(full_edges)} edges, {n_tri_before} closed triangles")
    print(f"loop error before pruning: median {np.median(errors_before):.2f} deg, "
          f"fraction > 10 deg: {(errors_before > 10).mean():.3f}")
    print(f"edges flagged 'always bad' across their triangles: {len(bad_edges)}")
    for (i, j) in sorted(bad_edges):
        print(f"  removing frame {i}-{j} (inliers={full_edges[(i,j)][1]})")

    cleaned = {k: v for k, v in rot_only.items() if k not in bad_edges}
    _, n_tri_after, errors_after = find_bad_edges(cleaned)
    print(f"\nafter removing {len(bad_edges)} edges: {len(cleaned)} edges, {n_tri_after} closed triangles")
    if len(errors_after):
        print(f"loop error after pruning: median {np.median(errors_after):.2f} deg, "
              f"fraction > 10 deg: {(errors_after > 10).mean():.3f}")

    # refit rotation averaging on the cleaned edge set
    edges_for_refine = [(i - 1, j - 1, R, full_edges[(i, j)][1], None, None) for (i, j), R in cleaned.items()]
    keep, labels, A = largest_component(n, edges_for_refine)
    print(f"\nlargest connected component after pruning: {len(keep)}/{n} frames")
    R_init = spanning_tree_init(n, edges_for_refine, keep)
    R_final, result = refine(n, edges_for_refine, keep, R_init)

    rel_edges = [(i, j, R, w) for (i, j, R, w, *_ ) in edges_for_refine if i in keep and j in keep]
    resids = []
    for i, j, R_meas, w in rel_edges:
        R_pred = R_final[j] @ R_final[i].T
        resids.append(float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_meas @ R_pred.T).as_rotvec()))))
    resids = np.array(resids)
    print(f"post-prune rotation-averaging residual: median {np.median(resids):.2f} deg, "
          f"mean {resids.mean():.2f} deg, fraction > 10 deg: {(resids > 10).mean():.3f}")

    excluded = sorted(set(range(n)) - keep)
    with open(OUT / "rotation_averaging_pruned_result.pkl", "wb") as fh:
        pickle.dump(dict(R_final=R_final, keep=keep, excluded=excluded, bad_edges=bad_edges,
                          n=n, resids=resids, errors_before=errors_before, errors_after=errors_after), fh)
    print(f"\nexcluded frames after pruning (1-indexed): {[e+1 for e in excluded]}")
    print("wrote rotation_averaging_pruned_result.pkl")


if __name__ == "__main__":
    main()
