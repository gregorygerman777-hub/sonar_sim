# Rerun the same pipeline (rotation averaging, triangle pruning, spectral
# cross-check) on the SIFT graph instead of ORB's, reusing the existing
# functions -- just pointed at sift_pairwise_all.pkl and its much richer,
# fully-connected 117-node graph.
import itertools
import pickle
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from rotation_averaging import largest_component, spanning_tree_init, refine
from spectral_sync import spectral_synchronize, triangle_errors

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
THRESHOLD = 20


def load_sift_edges(threshold):
    with open(OUT / "sift_pairwise_all.pkl", "rb") as fh:
        d = pickle.load(fh)
    n = d["n_frames"]
    edges = [(r["i"] - 1, r["j"] - 1, r["R"], r["inliers"], r["epi_err"], r["r_h"])
              for r in d["results"] if r["inliers"] >= threshold and "R" in r]
    return n, edges


def iterative_prune(rot_edges, min_triangles=2):
    current = dict(rot_edges)
    round_no = 0
    history = []
    while True:
        round_no += 1
        errs, edge_bad, edge_total, n_tri = triangle_errors(current)
        bad = {e for e in edge_total if edge_total[e] >= min_triangles and edge_bad[e] == edge_total[e]}
        frac_bad = float((errs > 10).mean()) if len(errs) else 0.0
        history.append(dict(round=round_no, n_edges=len(current), n_triangles=n_tri,
                             median_loop_deg=float(np.median(errs)) if len(errs) else None,
                             frac_over_10=frac_bad, n_removed=len(bad)))
        print(f"  round {round_no}: {len(current)} edges, {n_tri} triangles, "
              f"median loop error {np.median(errs) if len(errs) else float('nan'):.2f} deg, "
              f"frac>10deg {frac_bad:.3f}, removing {len(bad)} edges")
        if not bad:
            break
        current = {k: v for k, v in current.items() if k not in bad}
    return current, history


def main():
    n, edges = load_sift_edges(THRESHOLD)
    print(f"=== SIFT graph, threshold {THRESHOLD}: {len(edges)} edges among {n} frames ===")
    keep, labels, A = largest_component(n, edges)
    print(f"largest connected component: {len(keep)}/{n} frames")
    excluded = sorted(set(range(n)) - keep)
    print(f"excluded (1-indexed): {[e+1 for e in excluded]}")

    R_init = spanning_tree_init(n, edges, keep)
    R_nls, result = refine(n, edges, keep, R_init)
    rel_edges = [(i, j, R, w) for (i, j, R, w, *_) in edges if i in keep and j in keep]
    resids = np.array([np.degrees(np.linalg.norm(Rotation.from_matrix(
        R_meas @ (R_nls[j] @ R_nls[i].T).T).as_rotvec())) for i, j, R_meas, w in rel_edges])
    print(f"\nNLS residual (pre-pruning): median {np.median(resids):.3f} deg, "
          f"mean {resids.mean():.3f} deg, frac>10deg {(resids>10).mean():.3f}")

    print(f"\n=== iterative triangle-consistency pruning ===")
    rot_only = {(i, j): R for i, j, R, w, *_ in edges}
    # triangle_errors expects 1-indexed dict keys matching (i,j) with i<j; our edges are 0-indexed here
    rot_only_1idx = {(i+1, j+1): R for (i, j), R in rot_only.items()}
    cleaned, history = iterative_prune(rot_only_1idx)
    print(f"converged after {len(history)} rounds: {len(rot_only_1idx)} -> {len(cleaned)} edges")

    weight_lookup = {(i+1, j+1): w for i, j, R, w, *_ in edges}
    edges_cleaned = [(i-1, j-1, R, weight_lookup[(i, j)], None, None) for (i, j), R in cleaned.items()]
    keep2, _, _ = largest_component(n, edges_cleaned)
    print(f"\nlargest connected component after pruning: {len(keep2)}/{n} frames")
    excluded2 = sorted(set(range(n)) - keep2)
    print(f"excluded after pruning (1-indexed): {[e+1 for e in excluded2]}")

    R_init2 = spanning_tree_init(n, edges_cleaned, keep2)
    R_nls2, result2 = refine(n, edges_cleaned, keep2, R_init2)
    rel_edges2 = [(i, j, R, w) for (i, j, R, w, *_) in edges_cleaned if i in keep2 and j in keep2]
    resids2 = np.array([np.degrees(np.linalg.norm(Rotation.from_matrix(
        R_meas @ (R_nls2[j] @ R_nls2[i].T).T).as_rotvec())) for i, j, R_meas, w in rel_edges2])
    print(f"\nNLS residual (post-pruning): median {np.median(resids2):.3f} deg, "
          f"mean {resids2.mean():.3f} deg, frac>10deg {(resids2>10).mean():.3f}")

    # spectral cross-check on the cleaned graph
    edges_weighted = {(i+1, j+1): (R, weight_lookup[(i+1, j+1)]) for i, j, R, w, *_ in edges_cleaned}
    keep2_1idx = {k+1 for k in keep2}
    R_spec, eigvals, top3, gap = spectral_synchronize(edges_weighted, keep2_1idx)
    print(f"\nspectral: top3 eigvals {top3.round(4)}, next {eigvals[-4]:.4f}, gap {gap:.4f}")

    def project_rotation(M):
        U, _, Vt = np.linalg.svd(M)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt
        return R

    order = sorted(keep2_1idx)
    ref = order[0]
    offset = project_rotation(R_nls2[ref-1] @ R_spec[ref].T)
    diffs = []
    for node in order:
        aligned = project_rotation(offset @ R_spec[node])
        diffs.append(np.degrees(np.linalg.norm(Rotation.from_matrix(
            project_rotation(aligned @ R_nls2[node-1].T)).as_rotvec())))
    diffs = np.array(diffs)
    print(f"spectral vs NLS agreement: median {np.median(diffs):.3f} deg, max {diffs.max():.3f} deg")

    with open(OUT / "sift_pipeline_result.pkl", "wb") as fh:
        pickle.dump(dict(keep=keep2, excluded=excluded2, R_nls=R_nls2, R_spec=R_spec,
                          resids=resids2, eigvals=eigvals, gap=gap, spectral_nls_diffs=diffs,
                          cleaned_edges=cleaned, n=n, history=history, weight_lookup=weight_lookup), fh)
    print("\nwrote sift_pipeline_result.pkl")


if __name__ == "__main__":
    main()
