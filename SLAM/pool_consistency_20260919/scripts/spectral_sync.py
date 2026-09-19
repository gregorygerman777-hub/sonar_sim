# Second, independent check on the rotation-averaging fit: spectral
# synchronization instead of nonlinear least squares. First prunes edges that
# fail their own triangle (3-view loop) checks down to a fixed point, then
# solves rotations as an eigenvalue problem (Singer 2011, extended to SO(3)
# by Arie-Nachimson et al 2012) instead of iterating from a spanning tree.
#
# Quick derivation: stack rotations as X = [R_1; ...; R_n] (3n x 3). A clean
# measurement gives R_j = R_ij R_i. Writing that as a linear map gives a
# 3n x 3n block matrix H (block (i,j) = w_ij R_ij, block (j,i) = w_ij R_ij^T)
# with H X = D X exactly when every measurement is exact (D = block-diagonal
# weighted degrees). So the true rotations sit in the top-3 eigenspace of
# H v = lambda D v -- no iteration, no initial guess needed.
#
# The gap between the 3rd and 4th eigenvalue is a free well-posedness check:
# for consistent data it's large, and it closes as measurements get noisier
# or more of them are wrong (same idea as the SDP-tightness result in
# Bandeira, Boumal & Singer 2017).
import itertools
import pickle
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.linalg import eigh
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from rotation_averaging import largest_component, spanning_tree_init, refine

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")
THRESHOLD = 20


def triangle_errors(edges_dict):
    adj = defaultdict(set)
    for (i, j) in edges_dict:
        adj[i].add(j); adj[j].add(i)
    triangles = []
    for i in sorted(adj):
        neigh = sorted(x for x in adj[i] if x > i)
        for a, b in itertools.combinations(neigh, 2):
            lo, hi = min(a, b), max(a, b)
            if lo in adj and hi in adj[lo]:
                triangles.append((i, a, b) if a < b else (i, b, a))
    edge_bad, edge_total, errs = defaultdict(int), defaultdict(int), []
    for (i, j, k) in triangles:
        loop = edges_dict[(i, k)].T @ (edges_dict[(j, k)] @ edges_dict[(i, j)])
        e = float(np.degrees(np.linalg.norm(Rotation.from_matrix(loop).as_rotvec())))
        errs.append(e)
        for edge in [(i, j), (j, k), (i, k)]:
            edge_total[edge] += 1
            if e > 10:
                edge_bad[edge] += 1
    return np.array(errs), edge_bad, edge_total, len(triangles)


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


def spectral_synchronize(edges_weighted, keep):
    """edges_weighted: dict (i,j)->(R_ij, weight), 1-indexed frame ids, i<j."""
    order = sorted(keep)
    pos = {node: k for k, node in enumerate(order)}
    m = len(order)
    H = np.zeros((3 * m, 3 * m))
    deg = np.zeros(m)
    for (i, j), (R, w) in edges_weighted.items():
        if i not in pos or j not in pos:
            continue
        a, b = pos[i], pos[j]
        H[3*a:3*a+3, 3*b:3*b+3] = w * R.T   # R_ij maps i-frame->j-frame per our convention;
        H[3*b:3*b+3, 3*a:3*a+3] = w * R     # block(i,j) should satisfy X_j ~ block(j,i) X_i etc.
        deg[a] += w
        deg[b] += w
    D = np.zeros_like(H)
    for a in range(m):
        D[3*a:3*a+3, 3*a:3*a+3] = deg[a] * np.eye(3)
    eigvals, eigvecs = eigh(H, D)  # ascending order
    top3_vals = eigvals[-3:][::-1]
    gap = eigvals[-3] - eigvals[-4] if m * 3 >= 4 else float("nan")
    top3_vecs = eigvecs[:, -3:][:, ::-1]  # (3m, 3)

    # Each 3x3 block equals R_true @ Q for one shared unknown matrix Q (the
    # gauge), and Q turns out to be proportional to an orthogonal matrix --
    # which means every block has three EXACTLY equal singular values. Found
    # out the hard way that doing a per-block SVD + "flip if det<0" breaks
    # here: with fully degenerate singular values that flip depends on
    # whatever basis LAPACK happens to pick, and it doesn't agree block to
    # block (a 3-node test case came back with two nodes right and one
    # exactly 180 degrees off). U @ V^T from a plain SVD doesn't have that
    # problem -- it's the polar decomposition's orthogonal factor, which is
    # unique regardless of how the SVD splits it -- so that's what's used,
    # and if the overall result comes out improper it's fixed once, from one
    # reference node, and applied the same way to everyone else.
    R_raw = {}
    for a, node in enumerate(order):
        block = top3_vecs[3*a:3*a+3, :]
        U, _, Vt = np.linalg.svd(block)
        R_raw[node] = U @ Vt
    ref_det = np.linalg.det(R_raw[order[0]])
    global_flip = np.diag([1.0, 1.0, -1.0]) if ref_det < 0 else np.eye(3)
    R_spectral = {node: R_raw[node] @ global_flip for node in order}
    return R_spectral, eigvals, top3_vals, gap


def main():
    with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
        d = pickle.load(fh)
    n = d["n_frames"]
    full = {(r["i"], r["j"]): (r["R"], r["inliers"]) for r in d["results"]
            if r["inliers"] >= THRESHOLD and r["R"] is not None}
    rot_only = {k: v[0] for k, v in full.items()}

    print(f"=== iterative triplet-consistency pruning (threshold {THRESHOLD}) ===")
    cleaned, history = iterative_prune(rot_only)
    print(f"converged after {len(history)} rounds: {len(rot_only)} -> {len(cleaned)} edges")

    edges_weighted = {k: (v, full[k][1]) for k, v in cleaned.items()}
    edges_for_nls = [(i - 1, j - 1, R, full[(i, j)][1], None, None) for (i, j), R in cleaned.items()]
    keep, labels, A = largest_component(n, edges_for_nls)
    print(f"\nlargest connected component after full pruning: {len(keep)}/{n} frames")

    R_init = spanning_tree_init(n, edges_for_nls, keep)
    R_nls, result = refine(n, edges_for_nls, keep, R_init)
    rel_edges = [(i, j, R, w) for (i, j, R, w, *_ ) in edges_for_nls if i in keep and j in keep]
    resids = []
    for i, j, R_meas, w in rel_edges:
        R_pred = R_nls[j] @ R_nls[i].T
        resids.append(float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_meas @ R_pred.T).as_rotvec()))))
    resids = np.array(resids)
    print(f"NLS (Huber) residual after full pruning: median {np.median(resids):.3f} deg, "
          f"fraction>10deg {(resids>10).mean():.3f}")

    # spectral synchronization on the SAME cleaned graph, 1-indexed keep set
    keep_1idx = {k + 1 for k in keep}
    R_spec, eigvals, top3, gap = spectral_synchronize(edges_weighted, keep_1idx)
    print(f"\nspectral synchronization: top-3 eigenvalues {top3.round(2)}, "
          f"next eigenvalue {eigvals[-4]:.3f}, spectral gap {gap:.3f}")

    # align spectral estimate to NLS estimate via a single global rotation (Procrustes on SO(3) using ref node)
    order = sorted(keep_1idx)
    ref = order[0]
    offset = R_nls[ref - 1] @ R_spec[ref].T
    diffs = []
    for node in order:
        R_spec_aligned = offset @ R_spec[node]
        R_nls_node = R_nls[node - 1]
        diff = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_spec_aligned @ R_nls_node.T).as_rotvec())))
        diffs.append(diff)
    diffs = np.array(diffs)
    print(f"agreement between spectral and NLS estimators (post-alignment): "
          f"median {np.median(diffs):.3f} deg, mean {diffs.mean():.3f} deg, max {diffs.max():.3f} deg")

    with open(OUT / "spectral_sync_result.pkl", "wb") as fh:
        pickle.dump(dict(history=history, keep=keep, excluded=sorted(set(range(n)) - keep),
                          resids_nls=resids, eigvals=eigvals, top3=top3, gap=gap,
                          spectral_nls_diffs=diffs, cleaned_edges=cleaned, n=n,
                          R_nls=R_nls, R_spec=R_spec), fh)
    print("\nwrote spectral_sync_result.pkl")


if __name__ == "__main__":
    main()
