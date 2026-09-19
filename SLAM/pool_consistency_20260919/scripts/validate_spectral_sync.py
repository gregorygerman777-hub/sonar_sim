"""Falsification test for spectral_synchronize(), required before trusting its
output on real data.

Claim under test: spectral_synchronize() correctly recovers absolute
rotations from noiseless pairwise relative rotation measurements, on the
EXACT edge topology of the real pruned graph (80 nodes, 155 edges), not a
generic random graph.

Refuting observation: on noiseless synthetic data with this topology, the
recovered rotations do not match ground truth to numerical precision, and/or
the spectral gap is not large. If that happens the method or its
implementation is broken and the real-data result is uninterpretable.

Two conditions are run:
  A. Noiseless: measured R_ij = R_j_true R_i_true^T exactly. Expected:
     near machine precision recovery, large spectral gap.
  B. Outlier sweep: a controlled fraction of edges replaced by an
     independent uniformly random rotation (matching the "wrong
     correspondence" failure mode identified on the real data in
     Sections 4.3/4.5 of the report), at a fixed base noise of 1 degree
     (matching the wrote OSCalibration accuracy scale) on the
     non-outlier edges. 10 seeds per fraction for a distribution, not a
     point estimate.
"""
import pickle
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from rotation_averaging import largest_component, spanning_tree_init, refine
from spectral_sync import spectral_synchronize, triangle_errors

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")


def load_real_topology():
    with open(OUT / "spectral_sync_result.pkl", "rb") as fh:
        d = pickle.load(fh)
    cleaned = d["cleaned_edges"]  # dict (i,j) 1-indexed -> R (real measured, not used here)
    keep = {k + 1 for k in d["keep"]}  # 1-indexed node ids actually used
    edge_list = [(i, j) for (i, j) in cleaned if i in keep and j in keep]
    return sorted(keep), edge_list


def random_rotation(rng):
    return Rotation.random(random_state=rng).as_matrix()


def make_synthetic(nodes, edge_list, rng, outlier_frac, base_noise_deg):
    truth = {n: random_rotation(rng) for n in nodes}
    edges_weighted = {}
    n_outliers = 0
    for (i, j) in edge_list:
        R_true_ij = truth[j] @ truth[i].T
        if rng.random() < outlier_frac:
            R_meas = random_rotation(rng)
            n_outliers += 1
        else:
            noise = Rotation.from_rotvec(rng.normal(scale=np.radians(base_noise_deg), size=3)).as_matrix()
            R_meas = noise @ R_true_ij
        edges_weighted[(i, j)] = (R_meas, 1.0)
    return truth, edges_weighted, n_outliers


def align_and_score(estimate, truth, nodes):
    """Both estimators use the convention R_ij = R_j R_i^T, whose gauge freedom
    is a RIGHT multiplication by a shared orthogonal Q (R_i -> R_i @ Q for
    every i simultaneously): (R_j Q)(R_i Q)^T = R_j R_i^T is invariant under
    this, and not under a left multiplication. Q is estimated from one
    reference node and divided out on the right for every node."""
    ref = nodes[0]
    Q = truth[ref].T @ estimate[ref]
    errs = []
    for n in nodes:
        recovered = estimate[n] @ Q.T
        errs.append(np.degrees(np.linalg.norm(Rotation.from_matrix(recovered @ truth[n].T).as_rotvec())))
    return np.array(errs)


def run_nls(nodes, edges_weighted):
    pos = {n: k for k, n in enumerate(nodes)}
    edges_for_nls = [(pos[i], pos[j], R, w, None, None) for (i, j), (R, w) in edges_weighted.items()]
    n = len(nodes)
    keep, _, _ = largest_component(n, edges_for_nls)
    R_init = spanning_tree_init(n, edges_for_nls, keep)
    R_final, _ = refine(n, edges_for_nls, keep, R_init)
    return {nodes[k]: R_final[k] for k in keep}


def main():
    nodes, edge_list = load_real_topology()
    print(f"real topology: {len(nodes)} nodes, {len(edge_list)} edges (loaded, not their rotations)")

    print("\n=== Condition A: noiseless, exact topology ===")
    rng = np.random.default_rng(0)
    truth, edges_weighted, _ = make_synthetic(nodes, edge_list, rng, outlier_frac=0.0, base_noise_deg=0.0)
    R_spec, eigvals, top3, gap = spectral_synchronize(edges_weighted, set(nodes))
    err_spec = align_and_score(R_spec, truth, nodes)
    R_nls = run_nls(nodes, edges_weighted)
    err_nls = align_and_score(R_nls, truth, nodes)
    print(f"  spectral: top3 eigvals {top3}, next {eigvals[-4]:.6f}, gap {gap:.6f}")
    print(f"  spectral recovery error vs ground truth: max {err_spec.max():.6f} deg, median {np.median(err_spec):.6f} deg")
    print(f"  NLS recovery error vs ground truth: max {err_nls.max():.6f} deg, median {np.median(err_nls):.6f} deg")
    if err_spec.max() > 1e-2:
        print("  FAIL: spectral method does not recover noiseless ground truth on this topology. "
              "Implementation bug confirmed; real-data spectral result is NOT trustworthy as reported.")
    else:
        print("  PASS: spectral method recovers noiseless ground truth to high precision on this exact topology.")

    print("\n=== Condition B: outlier-fraction sweep, 10 seeds each, base noise 1 deg ===")
    fractions = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40]
    rows = []
    for frac in fractions:
        gaps, med_loops, frac_over10, nls_med_errs = [], [], [], []
        for seed in range(10):
            rng = np.random.default_rng(1000 + seed)
            truth, edges_weighted, n_out = make_synthetic(nodes, edge_list, rng, frac, base_noise_deg=1.0)
            R_spec, eigvals, top3, gap = spectral_synchronize(edges_weighted, set(nodes))
            gaps.append(gap)
            rot_only = {k: v[0] for k, v in edges_weighted.items()}
            errs, _, _, _ = triangle_errors(rot_only)
            med_loops.append(float(np.median(errs)) if len(errs) else np.nan)
            frac_over10.append(float((errs > 10).mean()) if len(errs) else np.nan)
            R_nls = run_nls(nodes, edges_weighted)
            err_nls = align_and_score(R_nls, truth, nodes)
            nls_med_errs.append(float(np.median(err_nls)))
        rows.append(dict(frac=frac,
                          gap_mean=float(np.mean(gaps)), gap_std=float(np.std(gaps)),
                          med_loop_mean=float(np.nanmean(med_loops)), med_loop_std=float(np.nanstd(med_loops)),
                          frac_over10_mean=float(np.nanmean(frac_over10)), frac_over10_std=float(np.nanstd(frac_over10)),
                          nls_err_mean=float(np.mean(nls_med_errs)), nls_err_std=float(np.std(nls_med_errs))))
        r = rows[-1]
        print(f"  outlier frac {frac:.2f}: gap {r['gap_mean']:.4f}+/-{r['gap_std']:.4f}, "
              f"median loop err {r['med_loop_mean']:.2f}+/-{r['med_loop_std']:.2f} deg, "
              f"frac>10deg {r['frac_over10_mean']:.3f}+/-{r['frac_over10_std']:.3f}, "
              f"NLS median recovery err {r['nls_err_mean']:.3f}+/-{r['nls_err_std']:.3f} deg")

    import json
    (OUT / "spectral_validation.json").write_text(json.dumps(rows, indent=2))
    print("\nwrote spectral_validation.json")


if __name__ == "__main__":
    main()
