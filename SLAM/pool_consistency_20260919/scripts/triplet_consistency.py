"""Stage F: three-view (triplet) cycle-consistency check, beyond pairwise.

Pairwise essential-matrix verification (Stage B) and global rotation averaging
(Stage D) both operate on the graph of 2-view measurements. This stage adds an
explicit N>2 check: for every triangle of frames (i, j, k) that all three
pairwise edges connect, the three independently measured relative rotations
should compose to the identity around the loop:

    R_ik  =~  R_jk @ R_ij      (equivalently  R_ij @ R_jk @ R_ki  =~  I)

This is the classical loop-consistency / cycle-consistency test used to reject
outlier pairwise measurements before rotation averaging (Zach, Klopschitz and
Pollefeys, "Disambiguating visual relations using loop constraints," CVPR
2010). It is independent of the global least-squares fit in Stage D: a
measurement can be scored here using only its own triangle, without any
global optimisation, and a pairwise edge that is wrong will show up in
*every* triangle it participates in, while a genuinely correct edge should
show up in consistent triangles with edges that are themselves correct.
"""
import itertools
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")


def main():
    with open(OUT / "pairwise_scaled_all.pkl", "rb") as fh:
        d = pickle.load(fh)
    n = d["n_frames"]

    for threshold in (8, 12, 15, 20):
        edges = {}
        for r in d["results"]:
            if r["inliers"] >= threshold and r["R"] is not None:
                edges[(r["i"], r["j"])] = r["R"]

        # adjacency for triangle enumeration
        adj = defaultdict(set)
        for (i, j) in edges:
            adj[i].add(j)
            adj[j].add(i)

        triangles = []
        nodes = sorted(adj)
        for i in nodes:
            neighbors = sorted(n for n in adj[i] if n > i)
            for a, b in itertools.combinations(neighbors, 2):
                lo, hi = min(a, b), max(a, b)
                if lo in adj and hi in adj[lo]:
                    triangles.append((i, a, b) if a < b else (i, b, a))

        errors = []
        edge_bad_count = defaultdict(int)
        edge_triangle_count = defaultdict(int)
        for (i, j, k) in triangles:
            R_ij = edges.get((i, j))
            R_jk = edges.get((j, k))
            R_ik = edges.get((i, k))
            if R_ij is None or R_jk is None or R_ik is None:
                continue
            loop = R_ik.T @ (R_jk @ R_ij)  # should be ~identity
            err_deg = float(np.degrees(np.linalg.norm(Rotation.from_matrix(loop).as_rotvec())))
            errors.append(err_deg)
            for e in [(i, j), (j, k), (i, k)]:
                edge_triangle_count[e] += 1
                if err_deg > 10:
                    edge_bad_count[e] += 1

        errors = np.array(errors)
        print(f"\n=== threshold {threshold}: {len(edges)} edges, {len(triangles)} closed triangles ===")
        if len(errors) == 0:
            print("  no closed triangles at this threshold")
            continue
        print(f"  loop error: median {np.median(errors):.2f} deg, mean {errors.mean():.2f} deg, "
              f"90th pct {np.percentile(errors, 90):.2f} deg, fraction > 10 deg: {(errors > 10).mean():.3f}")

        # edges implicated in every triangle they appear in being bad
        always_bad = [(e, edge_bad_count[e], edge_triangle_count[e]) for e in edge_triangle_count
                      if edge_triangle_count[e] >= 2 and edge_bad_count[e] == edge_triangle_count[e]]
        always_bad.sort(key=lambda x: -x[2])
        print(f"  edges bad in EVERY triangle they appear in (>=2 triangles): {len(always_bad)}")
        for (i, j), bad, total in always_bad[:10]:
            print(f"    frame {i}-{j}: bad in {bad}/{total} triangles")

    return


if __name__ == "__main__":
    main()
