# Graph-level analysis of the pairwise verification results:
# - matchability vs frame separation (checks the wide-baseline hypothesis)
# - loop-closure search among verified pairs at large separation
# - does tracking quality correlate with blur / high-frequency energy
# - calibration sensitivity, scaled vs raw K
import pickle
from pathlib import Path

import numpy as np
from scipy import stats

OUT = Path("/Users/gregsobe/sonar_sim/tmp/pool_vo_20260919/phd")


def load(name):
    with open(OUT / f"pairwise_{name}.pkl", "rb") as fh:
        return pickle.load(fh)


def main():
    scaled_all = load("scaled_all")
    raw_all = load("raw_all")
    scaled_consec = load("scaled_consecutive")
    raw_consec = load("raw_consecutive")
    with open(OUT / "features.pkl", "rb") as fh:
        frames = pickle.load(fh)["frames"]

    results = scaled_all["results"]
    n = scaled_all["n_frames"]

    # --- 1. Matchability vs frame separation -------------------------------
    sep = np.array([abs(r["j"] - r["i"]) for r in results])
    inliers = np.array([r["inliers"] for r in results])
    matches = np.array([r["matches"] for r in results])
    bins = np.arange(1, sep.max() + 2)
    decay = []
    for s in bins[:-1]:
        m = sep == s
        if m.sum() == 0:
            continue
        decay.append(dict(sep=int(s), n_pairs=int(m.sum()),
                           mean_inliers=float(inliers[m].mean()),
                           median_inliers=float(np.median(inliers[m])),
                           frac_verified=float((inliers[m] >= 20).mean())))
    print("--- matchability decay (selected separations) ---")
    for row in decay:
        if row["sep"] in (1, 2, 3, 5, 10, 20, 30, 50, 80, 116):
            print(row)

    # --- 2. Loop closures: high-inlier pairs at large separation ------------
    VERIFIED_INLIERS = 30
    LOOP_MIN_SEP = 15
    loop_candidates = [r for r in results if r["inliers"] >= VERIFIED_INLIERS and abs(r["j"] - r["i"]) >= LOOP_MIN_SEP]
    loop_candidates.sort(key=lambda r: -r["inliers"])
    print(f"\n--- loop-closure candidates (inliers>={VERIFIED_INLIERS}, |i-j|>={LOOP_MIN_SEP}): {len(loop_candidates)} ---")
    for r in loop_candidates[:15]:
        print(f"  frame {r['i']:3d} - {r['j']:3d}  sep {r['j']-r['i']:3d}  inliers {r['inliers']:4d}  "
              f"matches {r['matches']:4d}  epi_err {r['mean_epi_err']:.3f}px  rot {r['rotation_deg']:.1f} deg")

    # --- 3. Per-frame quality vs tracking: correlate blur/caustic with matchability
    # attach each frame's mean inlier count over its consecutive edges
    consec = {r["i"]: r for r in scaled_consec["results"]}
    frame_stats = []
    for k in range(1, n):
        r = consec.get(k)
        if r is None:
            continue
        f_prev, f_cur = frames[k - 1], frames[k]
        frame_stats.append(dict(step=k, inliers=r["inliers"], matches=r["matches"],
                                 lap_var_prev=f_prev["lap_var"], lap_var_cur=f_cur["lap_var"],
                                 hf_prev=f_prev["high_freq_energy"], hf_cur=f_cur["high_freq_energy"],
                                 contrast_prev=f_prev["contrast"]))
    inl = np.array([r["inliers"] for r in frame_stats])
    lap = np.array([r["lap_var_cur"] for r in frame_stats])
    hf = np.array([r["hf_cur"] for r in frame_stats])
    contrast = np.array([r["contrast_prev"] for r in frame_stats])
    r_lap, p_lap = stats.pearsonr(inl, lap)
    r_hf, p_hf = stats.pearsonr(inl, hf)
    r_con, p_con = stats.pearsonr(inl, contrast)
    print(f"\n--- correlation of consecutive-step inlier count with per-frame image stats (n={len(inl)}) ---")
    print(f"  Laplacian variance (sharpness): r={r_lap:.3f}, p={p_lap:.2e}")
    print(f"  high-frequency energy fraction (proxy for caustic/ripple clutter): r={r_hf:.3f}, p={p_hf:.2e}")
    print(f"  intensity std (contrast): r={r_con:.3f}, p={p_con:.2e}")

    # --- 4. Calibration sensitivity on the consecutive chain -----------------
    print("\n--- calibration sensitivity: raw vs width-scaled K, consecutive pairs ---")
    rot_scaled = np.array([r["rotation_deg"] for r in scaled_consec["results"]])
    rot_raw = np.array([r["rotation_deg"] for r in raw_consec["results"]])
    valid = np.isfinite(rot_scaled) & np.isfinite(rot_raw)
    print(f"  valid steps under both K: {valid.sum()}/{len(rot_scaled)}")
    print(f"  mean |rotation difference| (deg): {np.mean(np.abs(rot_scaled[valid]-rot_raw[valid])):.3f}")
    tdir_scaled = np.array([r["t_dir"] if r["t_dir"] is not None else [np.nan]*3 for r in scaled_consec["results"]])
    tdir_raw = np.array([r["t_dir"] if r["t_dir"] is not None else [np.nan]*3 for r in raw_consec["results"]])
    valid_t = np.isfinite(tdir_scaled).all(axis=1) & np.isfinite(tdir_raw).all(axis=1)
    cos_angle = np.sum(tdir_scaled[valid_t] * tdir_raw[valid_t], axis=1)
    cos_angle = np.clip(cos_angle, -1, 1)
    ang_diff = np.degrees(np.arccos(cos_angle))
    print(f"  mean translation-direction angle difference (deg): {np.mean(ang_diff):.2f}, max: {np.max(ang_diff):.2f}")

    # save for plotting / report stage
    import json
    summary = dict(decay=decay,
                    loop_candidates=[{k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                       for k, v in r.items() if k not in ("R", "t_dir")}
                                      for r in loop_candidates[:30]],
                    correlations=dict(lap_var=dict(r=r_lap, p=p_lap), high_freq=dict(r=r_hf, p=p_hf),
                                       contrast=dict(r=r_con, p=p_con)),
                    calibration_sensitivity=dict(mean_abs_rotation_diff_deg=float(np.mean(np.abs(rot_scaled[valid]-rot_raw[valid]))),
                                                  mean_translation_angle_diff_deg=float(np.mean(ang_diff)),
                                                  max_translation_angle_diff_deg=float(np.max(ang_diff))))
    (OUT / "graph_summary.json").write_text(json.dumps(summary, indent=2, default=float))
    print(f"\nwrote {OUT / 'graph_summary.json'}")


if __name__ == "__main__":
    main()
