"""Score a run directory against ground truth with evo.

    python SLAM/validation/evaluate.py SLAM/validation/results/<run>   (dataset read from run_meta.json)

* Associates estimate and ground truth by timestamp (evo.core.sync).
* Aligns with Sim(3) Umeyama (monocular scale is unobservable) and reports the recovered scale.
* ATE: RMSE, mean, median, max of the translation error after alignment.
* RPE: translation and rotation error over a fixed travelled distance (default 1 m of ground truth
  path), computed on the Sim(3) aligned estimate.
* Coverage: the fraction of frames that received a pose. A run that posed less than 95 % of its
  frames is flagged PARTIAL; its ATE covers only the posed frames and must be read with that fraction.
Writes metrics.json into the run directory. Ground truth is only read here, never by the SLAM.
"""

import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import export  # noqa: E402
import sequences  # noqa: E402

PARTIAL_THRESHOLD = 0.95
COLLINEAR_RATIO = 0.05   # second / first singular value of the centred ground truth positions


def rotation_is_constrained(positions, ratio=COLLINEAR_RATIO):
    """Whether a position only alignment (Umeyama) determines the full rotation. On a nearly straight path the
    rotation about the path is fixed only by noise, so an orientation error after that alignment is arbitrary
    (KITTI 04: 36 degrees of 'error' from a trajectory whose relative rotations match ground truth to 0.2 degrees)."""
    p = np.asarray(positions, float)
    if len(p) < 3:
        return False, 0.0
    sv = np.linalg.svd(p - p.mean(0), compute_uv=False)
    r = float(sv[1] / sv[0]) if sv[0] > 0 else 0.0
    return r >= ratio, r


def to_evo(timestamps, R_wc, t_wc):
    q = Rotation.from_matrix(np.asarray(R_wc)).as_quat()        # x y z w
    return PoseTrajectory3D(positions_xyz=np.asarray(t_wc, float), orientations_quat_wxyz=q[:, [3, 0, 1, 2]],
                            timestamps=np.asarray(timestamps, float))


def path_length(xyz):
    return float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))) if len(xyz) > 1 else 0.0


def default_rpe_delta_m(dataset_name):
    """RPE step: 1 m of ground truth travel, except on KITTI, whose frames are about 1.4 m apart (no two frames are
    1 m apart there); it uses 100 m, the shortest segment of the KITTI odometry benchmark's own metric."""
    return 100.0 if dataset_name.startswith("kitti_") else 1.0


def evaluate(run_dir, max_diff=None, rpe_delta_m=None, trajectory_file="trajectory_tum.txt", dataset=None):
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "run_meta.json").read_text())
    seq = sequences.get(dataset or meta.get("dataset_key") or meta["dataset"])
    gt_t, gt_R, gt_p = seq.ground_truth
    if rpe_delta_m is None:
        rpe_delta_m = default_rpe_delta_m(seq.name)
    ts, R, p = export.read_tum(run_dir / trajectory_file)
    result = dict(run=run_dir.name, dataset=meta["dataset"], method=meta.get("method", "monoslam"),
                  frontend=meta.get("frontend"), frames_total=meta["frames_total"],
                  frames_posed=int(len(ts)), fraction_posed=len(ts) / max(meta["frames_total"], 1))
    if len(ts) < 3:
        # Not an error of the run or of this script: the method posed too few frames for anything to be scored.
        result.update(status="TOO FEW POSED", reason="fewer than 3 posed frames")
        (run_dir / "metrics.json").write_text(json.dumps(result, indent=2))
        return result
    if max_diff is None:
        step = np.median(np.diff(gt_t)) if len(gt_t) > 1 else 0.02
        max_diff = min(0.02, 0.5 * step) if step > 0.001 else 0.01
    ref = to_evo(gt_t, gt_R, gt_p)
    est = to_evo(ts, R, p)
    ref_a, est_a = sync.associate_trajectories(ref, est, max_diff=max_diff)
    n = ref_a.num_poses
    # Frames that could have been scored: every frame of the run that has a ground truth pose.
    frame_ts = np.array(meta.get("frame_timestamps", []), float)
    if len(frame_ts):
        gt_sorted = np.sort(gt_t)
        pos = np.clip(np.searchsorted(gt_sorted, frame_ts), 1, len(gt_sorted) - 1)
        nearest = np.minimum(np.abs(gt_sorted[pos] - frame_ts), np.abs(gt_sorted[pos - 1] - frame_ts))
        scorable = int(np.sum(nearest <= max_diff))
    else:
        scorable = None
    est_aligned = copy.deepcopy(est_a)
    r_a, t_a, s = est_aligned.align(ref_a, correct_scale=True)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref_a, est_aligned))
    ape_stats = ape.get_all_statistics()
    ape_rot = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    ape_rot.process_data((ref_a, est_aligned))
    gt_len = path_length(ref_a.positions_xyz)
    constrained, spread = rotation_is_constrained(ref_a.positions_xyz)
    rpe_t = rpe_r = None
    if gt_len > 2 * rpe_delta_m:
        try:
            m_t = metrics.RPE(metrics.PoseRelation.translation_part, delta=rpe_delta_m,
                              delta_unit=metrics.Unit.meters, all_pairs=True)
            m_t.process_data((ref_a, est_aligned))
            m_r = metrics.RPE(metrics.PoseRelation.rotation_angle_deg, delta=rpe_delta_m,
                              delta_unit=metrics.Unit.meters, all_pairs=True)
            m_r.process_data((ref_a, est_aligned))
            rpe_t, rpe_r = m_t.get_all_statistics(), m_r.get_all_statistics()
        except Exception as exc:  # noqa: BLE001  (too few pairs at this delta)
            result["rpe_error"] = str(exc)
    result.update(
        status="PARTIAL" if result["fraction_posed"] < PARTIAL_THRESHOLD else "OK",
        associated_poses=int(n), scorable_frames=scorable,
        fraction_of_scorable_posed=(n / scorable) if scorable else None,
        association_max_diff=max_diff,
        sim3_scale=float(s), sim3_rotation=r_a.tolist(), sim3_translation=t_a.tolist(),
        gt_path_length_m=gt_len,
        ate_m={k: float(v) for k, v in ape_stats.items()},
        ate_rmse_percent_of_path=100.0 * ape_stats["rmse"] / gt_len if gt_len > 0 else None,
        orientation_error_deg=({k: float(v) for k, v in ape_rot.get_all_statistics().items()}
                               if constrained else None),
        orientation_error_note=(None if constrained else
                                f"not determined: ground truth path nearly straight (singular value ratio {spread:.3f} "
                                f"< {COLLINEAR_RATIO}), so the alignment's rotation about it is arbitrary; see RPE"),
        rpe_delta_m=rpe_delta_m,
        rpe_translation_m={k: float(v) for k, v in rpe_t.items()} if rpe_t else None,
        rpe_rotation_deg={k: float(v) for k, v in rpe_r.items()} if rpe_r else None,
    )
    # Map accuracy for the synthetic scene, where the true surface is known.
    height_file = sequences.DATA / "synthetic" / seq.name / "height.npy"
    if height_file.exists() and (run_dir / "points.ply").exists():
        xyz, _ = export.read_ply(run_dir / "points.ply")
        if len(xyz):
            result["map_to_surface_m"] = map_surface_error(xyz, r_a, t_a, s, np.load(height_file))
    result["segments"] = evaluate_segments(run_dir, ref, max_diff)
    if result["segments"]:
        errs = np.concatenate([seg.pop("_errors") for seg in result["segments"]])
        result["all_maps"] = dict(maps=len(result["segments"]),
                                  frames_posed=int(sum(seg["frames_posed"] for seg in result["segments"])),
                                  associated_poses=int(len(errs)),
                                  ate_rmse_m_each_map_aligned_separately=float(np.sqrt(np.mean(errs ** 2))))
    (run_dir / "metrics.json").write_text(json.dumps(result, indent=2))
    np.savetxt(run_dir / "aligned_estimate_tum.txt",
               np.column_stack((est_aligned.timestamps, est_aligned.positions_xyz,
                                est_aligned.orientations_quat_wxyz[:, [1, 2, 3, 0]])),
               fmt="%.9f", header="timestamp tx ty tz qx qy qz qw (Sim3 aligned to ground truth)")
    np.savetxt(run_dir / "associated_gt_tum.txt",
               np.column_stack((ref_a.timestamps, ref_a.positions_xyz, ref_a.orientations_quat_wxyz[:, [1, 2, 3, 0]])),
               fmt="%.9f", header="timestamp tx ty tz qx qy qz qw (ground truth at the associated stamps)")
    return result


def evaluate_segments(run_dir, ref, max_diff):
    """ATE of every map of a multi-map run, each map aligned by its own Sim(3) (maps have unrelated scales)."""
    files = [("primary", run_dir / "trajectory_tum.txt")]
    files += [(p.parent.name, p) for p in sorted((run_dir / "maps").glob("map_*/trajectory_tum.txt"))]
    if len(files) == 1:
        return []
    out = []
    for name, path in files:
        ts, R, p = export.read_tum(path)
        seg = dict(map=name, frames_posed=int(len(ts)))
        if len(ts) >= 3:
            try:
                ref_a, est_a = sync.associate_trajectories(ref, to_evo(ts, R, p), max_diff=max_diff)
                est_a = copy.deepcopy(est_a)
                _, _, s = est_a.align(ref_a, correct_scale=True)
                e = np.linalg.norm(est_a.positions_xyz - ref_a.positions_xyz, axis=1)
                if name != "primary":
                    np.savetxt(path.parent / "aligned_estimate_tum.txt",
                               np.column_stack((est_a.timestamps, est_a.positions_xyz,
                                                est_a.orientations_quat_wxyz[:, [1, 2, 3, 0]])),
                               fmt="%.9f", header="timestamp tx ty tz qx qy qz qw (this map, own Sim3 to ground truth)")
                seg.update(associated_poses=int(len(e)), ate_rmse_m=float(np.sqrt(np.mean(e ** 2))),
                           gt_path_length_m=path_length(ref_a.positions_xyz), sim3_scale=float(s), _errors=e)
            except Exception as exc:  # noqa: BLE001  (e.g. a map too short to align)
                seg.update(error=str(exc), _errors=np.empty(0))
        else:
            seg["_errors"] = np.empty(0)
        out.append(seg)
    return out


def map_surface_error(xyz, R, t, s, height, extent=3.0):
    """Vertical distance from each aligned map point to the true height field (2 cm grid)."""
    import synthetic  # noqa: F401  (for EXTENT consistency)
    aligned = s * xyz @ np.asarray(R).T + np.asarray(t)
    res = 2 * extent / height.shape[0]
    rows = np.clip(((aligned[:, 1] + extent) / res).astype(int), 0, height.shape[0] - 1)
    cols = np.clip(((aligned[:, 0] + extent) / res).astype(int), 0, height.shape[1] - 1)
    inside = (np.abs(aligned[:, 0]) < extent) & (np.abs(aligned[:, 1]) < extent)
    d = np.abs(aligned[:, 2] - height[rows, cols])
    d = np.where(inside, d, np.inf)
    finite = d[np.isfinite(d)]
    return dict(points=int(len(xyz)), median=float(np.median(finite)) if len(finite) else None,
                p90=float(np.percentile(finite, 90)) if len(finite) else None,
                fraction_within_5cm=float(np.mean(d < 0.05)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--rpe-delta-m", type=float, default=None, help="default: 1 m (100 m on KITTI)")
    args = parser.parse_args()
    for run in args.runs:
        r = evaluate(run, rpe_delta_m=args.rpe_delta_m)
        if r.get("ate_m"):
            print(f"{run.name}: {r['status']} posed {r['fraction_posed']:.1%}, ATE RMSE {r['ate_m']['rmse']:.4f} m "
                  f"({r['ate_rmse_percent_of_path']:.2f} % of {r['gt_path_length_m']:.2f} m), scale {r['sim3_scale']:.4f}")
        else:   # the method posed too few frames to score: a result of the run, not an error of this script
            print(f"{run.name}: {r['status']} ({r['frames_posed']}/{r['frames_total']} frames posed, nothing to score)")


if __name__ == "__main__":
    main()
