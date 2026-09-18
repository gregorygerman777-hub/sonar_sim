"""Rescore the public DFKI ARIS sonar runs in KITTI odometry form.

Reads the frozen per frame trajectories that SLAM/research/evaluate.py wrote
for the eight complete recordings (stride 1), writes them as KITTI pose files
beside the gantry ground truth, and scores them with the devkit metric at
segment lengths scaled to the 0.7 to 2.9 m tank passes. No estimation is
rerun here; the estimator and its outputs are the 2026-09-14 ones.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "SLAM/kitti_benchmark_20260916"), str(ROOT / "python")]

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import kitti_odometry as kitti  # noqa: E402
import slam  # noqa: E402
import sonar  # noqa: E402

RUNS = ROOT / "SLAM/research/runs"
LENGTHS_M = (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0)   # KITTI's 100 to 800 m scaled by 1/400
STEP_FRAMES = 1
METHODS = ("sonar_odometry", "sonar_slam")
LABELS = {"sonar_odometry": "sonar odometry", "sonar_slam": "sonar SLAM (loop closures)"}
COLORS = {"sonar_odometry": "#9ca3af", "sonar_slam": "#2563eb"}
SPLITS = {"development": ["2023-09-20_180722", "2023-09-21_100927", "2023-09-21_105856",
                          "2023-09-21_123500", "2023-09-21_124735"],
          "heldout": ["2023-09-20_171105", "2023-09-20_172513", "2023-09-20_172851"]}


def compose(pose, relative):
    xy = slam.transform_points(np.asarray(relative[:2])[None, :], pose)[0]
    return np.array([xy[0], xy[1], pose[2] + relative[2]])


def densify(odometry, key_rows, optimized):
    full = np.empty_like(odometry)
    bounds = list(key_rows) + [len(odometry)]
    for i, key in enumerate(key_rows):
        for frame in range(key, bounds[i + 1]):
            full[frame] = compose(optimized[i], sonar.relative_pose_2d(odometry[key], odometry[frame]))
    return full


def load_run(split, sequence):
    folder = RUNS / split / f"{sequence}_stride1"
    odometry = pd.read_csv(folder / "odometry.csv")
    keyframes = pd.read_csv(folder / "keyframes.csv")
    metrics = json.loads((folder / "metrics.json").read_text())
    truth = odometry[["gt_x", "gt_y", "gt_yaw"]].to_numpy(float)
    poses = odometry[["x", "y", "yaw"]].to_numpy(float)
    row_of = {int(f): i for i, f in enumerate(odometry.frame_id)}
    key_rows = [row_of[int(f)] for f in keyframes.frame_id]
    optimized = keyframes[["after_x", "after_y", "after_yaw"]].to_numpy(float)
    return dict(times=odometry.time_s.to_numpy(float), truth=truth, tilt=metrics["tilt_minmax_deg"],
                loops=metrics["loop_constraints"], rejected=metrics["rejected_pairs"],
                trajectories={"sonar_odometry": poses, "sonar_slam": densify(poses, key_rows, optimized)})


def score(truth, times, poses):
    truth_m = kitti.relative_to_first(kitti.se2_trajectory_to_matrices(truth))
    est_m = kitti.relative_to_first(kitti.se2_trajectory_to_matrices(poses))
    errors = kitti.sequence_errors(truth_m, est_m, LENGTHS_M, STEP_FRAMES, times)
    _, ate = kitti.aligned_ate(poses[:, :2], truth[:, :2])
    stationary = float(np.sqrt(np.mean(np.sum((truth[:, :2] - truth[:, :2].mean(0)) ** 2, axis=1))))
    return dict(**kitti.average_errors(errors), ate_m=ate, stationary_ate_m=stationary,
                by_length=kitti.errors_by_length(errors, LENGTHS_M), _errors=errors, _matrices=(truth_m, est_m))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=HERE / "results")
    args = parser.parse_args()
    out = args.output
    (out / "dataset/poses").mkdir(parents=True, exist_ok=True)
    for method in METHODS:
        (out / f"results/{method}/data").mkdir(parents=True, exist_ok=True)
    report = dict(source="SLAM/research/runs (evaluate.py, 2026-09-14), stride 1",
                  protocol_sha256=hashlib.sha256((HERE / "PROTOCOL.md").read_bytes()).hexdigest(),
                  lengths_m=list(LENGTHS_M), step_frames=STEP_FRAMES, sequences=[])
    pooled = {m: [] for m in METHODS}
    pooled_split = {s: {m: [] for m in METHODS} for s in SPLITS}
    figures = []
    for split, sequences in SPLITS.items():
        for sequence in sequences:
            run = load_run(split, sequence)
            truth, times = run["truth"], run["times"]
            row = dict(sequence=sequence, split=split, frames=len(truth),
                       path_length_m=float(np.linalg.norm(np.diff(truth[:, :2], axis=0), axis=1).sum()),
                       duration_s=float(times[-1] - times[0]), tilt_deg=run["tilt"], loops=run["loops"],
                       rejected_pairs=run["rejected"], methods={})
            for method in METHODS:
                scored = score(truth, times, run["trajectories"][method])
                truth_m, est_m = scored.pop("_matrices")
                kitti.write_poses(out / f"dataset/poses/{sequence}.txt", truth_m)
                kitti.write_poses(out / f"results/{method}/data/{sequence}.txt", est_m)
                errors = scored.pop("_errors")
                pooled[method].extend(errors)
                pooled_split[split][method].extend(errors)
                row["methods"][method] = scored
            (out / f"dataset/{sequence}_times.txt").write_text("\n".join(f"{t:.6e}" for t in times) + "\n")
            report["sequences"].append(row)
            figures.append((sequence, split, truth, run["trajectories"]))
            print(split, sequence, " ".join(f"{m}: {row['methods'][m]['t_err_percent']:.1f}%/{row['methods'][m]['r_err_deg_per_m']:.2f}deg/m"
                                            for m in METHODS), "ATE", " ".join(f"{row['methods'][m]['ate_m']:.3f}" for m in METHODS))
    report["overall"] = {m: dict(**kitti.average_errors(pooled[m]), by_length=kitti.errors_by_length(pooled[m], LENGTHS_M))
                         for m in METHODS}
    report["by_split"] = {s: {m: kitti.average_errors(pooled_split[s][m]) for m in METHODS} for s in SPLITS}
    (out / "metrics.json").write_text(json.dumps(report, indent=2))
    write_table(out, report)
    plot(out, figures, report)


def write_table(out, report):
    md = ["| Recording | Split | Frames | Path (m) | Tilt (deg) | Loops | " + " | ".join(f"{LABELS[m]} t / r / ATE" for m in METHODS) + " | Stationary ATE |",
          "|---|---|---:|---:|---:|---:|" + "|".join("---:" for _ in METHODS) + "|---:|"]
    for s in report["sequences"]:
        cells = [f"{s['methods'][m]['t_err_percent']:.1f} % / {s['methods'][m]['r_err_deg_per_m']:.2f} / {s['methods'][m]['ate_m']:.3f} m" for m in METHODS]
        md.append(f"| {s['sequence']} | {s['split']} | {s['frames']} | {s['path_length_m']:.2f} | {s['tilt_deg'][0]:.0f} | {s['loops']} | "
                  + " | ".join(cells) + f" | {s['methods']['sonar_odometry']['stationary_ate_m']:.3f} m |")
    for split in SPLITS:
        md.append(f"| all {split} | | | | | | " + " | ".join(
            f"{report['by_split'][split][m]['t_err_percent']:.1f} % / {report['by_split'][split][m]['r_err_deg_per_m']:.2f}" for m in METHODS) + " | |")
    (out / "summary.md").write_text("\n".join(md) + "\n")


def plot(out, figures, report):
    figure, axes = plt.subplots(2, 4, figsize=(18, 9), constrained_layout=True)
    for axis, (sequence, split, truth, trajectories) in zip(axes.ravel(), figures):
        gt = truth[:, :2] - truth[0, :2]
        axis.plot(*gt.T, color="black", linestyle="--", linewidth=1.6, label="gantry ground truth")
        for method in METHODS:
            poses = trajectories[method]
            first = kitti.se2_to_matrix(truth[0]) @ np.linalg.inv(kitti.se2_to_matrix(poses[0]))
            world = np.array([(first @ kitti.se2_to_matrix(p))[:2, 3] for p in poses]) - truth[0, :2]
            axis.plot(*world.T, color=COLORS[method], linewidth=1.3, label=LABELS[method])
        axis.plot(0, 0, marker="o", color="black", markersize=5)
        centre = 0.5 * (gt.min(axis=0) + gt.max(axis=0))
        half = 0.5 * np.ptp(gt, axis=0).max() + 1.0
        axis.set_xlim(centre[0] - half, centre[0] + half)
        axis.set_ylim(centre[1] - half, centre[1] + half)
        axis.set_aspect("equal")
        axis.grid(alpha=0.25)
        axis.set_title(f"{sequence} ({split})", fontsize=10)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", ncol=3, fontsize=10, bbox_to_anchor=(0.5, -0.03))
    figure.suptitle("DFKI ARIS recordings: horizontal trajectories in the first frame gauge, no alignment", fontsize=13)
    figure.savefig(out / "trajectories.png", dpi=140, bbox_inches="tight")
    plt.close(figure)

    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 4.2), constrained_layout=True)
    for method in METHODS:
        rows = report["overall"][method]["by_length"]
        x = [r["length"] for r in rows]
        left.plot(x, [r["t_err_percent"] for r in rows], marker="o", color=COLORS[method], label=LABELS[method])
        right.plot(x, [r["r_err_deg_per_m"] for r in rows], marker="o", color=COLORS[method], label=LABELS[method])
    left.set(xlabel="path length (m)", ylabel="translation error (%)")
    right.set(xlabel="path length (m)", ylabel="rotation error (deg/m)")
    for axis in (left, right):
        axis.grid(alpha=0.25)
        axis.set_ylim(bottom=0)
    left.legend(fontsize=9)
    figure.suptitle("KITTI error versus segment length, eight recordings pooled", fontsize=12)
    figure.savefig(out / "error_by_length.png", dpi=140)
    plt.close(figure)


if __name__ == "__main__":
    main()
