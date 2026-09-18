"""Render the catalogue, run every method, and score in KITTI odometry form.

Two imaging conditions are run on identical rendered pings: `compensated`
applies the range gain of `imaging.py` before feature extraction, as sonar
hardware does; `raw` uses the simulator output directly, as the earlier reports
did. Writes, under --output (default results/ beside this file):
  dataset/poses/SS.txt, dataset/sequences/SS/times.txt   ground truth, KITTI layout
  results/CONDITION/METHOD/data/SS.txt                    estimates, KITTI layout
  metrics.json, summary.csv, summary.md                   scores, both conditions
  trajectories_CONDITION.png, error_by_length_CONDITION.png,
  error_by_speed_CONDITION.png, per_sequence_CONDITION.png
Ground truth is read by the scorer only, after every estimate is complete.
"""

import argparse
import hashlib
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT), str(ROOT / "python"), str(ROOT / "SLAM/research")]

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import imaging  # noqa: E402
import kitti_odometry as kitti  # noqa: E402
import sequences  # noqa: E402
import slam  # noqa: E402
import slam_experiment  # noqa: E402
import sonar  # noqa: E402
from evaluate import estimate  # noqa: E402  (SLAM/research, unchanged)

LENGTHS_M = (2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0)
STEP_FRAMES = 1
CONDITIONS = ("compensated", "raw")
FREQUENCY_HZ = 1.2e6
METHODS = ("sonar_odometry", "sonar_slam", "imu_dead_reckoning", "sonar_imu_slam")
LABELS = {"sonar_odometry": "sonar odometry", "sonar_slam": "sonar SLAM",
          "imu_dead_reckoning": "IMU dead reckoning", "sonar_imu_slam": "sonar + IMU SLAM"}
COLORS = {"sonar_odometry": "#9ca3af", "sonar_slam": "#2563eb",
          "imu_dead_reckoning": "#f59e0b", "sonar_imu_slam": "#0f766e"}
STYLES = {"sonar_odometry": "-", "sonar_slam": "-", "imu_dead_reckoning": "--", "sonar_imu_slam": "-"}


def compose(pose, relative):
    xy = slam.transform_points(np.asarray(relative[:2])[None, :], pose)[0]
    return np.array([xy[0], xy[1], pose[2] + relative[2]])


def densify(odometry, keys, optimized):
    """Carry optimised keyframe poses to every frame with the sequential odometry."""
    full = np.empty_like(odometry)
    bounds = list(keys) + [len(odometry)]
    for i, key in enumerate(keys):
        for frame in range(key, bounds[i + 1]):
            full[frame] = compose(optimized[i], sonar.relative_pose_2d(odometry[key], odometry[frame]))
    return full


def run_methods(simulator, objects, truth, times, index, gain):
    """Every estimator sees rendered pings and timestamps only."""
    positions = sequences.positions_for(truth)
    images, features, descriptors = slam_experiment.render_survey(simulator, objects, truth, positions, gain)
    inputs = dict(ids=np.arange(len(truth)), times=times, points=features, desc=descriptors)

    started = time.perf_counter()
    sonar_only = estimate(inputs, 1)
    sonar_seconds = time.perf_counter() - started
    started = time.perf_counter()
    fused = slam_experiment.fuse_sonar_imu(simulator, objects, truth, positions, images,
                                           features, descriptors, imu_seed=20260913 + index)
    fused_seconds = time.perf_counter() - started

    trajectories = {
        "sonar_odometry": sonar_only["poses"],
        "sonar_slam": densify(sonar_only["poses"], sonar_only["keys"], sonar_only["optimized"]),
        "imu_dead_reckoning": fused["imu"],
        "sonar_imu_slam": fused["optimized"],
    }
    diagnostics = {
        "sonar_odometry": dict(rejected_pairs=int(sum(not e["accepted"] for e in sonar_only["edges"])),
                               seconds=sonar_seconds),
        "sonar_slam": dict(keyframes=int(len(sonar_only["keys"])), loops=int(len(sonar_only["loops"])),
                           seconds=sonar_seconds),
        "imu_dead_reckoning": dict(seconds=fused_seconds),
        "sonar_imu_slam": dict(scan_constraints=int(len(fused["scan_edges"])),
                               loops=int(len(fused["loop_edges"])), seconds=fused_seconds),
    }
    feature_count = int(sum(map(len, features)))
    return trajectories, diagnostics, feature_count


def score(truth, times, trajectories):
    truth_matrices = kitti.relative_to_first(kitti.se2_trajectory_to_matrices(truth))
    stationary = float(np.sqrt(np.mean(np.sum((truth[:, :2] - truth[:, :2].mean(0)) ** 2, axis=1))))
    per_method = {}
    for method, poses in trajectories.items():
        matrices = kitti.relative_to_first(kitti.se2_trajectory_to_matrices(poses))
        errors = kitti.sequence_errors(truth_matrices, matrices, LENGTHS_M, STEP_FRAMES, times)
        _, ate = kitti.aligned_ate(poses[:, :2], truth[:, :2])
        per_method[method] = dict(
            **kitti.average_errors(errors), ate_m=ate, stationary_ate_m=stationary,
            by_length=kitti.errors_by_length(errors, LENGTHS_M),
            by_speed=kitti.errors_by_speed(errors),
            _errors=errors, _matrices=matrices)
    return truth_matrices, per_method


def environment():
    def git(*args):
        try:
            return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
        except Exception:  # noqa: BLE001
            return None
    return dict(python=platform.python_version(), numpy=np.__version__, platform=platform.platform(),
                git_commit=git("rev-parse", "HEAD"), git_dirty=bool(git("status", "--porcelain")),
                protocol_sha256=hashlib.sha256((HERE / "PROTOCOL.md").read_bytes()).hexdigest(),
                lengths_m=list(LENGTHS_M), step_frames=STEP_FRAMES,
                ping_period_s=sequences.PING_PERIOD_S)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=HERE / "results")
    args = parser.parse_args()
    out = args.output
    (out / "dataset/poses").mkdir(parents=True, exist_ok=True)
    for condition in CONDITIONS:
        for method in METHODS:
            (out / f"results/{condition}/{method}/data").mkdir(parents=True, exist_ok=True)

    simulator = slam_experiment.build_simulator()
    gains = {"compensated": imaging.range_compensation(simulator, FREQUENCY_HZ), "raw": None}
    report = dict(environment=environment(), conditions={c: dict(sequences=[]) for c in CONDITIONS})
    pooled = {c: {m: [] for m in METHODS} for c in CONDITIONS}
    figures = {c: [] for c in CONDITIONS}
    for index, spec in enumerate(sequences.catalogue()):
        truth, objects = spec["build"]()
        times = sequences.times_for(truth)
        name = spec["name"]
        path_length = float(np.linalg.norm(np.diff(truth[:, :2], axis=0), axis=1).sum())
        truth_matrices = kitti.relative_to_first(kitti.se2_trajectory_to_matrices(truth))
        (out / f"dataset/sequences/{name}").mkdir(parents=True, exist_ok=True)
        kitti.write_poses(out / f"dataset/poses/{name}.txt", truth_matrices)
        kitti.write_times(out / f"dataset/sequences/{name}/times.txt", times)
        for condition in CONDITIONS:
            trajectories, diagnostics, feature_count = run_methods(
                simulator, objects, truth, times, index, gains[condition])
            # Ground truth is consulted only from here on.
            _, per_method = score(truth, times, trajectories)
            row = dict(sequence=name, shape=spec["shape"], seed=spec["seed"], frames=len(truth),
                       path_length_m=path_length, duration_s=float(times[-1]),
                       mean_speed_m_per_s=path_length / float(times[-1]), spheres=len(objects),
                       features=feature_count, features_per_ping=feature_count / len(truth), methods={})
            for method in METHODS:
                kitti.write_poses(out / f"results/{condition}/{method}/data/{name}.txt",
                                  per_method[method]["_matrices"])
                pooled[condition][method].extend(per_method[method]["_errors"])
                row["methods"][method] = {k: v for k, v in per_method[method].items() if not k.startswith("_")}
                row["methods"][method].update(diagnostics[method])
            report["conditions"][condition]["sequences"].append(row)
            figures[condition].append((name, spec["shape"], truth, trajectories))
            print(condition, name, spec["shape"], " ".join(
                f"{m}: {row['methods'][m]['t_err_percent']:.2f}%/{row['methods'][m]['r_err_deg_per_m']:.3f}deg/m"
                for m in METHODS), flush=True)

    for condition in CONDITIONS:
        block = report["conditions"][condition]
        block["overall"] = {m: dict(**kitti.average_errors(pooled[condition][m]),
                                    by_length=kitti.errors_by_length(pooled[condition][m], LENGTHS_M),
                                    by_speed=kitti.errors_by_speed(pooled[condition][m]),
                                    mean_ate_m=float(np.mean([s["methods"][m]["ate_m"] for s in block["sequences"]])))
                            for m in METHODS}
        plot_trajectories(out, figures[condition], condition)
        plot_error_curves(out, block, condition)
        plot_per_sequence(out, block, condition)
    (out / "metrics.json").write_text(json.dumps(report, indent=2))
    write_tables(out, report)
    for condition in CONDITIONS:
        print(condition, json.dumps({m: {k: v for k, v in report["conditions"][condition]["overall"][m].items()
                                         if not isinstance(v, list)} for m in METHODS}, indent=2))


def write_tables(out, report):
    header = ["condition", "sequence", "shape", "frames", "path_m", "speed_m_s", "features_per_ping"]
    for method in METHODS:
        header += [f"{method}_t_err_pct", f"{method}_r_err_deg_m", f"{method}_ate_m"]
    lines = [",".join(header)]
    md = []
    for condition in CONDITIONS:
        block = report["conditions"][condition]
        md += [f"### {condition} imagery", "",
               "| Seq | Shape | Frames | Path (m) | Feat./ping | " + " | ".join(
                   f"{LABELS[m]} t / r / ATE" for m in METHODS) + " |",
               "|---|---|---:|---:|---:|" + "|".join("---:" for _ in METHODS) + "|"]
        for s in block["sequences"]:
            values = [condition, s["sequence"], s["shape"], s["frames"], f"{s['path_length_m']:.1f}",
                      f"{s['mean_speed_m_per_s']:.2f}", f"{s['features_per_ping']:.1f}"]
            cells = []
            for m in METHODS:
                r = s["methods"][m]
                values += [f"{r['t_err_percent']:.3f}", f"{r['r_err_deg_per_m']:.4f}", f"{r['ate_m']:.3f}"]
                cells.append(f"{r['t_err_percent']:.2f} % / {r['r_err_deg_per_m']:.3f} / {r['ate_m']:.3f} m")
            lines.append(",".join(str(v) for v in values))
            md.append(f"| {s['sequence']} | {s['shape']} | {s['frames']} | {s['path_length_m']:.1f} | "
                      f"{s['features_per_ping']:.1f} | " + " | ".join(cells) + " |")
        overall = []
        for m in METHODS:
            r = block["overall"][m]
            overall.append(f"{r['t_err_percent']:.2f} % / {r['r_err_deg_per_m']:.3f} / {r['mean_ate_m']:.3f} m")
        md.append("| **all** | mean over all segments | | | | " + " | ".join(f"**{c}**" for c in overall) + " |")
        md.append("")
    (out / "summary.csv").write_text("\n".join(lines) + "\n")
    (out / "summary.md").write_text("\n".join(md))


def plot_trajectories(out, figures, condition):
    figure, axes = plt.subplots(2, 5, figsize=(22, 9.5), constrained_layout=True)
    for axis, (name, shape, truth, trajectories) in zip(axes.ravel(), figures):
        gt = truth[:, :2] - truth[0, :2]
        axis.plot(*gt.T, color="black", linestyle="--", linewidth=1.6, label="ground truth")
        extent = [gt]
        for method in METHODS:
            poses = trajectories[method]
            # Show each estimate in the ground-truth gauge of its first frame, as KITTI plots do.
            first = kitti.se2_to_matrix(truth[0]) @ np.linalg.inv(kitti.se2_to_matrix(poses[0]))
            world = np.array([(first @ kitti.se2_to_matrix(p))[:2, 3] for p in poses]) - truth[0, :2]
            axis.plot(*world.T, color=COLORS[method], linestyle=STYLES[method], linewidth=1.3, label=LABELS[method])
            if method == "sonar_slam":
                extent.append(world)
        axis.plot(gt[0, 0], gt[0, 1], marker="o", color="black", markersize=5)
        # Square window around truth and the sonar SLAM estimate; other methods may leave it.
        points = np.vstack(extent)
        centre = 0.5 * (points.min(axis=0) + points.max(axis=0))
        half = 0.5 * np.ptp(points, axis=0).max() + 1.0
        axis.set_xlim(centre[0] - half, centre[0] + half)
        axis.set_ylim(centre[1] - half, centre[1] + half)
        axis.set_title(f"{name}: {shape}", fontsize=10)
        axis.set_aspect("equal")
        axis.grid(alpha=0.25)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", ncol=5, fontsize=10, bbox_to_anchor=(0.5, -0.03))
    figure.suptitle(f"Trajectories in the first frame gauge, no alignment ({condition} imagery)", fontsize=13)
    figure.savefig(out / f"trajectories_{condition}.png", dpi=140, bbox_inches="tight")
    plt.close(figure)


def _curves(out, block, key, xlabel, filename, title):
    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 4.2), constrained_layout=True)
    for method in METHODS:
        rows = block["overall"][method][key]
        if not rows:
            continue
        x = [r["length" if key == "by_length" else "speed"] for r in rows]
        left.plot(x, [r["t_err_percent"] for r in rows], marker="o", color=COLORS[method],
                  linestyle=STYLES[method], label=LABELS[method])
        right.plot(x, [r["r_err_deg_per_m"] for r in rows], marker="o", color=COLORS[method],
                   linestyle=STYLES[method], label=LABELS[method])
    left.set(xlabel=xlabel, ylabel="translation error (%)")
    right.set(xlabel=xlabel, ylabel="rotation error (deg/m)")
    for axis in (left, right):
        axis.grid(alpha=0.25)
        axis.set_ylim(bottom=0)
    left.legend(fontsize=9)
    figure.suptitle(title, fontsize=12)
    figure.savefig(out / filename, dpi=140)
    plt.close(figure)


def plot_error_curves(out, block, condition):
    _curves(out, block, "by_length", "path length (m)", f"error_by_length_{condition}.png",
            f"KITTI error versus segment length, all sequences pooled ({condition} imagery)")
    _curves(out, block, "by_speed", "speed (m/s)", f"error_by_speed_{condition}.png",
            f"KITTI error versus speed, all sequences pooled ({condition} imagery)")


def plot_per_sequence(out, block, condition):
    names = [s["sequence"] for s in block["sequences"]]
    x = np.arange(len(names))
    width = 0.2
    figure, (left, right) = plt.subplots(1, 2, figsize=(14, 4.2), constrained_layout=True)
    for k, method in enumerate(METHODS):
        t = [s["methods"][method]["t_err_percent"] for s in block["sequences"]]
        r = [s["methods"][method]["r_err_deg_per_m"] for s in block["sequences"]]
        left.bar(x + (k - 1.5) * width, t, width, color=COLORS[method], label=LABELS[method])
        right.bar(x + (k - 1.5) * width, r, width, color=COLORS[method], label=LABELS[method])
    for axis, label in ((left, "translation error (%)"), (right, "rotation error (deg/m)")):
        axis.set_xticks(x, names)
        axis.set(xlabel="sequence", ylabel=label)
        axis.grid(axis="y", alpha=0.25)
    left.legend(fontsize=9)
    figure.suptitle(f"KITTI errors per sequence ({condition} imagery)", fontsize=12)
    figure.savefig(out / f"per_sequence_{condition}.png", dpi=140)
    plt.close(figure)


if __name__ == "__main__":
    main()
