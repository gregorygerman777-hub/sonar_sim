"""Run stereo and monocular visual odometry on KITTI odometry and score with the devkit metric.

Writes, under --output (default results/ beside this file):
  results/METHOD/data/SS.txt        estimates in KITTI layout, world = camera 0 at frame 0
  diagnostics/SS.json               per frame matches, inliers, fallbacks and timing
  metrics.json, summary.csv, summary.md, trajectories.png, error_by_length.png,
  error_by_speed.png, per_sequence.png
Ground truth is read only by the scorer and, for `mono_vo_gt_scale`, to set the
length of each monocular step after the direction has been estimated.
"""

import argparse
import hashlib
import json
import platform
import subprocess
import sys
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT / "SLAM/kitti_benchmark_20260916")]

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import cv2  # noqa: E402
import kitti_dataset  # noqa: E402
import kitti_odometry as kitti  # noqa: E402
import stereo_vo  # noqa: E402

METHODS = ("stereo_vo", "mono_vo_gt_scale")
LABELS = {"stereo_vo": "stereo VO", "mono_vo_gt_scale": "monocular VO, ground truth scale"}
COLORS = {"stereo_vo": "#2563eb", "mono_vo_gt_scale": "#f59e0b"}
DEFAULT_DATA = ROOT / "data_external/kitti_odometry/dataset"


def run_sequence(args):
    """Estimate one sequence without ground truth; returns unit-scale mono motions and stereo motions."""
    sequence, data, max_frames = args
    data = Path(data)
    sequence_dir = data / "sequences" / sequence
    calibration = kitti_dataset.load_calibration(sequence_dir)
    left_paths = kitti_dataset.frame_paths(sequence_dir, 0)[:max_frames]
    right_paths = kitti_dataset.frame_paths(sequence_dir, 1)[:max_frames]
    front = stereo_vo.StereoFrontEnd(calibration["K"], calibration["baseline_m"])
    stereo_motions, mono_motions, diagnostics = [], [], []
    previous, previous_motion = None, np.eye(4)
    started = time.perf_counter()
    for index, (left_path, right_path) in enumerate(zip(left_paths, right_paths)):
        frame_started = time.perf_counter()
        left = kitti_dataset.read_gray(left_path)
        right = kitti_dataset.read_gray(right_path)
        current = front.extract(left, right)
        if previous is not None:
            motion, stereo_info = front.motion_pnp(previous, current, guess=previous_motion)
            stereo_fallback = motion is None
            if stereo_fallback:
                motion = previous_motion.copy()   # constant velocity when matching fails
            stereo_motions.append(motion)
            previous_motion = motion
            direction, mono_info = front.motion_essential(previous, current)
            mono_fallback = direction is None
            if mono_fallback:
                direction = np.eye(4)
                direction[:3, :3] = motion[:3, :3]
                direction[:3, 3] = motion[:3, 3] / max(np.linalg.norm(motion[:3, 3]), 1e-9)
            mono_motions.append(direction)
            diagnostics.append(dict(frame=index, features=int(len(current.keypoints)),
                                    depth_points=int(np.isfinite(current.points3d[:, 2]).sum()),
                                    stereo_matches=stereo_info["matches"], stereo_inliers=stereo_info["inliers"],
                                    stereo_fallback=bool(stereo_fallback), mono_matches=mono_info["matches"],
                                    mono_inliers=mono_info["inliers"], mono_fallback=bool(mono_fallback),
                                    seconds=time.perf_counter() - frame_started))
        previous = current
    return dict(sequence=sequence, frames=len(left_paths), stereo=np.array(stereo_motions),
                mono_unit=np.array(mono_motions), diagnostics=diagnostics,
                seconds=time.perf_counter() - started)


def scale_mono_with_ground_truth(mono_unit, truth):
    """Give each unit-direction monocular step the ground-truth step length (scoring-side operation)."""
    motions = []
    for k, motion in enumerate(mono_unit):
        step = np.linalg.inv(truth[k]) @ truth[k + 1]
        scaled = motion.copy()
        scaled[:3, 3] = motion[:3, 3] * np.linalg.norm(step[:3, 3])
        motions.append(scaled)
    return np.array(motions)


def score(truth, estimate, times):
    errors = kitti.sequence_errors(truth, estimate, kitti.KITTI_LENGTHS_M, kitti.KITTI_STEP_FRAMES, times)
    _, ate = kitti.aligned_ate_se3(estimate, truth)
    return dict(**kitti.average_errors(errors), ate_m=ate,
                by_length=kitti.errors_by_length(errors, kitti.KITTI_LENGTHS_M),
                by_speed=kitti.errors_by_speed(errors, bin_width=2.0), _errors=errors)


def md5_of(path, chunk=1 << 24):
    digest = hashlib.md5()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(chunk), b""):
            digest.update(block)
    return digest.hexdigest()


def environment(data):
    def git(*args):
        try:
            return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
        except Exception:  # noqa: BLE001
            return None
    archive = Path(data).parent / "data_odometry_gray.zip"
    return dict(python=platform.python_version(), numpy=np.__version__, opencv=cv2.__version__,
                platform=platform.platform(), git_commit=git("rev-parse", "HEAD"),
                git_dirty=bool(git("status", "--porcelain")),
                protocol_sha256=hashlib.sha256((HERE / "PROTOCOL.md").read_bytes()).hexdigest(),
                archive_md5=md5_of(archive) if archive.exists() else None,
                settings=asdict(stereo_vo.Settings()), lengths_m=list(kitti.KITTI_LENGTHS_M),
                step_frames=kitti.KITTI_STEP_FRAMES)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output", type=Path, default=HERE / "results")
    parser.add_argument("--sequences", nargs="*", default=kitti_dataset.GROUND_TRUTH_SEQUENCES)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--skip-md5", action="store_true")
    parser.add_argument("--plot-only", action="store_true", help="redraw figures from saved results")
    args = parser.parse_args()
    out = args.output
    if args.plot_only:
        report = json.loads((out / "metrics.json").read_text())
        figures = []
        for row in report["sequences"]:
            sequence = row["sequence"]
            truth = kitti_dataset.load_ground_truth(args.data / "poses", sequence)[:row["frames"]]
            estimates = {m: kitti.read_poses(out / f"results/{m}/data/{sequence}.txt") for m in METHODS}
            figures.append((sequence, truth, estimates))
        write_tables(out, report)
        plot_trajectories(out, figures)
        plot_error_curves(out, report)
        plot_per_sequence(out, report)
        return
    for method in METHODS:
        (out / f"results/{method}/data").mkdir(parents=True, exist_ok=True)
    (out / "diagnostics").mkdir(parents=True, exist_ok=True)

    jobs = [(sequence, str(args.data), args.max_frames) for sequence in args.sequences]
    with Pool(min(args.workers, len(jobs))) as pool:
        runs = pool.map(run_sequence, jobs)

    report = dict(environment=environment(args.data) if not args.skip_md5 else
                  dict(environment(args.data.parent / "missing"), archive_md5=None), sequences=[])
    pooled = {m: [] for m in METHODS}
    figures = []
    for run in runs:
        sequence = run["sequence"]
        truth = kitti_dataset.load_ground_truth(args.data / "poses", sequence)[:run["frames"]]
        times = kitti_dataset.load_times(args.data / "sequences" / sequence)[:run["frames"]]
        estimates = {"stereo_vo": stereo_vo.chain(run["stereo"]),
                     "mono_vo_gt_scale": stereo_vo.chain(scale_mono_with_ground_truth(run["mono_unit"], truth))}
        diagnostics = run["diagnostics"]
        row = dict(sequence=sequence, frames=run["frames"], path_length_m=float(kitti.trajectory_distances(truth)[-1]),
                   duration_s=float(times[-1] - times[0]), seconds=run["seconds"],
                   seconds_per_frame=run["seconds"] / max(run["frames"], 1),
                   mean_features=float(np.mean([d["features"] for d in diagnostics])),
                   mean_depth_points=float(np.mean([d["depth_points"] for d in diagnostics])),
                   mean_stereo_inliers=float(np.mean([d["stereo_inliers"] for d in diagnostics])),
                   stereo_fallbacks=int(sum(d["stereo_fallback"] for d in diagnostics)),
                   mono_fallbacks=int(sum(d["mono_fallback"] for d in diagnostics)), methods={})
        for method in METHODS:
            kitti.write_poses(out / f"results/{method}/data/{sequence}.txt", estimates[method])
            scored = score(truth, estimates[method], times)
            pooled[method].extend(scored.pop("_errors"))
            row["methods"][method] = scored
        (out / f"diagnostics/{sequence}.json").write_text(json.dumps(diagnostics))
        report["sequences"].append(row)
        figures.append((sequence, truth, estimates))
        print(sequence, f"{run['frames']} frames", f"{row['seconds_per_frame'] * 1000:.0f} ms/frame",
              " ".join(f"{m}: {row['methods'][m]['t_err_percent']:.2f}%/{row['methods'][m]['r_err_deg_per_m']:.4f}"
                       for m in METHODS), flush=True)

    report["overall"] = {m: dict(**kitti.average_errors(pooled[m]),
                                 by_length=kitti.errors_by_length(pooled[m], kitti.KITTI_LENGTHS_M),
                                 by_speed=kitti.errors_by_speed(pooled[m], bin_width=2.0),
                                 mean_ate_m=float(np.mean([s["methods"][m]["ate_m"] for s in report["sequences"]])))
                         for m in METHODS}
    (out / "metrics.json").write_text(json.dumps(report, indent=2))
    write_tables(out, report)
    plot_trajectories(out, figures)
    plot_error_curves(out, report)
    plot_per_sequence(out, report)
    print(json.dumps({m: {k: v for k, v in report["overall"][m].items() if not isinstance(v, list)}
                      for m in METHODS}, indent=2))


def write_tables(out, report):
    header = ["sequence", "frames", "path_m", "ms_per_frame", "mean_stereo_inliers", "stereo_fallbacks"]
    for method in METHODS:
        header += [f"{method}_t_err_pct", f"{method}_r_err_deg_m", f"{method}_ate_m"]
    lines = [",".join(header)]
    md = ["| Seq | Frames | Path (m) | ms/frame | Inliers | " + " | ".join(f"{LABELS[m]} t / r / ATE" for m in METHODS) + " |",
          "|---|---:|---:|---:|---:|" + "|".join("---:" for _ in METHODS) + "|"]
    for s in report["sequences"]:
        values = [s["sequence"], s["frames"], f"{s['path_length_m']:.0f}", f"{s['seconds_per_frame'] * 1000:.0f}",
                  f"{s['mean_stereo_inliers']:.0f}", s["stereo_fallbacks"]]
        cells = []
        for m in METHODS:
            r = s["methods"][m]
            values += [f"{r['t_err_percent']:.3f}", f"{r['r_err_deg_per_m']:.5f}", f"{r['ate_m']:.2f}"]
            cells.append(f"{r['t_err_percent']:.2f} % / {r['r_err_deg_per_m']:.4f} / {r['ate_m']:.1f} m")
        lines.append(",".join(str(v) for v in values))
        md.append(f"| {s['sequence']} | {s['frames']} | {s['path_length_m']:.0f} | {s['seconds_per_frame'] * 1000:.0f} | "
                  f"{s['mean_stereo_inliers']:.0f} | " + " | ".join(cells) + " |")
    overall = []
    for m in METHODS:
        r = report["overall"][m]
        overall.append(f"{r['t_err_percent']:.2f} % / {r['r_err_deg_per_m']:.4f} / {r['mean_ate_m']:.1f} m")
    md.append("| all | mean over all segments | | | | " + " | ".join(overall) + " |")
    (out / "summary.csv").write_text("\n".join(lines) + "\n")
    (out / "summary.md").write_text("\n".join(md) + "\n")


def plot_trajectories(out, figures):
    count = len(figures)
    cols = 4
    rows = int(np.ceil(count / cols))
    figure, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 4.6 * rows), constrained_layout=True)
    axes = np.atleast_1d(axes).ravel()
    for axis, (sequence, truth, estimates) in zip(axes, figures):
        axis.plot(truth[:, 0, 3], truth[:, 2, 3], color="black", linestyle="--", linewidth=1.5, label="ground truth")
        for method in METHODS:
            axis.plot(estimates[method][:, 0, 3], estimates[method][:, 2, 3], color=COLORS[method],
                      linewidth=1.2, label=LABELS[method])
        axis.plot(truth[0, 0, 3], truth[0, 2, 3], marker="o", color="black", markersize=5)
        points = truth[:, [0, 2], 3]
        centre = 0.5 * (points.min(axis=0) + points.max(axis=0))
        half = 0.5 * np.ptp(points, axis=0).max() * 1.15 + 20.0
        axis.set_xlim(centre[0] - half, centre[0] + half)
        axis.set_ylim(centre[1] - half, centre[1] + half)
        axis.set_title(f"sequence {sequence}", fontsize=11)
        axis.set_aspect("equal")
        axis.grid(alpha=0.25)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("z (m)")
    for axis in axes[count:]:
        axis.axis("off")
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower right", fontsize=11, bbox_to_anchor=(0.98, 0.04))
    figure.suptitle("KITTI odometry: top view in the frame of the first camera pose, no alignment", fontsize=13)
    figure.savefig(out / "trajectories.png", dpi=130)
    plt.close(figure)


def _curves(out, report, key, xlabel, filename, title):
    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 4.2), constrained_layout=True)
    for method in METHODS:
        rows = report["overall"][method][key]
        if not rows:
            continue
        x = [r["length" if key == "by_length" else "speed"] for r in rows]
        left.plot(x, [r["t_err_percent"] for r in rows], marker="o", color=COLORS[method], label=LABELS[method])
        right.plot(x, [r["r_err_deg_per_m"] for r in rows], marker="o", color=COLORS[method], label=LABELS[method])
    left.set(xlabel=xlabel, ylabel="translation error (%)")
    right.set(xlabel=xlabel, ylabel="rotation error (deg/m)")
    for axis in (left, right):
        axis.grid(alpha=0.25)
        axis.set_ylim(bottom=0)
    left.legend(fontsize=9)
    figure.suptitle(title, fontsize=12)
    figure.savefig(out / filename, dpi=140)
    plt.close(figure)


def plot_error_curves(out, report):
    _curves(out, report, "by_length", "path length (m)", "error_by_length.png",
            "KITTI error versus segment length, sequences 00 to 10 pooled")
    _curves(out, report, "by_speed", "speed (m/s)", "error_by_speed.png",
            "KITTI error versus speed, sequences 00 to 10 pooled")


def plot_per_sequence(out, report):
    names = [s["sequence"] for s in report["sequences"]]
    x = np.arange(len(names))
    width = 0.38
    figure, (left, right) = plt.subplots(1, 2, figsize=(14, 4.2), constrained_layout=True)
    for k, method in enumerate(METHODS):
        t = [s["methods"][method]["t_err_percent"] for s in report["sequences"]]
        r = [s["methods"][method]["r_err_deg_per_m"] for s in report["sequences"]]
        left.bar(x + (k - 0.5) * width, t, width, color=COLORS[method], label=LABELS[method])
        right.bar(x + (k - 0.5) * width, r, width, color=COLORS[method], label=LABELS[method])
    for axis, label in ((left, "translation error (%)"), (right, "rotation error (deg/m)")):
        axis.set_xticks(x, names)
        axis.set(xlabel="sequence", ylabel=label)
        axis.grid(axis="y", alpha=0.25)
    left.legend(fontsize=9)
    figure.suptitle("KITTI errors per sequence", fontsize=12)
    figure.savefig(out / "per_sequence.png", dpi=140)
    plt.close(figure)


if __name__ == "__main__":
    main()
