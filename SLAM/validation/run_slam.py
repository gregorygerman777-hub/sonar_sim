"""Run the monocular SLAM on one dataset and write the standard outputs.

    python SLAM/validation/run_slam.py --dataset tum_freiburg1_xyz --frontend orb

Writes SLAM/validation/results/<dataset>_<frontend>/:
    trajectory_tum.txt   timestamp tx ty tz qx qy qz qw, camera to world, tracked frames only
    points.ply           triangulated map points (xyz, rgb)
    frames.csv           per frame status (tracked, keyframe, relocalized, localized_after, untracked)
    run_meta.json        dataset, front end, settings, frame counts, runtime, git commit
The SLAM never sees ground truth. Parameters are the defaults in monoslam/system.py for every dataset.
"""

import argparse
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import export  # noqa: E402
from monoslam.system import MonoSLAM, Settings  # noqa: E402
import sequences  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--frontend", choices=("orb", "sift"), default="orb")
    parser.add_argument("--stride", type=int, default=1, help="use every n-th frame")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--out", type=Path, default=HERE / "results")
    parser.add_argument("--tag", default="")
    parser.add_argument("--set", nargs="*", default=[], metavar="NAME=VALUE",
                        help="override a Settings field (diagnostic experiments only; recorded in run_meta.json)")
    args = parser.parse_args()

    seq = sequences.get(args.dataset).subsample(args.stride)
    n = len(seq) if args.max_frames is None else min(len(seq), args.max_frames)
    out = args.out / f"{seq.name}_{args.frontend}{args.tag}"
    out.mkdir(parents=True, exist_ok=True)
    log_file = open(out / "log.txt", "w")

    def log(msg):
        print(msg, flush=True)
        log_file.write(msg + "\n")
        log_file.flush()

    settings = Settings(frontend=args.frontend)
    for item in args.set:
        name, value = item.split("=", 1)
        default = getattr(settings, name)
        setattr(settings, name, type(default)(value) if not isinstance(default, bool) else value.lower() == "true")
    log(f"{seq.name}: {n} frames, {seq.size[0]}x{seq.size[1]}, front end {args.frontend}")
    started = time.perf_counter()
    # A map that stays lost for settings.new_map_after_lost frames is closed and a new map is started
    # (like ORB-SLAM3's Atlas or COLMAP's multiple models). The map that poses the most frames is the
    # primary output; the others are written to maps/. Maps are never merged.
    maps = []
    slam = MonoSLAM(seq.K, seq.size, settings, log=log)
    first = 0
    lost = 0
    for i in range(n):
        gray, color = seq.load(i)
        rec = slam.process(i, float(seq.timestamps[i]), gray, color)
        lost = lost + 1 if (slam.initialized and rec.status == "untracked") else 0
        if lost >= settings.new_map_after_lost and i < n - 1:
            log(f"  frame {i}: lost for {lost} frames, closing map {len(maps)} and starting a new map")
            maps.append((slam, first))
            slam = MonoSLAM(seq.K, seq.size, settings, log=log)
            first, lost = i + 1, 0
        if i % 50 == 0 or i == n - 1:
            log(f"  frame {i}/{n}: {rec.status}, inliers {rec.inliers}, map {len(maps)}, keyframes {len(slam.kfs)}, "
                f"points {sum(1 for p in slam.points.values() if not p.bad)}, "
                f"median depth {slam.median_depth():.3g}, "
                f"{time.perf_counter() - started:.0f} s")
    maps.append((slam, first))
    track_seconds = time.perf_counter() - started
    log("global bundle adjustment and final localization pass")
    results = []
    for m, (s_, first_) in enumerate(maps):
        if not s_.initialized:
            continue
        s_.finish()
        traj = s_.trajectory()
        posed = [p for p in traj if p["status"] != "untracked"]
        results.append(dict(map=m, first_frame=first_, slam=s_, traj=traj, posed=posed))
    seconds = time.perf_counter() - started
    commit, dirty = export.git_commit(HERE.parents[1])
    maps_meta = [dict(map=r["map"], first_frame=r["first_frame"], frames_posed=len(r["posed"]),
                      keyframes=len(r["slam"].kfs), points=int(len(r["slam"].map_points()[0])))
                 for r in results]
    status = {i: ("untracked", -1, 0) for i in range(n)}
    for r in results:
        for p in r["traj"]:
            if p["status"] != "untracked" and status[p["index"]][0] == "untracked":
                status[p["index"]] = (p["status"], r["map"], p.get("inliers", 0))
    with open(out / "frames.csv", "w") as fh:
        fh.write("index,timestamp,status,map,inliers\n")
        for i in range(n):
            st, m, inl = status[i]
            fh.write(f"{i},{seq.timestamps[i]:.9f},{st},{m},{inl}\n")
    if not results:
        export.write_tum(out / "trajectory_tum.txt", [], [], [])
        export.write_ply(out / "points.ply", np.empty((0, 3)))
        export.write_meta(out / "run_meta.json", dataset=seq.name, dataset_key=args.dataset, method="monoslam",
                          frontend=args.frontend, stride=args.stride, frame_timestamps=[float(x) for x in seq.timestamps[:n]],
                          frames_total=n, frames_posed=0, fraction_posed=0.0, maps=[], settings=asdict(settings),
                          K=seq.K, image_size=seq.size, runtime_seconds=seconds, git_commit=commit, git_dirty=dirty)
        log("done: no map was initialized")
        return
    primary = max(results, key=lambda r: len(r["posed"]))
    for r in results:
        target = out if r is primary else out / "maps" / f"map_{r['map']}"
        posed = r["posed"]
        export.write_tum(target / "trajectory_tum.txt", [p["timestamp"] for p in posed],
                         [p["R_wc"] for p in posed], [p["t_wc"] for p in posed])
        xyz, rgb, _ = r["slam"].map_points()
        export.write_ply(target / "points.ply", xyz, rgb)
    slam, posed = primary["slam"], primary["posed"]
    xyz, rgb, n_obs = slam.map_points()
    kf_frames = [kf.frame for kf in slam.kfs]
    np.savetxt(out / "keyframes.txt", np.array([float(seq.timestamps[i]) for i in kf_frames]), fmt="%.9f",
               header="timestamps of keyframes")
    summary = slam.summary()
    export.write_meta(out / "run_meta.json", dataset=seq.name, dataset_key=args.dataset, method="monoslam",
                      frontend=args.frontend, stride=args.stride,
                      frame_timestamps=[float(x) for x in seq.timestamps[:n]],
                      frames_total=n, frames_posed=len(posed), fraction_posed=len(posed) / max(n, 1),
                      primary_map=primary["map"], maps=maps_meta,
                      frames_posed_any_map=sum(1 for v in status.values() if v[0] != "untracked"),
                      status_counts=summary["status_counts"], keyframes=summary["n_keyframes"],
                      map_points=int(len(xyz)), median_observations_per_point=float(np.median(n_obs)) if len(n_obs) else 0,
                      initialization=summary["init"], stats=summary["stats"], settings=asdict(settings),
                      reprojection=summary["reprojection"],
                      K=seq.K, image_size=seq.size, dataset_notes=seq.notes,
                      runtime_seconds=seconds, tracking_seconds=track_seconds,
                      seconds_per_frame=seconds / max(n, 1), git_commit=commit, git_dirty=dirty)
    log(f"done: {len(posed)}/{n} frames posed by the primary map ({len(results)} map(s): {maps_meta}), "
        f"{summary['n_keyframes']} keyframes, {len(xyz)} points, {seconds:.0f} s. Status: {summary['status_counts']}")


if __name__ == "__main__":
    main()
