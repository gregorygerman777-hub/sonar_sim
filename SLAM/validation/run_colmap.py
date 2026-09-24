"""COLMAP baseline (pycolmap): SIFT features, sequential (or exhaustive) matching, incremental mapping.

    python SLAM/validation/run_colmap.py --dataset tum_freiburg1_xyz [--matcher sequential|exhaustive]

COLMAP gets exactly the images and intrinsics our SLAM gets: the undistorted frames written by the
same dataset adapter, a single PINHOLE camera with the adapter's K, and focal length, principal point
and distortion held fixed. Writes the same outputs as run_slam.py into results/<dataset>_colmap_<matcher>/.
If COLMAP splits the sequence into several models, the largest is exported and all model sizes are
recorded in run_meta.json.
"""

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pycolmap

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import export, geometry as geo  # noqa: E402
import sequences  # noqa: E402


def cam_from_world(image):
    pose = image.cam_from_world
    pose = pose() if callable(pose) else pose
    return pose.rotation.matrix(), np.asarray(pose.translation, float)


def colmap_pinhole_params(K):
    """PINHOLE parameters for COLMAP from an OpenCV style K. COLMAP puts the centre of the top left pixel at
    (0.5, 0.5) (its keypoints come out half a pixel from OpenCV's, tests/test_colmap.py); OpenCV and every
    calibration used here put it at (0, 0). So the principal point moves by half a pixel (CHANGELOG entry 17)."""
    return [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]) + 0.5, float(K[1, 2]) + 0.5]


def check_overwrite(out, stride, overwrite=False):
    """The output name has no stride in it (the reported fr3 and AQUALOC runs use every 3rd and 2nd frame under the
    plain name), so refuse to replace an existing run made with a different stride."""
    meta = Path(out) / "run_meta.json"
    if meta.exists() and not overwrite:
        old = int(json.loads(meta.read_text()).get("stride", 1))
        if old != stride:
            raise SystemExit(f"{out} holds a stride {old} run; refusing to replace it with stride {stride} "
                             f"(pass --out elsewhere, or --overwrite)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--matcher", choices=("sequential", "exhaustive"), default="sequential")
    parser.add_argument("--overlap", type=int, default=10)
    parser.add_argument("--out", type=Path, default=HERE / "results")
    parser.add_argument("--work", type=Path, default=sequences.DATA / "colmap_work")
    parser.add_argument("--overwrite", action="store_true", help="replace an existing run made with another stride")
    args = parser.parse_args()

    seq = sequences.get(args.dataset).subsample(args.stride)
    name = f"{seq.name}_colmap_{args.matcher}"
    out = args.out / name
    check_overwrite(out, args.stride, args.overwrite)
    out.mkdir(parents=True, exist_ok=True)
    work = args.work / name
    if work.exists():
        shutil.rmtree(work)
    images = work / "images"
    images.mkdir(parents=True)
    started = time.perf_counter()
    for i in range(len(seq)):
        gray, color = seq.load(i)
        cv2.imwrite(str(images / f"{i:06d}.png"), color if color is not None else gray)
    K = seq.K
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "PINHOLE"
    reader.camera_params = ",".join(repr(v) for v in colmap_pinhole_params(K))
    extraction = pycolmap.FeatureExtractionOptions()
    extraction.sift.max_num_features = 8192
    db = work / "database.db"
    pycolmap.extract_features(db, images, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader,
                              extraction_options=extraction, device=pycolmap.Device.cpu)
    if args.matcher == "sequential":
        pairing = pycolmap.SequentialPairingOptions()
        pairing.overlap = args.overlap
        pairing.loop_detection = False
        pycolmap.match_sequential(db, pairing_options=pairing, device=pycolmap.Device.cpu)
    else:
        pycolmap.match_exhaustive(db, device=pycolmap.Device.cpu)
    options = pycolmap.IncrementalPipelineOptions()
    options.ba_refine_focal_length = False
    options.ba_refine_principal_point = False
    options.ba_refine_extra_params = False
    options.mapper.abs_pose_refine_focal_length = False
    options.mapper.abs_pose_refine_extra_params = False
    options.random_seed = 0
    options.mapper.random_seed = 0
    options.triangulation.random_seed = 0
    (work / "sparse").mkdir()
    models = pycolmap.incremental_mapping(db, images, work / "sparse", options)
    seconds = time.perf_counter() - started
    sizes = {int(k): int(m.num_reg_images()) for k, m in models.items()}
    n = len(seq)
    meta = dict(dataset=seq.name, dataset_key=args.dataset, method=f"colmap_{args.matcher}", frontend="sift",
                stride=args.stride, frame_timestamps=[float(x) for x in seq.timestamps],
                frames_total=n, models=sizes, runtime_seconds=seconds, K=K, colmap_camera_params=colmap_pinhole_params(K),
                pycolmap_version=pycolmap.__version__, notes="largest model exported; fixed PINHOLE intrinsics (principal point + 0.5 px for COLMAP's pixel convention)")
    if not models:
        export.write_meta(out / "run_meta.json", frames_posed=0, fraction_posed=0.0, **meta)
        export.write_tum(out / "trajectory_tum.txt", [], [], [])
        export.write_ply(out / "points.ply", np.empty((0, 3)))
        print(f"{name}: COLMAP produced no model")
        return
    best = max(models.values(), key=lambda m: m.num_reg_images())
    rows = []
    for image in best.images.values():
        if not getattr(image, "has_pose", True):
            continue
        index = int(Path(image.name).stem)
        R, t = cam_from_world(image)
        Rwc, twc = geo.invert(R, t)
        rows.append((index, float(seq.timestamps[index]), Rwc, twc))
    rows.sort(key=lambda r: r[0])
    export.write_tum(out / "trajectory_tum.txt", [r[1] for r in rows], [r[2] for r in rows], [r[3] for r in rows])
    xyz = np.array([p.xyz for p in best.points3D.values()]).reshape(-1, 3)
    rgb = np.array([p.color for p in best.points3D.values()], np.uint8).reshape(-1, 3)
    track = np.array([p.track.length() for p in best.points3D.values()])
    export.write_ply(out / "points.ply", xyz, rgb)
    posed = {r[0] for r in rows}
    with open(out / "frames.csv", "w") as fh:
        fh.write("index,timestamp,status,inliers\n")
        for i in range(n):
            fh.write(f"{i},{seq.timestamps[i]:.9f},{'registered' if i in posed else 'untracked'},0\n")
    export.write_meta(out / "run_meta.json", frames_posed=len(rows), fraction_posed=len(rows) / max(n, 1),
                      reprojection=dict(mean_px=float(best.compute_mean_reprojection_error())),
                      map_points=int(len(xyz)), median_observations_per_point=float(np.median(track)) if len(track) else 0,
                      **meta)
    print(f"{name}: {len(rows)}/{n} registered in the largest of {len(models)} model(s) {sizes}, "
          f"{len(xyz)} points, {seconds:.0f} s")


if __name__ == "__main__":
    main()
