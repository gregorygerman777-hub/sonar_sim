"""Export the pool reconstruction (camera trajectory and 3D model) as data files for Dr. Negahdaripour.

    python SLAM/validation/export_pool_deliverable.py [--run results/pool_raw_colmap_exhaustive]
                                                      [--out <dir>] [--metres-per-unit X]

Writes into <out> (default <run>/export):
    pool_reconstruction.mat  K; image_names; Rt (3 x 4 x N, [R | t] per frame, the layout of Final_Proj in
                             OSCalibration.mat); P = K Rt (3 x 4 x N); C (N x 3 camera centres, the trajectory);
                             points (M x 3) and colors (M x 3) of the 3D model; units; frame description
    camera_trajectory.csv    one row per frame: image, frame, camera centre, R (row major), t
    model.ply                the 3D model points with colour
    README.txt               conventions

Convention: a world point X (column) projects to pixel x ~ K (R X + t); the camera centre is C = -R' t. Pixel
coordinates put the centre of the top left pixel at (0, 0), as OpenCV does and as K from OSCalibration.mat is used
throughout (MATLAB indexes that pixel as (1, 1)).
Frame: the reconstruction is rotated and translated (a similarity, which changes no projection) so that z is the
height above the fitted pool floor plane, x runs along the floor lane stripes, and the origin is the floor point
under the centre of the camera path. Units are the reconstruction's own (monocular, arbitrary) unless
--metres-per-unit is given, e.g. from the real lane stripe width (see pool_scale.py).
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "datasets")]

from monoslam import export, geometry as geo  # noqa: E402
import pool_scale  # noqa: E402
import sequences  # noqa: E402


def to_frame(R_cw, t_cw, X, B, origin, scale=1.0):
    """Express cameras (world to camera R_cw, t_cw; arrays of N) and points X in the frame with rows B (3 x 3,
    right handed) at `origin`, multiplied by `scale`. Projections are unchanged."""
    R_new = np.einsum("nij,kj->nik", R_cw, B)                         # R B'
    t_new = scale * (np.einsum("nij,j->ni", R_cw, origin) + t_cw)     # s (R o + t)
    X_new = scale * (np.asarray(X) - origin) @ B.T
    return R_new, t_new, X_new


def floor_frame(xyz, centres, frames, K, R_wc, t_wc):
    """Rows (x along the stripes, y, z up from the floor) and the origin under the centre of the camera path."""
    c0, n, _ = pool_scale.fit_plane(xyz, centres)
    height = float(np.median((centres - c0) @ n))
    centre = centres.mean(0) - ((centres.mean(0) - c0) @ n) * n
    _, across = pool_scale.measure(frames, K, list(zip(R_wc, t_wc)), c0, n, centre, height)   # camera to world
    x = np.cross(n, across)
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    return np.vstack((x, y, n)), centre, height


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, default=HERE / "results" / f"pool_{sequences.POOL_PRIMARY}_colmap_exhaustive")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--metres-per-unit", type=float, default=None)
    args = parser.parse_args()
    out = args.out or args.run / "export"
    out.mkdir(parents=True, exist_ok=True)
    meta = json.loads((args.run / "run_meta.json").read_text())
    K = np.asarray(meta["K"], float)
    ts, R_wc, t_wc = export.read_tum(args.run / "trajectory_tum.txt")
    xyz, rgb = export.read_ply(args.run / "points.ply")
    seq = sequences.get(meta["dataset_key"])
    frames_used = [int(round(x)) for x in ts]                          # pool timestamps are the frame numbers
    names = [Path(seq.paths[f - 1]).name for f in frames_used]
    gray = [seq.load(f - 1)[0] for f in frames_used]
    # world to camera for export and for the frame computation
    R_cw = np.array([geo.invert(R, t)[0] for R, t in zip(R_wc, t_wc)])
    t_cw = np.array([geo.invert(R, t)[1] for R, t in zip(R_wc, t_wc)])
    B, origin, height = floor_frame(xyz, t_wc, gray, K, R_wc, t_wc)
    scale = args.metres_per_unit or 1.0
    units = "metres" if args.metres_per_unit else "reconstruction units (monocular: metric scale unknown)"
    R_out, t_out, X_out = to_frame(R_cw, t_cw, xyz, B, origin, scale)
    C_out = -np.einsum("nji,nj->ni", R_out, t_out)
    Rt = np.concatenate((R_out, t_out[:, :, None]), axis=2).transpose(1, 2, 0)   # 3 x 4 x N
    P = np.einsum("ij,jkn->ikn", K, Rt)
    frame_note = ("z: height above the fitted pool floor plane; x: along the floor lane stripes; origin: floor point "
                  "under the centre of the camera path. x_pixel ~ K (R X + t), C = -R' t. Pixel coordinates put the centre "
                  "of the top left pixel at (0, 0) (OpenCV); in MATLAB, image(v + 1, u + 1) is the pixel at (u, v).")
    import scipy.io
    scipy.io.savemat(str(out / "pool_reconstruction.mat"), dict(
        K=K, image_names=np.array(names, dtype=object), frame_numbers=np.array(frames_used, float),
        Rt=Rt, P=P, C=C_out, points=X_out, colors=rgb if rgb is not None else np.zeros((len(X_out), 3), np.uint8),
        units=units, frame=frame_note, source=f"{args.run.name}: COLMAP {pool_scale_note(meta)}"), do_compression=True)
    with open(out / "camera_trajectory.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["image", "frame", "Cx", "Cy", "Cz"] + [f"r{i}{j}" for i in (1, 2, 3) for j in (1, 2, 3)]
                   + ["tx", "ty", "tz"])
        for k, (name, f) in enumerate(zip(names, frames_used)):
            w.writerow([name, f] + [f"{v:.6f}" for v in C_out[k]] + [f"{v:.9f}" for v in R_out[k].ravel()]
                       + [f"{v:.6f}" for v in t_out[k]])
    export.write_ply(out / "model.ply", X_out, rgb)
    floor_z = X_out[:, 2]
    (out / "README.txt").write_text(
        f"Pool reconstruction exported from {args.run.name} (all {len(names)} frames registered).\n\n"
        f"Units: {units}.\nFrame: {frame_note}\n\n"
        "pool_reconstruction.mat (MATLAB: load('pool_reconstruction.mat')):\n"
        "  K            3x3 camera matrix used (fixed, not refined)\n"
        "  image_names  file name of each frame (opt<n>.bmp); frame_numbers the n\n"
        "  Rt           3x4xN, [R | t] of each frame, same layout as Final_Proj in OSCalibration.mat\n"
        "  P            3x4xN, K * Rt\n"
        "  C            Nx3 camera centres: the estimated camera trajectory\n"
        "  points       Mx3 3D model points; colors Mx3 (RGB 0..255)\n"
        "camera_trajectory.csv: the same cameras, one row per frame (R row major).\n"
        "model.ply: the 3D model (MeshLab, CloudCompare).\n\n"
        f"Camera height above the floor: median {np.median(C_out[:, 2]):.3f}; median height of the model points "
        f"above the floor: {np.median(floor_z):.3f} (most points lie on the floor and the pebble mat).\n"
        f"{100 * np.mean(floor_z > np.median(C_out[:, 2])):.1f} % of the points lie above the cameras: the lane ropes "
        "at the surface and features seen in reflections in the underside of the moving water surface. The "
        "reflection points are not physical structure; they were left in rather than removed by hand.\n")
    print(f"wrote {out}: {len(names)} cameras, {len(X_out)} points; camera height median "
          f"{np.median(C_out[:, 2]):.3f}, ranges x {np.ptp(C_out[:, 0]):.2f} y {np.ptp(C_out[:, 1]):.2f}")


def pool_scale_note(meta):
    return f"{meta.get('method', '')}, pycolmap {meta.get('pycolmap_version', '?')}, fixed PINHOLE K"


if __name__ == "__main__":
    main()
