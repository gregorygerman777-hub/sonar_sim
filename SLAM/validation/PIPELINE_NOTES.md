# Pipeline notes (Step 0, written 2026-09-23 before any code changes)

This file records what the optical pipeline actually was before the ground truth
validation work in this folder began. It was written from reading every file under
`SLAM/`, with the pool assessment (`SLAM/pool_consistency_20260919/`) and the KITTI
benchmark (`SLAM/optical_benchmark_20260918/`) read in full.

## What exists

| Piece | File | What it does |
|---|---|---|
| Front end | `SLAM/optical_benchmark_20260918/stereo_vo.py` | ORB (4000 candidates, 12 x 4 bucketing grid, 60 per cell), Lowe ratio 0.8 matching, stereo block matching, PnP RANSAC for stereo, 5 point essential matrix + `recoverPose` for monocular |
| KITTI runner | `SLAM/optical_benchmark_20260918/run_kitti.py` | Entry point for the KITTI benchmark. Runs stereo VO and monocular VO frame to frame and scores with the KITTI devkit metric |
| KITTI I/O | `SLAM/optical_benchmark_20260918/kitti_dataset.py` | Reads `calib.txt`, `times.txt`, image folders, ground truth poses |
| Pool analysis | `SLAM/pool_consistency_20260919/scripts/*.py` | Pairwise essential matrix verification over all 6,786 pairs, rotation averaging, spectral synchronization, translation direction synchronization. Paths are hardcoded to `/Users/gregsobe/sonar_sim/...` |
| Sonar SLAM | `core/slam.cpp`, `python/slam.py` | Planar sonar SLAM (not optical; out of scope here) |

## Entry points

* KITTI: `python SLAM/optical_benchmark_20260918/run_kitti.py --data <kitti>/dataset`
* Pool: the numbered scripts in `SLAM/pool_consistency_20260919/scripts/`, run in the order in that README, after editing hardcoded paths.
* There was no general entry point that takes an arbitrary image folder plus calibration.

## Input formats

* KITTI: `sequences/SS/image_0/*.png`, `image_1/*.png`, `calib.txt` (P0..P3 as 3x4), `times.txt`, `poses/SS.txt`.
* Pool: `opt1.bmp` .. `opt117.bmp` (1024 x 768, 24 bit color), `OSCalibration.mat` with
  `K = [1403.461, 0, 476.517; 0, 1403.461, 392.916; 0, 0, 1]`, no distortion coefficients,
  plus `Ro2s`, `To2s` (optical to sonar extrinsic), `Final_Proj` (3 x 4 x 28), `rmin`, `rmax`, `caseno`.
  The pool report uses a width scaled K (fx = 1507.97, cx = 512.0) on the hypothesis of a
  horizontal resize after calibration.
* Images are always converted to 8 bit grayscale before feature extraction. No undistortion
  is applied anywhere (KITTI images are already rectified).

## Output formats

* KITTI runner: poses in KITTI layout (12 numbers per line, row major 3 x 4 `T_w_cam`),
  `metrics.json`, `diagnostics/SS.json`, PNG figures.
* Pool scripts: pickles of pairwise results and rotation estimates, PNG figures, a PDF.
* **No run writes 3D points.** No PLY or any other point cloud is produced anywhere.

## Conventions

* Relative motion `T_cur_prev` maps points from the previous camera frame into the current
  camera frame (what `solvePnP` and `recoverPose` return).
* Absolute poses are chained as `T_w_cur = T_w_prev @ inv(T_cur_prev)`, so the stored poses are
  **camera to world** (`T_w_cam`), with world equal to camera 0 at frame 0.
* Rotations are 3 x 3 matrices in the front end; `scipy.spatial.transform.Rotation` rotation
  vectors inside the stereo refinement. No quaternions are written anywhere, so there was no
  quaternion order to get wrong. (The TUM exporter added in this work writes `qx qy qz qw`, scalar last.)
* Camera axes: OpenCV (x right, y down, z forward).

## Front ends

* ORB: the default everywhere (`stereo_vo.make_orb`).
* SIFT: only inside the pool scripts (`sift_crossblock.py`, `sift_full_pairwise.py`,
  `sift_full_pipeline.py`), via `cv2.SIFT_create`, not in `stereo_vo.py`.

## The key finding from Step 0

**The monocular path isn't SLAM.** `motion_essential` returns a rotation and a *unit*
translation per frame pair. In the KITTI benchmark each step's length comes from
ground truth (`mono_vo_gt_scale`). There's no map, no triangulation, no tracking against
3D points, no keyframes and no bundle adjustment. So:

1. It can't output a 3D scene model: nothing is ever triangulated.
2. It can't output a scale consistent trajectory without ground truth: each step's
   translation length is unknown, and chaining unit steps gives the wrong shape.
3. On the pool data, each frame's orientation depended only on pairwise measurements. That's
   why the pool assessment had to turn to rotation averaging rather than tracking.

So "Step 0: add a minimal exporter" can't be satisfied without building the missing back end.
A keyframe based monocular SLAM system (`monoslam/`) was added on top of the existing
feature and matching code. It covers initialization, map point triangulation, tracking against the map,
keyframes, and local and global bundle adjustment. The existing `stereo_vo.py` is not modified,
so the KITTI results stay reproducible.
