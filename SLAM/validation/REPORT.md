# Monocular SLAM: validation against ground truth, then Dr. Negahdaripour's pool data

*Gregory German, September 2026. Everything here is reproducible with `SLAM/validation/run_all.sh`.*

## Conclusions first

1. **The implementation is correct where the answer is known exactly.** On a rendered scene resembling the pool (an arc
   around a rock on tiles and pebbles, exact ground truth), the monocular SLAM recovers the 10 m trajectory to
   **0.52 cm ATE (0.05 %) with ORB and 0.23 cm (0.02 %) with SIFT**, and its 3D map lies a **median 1.10 cm (ORB) /
   0.62 cm (SIFT) from the true surface**. COLMAP on the same images: 0.29 cm (map 0.76 cm).
2. **On KITTI, the benchmark Dr. Negahdaripour named, it tracks, and its error is monocular drift.** With SIFT it tracks
   every frame of 10 of the 11 sequences 00 to 10 in one map (01, a highway, splits into 4 maps); with ORB, 4 of the 11,
   the others splitting into 2 to 8 maps where tracking is lost. Where it tracks every frame, the ATE after one Sim(3)
   alignment is 0.3 % of the path on the two short sequences and 1.1 to 4.9 % on the long ones. The RPE over 100 m shows
   why: with no loop closure the monocular scale drifts along a long run (on 00 with SIFT, 100 m of road measures a
   median 0.46 times that in the aligned estimate). COLMAP, on the same frames: 0.65 m on 04 (ours 1.05 and 1.20 m); on
   the 07 loop, without loop detection, 16.98 m (ours 10.11 m with ORB on its main map, 16.61 m with SIFT).
3. **On real underwater imagery it is accurate while it tracks, but it loses track.** AQUALOC harbor 07:
   0.93 cm ATE over the 8.6 m tracked by its main map (ORB), but that is 42 % of the frames; after tracking losses the
   run continues in 3 maps (all maps together 0.81 cm, each aligned separately). COLMAP also splits this sequence
   (3 models, the largest 43 % of the frames), so part of the difficulty is the data: the ORB run loses track where the
   image goes black and then saturated (frames 742 to 782) and where the camera leaves a rock for featureless, turbid
   seabed (frames 1730 to 1883). With SIFT, a closed map is reopened when the camera comes back, and the main map
   covers 758 frames at 0.68 cm.
4. **On the TUM benchmark it is 1.7 to 3.9 times less accurate than COLMAP.** fr1/xyz: 1.58 cm (ORB), 2.35 cm (SIFT)
   against COLMAP 0.92 cm; fr3/long_office: 7.72 cm (ORB), 5.28 cm (SIFT) against 1.98 cm. The gap was investigated
   (initialisation, homography choice, per frame jitter) and none of those explain it; it is a smooth drift consistent
   with having no loop closure and using only keyframe observations (`CHANGELOG.md`, entry 6).
5. **The validation found bugs, and each is fixed with a regression test that fails on the old code.** Ground truth
   exposed two scale bugs, both invisible without it: a monocular scale blow up (about 2,500x) in local bundle
   adjustment, and a scale collapse caused by map point fusion (entries 1 and 7). A code review against the ORB-SLAM
   design the code follows found two deviations in how map points are counted and culled (entry 16). Measurement found
   a half pixel principal point error in the COLMAP baseline and in the synthetic scenes (entries 17 and 19). Checks of
   the evaluation found three errors in the metrics themselves (entries 11, 14 and 22). Every run was then repeated
   with the final code, and every number here comes from those runs (`CHANGELOG.md`, entries 15 to 23, has the before
   and after numbers, including where they got worse).
6. **Pool data: our SLAM tracks 12 of the 117 stills; a multi view reconstruction registers all of them.** The
   longest map of our sequential SLAM holds 12 consecutive stills (opt28 to opt39); over the sequence it starts 4 maps,
   and none of the other three grows beyond 4 frames. COLMAP registers **all 117 frames in one model** (mean
   reprojection error 0.92 px), showing the camera circling the rock target about twice at constant height, with the
   3D map concentrated on the rock and pebble patch. Where both pose the same frames, their relative rotations agree
   to a median of 0.6 degrees (ORB; 0.8 with SIFT). There is no ground truth, so the reconstruction is plausible and
   internally consistent, not verified. All pool numbers use the camera matrix Dr. Negahdaripour confirmed on
   September 23 (the raw K in `OSCalibration.mat`); the earlier width scaled K gives the same picture.
7. **A likely cause of the tracking failures on the pool is moving caustics.** In the synthetic scene with
   moving caustics added, frame to frame tracking (ours, ORB and SIFT) and even COLMAP fail once consecutive frames are
   0.4 s and 5 degrees apart, while the same scene without caustics works at 15 degrees per frame. The real pool
   caustics are milder than the synthetic ones (COLMAP succeeds on the real frames), so this shows the mechanism,
   not its size on the real data.

## Results table

Full table (every run, RPE, runtimes, per map numbers): `results/SUMMARY.md` and `results/summary.csv`. ATE is after
Sim(3) alignment on the frames each method posed, and "ATE / path" is a percentage of their ground truth path.

| Dataset | Method | Frames posed | ATE RMSE | ATE / path | Map error (median) |
|---|---|---|---|---|---|
| Synthetic pool (exact GT, 10.0 m) | ours ORB | 240/240 | 0.52 cm | 0.05 % | 1.10 cm |
| | ours SIFT | 240/240 | 0.23 cm | 0.02 % | 0.62 cm |
| | COLMAP | 240/240 | 0.29 cm | 0.03 % | 0.76 cm |
| Synthetic pool + moving caustics | ours ORB | 240/240 | 1.95 cm | 0.19 % | 3.08 cm |
| | ours SIFT | 240/240 | 0.58 cm | 0.06 % | 1.72 cm |
| | COLMAP | 240/240 | 0.51 cm | 0.05 % | 1.30 cm |
| TUM fr1/xyz (8.0 m) | ours ORB | 798/798 | 1.58 cm | 0.20 % | |
| | ours SIFT | 798/798 | 2.35 cm | 0.29 % | |
| | COLMAP | 798/798 | 0.92 cm | 0.11 % | |
| TUM fr3/long_office (22.1 m) | ours ORB | 2585/2585 | 7.72 cm | 0.35 % | |
| | ours SIFT | 2585/2585 | 5.28 cm | 0.24 % | |
| | COLMAP (1 frame in 3) | 862/862 | 1.98 cm | 0.09 % | |
| AQUALOC harbor 07 (23.9 m) | ours ORB | 947/2261 main map (2066 in 3 maps) | 0.93 cm (0.81 cm all maps) | 0.11 % | |
| | ours SIFT | 758/2261 main map (1690 in 4 maps) | 0.68 cm (0.54 cm all maps) | 0.08 % | |
| | COLMAP (1 frame in 2) | 482/1131 largest of 3 models | 0.54 cm | 0.06 % | |
| KITTI 00 to 10 | ours ORB and SIFT; COLMAP on 04 and 07 | see the KITTI section | | | |
| Pool (no ground truth, raw K) | ours ORB / SIFT | 12 / 11 of 117 | n/a | | |
| | COLMAP exhaustive | 117/117 | n/a | | |

## KITTI odometry 00 to 10

Dr. Negahdaripour named KITTI as the benchmark with ground truth. The left grayscale camera is used alone
(monocular), every frame, with each sequence's rectified intrinsics and the same default settings as every other
dataset; nothing was tuned for KITTI. ATE is after one Sim(3) alignment over the frames the primary map posed, as a
percentage of their ground truth path. RPE is the translation (metres) and rotation (degrees) error over 100 m of
ground truth travel, the shortest segment of the KITTI benchmark's own metric; KITTI frames are about 1 m apart, so the
1 m RPE used elsewhere is undefined. COLMAP (sequential matching, overlap 10, no loop detection) was run on 04, the
shortest sequence, and on 07, a loop. Generated from `results/summary.csv`:

| Seq | Length | ours ORB | ours SIFT | COLMAP (sequential) |
|---|---|---|---|---|
| 00 | 3724 m | 3540/4541, 3 maps: 53.85 m (1.79 %), RPE 22.8 m, 1.02 deg | 4541/4541: 107.14 m (2.88 %), RPE 50.7 m, 0.90 deg | not run |
| 01 | 2453 m | 574/1101, 8 maps: 173.47 m (12.37 %), RPE 78.7 m, 7.19 deg | 478/1101, 4 maps: 7.07 m (0.63 %), RPE 5.5 m, 0.55 deg | not run |
| 02 | 5067 m | 2360/4661, 2 maps: 22.83 m (0.87 %), RPE 9.5 m, 0.77 deg | 4661/4661: 121.51 m (2.40 %), RPE 40.5 m, 0.49 deg | not run |
| 03 | 561 m | 396/801, 2 maps: 1.21 m (0.43 %), RPE 2.0 m, 0.62 deg | 801/801: 1.51 m (0.27 %), RPE 1.9 m, 0.47 deg | not run |
| 04 | 394 m | 271/271: 1.05 m (0.27 %), RPE 1.7 m, 0.35 deg | 271/271: 1.20 m (0.30 %), RPE 1.8 m, 0.16 deg | 271/271: 0.65 m (0.16 %), RPE 1.0 m, 0.13 deg |
| 05 | 2206 m | 2761/2761: 24.95 m (1.13 %), RPE 18.0 m, 0.69 deg | 2761/2761: 51.70 m (2.34 %), RPE 34.1 m, 0.49 deg | not run |
| 06 | 1233 m | 1101/1101: 54.83 m (4.45 %), RPE 28.8 m, 0.50 deg | 1101/1101: 60.51 m (4.91 %), RPE 31.7 m, 0.37 deg | not run |
| 07 | 695 m | 1060/1101, 2 maps: 10.11 m (1.48 %), RPE 10.9 m, 0.72 deg | 1101/1101: 16.61 m (2.39 %), RPE 16.1 m, 0.55 deg | 1101/1101: 16.98 m (2.44 %), RPE 16.7 m, 1.09 deg |
| 08 | 3223 m | 2812/4071, 4 maps: 41.34 m (1.91 %), RPE 25.0 m, 0.85 deg | 4071/4071: 114.76 m (3.56 %), RPE 55.4 m, 0.59 deg | not run |
| 09 | 1705 m | 760/1591, 4 maps: 2.09 m (0.27 %), RPE 2.7 m, 0.70 deg | 1591/1591: 80.76 m (4.74 %), RPE 27.4 m, 0.45 deg | not run |
| 10 | 920 m | 1201/1201: 10.36 m (1.13 %), RPE 8.6 m, 1.22 deg | 1201/1201: 15.84 m (1.72 %), RPE 15.5 m, 0.79 deg | not run |

What it shows:
* **Tracking.** With SIFT, every frame of 10 of the 11 sequences is tracked in one map; on 01 (a highway) tracking is
  lost and the run splits into 4 maps. With ORB, 04, 05, 06 and 10 are tracked in one map, and the other seven split
  into 2 to 8 maps where tracking is lost; the ATE of a split run covers its main map only.
* **Accuracy.** In the runs that track every frame, the error grows with the length of the run: 0.3 % of the path
  on the short 03 and 04, 1.1 to 4.9 % on the others (a split run's ATE covers its main map only: 12.4 % on 01 with
  ORB, over 574 frames). Most of it is scale drift. Without loop closure a monocular map's scale changes
  along the run and one Sim(3) cannot absorb it: on 00 with SIFT, a 100 m stretch of road measures a median 0.46 times
  that in the aligned estimate (0.17 to 1.31 times between the 5th and 95th percentiles), against 1.00 (0.96 to 1.02)
  on 04. Rotation drift is small: under 1.3 degrees per 100 m except on 01 with ORB.
* **ORB against SIFT.** ORB loses track more often, but drifts less while it tracks (05: 24.95 m against 51.70 m over
  the same 2761 frames; 10: 10.36 m against 15.84 m).
* **COLMAP** is more accurate on 04 (0.65 m against 1.05 and 1.20 m). On the 07 loop, with no loop detection, it
  drifts as our SLAM does: 16.98 m, against 16.61 m (SIFT) and 10.11 m (ORB, over the 1060 frames of its main map).
* **Effect of the fixes.** With ORB, 02 and 03 now lose track once where the code before entries 15 to 19 tracked
  every frame. On 03 that was traced to a knife edge: the tracked inliers fall exactly to the minimum of 30 at frame
  395 in every version of the code (`CHANGELOG.md`, after entry 19).

## Figures

Each run has four figures in `figures/<dataset>/<run>/`: `1_trajectory.png` (estimated trajectory
against ground truth, top and side views), `2_model.png` (the 3D map points), `3_overlay.png` (the
trajectory and camera frustums drawn over the 3D map), `4_overlay_top.png` (the same seen from above, map points
coloured by height, arrows for the viewing direction), plus an interactive `3_overlay.html` (at most 100,000 map
points). `REPORT_summary.pdf` (three pages, for e-mail) shows the validation on ground truth first, then our SLAM's
trajectory and 3D model on the pool frames, then the reconstruction of all 117 frames with the trajectory over the
model.

Key figures:

| What | Figure |
|---|---|
| KITTI 00 with SIFT, trajectory against ground truth (every frame, one map) | `figures/kitti_00/kitti_00_sift/1_trajectory.png` |
| KITTI 07, ours (ORB) and COLMAP against ground truth | `figures/kitti_07/kitti_07_orb/1_trajectory.png` |
| Synthetic pool, trajectory over the 3D map (exact ground truth) | `figures/synthetic_pool/synthetic_pool_orb/3_overlay.png` |
| Synthetic pool, 3D map alone | `figures/synthetic_pool/synthetic_pool_orb/2_model.png` |
| AQUALOC harbor 07, trajectory vs ground truth, all maps | `figures/aqualoc_harbor_07/aqualoc_harbor_07_orb/1_trajectory.png` |
| TUM fr1/xyz, ours vs COLMAP vs ground truth | `figures/tum_freiburg1_xyz/tum_freiburg1_xyz_sift/1_trajectory.png` |
| TUM fr3/long_office, trajectory | `figures/tum_freiburg3_long_office_household/tum_freiburg3_long_office_household_sift/1_trajectory.png` |
| **Pool, our SLAM: trajectory / 3D model / trajectory over the model (12 frames, raw K)** | `figures/pool_raw/pool_raw_orb/1_trajectory.png`, `2_model.png`, `3_overlay.png` |
| **Pool, COLMAP: trajectory over the 3D model (all 117 frames, raw K)** | `figures/pool_raw/pool_raw_colmap_exhaustive/3_overlay.png` |
| **Pool, COLMAP: trajectory alone / 3D model alone** | `.../pool_raw_colmap_exhaustive/1_trajectory.png`, `2_model.png` |
| Pool, COLMAP: trajectory over the 3D model, seen from above | `.../pool_raw_colmap_exhaustive/4_overlay_top.png` |
| Pool, width scaled K (comparison) | `figures/pool_width_scaled/pool_width_scaled_colmap_exhaustive/3_overlay.png` |
| Pool: camera positions coloured by frame number | `figures/pool/frame_order_exhaustive.png` |
| Pool: which frames each method could pose, with a floor caustic index | `figures/pool/coverage.png` |
| Synthetic: frame spacing vs moving caustics | `figures/pool/spacing_experiment.png` |

## Bugs found and fixed

Full details, tests and before/after numbers are in `CHANGELOG.md`. Each fix has a test that fails on the old code.

1. **Scale blow up in local BA (found by ground truth).** Only one keyframe held the 7 dof monocular gauge, so the map
   grew about 2,500 times within 50 frames of fr1/xyz while the trajectory shape still looked right. Now at least two
   keyframes are fixed and LM damping has a floor (entry 1).
2. **Tracking local map** now follows covisibility (as in ORB-SLAM) instead of "the last 10 keyframes" (entry 2).
3. **Map point fusion** was added, then **turned off** because it collapsed the scale on fr3 (median depth 1 to
   0.0013; found by ground truth, entry 7).
4. **Map point statistics (found by code review):** visibility was counted per search attempt instead of once per
   tracked frame, and the found ratio test culled established points for ever instead of only points on probation, as
   ORB-SLAM does (entry 16).
5. **Relocalization and lost maps:** a lost map is replaced by a new one instead of the run ending (entry 4);
   relocalization searches the whole map by place recognition, and a closed map is reopened when the camera returns
   to it (entry 15). Maps are never merged.
6. **Half pixel principal point errors (found by measurement):** in the COLMAP baseline, which uses a different pixel
   convention (entry 17), and in the synthetic scenes' `K.txt` (entry 19).
7. **Errors in the evaluation itself:** an orientation error reported on nearly straight paths, where the alignment
   cannot determine it (entry 11); an empty RPE on KITTI (entry 14); RPE segments measured along the estimate instead
   of the ground truth, which distorted the KITTI RPE by up to a factor of 2.4 (entry 22).
8. **Reproducibility and bookkeeping:** `run_all.sh` did not reproduce the reported runs (entry 13); a frame stride run
   overwrote a full rate run (entry 10); reruns were recorded as modified code (entry 18); a rerun kept files of the
   earlier run, and the commit was read when a run ended rather than when it started (entry 21); smaller reporting
   fixes (entries 20 and 23).
9. **Runtime:** sparse reduced camera system and an LM stopping rule, with no change to results (entry 5).
10. **fr1/xyz accuracy gap to COLMAP:** investigated, not resolved; stated as a limitation (entry 6).

Two tried changes were **not** adopted because they failed the synthetic unit test (ORB-SLAM's keyframe rule
and a keyframe rule based on the tracked count both cut keyframes and doubled the map error); that decision was taken on
the unit test, not on benchmark numbers. Some fixes made benchmark numbers worse (fr3 with ORB, KITTI 01, 02 and 03 with
ORB); they were kept because they were decided on engineering grounds, and changing them back because of those numbers
would be tuning on ground truth.

## The pool data

Inputs: `opt1.bmp` ... `opt117.bmp` (1024 x 768) and `OSCalibration.mat`. On September 23 Dr. Negahdaripour
confirmed the camera matrix as K = [1403.5 0 476.5; 0 1403.5 392.9; 0 0 1], which is the K stored in the
calibration file (`raw`; the file holds it to more digits, 1403.461, 476.517 and 392.916, which are used). That is the
primary configuration and every pool number below uses it. The September 19 report used a `width_scaled` hypothesis
(fx scaled by 1024/953 and cx = 512, because the stored principal point implies a 953 x 786 image, not 1024 x 768). It
is kept as a comparison. K is used in OpenCV's pixel convention, with the centre of the top left pixel at (0, 0). If
the calibration came from a MATLAB toolbox that puts that centre at (1, 1), the principal point used here is 1 px too
far right and down; the files delivered cannot tell, and at 0.92 px mean reprojection error it would not change the
picture.

**Raw K against width scaled K** (same frames, same settings, COLMAP with fixed intrinsics;
`results/pool_analysis.json`, key `k_model_comparison`):

| COLMAP matcher | K | Frames registered | Models | Mean reprojection error | Map points |
|---|---|---|---|---|---|
| exhaustive | **raw (confirmed)** | **117/117** | 1 | **0.92 px** | 12,152 |
| exhaustive | width scaled | 117/117 | 1 | 0.94 px | 12,154 |
| sequential | **raw (confirmed)** | 110/117 | 1 | 0.81 px | 9,957 |
| sequential | width scaled | 110/117 | 1 | 0.82 px | 9,979 |

With either matcher the two K models register the same frames, the raw K has a slightly lower reprojection error, and
with exhaustive matching the two trajectories agree to 0.6 % of the trajectory extent (RMS after Sim(3)), with
relative rotations between consecutive frames agreeing to a median of 0.3 degrees. Both sequential models miss opt57 to
opt62 and opt117. So the raw K does not give a visibly worse reconstruction. The question of why (2 cx, 2 cy) =
(953, 786) differs from the 1024 x 768 frame size (cropping or resizing) is still open but does not affect these
results materially.

**What each method could do** with the raw K (`results/pool_analysis.json`, `figures/pool/coverage.png`):

| Method | Frames posed | Notes |
|---|---|---|
| Ours, ORB | 12 (largest map, opt28 to opt39), 21 over 4 maps | the other maps hold 2 to 4 frames each |
| Ours, SIFT | 11 (largest map, opt28 to opt38), 28 over 6 maps | the other maps hold 2 to 6 frames each |
| COLMAP, sequential matching | 110 in one model | opt57 to opt62 and opt117 not in it; 0.81 px mean reprojection error |
| COLMAP, exhaustive matching | **117 in one model** | 0.92 px mean reprojection error; 12,152 map points |

**Skipping frames** (suggested for an initial result): with every 2nd or every 3rd frame, our SLAM poses only the two
frames it initialises on (2/59 and 2/39, ORB and SIFT alike) and then never re-initialises. The stills are already
about 2 s apart, so skipping widens the gaps that break tracking (`CHANGELOG.md`, entry 10).

(Frame numbers are the n in `opt<n>.bmp`.)

**Consistency checks without ground truth** (raw K).
* COLMAP sequential vs exhaustive, 110 common frames: positions agree to 2.2 % of the trajectory extent (RMS after
  Sim(3)), and the relative rotation between consecutive frames agrees to a median of 0.19 degrees (max 10 degrees,
  at one step).
* Ours vs COLMAP exhaustive on the frames both pose (12 with ORB, 11 with SIFT): positions within 0.7 % (ORB) and
  2.1 % (SIFT) of extent, relative rotations within a median of 0.6 (ORB) and 0.8 (SIFT) degrees. (The absolute
  rotation after an alignment on positions only differs by a median of 2 (ORB) to 9 (SIFT) degrees, but with only 11
  or 12 nearly collinear common frames that alignment's rotation is poorly determined, which is why the alignment
  free comparison is used.)
* The reconstruction is physically plausible: the cameras move on a near circle at nearly constant height around
  the rock and pebble patch, all looking inwards, and the densest part of the map is the patch itself.

**Checks that the pool outputs are right** (no ground truth, so these are consistency checks).
* *Reproducible from the raw inputs* (checked on 2026-09-24, before entry 17). A fresh run on `opt1.bmp` ...
  `opt117.bmp` and `OSCalibration.mat` into an empty directory reproduced our SLAM results bit for bit, and the COLMAP
  exhaustive model to 0.17 % of the trajectory extent (COLMAP is multithreaded and not bit reproducible). The
  sequential COLMAP model repeated only to 1.8 %, the same size as its disagreement with the exhaustive one.
* *Metric consistency of K and the poses.* Every frame warped onto the fitted floor plane with its own pose shows the
  lane stripes straight, parallel and of constant width, and the two stripes measure nearly the same width (0.451 and
  0.432 units) in different frames (`pool_scale.py`). A wrong K or wrong poses would bend or shear them.
* *The model reprojects onto the images.* Projected with each frame's pose, the 3D points land on the rock target, the
  pebbles, the lane rope floats and the wall targets (checked on opt1, opt40, opt80 and opt110; the image is not in the
  repository because the frames are unpublished).
* *Known spurious points.* 6.8 % of the points lie above the cameras' median height. Some are real (lane rope floats at
  the surface); others are triangulated from the mirror images of the wall targets in the underside of the moving water
  surface. The latter sit about 1.2 to 1.5 units up and a median 2.7 units from the centre, inside the camera circle
  (radius about 3.8) where no structure exists, so they are not physical. They were left in the model and are flagged
  here instead of removed by hand.

**Data files** (`export_pool_deliverable.py`, in `results/pool_raw_colmap_exhaustive/export/`): the trajectory and
model in a frame tied to the pool (z up from the floor plane, x along the lane stripes, origin under the centre of the
camera path) as `pool_reconstruction.mat` (per frame `Rt` as 3 x 4 x N in the layout of `Final_Proj` in
`OSCalibration.mat`, `P = K Rt`, camera centres `C`, `points`, `colors`), `camera_trajectory.csv` and `model.ply`.
Units are the reconstruction's own until the stripe width is known (`--metres-per-unit` then rescales). Our SLAM's own
trajectory and map are `results/pool_raw_orb/trajectory_tum.txt` and `points.ply`.

**Capture timing.** The only timing information delivered with the frames is the files' modification times
(2014-03-07). Consecutive frames are a median 2 s apart, except for one gap of **574 s (9.6 minutes) between
opt105 and opt106**. In the COLMAP reconstruction (`figures/pool/frame_order_exhaustive.png`), frames 1 to about 50
make one lap around the target, frames 50 to 105 a second lap, and frames 106 to 117 (after the gap) continue on
the same circle. So the set looks like one continuous circling pass plus a short second session taken from the same
path, not unrelated passes. This is inferred from file times and the reconstruction; the capture log would settle it.

**Why the sequential SLAM fails here.** The frames are stills a median 2 s apart with large and irregular viewpoint
changes, and many show moving sunlight caustics on the floor and moving reflections of the water surface. Both
violate what a frame to frame tracker relies on (small motion, a static scene). The synthetic experiment isolates
the caustic effect (`figures/pool/spacing_experiment.png`): with the same scene, tracking survives 15 degrees
between frames when the lighting is static, but fails at 5 degrees when the caustics have moved for 0.4 s between
frames, for ORB, SIFT and COLMAP alike. On the real frames our main map holds in frames 28 to 39, where a crude
floor caustic index is low, but other factors (irregular spacing, reflections) were not isolated. COLMAP succeeds on
the real data because it matches every frame against many others and verifies each pair geometrically before building
one global model, rather than relying on the previous frame.

**What this means for the questions in my September 22 email.**
* A multi view, global method (COLMAP's incremental SfM) handles this sequence where a better descriptor alone
  did not: SIFT inside the sequential SLAM still loses track, while COLMAP (also SIFT) registers every frame.
* The scale of the pool reconstruction is unknown (monocular). No sonar frames were delivered for the same instants,
  so the sonar to optical extrinsic in `OSCalibration.mat` cannot fix it. The floor's dark lane stripes can:
  `pool_scale.py` rectifies each frame onto the fitted floor plane and measures the two stripes beside the mat at
  0.451 and 0.432 model units wide (6 and 8 detections in different frames), 3.28 units apart, with the camera a
  median 0.97 units above the floor. One known length fixes everything: metres per unit = true stripe width / 0.442.
  For a stripe between 0.20 and 0.30 m that would put the camera 0.44 to 0.66 m above the floor, but this is
  conditional; the real stripe width or lane spacing is needed (`results/pool_scale.json`, `CHANGELOG.md` entry 9).

## Method (details)

**What was built.** The existing optical code (`SLAM/optical_benchmark_20260918/stereo_vo.py`) is
frame to frame visual odometry. Its monocular mode takes each step's length from ground truth. It has no map,
so it could never output a 3D model or a scale consistent trajectory (see `PIPELINE_NOTES.md`).
`monoslam/` adds a keyframe based monocular SLAM on the same ORB settings, following the monocular
design of ORB-SLAM without loop closure:

* initialisation from an essential matrix or homography (model selection by the ORB-SLAM score ratio), with
  every motion hypothesis triangulated and rejected if the runner up is within 75 % of the best;
* tracking by constant velocity prediction, projection guided matching against a covisibility local map
  and robust motion only BA, with PnP RANSAC as a fallback;
* relocalization by PnP RANSAC against the 30 most recent keyframes plus up to 10 found by place recognition over the
  whole map (keyframes that observe at least 15 of the map points the frame matches);
* keyframes when tracking weakens; triangulation against covisible keyframes with epipolar, cheirality,
  reprojection and 1 degree parallax checks; local BA over the covisible window;
* map point culling as in ORB-SLAM's `MapPointCulling`: a point on probation (up to 3 keyframes old) is culled if it
  was found in under 25 % of the frames where it was visible (visibility counted once per tracked frame), or if 2 or
  fewer keyframes observe it after 2 keyframes; outlier observations are removed after BA
  (duplicate point fusion is implemented but off, because it collapsed the scale on fr3: `CHANGELOG.md`, entry 7);
* global BA at the end, then every frame's pose recomposed from its reference keyframe;
* a new map when a map is lost for 10 frames; while the new map is not initialized, each frame is also tried against
  the closed maps, and a map that relocalizes it is reopened (maps are never merged; the largest is reported);
* BA is Levenberg-Marquardt with the Schur complement and a Huber kernel, analytic Jacobians (`monoslam/ba.py`).

**Datasets.**

| Dataset | Why | Ground truth | Frames |
|---|---|---|---|
| Synthetic pool (`datasets/synthetic.py`) | Isolates the pool's conditions: an arc around a rock on tiles and pebbles | Exact (rendered) | 240 |
| Synthetic pool + moving caustics | Same scene, adding the moving light network seen on the real pool floor | Exact | 240 |
| KITTI odometry 00 to 10, left grayscale camera | The standard driving benchmark, named by Dr. Negahdaripour | GPS/INS | 23,201 |
| TUM RGB-D fr1/xyz, RGB stream | Standard monocular benchmark, easy motion | Motion capture | 798 |
| TUM RGB-D fr3/long_office_household | Standard monocular benchmark, 21 m loop | Motion capture | 2,585 |
| AQUALOC harbor 07 | Real underwater, monocular, fisheye (undistorted to pinhole) | COLMAP photogrammetry (2021 rescale) | 2,261 |

EuRoC MAV was the first choice. It could not be downloaded because its only host (ETH Research Collection)
answered every request with HTTP 429 "Rate Limited" on 2026-09-23, from both the cloud sandbox and a home connection.
The adapter (`sequences.euroc`) is written for the standard layout and is ready if the files are obtained.
TUM RGB-D was used in its place. AQUALOC was chosen over the Eiffel Tower dataset because a single harbor
sequence is 357 MB against tens of GB. **Caveat:** AQUALOC's ground truth was itself computed with COLMAP, so the
COLMAP baseline is not independent of it on that sequence.

**Evaluation** (`evaluate.py`, using `evo` 1.37.1). Estimate and ground truth are associated by timestamp and aligned
with Sim(3) Umeyama, because monocular scale is unobservable. ATE is the translation RMSE, mean, median and max after
alignment. RPE is over 1 m of ground truth travel (100 m on KITTI), between poses that far apart along the ground
truth. A run that poses fewer than 95 % of its frames is flagged PARTIAL, and its ATE covers only the posed frames; a
run that poses fewer than 3 frames is TOO FEW POSED (a result, nothing to score). For the synthetic scenes the aligned
map points are also compared with the true surface.

**Baseline** (`run_colmap.py`, pycolmap 4.2.0). COLMAP incremental SfM with SIFT and sequential matching (overlap 10,
no loop detection), given exactly the same undistorted images and fixed intrinsics (the principal point moved by half
a pixel into COLMAP's pixel convention, `CHANGELOG.md` entry 17). On the two long sequences COLMAP was run on every
2nd (AQUALOC) or 3rd (fr3) frame to keep its runtime reasonable, and on KITTI only on 04 and 07. That's stated in each
run's `run_meta.json`.

**What was not tuned.** All datasets use the same defaults in `monoslam/system.py`. No parameter was chosen
by looking at a ground truth error. `CHANGELOG.md` lists every change made after the first run, why, and the
before and after numbers.

## Limitations

* No loop closure, so drift on long runs is not corrected. On KITTI it is most of the error (scale drift), and on fr3
  it is the likely cause of the gap to COLMAP.
* Maps are not merged after a tracking loss (a closed map is only reopened when the camera relocalizes in it), so a
  run that loses track reports its largest map as primary. With ORB, 7 of the 11 KITTI sequences lose track at least
  once.
* Tracking can sit on a knife edge: on KITTI 03 with ORB the tracked inliers fall exactly to the minimum of 30 at
  frame 395, and whether the run holds there depended on small, correct code changes (`CHANGELOG.md`, after entry 19).
* On KITTI a keyframe is made on 91 to 100 % of the posed frames (the forward motion moves points out of view quickly),
  which makes the runs slow and the maps dense. It was left as is.
* Python implementation: about 0.1 to 1.3 s per frame. That's fine for validation but not real time.
* The synthetic caustics are a procedural approximation of real caustics, not a physical light simulation.
