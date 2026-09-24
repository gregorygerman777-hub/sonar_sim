# Monocular SLAM: validation against ground truth, then Dr. Negahdaripour's pool data

*Gregory German, September 2026. Everything here is reproducible with `SLAM/validation/run_all.sh`.*

## Conclusions first

1. **The implementation is correct where the answer is known exactly.** On a rendered scene resembling the pool (an arc
   around a rock on tiles and pebbles, exact ground truth), the monocular SLAM recovers the 10 m trajectory to
   **0.57 cm ATE (0.06 %) with ORB and 0.20 cm (0.02 %) with SIFT**, and its 3D map lies a **median 0.85 cm (ORB) /
   0.54 cm (SIFT) from the true surface**. COLMAP on the same images: 0.29 cm.
2. **On real underwater imagery it is accurate while it tracks, but it loses track.** AQUALOC harbor 07:
   0.83 cm ATE over the 8.6 m tracked by its main map (ORB), but that is 42 % of the frames; after tracking losses
   the run is split into 3 maps, one of them poor (all maps together 11 cm). COLMAP also splits this sequence
   (3 models, largest 43 % of frames), so part of the difficulty is the data.
3. **On the standard TUM benchmark it is 2 to 4 times less accurate than COLMAP.** fr1/xyz: 3.9 cm (ORB),
   2.2 cm (SIFT) against COLMAP 0.9 cm; fr3/long_office: 5.8 cm (ORB), 7.3 cm (SIFT) against 2.1 cm. The gap was
   investigated (initialisation, homography choice, per frame jitter) and none of those explain it; it is a smooth
   drift consistent with having no loop closure and using only keyframe observations (`CHANGELOG.md`, entry 6).
4. **Ground truth found two real bugs**, both invisible without it: a monocular scale blow up (about 2,500x) in
   local bundle adjustment, and a scale collapse caused by map point fusion. Both are fixed and covered by
   regression tests that fail on the old code (`CHANGELOG.md`, entries 1 and 7).
5. **Pool data: the trajectory and 3D model come from COLMAP, not from the sequential SLAM.** The sequential SLAM
   holds only 10 to 11 consecutive stills (frames 28 to 39) before losing track; COLMAP registers **all 117 frames
   in one model** (mean reprojection error 0.94 px) showing the camera circling the rock target about twice at
   constant height, with the 3D map concentrated on the rock and pebble patch. Where both methods pose the same
   frames their relative rotations agree to 0.5 to 1 degree. There is no ground truth, so this reconstruction is
   plausible and internally consistent, not verified.
6. **A likely cause of the tracking failures on the pool is moving caustics.** In the synthetic scene with
   moving caustics added, frame to frame tracking (ours, ORB and SIFT) and even COLMAP fail once consecutive frames are
   0.4 s and 5 degrees apart, while the same scene without caustics works at 15 degrees per frame. The real pool
   caustics are milder than the synthetic ones (COLMAP succeeds on the real frames), so this shows the mechanism,
   not its size on the real data.

## Results table

Full table (every run, RPE, runtimes, per map numbers): `results/SUMMARY.md` and `results/summary.csv`.

| Dataset | Method | Frames posed | ATE RMSE | ATE / path | Map error (median) |
|---|---|---|---|---|---|
| Synthetic pool (exact GT, 10.0 m) | ours ORB | 240/240 | 0.57 cm | 0.06 % | 0.85 cm |
| | ours SIFT | 240/240 | 0.20 cm | 0.02 % | 0.54 cm |
| | COLMAP | 240/240 | 0.29 cm | 0.03 % | 0.76 cm |
| Synthetic pool + moving caustics | ours ORB | 240/240 | 1.86 cm | 0.19 % | 2.44 cm |
| | ours SIFT | 240/240 | 0.73 cm | 0.07 % | 1.17 cm |
| | COLMAP | 240/240 | 0.48 cm | 0.05 % | 1.25 cm |
| TUM fr1/xyz (8.0 m) | ours ORB | 798/798 | 3.89 cm | 0.49 % | |
| | ours SIFT | 798/798 | 2.17 cm | 0.27 % | |
| | COLMAP | 798/798 | 0.91 cm | 0.11 % | |
| TUM fr3/long_office (22.1 m) | ours ORB | 2585/2585 | 5.77 cm | 0.26 % | |
| | ours SIFT | 2585/2585 | 7.28 cm | 0.33 % | |
| | COLMAP (1 frame in 3) | 862/862 | 2.06 cm | 0.09 % | |
| AQUALOC harbor 07 | ours ORB | 947/2261 main map (2064 in 3 maps) | 0.83 cm (11.3 cm all maps) | 0.10 % | |
| | ours SIFT | 502/2261 main map (1674 in 5 maps) | 0.54 cm (0.46 cm all maps) | 0.14 % | |
| | COLMAP (1 frame in 2) | 482/1131 largest of 3 models | 0.56 cm | 0.06 % | |
| Pool (no ground truth) | ours ORB / SIFT | 10 / 11 of 117 | n/a | | |
| | COLMAP exhaustive | 117/117 | n/a | | |

## Figures

Each dataset has three figures in `figures/<dataset>/<run>/`: `1_trajectory.png` (estimated trajectory
against ground truth, top and side views), `2_model.png` (the 3D map points), `3_overlay.png` (the
trajectory and camera frustums drawn over the 3D map), plus an interactive `3_overlay.html`.

Key figures:

| What | Figure |
|---|---|
| Synthetic pool, trajectory over the 3D map (exact ground truth) | `figures/synthetic_pool/synthetic_pool_orb/3_overlay.png` |
| Synthetic pool, 3D map alone | `figures/synthetic_pool/synthetic_pool_orb/2_model.png` |
| AQUALOC harbor 07, trajectory vs ground truth, all maps | `figures/aqualoc_harbor_07/aqualoc_harbor_07_orb/1_trajectory.png` |
| TUM fr1/xyz, ours vs COLMAP vs ground truth | `figures/tum_freiburg1_xyz/tum_freiburg1_xyz_sift/1_trajectory.png` |
| TUM fr3/long_office, trajectory | `figures/tum_freiburg3_long_office_household/tum_freiburg3_long_office_household_sift/1_trajectory.png` |
| **Pool: trajectory over the 3D model (COLMAP, all 117 frames)** | `figures/pool_width_scaled/pool_width_scaled_colmap_exhaustive/3_overlay.png` |
| **Pool: trajectory alone / 3D model alone** | `.../pool_width_scaled_colmap_exhaustive/1_trajectory.png`, `2_model.png` |
| Pool: camera positions coloured by frame number | `figures/pool/frame_order_exhaustive.png` |
| Pool: which frames each method could pose, with a floor caustic index | `figures/pool/coverage.png` |
| Synthetic: frame spacing vs moving caustics | `figures/pool/spacing_experiment.png` |

## Bugs found and fixed

Full details, tests and before/after numbers are in `CHANGELOG.md`. In short:

1. **Scale blow up in local BA (fixed).** Only one keyframe held the 7 dof monocular gauge, so the map grew
   about 2,500 times within 50 frames of fr1/xyz while the trajectory shape still looked right. Now at least two
   keyframes are fixed and LM damping has a floor. Regression test fails before, passes after.
2. **Tracking local map** now follows covisibility (as in ORB-SLAM) instead of "the last 10 keyframes".
3. **Map point fusion** was added, then **turned off** (entry 7) because it collapsed the scale on fr3
   (median depth 1 to 0.0013). Regression test (slow) fails with fusion, passes without.
4. **A lost map is replaced by a new one** instead of the run ending; maps are reported separately, never merged.
5. **Runtime:** sparse reduced camera system and an LM stopping rule (no change to results).
6. **fr1/xyz accuracy gap to COLMAP:** investigated, not resolved; stated as a limitation.

Two tried changes were **not** adopted because they failed the synthetic unit test (ORB-SLAM's keyframe rule
and a keyframe rule based on the tracked count both cut keyframes and doubled the map error); that decision was taken on the
unit test, not on benchmark numbers.

## The pool data

Inputs: `opt1.bmp` ... `opt117.bmp` (1024 x 768) and `OSCalibration.mat`. The calibration implies a
953 x 786 image, so two intrinsics hypotheses were run: `width_scaled` (fx scaled by 1024/953, cx = 512;
the hypothesis used in the September 19 report) and `raw` (K exactly as stored). Both give the same coverage.

**What each method could do** (`results/pool_analysis.json`, `figures/pool/coverage.png`):

| Method | Frames posed | Notes |
|---|---|---|
| Ours, ORB | 10 (largest map), 19 over 4 maps | loses track within a dozen frames every time it restarts |
| Ours, SIFT | 11 (largest map), 26 over 5 maps | same |
| COLMAP, sequential matching | 110 in one model | frames 56 to 61 and 116 not registered; 0.82 px mean reprojection error |
| COLMAP, exhaustive matching | **117 in one model** | 0.94 px mean reprojection error; 12,117 map points |

**Consistency checks without ground truth.**
* COLMAP sequential vs exhaustive, 110 common frames: positions agree to 3 % of the trajectory extent (RMS after
  Sim(3)), and the relative rotation between consecutive frames agrees to a median of 0.26 degrees (max 12 degrees,
  at one step).
* Ours vs COLMAP on the frames both pose: positions within 1 to 3 % of extent, relative rotations within a median of
  0.5 (ORB) to 1.0 (SIFT) degrees. (The absolute rotation after a alignment on positions only differs by about 10 degrees,
  but with only 10 nearly collinear common frames that alignment's rotation is poorly determined, which is why the
  alignment free comparison is used.)
* The reconstruction is physically plausible: the cameras move on a near circle at nearly constant height around
  the rock and pebble patch, all looking inwards, and the densest part of the map is the patch itself.

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
frames, for ORB, SIFT and COLMAP alike. On the real frames our maps only hold in frames 28 to 39, where a crude
floor caustic index is low, but other factors (irregular spacing, reflections) were not isolated. COLMAP succeeds on
the real data because it matches every frame against many others and verifies each pair geometrically before building
one global model, rather than relying on the previous frame.

**What this means for the questions in my September 22 email.**
* A multi view, global method (COLMAP's incremental SfM) handles this sequence where a better descriptor alone
  did not: SIFT inside the sequential SLAM still loses track, while COLMAP (also SIFT) registers every frame.
* The scale of the pool reconstruction is unknown (monocular). It could be fixed from a known length in the scene,
  e.g. the pool tile or lane marking width, or from the sonar to optical extrinsic in `OSCalibration.mat` if the sonar
  data for the same instants is available.

## Method (details)

**What was built.** The existing optical code (`SLAM/optical_benchmark_20260918/stereo_vo.py`) is
frame to frame visual odometry. Its monocular mode takes each step's length from ground truth. It has no map,
so it could never output a 3D model or a scale consistent trajectory (see `PIPELINE_NOTES.md`).
`monoslam/` adds a keyframe based monocular SLAM on the same ORB settings, following the monocular
design of ORB-SLAM without loop closure:

* initialisation from an essential matrix or homography (model selection by the ORB-SLAM score ratio), with
  every motion hypothesis triangulated and rejected if the runner up is within 75 % of the best;
* tracking by constant velocity prediction, projection guided matching against a covisibility local map
  and robust motion only BA; PnP RANSAC and relocalisation as fallbacks;
* keyframes when tracking weakens; triangulation against covisible keyframes with epipolar, cheirality,
  reprojection and 1 degree parallax checks; duplicate point fusion; local BA over the covisible window;
  outlier and weak point culling;
* global BA at the end, then every frame's pose recomposed from its reference keyframe;
* a new map when a map is lost for 10 frames (maps are not merged; the largest is reported);
* BA is Levenberg-Marquardt with the Schur complement and a Huber kernel, analytic Jacobians (`monoslam/ba.py`).

**Datasets.**

| Dataset | Why | Ground truth | Frames |
|---|---|---|---|
| Synthetic pool (`datasets/synthetic.py`) | Isolates the pool's conditions: an arc around a rock on tiles and pebbles | Exact (rendered) | 240 |
| Synthetic pool + moving caustics | Same scene, adding the moving light network seen on the real pool floor | Exact | 240 |
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
with Sim(3) Umeyama, because monocular scale is unobservable. ATE is the translation RMSE, mean, median and max after alignment. RPE is
per 1 m of ground truth travel. A run that poses fewer than 95 % of its frames is flagged PARTIAL, and its ATE covers
only the posed frames. For the synthetic scenes the aligned map points are also compared with the true surface.

**Baseline** (`run_colmap.py`, pycolmap 4.2.0). COLMAP incremental SfM with SIFT and sequential matching (overlap 10),
given exactly the same undistorted images and fixed intrinsics. On the two long sequences COLMAP was run on every
2nd (AQUALOC) or 3rd (fr3) frame to keep its runtime reasonable. That's stated in each run's `run_meta.json`.

**What was not tuned.** All datasets use the same defaults in `monoslam/system.py`. No parameter was chosen
by looking at a ground truth error. `CHANGELOG.md` lists every change made after the first run, why, and the
before and after numbers.

## Limitations

* No loop closure, so drift on long loops (fr3) is not corrected.
* Maps are not merged after tracking loss, so a run that loses track reports only its largest map as primary.
* Python implementation: about 0.5 to 2 s per frame. That's fine for validation but not real time.
* The synthetic caustics are a procedural approximation of real caustics, not a physical light simulation.
