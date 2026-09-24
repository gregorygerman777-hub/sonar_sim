# Debug log for the monocular SLAM (Step 5)

Every change made after the first ground truth run is listed here, with the test that covers it and the
before and after numbers. Unless noted, "fr1/xyz" means TUM RGB-D freiburg1_xyz, all 798 frames, ORB
front end, ATE after Sim(3) alignment.

The rule was: no parameter is chosen by looking at ground truth error. A change is kept or dropped on
engineering grounds (a demonstrated defect, a unit test, runtime, or following the published design it
copies), decided **before** the benchmark number is known. The benchmark number is recorded here either way,
including when it got worse.

## v0: first complete run

fr1/xyz: 798/798 frames posed, 538 keyframes, 24,554 points, ATE RMSE 0.0453 m (0.57 % of 8.01 m).
**The Sim(3) scale came out at 0.00038**, meaning the map had grown about 2,600 times larger than its
initialised size (median scene depth 1). The shape was still right, which is why the ATE looked reasonable.

## 1. Monocular scale blew up in local bundle adjustment (bug, fixed)

* **Symptom:** the logged median scene depth went from 1 at initialisation to 2,470 by frame 50 of fr1/xyz.
* **Cause:** monocular BA has a 7 dof gauge (rigid pose plus scale). Early on every keyframe that observes the
  local points is inside the local window, so only keyframe 0 was held fixed. That pins the pose but not the scale.
  With Levenberg-Marquardt damping allowed to fall to 1e-12, steps along the almost flat scale direction were
  unbounded. Low parallax homography initialisation (frames 0 and 2) makes that direction flatter still.
* **Fix:** local BA now keeps at least two keyframes fixed, fixing the oldest free ones if needed. ORB-SLAM3 does the same.
  LM damping has a floor of 1e-7. After global BA the map is rescaled so the first keyframe's median depth is 1
  (a cosmetic gauge choice with no effect on the aligned error).
* **Test:** `tests/test_regressions.py::TestScaleGauge` runs the first 60 frames of fr1/xyz and requires the
  median depth to stay within 0.2 to 5. With `MONOSLAM_OLD_BEHAVIOUR=1` (one fixed keyframe, damping floor
  1e-12) it **fails** with depth 2.86e3. With the fix it **passes**. A synthetic feature version of the test
  could not reproduce the blow up (clean data, good parallax), so the regression test uses the real sequence.
* **fr1/xyz after:** ATE RMSE 0.0434 m (0.54 %), scale 1.227, 541 keyframes.

## 2. Tracking local map and keyframe rule (design, partly adopted)

* **Observation (no ground truth needed):** 538 keyframes for 798 frames of a small desk scene. The tracking
  local map was "points of the last 10 keyframes", so on revisiting an area the older points weren't
  searched, tracking counts dropped, and a new keyframe with duplicate points was created.
* **Change adopted:** the local map is now built as in ORB-SLAM. It takes the keyframes that observe the points
  tracked in the last frame (top 10 by shared points), their 3 best covisible neighbours, and the last 3 keyframes.
  `Settings.covisible_local_map=False` restores the old behaviour.
* **Change tried and not adopted:** ORB-SLAM's keyframe test counts only reference keyframe points seen by
  3 or more keyframes. On the synthetic feature unit test it cut keyframes from 7 to 5, doubled the map error
  (median point error 0.9 cm to 4.2 cm) and failed the test, so it's off (`kf_ref_min_obs=0`).
  This was decided on the unit test, before any benchmark run with it.
* **Test:** unit tests pass with either local map (end to end synthetic features: ATE under 2 cm on a 6 m arc).
* **fr1/xyz after:** ATE RMSE 0.0379 m (0.47 %), 510 keyframes, 19,673 points.

## 3. Duplicate map points were never fused (missing component, added)

* **Observation (no ground truth needed):** in the fr1/xyz map, points form streaks along viewing rays, and there
  are 24,596 points for a desk. New points were triangulated only from unmatched features, and the same
  physical point, triangulated from different keyframe pairs, was never merged. Each copy got few observations
  and a poorly constrained depth.
* **Change:** after triangulation, each new keyframe's points are projected into its 10 best covisible keyframes
  and the neighbours' points into it. A projection within the chi-square gate, with a descriptor match,
  either adds an observation or merges the two points (ORB-SLAM `SearchInNeighbors` / `Fuse`).
  `Settings.fuse_neighbors=0` disables it.
* **Test:** end to end synthetic feature test. On that test fusion changes little (the synthetic descriptors are
  unique, so duplicates are rare there). The map sanity bound was loosened from 2 to 3 cm. True points are
  about 15 cm apart, so a wrong map still fails. One seed moved from 0.95 to 2.2 cm, with keyframes going from 7 to 6.
* **fr1/xyz after:** ATE RMSE 0.0381 m (0.48 %), unchanged in effect, with about 650 keyframes. **Fusion was
  later turned off again; see entry 7.** With fusion off, the map sanity bound in the unit test is back at 2 cm.

## 4. A lost map was never replaced (robustness, added)

* **Observation:** on the pool sequence the map initialised on frames 1 and 2, lost track at frame 4, and
  stayed lost for the remaining 113 frames (3/117 posed). Nothing could restart mapping.
* **Change:** `run_slam.py` closes a map after 10 consecutive lost frames and starts a new one, as ORB-SLAM3's
  Atlas and COLMAP's multiple models do. Maps are **not** merged. The map posing the most frames is the primary
  output, and every other map is written to `maps/map_k/`. `run_meta.json` lists all maps. On a sequence that
  never loses track this changes nothing.

## 5. Runtime: keyframe count and bundle adjustment cost (performance, partly adopted)

* **Observation:** with fusion on, ORB runs made a keyframe on nearly every frame (fr1/xyz: about 650 keyframes
  for 798 frames). The final global BA then took over 10 minutes, because the reduced camera system was solved
  densely and the LM loop kept retrying tiny steps.
* **Tried and not adopted:** a keyframe rule comparing with the number of points *tracked* when the last keyframe
  was made (`kf_reference="tracked"`). On the synthetic feature unit test it cut keyframes from 6 to 5 and
  doubled the map error. This system uses only keyframe observations in BA, so fewer keyframes means less
  information. Decided on the unit test, before any benchmark run with it.
* **Adopted (no change to the solution, only to speed):** the reduced camera system is kept sparse and solved
  by sparse LU above 150 cameras (it's banded by covisibility). LM stops when an accepted step lowers the
  robust cost by less than 1e-6 of itself. Runs set single threaded BLAS so parallel runs don't oversubscribe
  the CPU. All unit tests are unchanged and pass. A 1,500 camera, 330,000 observation synthetic BA takes about
  5 s per iteration.
* **All results in `results/` were produced after this entry, with one code version** (commit in each
  `run_meta.json`). The intermediate fr1/xyz runs for entries 1 to 3 are kept outside the repository and only
  their numbers are quoted here.

## 6. fr1/xyz is several times worse than COLMAP (investigated, not resolved)

With the final code, fr1/xyz scores ATE 3.89 cm with ORB and 2.17 cm with SIFT (3.81 and 2.06 cm while fusion was on). The COLMAP baseline on the same
undistorted images and intrinsics scores 0.91 cm, and published ORB-SLAM monocular results on this sequence are around
1 cm. The plan says a result much worse than an established method is a bug until shown otherwise, so this was
investigated with diagnostic runs (kept outside the repository because they change settings; they were run while
fusion was still on, entry 7, which does not change the fr1/xyz number materially):

| Hypothesis | Test | fr1/xyz ORB ATE |
|---|---|---|
| (code at the time, fusion on) | | 3.81 cm |
| Ambiguous initialisation: the homography chosen at frames 0 and 2 had a runner up with 72 % of its points, close to the 75 % cut | Require runner up under 50 % (initialises at frames 0 and 23 instead) | 3.31 cm |
| Homography decomposition itself | Essential matrix only (`homography_ratio=1.1`) | 3.83 cm |
| Jitter of non keyframe poses | ATE on keyframes only vs all frames (v2 run) | 3.70 vs 3.79 cm |

So it isn't the initialisation, the model choice, or per frame jitter. The error is a smooth distortion present over
the whole run (per 100 frame blocks: 4.6, 4.4, 3.8, 2.2, 3.4, 3.8, 3.6, 3.7 cm). SIFT roughly halves it, and COLMAP is
flat around 0.5 to 1 cm. The most likely remaining causes are feature localisation on the motion blurred,
rolling shutter Kinect images (ORB suffers more than SIFT) and the lack of a global optimisation that uses every frame's
observations: only keyframe observations enter BA here, while COLMAP bundles all 798 images with long tracks and
also matches frames far apart in time (its "quadratic overlap" option), which acts like loop closure. Nothing in these tests pointed
to a code defect, and on the synthetic and AQUALOC sequences the same code is at the millimetre level. **This stays
an open limitation, stated in the report.** No setting was changed because of it.

## 7. Point fusion collapsed the monocular scale on fr3 (bug, fusion turned off)

* **Symptom (no ground truth needed):** in the fr3/long_office_household ORB run, the logged median scene depth
  (1 at initialisation) went 0.67 (frame 600) to 0.18 (800) to 0.07 (1000) to 0.0013 (1450). The map was
  shrinking by orders of magnitude. The camera circles a desk at a roughly constant distance, so this isn't real.
* **Diagnosis:** the same 900 frames rerun with only `fuse_neighbors` changed. Without fusion the depth is 0.36 at
  frame 700 (a real close pass) and recovers to 0.59 at frame 800. With fusion it keeps falling. Fusion also made
  a keyframe on almost every frame: fused observations raise the reference keyframe's point count, which the
  keyframe test compares against. A likely mechanism is merges of points whose depths are only loosely
  determined, which lets local BA pull structure and cameras together. It was not pinned down further.
* **Fix:** fusion is off by default (`fuse_neighbors=0`). The code stays in for anyone who wants to fix it.
  Every monoslam result in `results/` was rerun after this change. The COLMAP runs were unaffected.
* **Test:** `tests/test_regressions.py::TestFusionScaleCollapse` (runs with `RUN_SLOW=1`, about 8 minutes)
  requires the median depth at frame 900 of fr3 to stay within 0.2 to 5. With `MONOSLAM_OLD_BEHAVIOUR=1` it
  switches fusion back on.

## 8. Pool intrinsics: the raw K is now primary (configuration, 2026-09-23)

* **Reason (external, not a benchmark number):** Dr. Negahdaripour confirmed the camera matrix as
  K = [1403.5 0 476.5; 0 1403.5 392.9; 0 0 1], the K stored in `OSCalibration.mat`. `datasets/sequences.py` now has
  `POOL_PRIMARY = "raw"`, `sequences.get("pool")` returns `pool_raw`, and `pool_analysis.py` uses it as the reference.
  `width_scaled` is kept as a comparison.
* **Test:** `tests/test_pool.py` checks the stored constant, the calibration file and the default pool sequence against
  the quoted K. `test_default_pool_uses_confirmed_k` **fails** on the old default (`pool_width_scaled`) and passes now.
* **Result (no ground truth on the pool):** COLMAP exhaustive, raw K: 117/117 frames in one model, 0.92 px mean
  reprojection error, 12,133 points (width scaled: 117/117, 0.94 px, 12,117). The two trajectories agree to 0.4 % of
  extent. COLMAP sequential, raw K: 106/117 in the largest of 2 models, 0.80 px (width scaled: 110/117 in 1, 0.82 px).
* **Report fix:** REPORT.md listed the sequential gaps as frames "56 to 61 and 116"; those were zero based indices.
  The frames are opt57 to opt62 and opt117.

## 9. Pool scale: stripe widths measured in model units (tooling, scale still conditional)

* **Why:** the pool model is monocular. No sonar frames exist for the same instants (only `opt*.bmp` were delivered),
  so the sonar extrinsic cannot fix the scale. `Final_Proj` in `OSCalibration.mat` holds 28 rigid calibration poses
  (translations 1 to 4.7 m), not poses of the pool stills.
* **Change:** `pool_scale.py` fits the floor plane to the COLMAP map, rectifies every frame onto it and measures the
  dark lane stripes (width at half depth, centre spacing) in model units. Writes `results/pool_scale.json`.
* **Defect found while building it (no ground truth needed):** the first version reported two large rocks on the mat
  as stripes (about 0.28 units wide): a rock covering most of the rows a frame sees pulls the median profile down.
  Fix: `is_elongated` requires the dip in each third of the seen rows along the stripe direction.
  `tests/test_pool_scale.py::TestElongation` **fails** with the check disabled and passes with it; the end to end
  test (rendered floor, known stripes, 12 views) recovers a 0.30 wide stripe to within 3 % and the spacing to 0.02.
* **Result (primary raw K model):** two stripes, 0.454 units wide (6 detections, frames 107 to 113) and 0.436 units
  (8 detections, frames 15 to 21 and 68; range 0.33 to 0.49), centres 3.27 units apart (7.3 widths). Camera height
  above the floor: median 0.97 units. The scale is **not** fixed: metres per unit = (true stripe width) / 0.445. For
  a 0.20 to 0.30 m stripe that is 0.45 to 0.67 m per unit (camera about 0.44 to 0.66 m above the floor). The real
  stripe width, or the lane spacing, is needed from Dr. Negahdaripour.
