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

*Update 2026-09-24: after entries 15 to 19 the numbers are 1.58 cm (ORB) and 2.35 cm (SIFT) against COLMAP's 0.92 cm
(table after entry 19); the gap is smaller but still there.*

With the code at the time, fr1/xyz scored ATE 3.89 cm with ORB and 2.17 cm with SIFT (3.81 and 2.06 cm while fusion was on). The COLMAP baseline on the same
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

## 10. Frame skipping on the pool, and a run naming bug (2026-09-24)

* **Bug:** `run_slam.py --stride N` without `--tag` wrote to the same directory as the full rate run, and the first
  pool stride runs overwrote `results/pool_raw_orb` and `results/pool_raw_sift` (restored from git before anything
  used them). Fix: `output_name` adds `_stride<N>` when no tag is given.
  `tests/test_pool.py::TestRunNames` **fails** on the old code and passes now. `run_colmap.py` has the same naming
  and was left alone because the existing fr3 and AQUALOC COLMAP runs (stride 3 and 2) use the unsuffixed names;
  pass `--out` or rename by hand when running it with a stride.
* **Result (Dr. Negahdaripour suggested skipping every other frame, or every 2 frames, for an initial result):**
  ours on `pool_raw` with every 2nd frame: 2/59 posed (ORB and SIFT); every 3rd frame: 2/39 (ORB and SIFT). Each run
  initialises on its first two frames (opt1 and opt3, or opt1 and opt4), loses track at the next frame and never
  initialises a new map in the remaining frames. Skipping frames makes this sequence harder, not easier: the stills are
  already a median 2 s apart, and the synthetic experiment (`spacing_experiment.py`) predicts the same direction.

## 11. Orientation error on nearly straight paths (evaluation bug, fixed)

* **Found on ground truth (KITTI 04, first 80 frames):** ATE 5 cm over 109 m but an "orientation error" of 36 degrees,
  constant over the run. Checked without any alignment: the camera's optical axis is 1.02 degrees from its direction
  of travel (ground truth 1.04), and the relative rotation over the run is 0.51 degrees (ground truth 0.36). The
  positions are collinear to 7 mm in 6.6 units, so the Sim(3) alignment, which uses positions only, cannot fix the
  rotation about the path; the 36 degrees was that arbitrary roll, not an error of the SLAM.
* **Fix:** `evaluate.py` reports the orientation error as not determined (with the reason) when the ground truth
  positions are nearly collinear (second over first singular value below 0.05). RPE rotation, which needs no
  alignment, is unaffected. `tests/test_evaluate.py` **fails** on the old code (27 degrees reported for an exactly
  right estimate) and passes now. None of the 21 runs scored so far is affected (all ratios at least 0.1).

## 12. Pool outputs verified and exported as data (2026-09-24)

* **Reproducibility:** a fresh run from the raw frames and `OSCalibration.mat` into an empty directory gives identical
  ORB and SIFT SLAM results, the COLMAP exhaustive model to 0.17 % of extent (0.925 vs 0.923 px) and the sequential
  model to 1.8 %. The inputs were checked: exactly opt1 to opt117, no duplicates by hash, all 1024 x 768 x 3.
* **Export:** `export_pool_deliverable.py` writes the trajectory and 3D model as `.mat` (the `Final_Proj` layout),
  `.csv` and `.ply` in a floor aligned frame. `tests/test_export.py` checks that the change of frame keeps every
  projection (to 1e-6 px on random cameras); on the real export the file reproduces the original projections to 4e-12 px.
* **Finding:** about 7 % of the model points lie above the cameras; part of them come from reflections in the water
  surface and are not physical (REPORT.md, pool section). They are flagged, not removed.

## 13. run_all.sh did not reproduce the reported runs (reproducibility bugs, fixed)

* It ran COLMAP at full rate on every ground truth set, but the reported fr3 and AQUALOC baselines use every 3rd and
  every 2nd frame; `run_colmap.py` does not put the stride in the directory name, so a full `run_all.sh` would have
  replaced the reported baselines with different runs. It now passes the reported strides, and `run_colmap.py`
  refuses to replace a run made with another stride (`tests/test_run_names.py` fails on the old code).
* `for r in results/pool_*` also matched `pool_analysis.json`, so the figure step would have stopped the script.
* It never made the frame spacing runs behind `spacing_experiment.png`, nor ran `spacing_experiment.py`,
  `pool_scale.py` or the export; it now does, plus the pool frame skipping runs and KITTI when present.

## 14. RPE on KITTI came out empty (evaluation bug, fixed)

* **Found on the first KITTI run scored (COLMAP, sequence 04):** `rpe_translation_m` and `rpe_rotation_deg` were empty,
  with evo reporting that a 1 m step "produced an empty index list". KITTI frames are about 1.4 m apart, so no two
  frames are 1 m apart; the RPE per 1 m used for the other datasets is undefined there, and the failure was silent
  in the summary (n/a).
* **Fix:** `evaluate.py` measures RPE over 100 m on KITTI (the shortest segment of the KITTI odometry benchmark's own
  metric) and over 1 m elsewhere; the step is stored in `metrics.json` and shown as "RPE over [m]" in
  `results/SUMMARY.md`. `tests/test_evaluate.py::TestRelativeError` fails on the old code and passes now. No existing
  result changes (every committed run had frames closer than 1 m).

## 15. Relocalization over the whole map, and reopening a closed map (robustness, adopted)

* **Why (engineering, decided before any benchmark run):** relocalization only tried the 30 most recent keyframes, and a
  map closed after 10 lost frames was never tried again. ORB-SLAM queries its whole keyframe database (bag of words)
  when it relocalizes, and ORB-SLAM3 relocalizes into the maps it already has.
* **Change:** place recognition over every map point adds the best keyframes from anywhere in the map to the recent
  ones (`reloc_place_candidates=10`, `reloc_place_min_votes=15`). While a new map is not initialized, each frame is
  first tried against the closed maps, most recent first; a map that relocalizes it is reopened and the empty new map
  is dropped (`reopen_maps=True`, in `run_slam.track`). Maps are still never merged.
* **Test:** `tests/test_relocalization.py` (`TestPlaceRecognition`, `TestReopenMap`) **fails** with the old behaviour
  and passes now. With both features off, the pool runs are bit identical to the runs before the change.
* **Result:** in the full rerun a relocalization happened in two runs only: AQUALOC harbor 07 SIFT (a closed map was
  reopened when the camera returned) and pool_width_scaled ORB (two relocalizations). A failed attempt changes
  nothing, so in every other run this entry had no effect. Numbers in the table after entry 19.

## 16. Map point statistics: visibility counted per search attempt, culling of established points (bugs, fixed)

Two deviations from the ORB-SLAM design the code follows, found by reading the code (no ground truth involved):
* `visible` was increased on every search attempt, so a frame whose search was retried counted a point up to four
  times, and frames that were then lost counted too. Now each tracked frame counts once every searched point that falls
  in the image under its final pose (ORB-SLAM `IncreaseVisible`).
* The found ratio test (found / visible < 0.25) was applied to every point once it was 2 keyframes old, for ever, so
  established points could be culled late in a run. Now only points on probation are tested, from the next keyframe
  until they are 3 keyframes old (ORB-SLAM `MapPointCulling`). Also: the probation list is cleared when an
  initialization is rejected.
* **Test:** `tests/test_map_statistics.py` (`TestVisibility`, `TestCulling`) **fails** on the old code and passes now.
* **Result:** the table after entry 19; apart from the two runs named in entry 15, every change there comes from this
  entry (and, on the synthetic scenes, from entry 19).

## 17. COLMAP baseline: principal point half a pixel off (baseline bug, fixed)

* **Found by measurement:** COLMAP puts the centre of the top left pixel at (0.5, 0.5); OpenCV, and every calibration
  used here, at (0, 0). A blob centred on OpenCV pixel (100, 60) is found by COLMAP at (100.5, 60.5). The K was
  passed unchanged, so every COLMAP baseline had its principal point half a pixel off.
* **Fix:** `run_colmap.colmap_pinhole_params` adds 0.5 px to the principal point (recorded as `colmap_camera_params`
  in `run_meta.json`). `tests/test_colmap.py` **fails** on the old code and passes now.
* **Result:** every COLMAP run was repeated (second table after entry 19). On the real sequences the changes are small
  (TUM fr3 2.06 to 1.98 cm, KITTI 04 0.66 to 0.65 m), except that the pool's sequential model now holds 110 of the
  117 frames in one model instead of 106 in the larger of two. On the synthetic scenes nothing was expected to change:
  their `K.txt` was half a pixel off the other way (entry 19), so for COLMAP the two errors cancelled, and the
  synthetic COLMAP numbers move only by COLMAP's run to run variation. COLMAP was also run on KITTI 07 for the first
  time: 1101/1101 frames, 16.98 m ATE (2.44 % of the path), about our SIFT run (16.61 m) and worse than our ORB run
  (10.11 m on the 1060 frames of its main map); its sequential matching has no loop detection, as our SLAM has no
  loop closure.

## 18. Every rerun was recorded as made with modified code (bookkeeping bug, fixed)

* `export.git_commit` counted any tracked file differing from HEAD. Results are tracked, so a rerun that rewrote them
  was recorded as `git_dirty`, which all 48 SLAM runs of the full rerun were. Generated outputs (`results/`,
  `figures/`, the summary PDF) no longer count. `tests/test_run_names.py::TestGitDirty` **fails** on the old code.
  The 48 `run_meta.json` files were corrected, each with a `git_dirty_note` saying so.

## 19. Synthetic scenes: K.txt half a pixel off (data bug, fixed)

* The renderer casts rays through pixel centres at column + 0.5, but `K.txt` stored cx = w/2, and every consumer
  reads K in OpenCV's convention (centres at integers). So our SLAM (and, after entry 17, COLMAP) used a principal
  point half a pixel off on the synthetic scenes. `K.txt` now holds cx = w/2 - 0.5, cy = h/2 - 0.5; the images are
  unchanged and the existing `K.txt` files were corrected the same way. `tests/test_synthetic.py` **fails** on the
  old code and passes now.

**Before and after entries 15, 16 and 19** (every SLAM run was repeated once with all of them). ATE after Sim(3) on the
frames posed by the primary map; "all maps" aligns every map by its own Sim(3). "Before" is the committed run, or for
KITTI the run made before these entries and kept outside the repository (`~/sonar_work/before_entry15_17/`; only ORB
had been run, and SIFT on 05). The table is generated from the metrics files, not typed.

| Run | Before | After |
|---|---|---|
| synthetic_pool_caustics_orb | 240/240, 1.86 cm (0.19 %) | 240/240, 1.95 cm (0.19 %) |
| synthetic_pool_caustics_sift | 240/240, 0.73 cm (0.07 %) | 240/240, 0.58 cm (0.06 %) |
| synthetic_pool_orb | 240/240, 0.57 cm (0.06 %) | 240/240, 0.52 cm (0.05 %) |
| synthetic_pool_sift | 240/240, 0.20 cm (0.02 %) | 240/240, 0.23 cm (0.02 %) |
| tum_freiburg1_xyz_orb | 798/798, 3.89 cm (0.49 %) | 798/798, 1.58 cm (0.20 %) |
| tum_freiburg1_xyz_sift | 798/798, 2.17 cm (0.27 %) | 798/798, 2.35 cm (0.29 %) |
| tum_freiburg3_long_office_household_orb | 2585/2585, 5.77 cm (0.26 %) | 2585/2585, 7.72 cm (0.35 %) |
| tum_freiburg3_long_office_household_sift | 2585/2585, 7.28 cm (0.33 %) | 2585/2585, 5.28 cm (0.24 %) |
| aqualoc_harbor_07_orb | 947/2261 (3 maps), 0.83 cm (0.10 %); all maps 2064 frames, 11.31 cm | 947/2261 (3 maps), 0.93 cm (0.11 %); all maps 2066 frames, 0.81 cm |
| aqualoc_harbor_07_sift | 502/2261 (5 maps), 0.54 cm (0.14 %); all maps 1674 frames, 0.46 cm | 758/2261 (4 maps), 0.68 cm (0.08 %); all maps 1690 frames, 0.54 cm |
| kitti_00_orb | 2693/4541 (4 maps), 44.38 m (1.89 %); all maps 4511 frames, 34.34 m | 3540/4541 (3 maps), 53.85 m (1.79 %); all maps 4521 frames, 47.67 m |
| kitti_00_sift | not run | 4541/4541, 107.14 m (2.88 %) |
| kitti_01_orb | 574/1101 (5 maps), 129.99 m (9.27 %); all maps 1056 frames, 120.96 m | 574/1101 (8 maps), 173.47 m (12.37 %); all maps 1031 frames, 146.33 m |
| kitti_01_sift | not run | 478/1101 (4 maps), 7.07 m (0.63 %); all maps 1066 frames, 5.73 m |
| kitti_02_orb | 4661/4661, 32.92 m (0.65 %) | 2360/4661 (2 maps), 22.83 m (0.87 %); all maps 4651 frames, 29.02 m |
| kitti_02_sift | not run | 4661/4661, 121.51 m (2.40 %) |
| kitti_03_orb | 801/801, 1.25 m (0.22 %) | 396/801 (2 maps), 1.21 m (0.43 %); all maps 790 frames, 0.87 m |
| kitti_03_sift | not run | 801/801, 1.51 m (0.27 %) |
| kitti_04_orb | 271/271, 0.85 m (0.22 %) | 271/271, 1.05 m (0.27 %) |
| kitti_04_sift | not run | 271/271, 1.20 m (0.30 %) |
| kitti_05_orb | 2761/2761, 24.25 m (1.10 %) | 2761/2761, 24.95 m (1.13 %) |
| kitti_05_sift | 2761/2761, 51.29 m (2.33 %) | 2761/2761, 51.70 m (2.34 %) |
| kitti_06_orb | 1101/1101, 50.93 m (4.13 %) | 1101/1101, 54.83 m (4.45 %) |
| kitti_06_sift | not run | 1101/1101, 60.51 m (4.91 %) |
| kitti_07_orb | 1060/1101 (2 maps), 10.24 m (1.50 %); all maps 1091 frames, 10.09 m | 1060/1101 (2 maps), 10.11 m (1.48 %); all maps 1091 frames, 9.96 m |
| kitti_07_sift | not run | 1101/1101, 16.61 m (2.39 %) |
| kitti_08_orb | 2811/4071 (4 maps), 41.12 m (1.90 %); all maps 4040 frames, 34.40 m | 2812/4071 (4 maps), 41.34 m (1.91 %); all maps 4041 frames, 34.62 m |
| kitti_08_sift | not run | 4071/4071, 114.76 m (3.56 %) |
| kitti_09_orb | 760/1591 (4 maps), 2.27 m (0.29 %); all maps 1561 frames, 5.47 m | 760/1591 (4 maps), 2.09 m (0.27 %); all maps 1561 frames, 5.05 m |
| kitti_09_sift | not run | 1591/1591, 80.76 m (4.74 %) |
| kitti_10_orb | 1201/1201, 10.08 m (1.10 %) | 1201/1201, 10.36 m (1.13 %) |
| kitti_10_sift | not run | 1201/1201, 15.84 m (1.72 %) |
| pool_raw_orb | 11/117 (4 maps) | 12/117 (4 maps) |
| pool_raw_sift | 11/117 (6 maps) | 11/117 (6 maps) |
| pool_width_scaled_orb | 10/117 (4 maps) | 11/117 (4 maps) |
| pool_width_scaled_sift | 11/117 (5 maps) | 11/117 (5 maps) |

What the table shows, better and worse alike (the changes were kept on the grounds given in each entry, decided before
these numbers existed):
* **Better:** TUM fr1/xyz ORB 3.89 to 1.58 cm; fr3 SIFT 7.28 to 5.28 cm; AQUALOC SIFT's main map 502 to 758 frames
  (the reopened map of entry 15); AQUALOC ORB over all maps 11.31 to 0.81 cm: its third map (375 frames) went from
  26.5 to 0.89 cm, while the main map (947 frames) and the second (742) barely changed.
* **Worse:** fr3 ORB 5.77 to 7.72 cm; KITTI 02 and 03 ORB now lose track once (2 maps; 2360/4661 and 396/801 frames in
  the main map, where the old code tracked every frame); KITTI 01 ORB splits into 8 maps instead of 5.
* **KITTI 03 was traced** (diagnostic runs kept outside the repository, `~/sonar_work/diag/kitti_03_knife_edge/`): the
  refactored code with the old behaviour reproduces the old run exactly (801/801 frames, ATE 1.2452 m); with either
  fix of entry 16 alone the run splits; turning entry 15 off changes nothing. In every version the tracked inliers fall
  to 30 at frame 395, the minimum (`track_min_inliers`); the old version stayed at exactly 30 for one more frame and
  then recovered. That is a knife edge, not a code error. KITTI 02 was not traced.
* **Mixed:** KITTI 00 ORB's main map grows from 2693 to 3540 of the 4541 frames (3 maps instead of 4), and its ATE
  from 44.38 to 53.85 m over that longer stretch (1.89 to 1.79 % of it).
* The other runs change less: on KITTI by at most 0.7 m, except 06 ORB (50.93 to 54.83 m); on the other datasets by
  at most 0.2 cm. On the pool, our SLAM still poses 11 or 12 of the 117 frames in its largest map.

**Before and after entry 17** (every COLMAP baseline; the last number is the mean reprojection error):

| Run | Before | After |
|---|---|---|
| synthetic_pool_caustics_colmap_exhaustive | 2/60, FAILED, 0.200 px | 2/60, TOO FEW POSED, 0.152 px |
| synthetic_pool_caustics_colmap_sequential | 240/240 (2 models), 0.48 cm (0.05 %), 0.487 px | 240/240, 0.51 cm (0.05 %), 0.486 px |
| synthetic_pool_colmap_sequential | 240/240, 0.29 cm (0.03 %), 0.472 px | 240/240, 0.29 cm (0.03 %), 0.473 px |
| tum_freiburg1_xyz_colmap_sequential | 798/798, 0.91 cm (0.11 %) | 798/798, 0.92 cm (0.11 %), 0.848 px |
| tum_freiburg3_long_office_household_colmap_sequential | 862/862, 2.06 cm (0.09 %), 0.747 px | 862/862, 1.98 cm (0.09 %), 0.747 px |
| aqualoc_harbor_07_colmap_sequential | 482/1131 (3 models), 0.56 cm (0.06 %), 0.285 px | 482/1131 (3 models), 0.54 cm (0.06 %), 0.285 px |
| kitti_04_colmap_sequential | 271/271, 0.66 m (0.17 %), 0.320 px | 271/271, 0.65 m (0.16 %), 0.320 px |
| kitti_07_colmap_sequential | not run | 1101/1101, 16.98 m (2.44 %), 0.359 px |
| pool_raw_colmap_exhaustive | 117/117, 0.923 px | 117/117, 0.924 px |
| pool_raw_colmap_sequential | 106/117 (2 models), 0.797 px | 110/117, 0.807 px |
| pool_width_scaled_colmap_exhaustive | 117/117, 0.936 px | 117/117, 0.940 px |
| pool_width_scaled_colmap_sequential | 110/117, 0.821 px | 110/117, 0.819 px |

## 20. Runs that pose too few frames were labelled FAILED (reporting fix)

* A run in which the method posed fewer than 3 frames (for example COLMAP on the caustic scene at 1 frame in 4, 2/60)
  has nothing to score. That is a result of the experiment, not a failed job or script, but `evaluate.py` labelled it
  FAILED. It is now TOO FEW POSED, with the reason in `metrics.json`, and `results/SUMMARY.md` explains the label.
  `tests/test_evaluate.py::TestTooFewFrames` **fails** on the old code and passes now.

## 21. A rerun kept files of the earlier run; the commit was read when a run ended (bookkeeping bugs, fixed)

* **Stale maps:** `run_slam.py` wrote into an existing run directory without clearing it. After a rerun with fewer
  maps, the earlier run's `maps/map_<k>` stayed, and `evaluate.py` and the figures use every `maps/map_*`. Found in
  `aqualoc_harbor_07_sift` (maps 3 and 4 of the 2026-09-23 run) and `kitti_00_orb` (a map 2 of an earlier run),
  before any number from them was committed or reported. Fix: `clear_previous_run` removes exactly what a run and its
  evaluation write, and nothing else. Both runs were repeated with the fix: every output is bit identical to the run
  before, minus the stale maps.
* **Recorded commit:** the commit was read when the run ended, but the code a run executes is what it imported when
  it started. `kitti_08_sift` ran from 17:44 to 18:04 and recorded a commit made at 18:02, and as `git_dirty`, from an
  unrelated edit of 18:04. Its SLAM code was the final one throughout, but the record was wrong. The commit is now
  read at the start. `kitti_08_sift` was repeated: bit identical, recorded at the right commit and clean.
* Also: `run_slam.main` opened `log.txt` and never closed it; each line is now appended and closed.
* **Tests:** `tests/test_run_names.py::TestRerunReplacesPreviousRun` (a rerun with fewer maps, a rerun with no map,
  the commit at the start) **fails** on the old code and passes now. `~/sonar_work/tools/audit_runs.py` (outside the
  repository) now also checks every run's maps against its `run_meta.json`, that its own log ends with its `done:`
  line, that its trajectory has as many poses as it reports, and that the SLAM code at its recorded commit equals the
  current code.

## 22. RPE pairs were chosen along the estimate, not along the ground truth (evaluation bug, fixed)

* **Found by an independent check of a KITTI number:** KITTI 00 SIFT scored an RPE over 100 m of 121.8 m, more than
  the segment itself. Recomputing it directly from the aligned trajectories gave 51 m. `evaluate.py` documents the
  RPE as over 1 m (100 m on KITTI) of ground truth travel, as the KITTI benchmark defines its segments, but evo
  chooses the pose pairs along the estimate unless `pairs_from_reference` is set. Where the monocular scale had
  drifted, a "100 m" segment spanned a different true length.
* **Fix:** both RPE metrics set `pairs_from_reference=True`. `tests/test_evaluate.py::TestRelativeError::
  test_segments_are_measured_along_the_ground_truth` (an estimate whose second half runs at half scale) **fails** on
  the old code (3.68 m against 3.04 m on ground truth segments) and passes now.
* **Result** (the same trajectories scored both ways; ATE is not affected): on KITTI the RPE over 100 m changes by
  58 % down to 23 % up (KITTI 00 SIFT 121.8 to 50.7 m, KITTI 10 SIFT 12.7 to 15.5 m). The RPE over 1 m of the other
  datasets changes by at most 6 %, except TUM fr1/xyz SIFT (4.19 to 3.24 cm). No conclusion in REPORT.md rested on an
  RPE number.

## 23. Smaller reporting and tooling fixes

* `summarize.py` listed a SLAM run that never initialized a map with 1 map (the caustic scene at 1 frame in 4 and 8);
  it now counts the maps, or COLMAP models, the run recorded. `tests/test_summary.py` fails on the old code.
* The summary PDF's compact table took the posed path of one method as the sequence length (KITTI 01 would have read
  1403 m, ORB's 574 posed frames). `evaluate.py` now records the ground truth length of the whole run
  (`run_path_length_m`) and the table uses it. It also labelled a run with nothing to score "failed", and the full
  table said "no ground truth" for a ground truth run that posed too few frames; both now say which it is.
  `tests/test_evaluate.py::TestRunLength` and `tests/test_report_pdf.py` fail on the old code.
* Reading the empty trajectory of a run that posed no frame printed a numpy "input contained no data" warning in every
  scoring log; the result was right, and the warning is gone. `tests/test_monoslam.py` covers it.
* The interactive `3_overlay.html` wrote every map point (KITTI maps have up to 413,000; files of up to 20 MB). It now
  shows a fixed random subset of at most 100,000 points and says so; the PNG figures still draw every point.
  `tests/test_figures.py` covers it.
