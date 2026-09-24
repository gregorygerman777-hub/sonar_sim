# Handoff: state of the ground truth validation (2026-09-23)

Written at the end of the Cowork session that built this folder, so work can continue in Claude Code.
The repo checkout this refers to is `~/sonar_work/sonar_sim` on Greg's Mac, with a Python 3.13 venv at
`~/sonar_work/venv` and all datasets under `~/sonar_work/data_external` (symlinked as `data_external/`, git ignored).

## Done

* Step 0: `PIPELINE_NOTES.md`. Key finding: the old monocular path was two view VO with ground truth scale, no map.
* Step 1: `monoslam/` (keyframe monocular SLAM, LM + Schur BA, TUM/PLY/meta export), `run_slam.py`, `tests/`.
* Step 2: `datasets/` (TUM RGB-D, AQUALOC harbor 07, EuRoC adapter unused because the ETH host rate limits,
  synthetic pool renderer with optional moving caustics, pool adapter with `width_scaled` / `raw` intrinsics).
* Steps 3 and 4: `evaluate.py` (evo, Sim(3), ATE, RPE, per map scores), `run_colmap.py` (pycolmap baseline).
* Step 5: `CHANGELOG.md` entries 1 to 7 (two real bugs fixed with regression tests, fr1 gap investigated).
* Steps 6 to 8: all runs in `results/`, figures in `figures/`, `REPORT.md`, `REPORT_summary.pdf` (was `REPORT_onepage.pdf`),
  `pool_analysis.py`, `spacing_experiment.py`. Everything is committed locally, **not pushed** (Greg's rule).

## Reproduce

```sh
cd ~/sonar_work/sonar_sim
export OMP_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1      # parallel runs otherwise oversubscribe the CPU
../venv/bin/python -m unittest discover -s SLAM/validation/tests          # fast tests (+ fr1 regression if data present)
RUN_SLOW=1 ../venv/bin/python -m unittest SLAM/validation/tests/test_regressions.py   # adds the 8 min fr3 test
PYTHON=../venv/bin/python SLAM/validation/run_all.sh --skip-downloads --skip-render   # everything, several hours
```
`~/sonar_work/runq.sh <jobs file> <parallelism>` runs a job list in parallel (logs in `~/sonar_work/logs/`).

## Open items, in priority order

1. **Email to Dr. Negahdaripour.** Draft: `~/sonar_work/email_draft_2026-09-23.txt` (not in the repo). Attach `REPORT_summary.pdf`. Greg's writing rule: no
   dashes or hyphens in drafted prose.
2. **Pool scale.** *Partly done 2026-09-23 (CHANGELOG entry 9): stripes measured in model units by `pool_scale.py`;
   needs the real stripe width or lane spacing from Dr. Negahdaripour to become metric. No sonar data for the same
   instants exists, so the extrinsic route is closed.* The pool reconstruction (COLMAP exhaustive, `results/pool_width_scaled_colmap_exhaustive/`) is in
   arbitrary units. Fix it from a known length (tile or lane line width) or from the sonar extrinsic in
   `OSCalibration.mat` (`Ro2s`, `To2s`) if sonar data for the same instants exists.
3. **Tracking robustness** (AQUALOC loses track, the pool frames can't be tracked). Candidates: relocalisation against
   all keyframes with a place recognition shortlist, map merging after a new map starts, a wider search after large
   motion. Evaluate on AQUALOC first (it has ground truth). Do not tune on the pool (no ground truth).
4. **Accuracy gap to COLMAP on TUM** (CHANGELOG entry 6): add non keyframe observations to BA, or loop closure.
5. **Point fusion** (CHANGELOG entry 7) is off because it collapsed the scale. If revisited, check the merge criteria
   (3D distance gate, scale level consistency like ORB-SLAM) with `TestFusionScaleCollapse`.
6. `run_all.sh` has not been run end to end in one go. The individual commands it runs have all been run.

## Rules this work followed (keep them)

* Never tune on ground truth; decide changes on engineering grounds or unit tests, then record the benchmark number
  either way in `CHANGELOG.md`.
* Every bug fix gets a test that fails before and passes after.
* Report failures and partial runs honestly (PARTIAL flag, per map numbers).
* The pool frames are unpublished: no pool photographs go in the repo (figures show reconstructions only).

## New from Dr. Negahdaripour (reply received 2026-09-23, after this work)

* He agreed with the plan ("Exactly. Good luck.") and said it is fine to **skip every other frame, or every 2 frames,
  for an initial result** (`run_slam.py --stride 2` or `--stride 3`; `run_colmap.py --stride N`).
* He gave the camera matrix directly: `K = [1403.5 0 476.5; 0 1403.5 392.9; 0 0 1]`. This is the **raw** K in
  `OSCalibration.mat` (to 4 significant figures), i.e. the `pool_raw` hypothesis, not the width scaled one used as
  primary so far.
  **Done (2026-09-23, CHANGELOG entry 8):** `pool_raw` is primary (`sequences.POOL_PRIMARY`). COLMAP exhaustive with
  the raw K: 117/117, 0.92 px (width scaled 117/117, 0.94 px; trajectories agree to 0.4 % of extent). Sequential:
  106/117 in the largest of 2 models, 0.80 px (width scaled 110/117, 0.82 px). REPORT.md, the one page PDF and the pool
  figures now use the raw K. The raw K is not visibly worse, so the (953, 786) vs 1024 x 768 question need not be raised.

## Feedback from Dr. Negahdaripour on the September 19 pairwise verification report (received 2026-09-24)

He wants (1) the implementation assessed on datasets with ground truth to find bugs, then run on his data, and
(2) the two key SLAM outputs, the estimated camera trajectory and the 3D model, shown separately and also with the
trajectory superimposed on the model. `REPORT_summary.pdf` is now laid out in exactly that order: page 1 validation
on ground truth (bugs found, accuracy table), page 2 pool trajectory and 3D model, page 3 trajectory over the model
(3D view and top view). Keep that structure in anything sent to him.

## Status 2026-09-24 (Claude Code session)

Done and committed on `main` (not pushed):
* `pool_raw` primary (entry 8); pool scale tooling (entry 9); frame skipping runs and the stride naming fix (entry 10);
  orientation metric fix (entry 11); pool outputs verified and exported as `.mat`/`.csv`/`.ply` (entry 12);
  `run_all.sh` reproducibility fixes (entry 13). `run_all.sh --only-eval` was run end to end in a clean worktree:
  exit 0, every committed number reproduced.
* `REPORT_summary.pdf` (3 pages, in the order Dr. Negahdaripour asked for). Email package outside the repo:
  `~/sonar_work/for_dr_negahdaripour/` (PDF, data files, `reprojection_check.jpg` made from his photos, which must
  stay out of the repo). Email draft: `~/sonar_work/email_draft_2026-09-23.txt`. Nothing has been sent.

In progress:
* KITTI 00 to 10 (he named KITTI; the new SLAM had never been run on it): ORB and SIFT with the code at `e292541`
  (runs record it), COLMAP on 04 and 07. Queue logs in `~/sonar_work/logs/jobs_kitti_*.log`. After they finish:
  `evaluate.py` each run, `make_figures.py`, `summarize.py`, `pool_analysis.py`, then add KITTI to REPORT.md and page 1
  of the summary PDF.
* Item 3 on branch `tracking-robustness` (worktree `~/sonar_work/dev_tracking`, commit `bb5c9f7`): place recognition
  over the whole map for relocalization, and reopening a closed map when the camera returns. Tests fail with the old
  behaviour; with both features off the pool runs are bit identical. Being scored on AQUALOC, then the pool. Merge
  into `main` only after KITTI has finished (queued KITTI jobs must keep the code version they started with), then
  rerun every run that lost track (AQUALOC, the pool, KITTI runs with several maps) and record the numbers either way.

### Update, later on 2026-09-24: code review ("fix any errors in the code") and a full rerun

Errors found and fixed, each with a test that fails on the old code (CHANGELOG entries 15 to 17 still to be written
with before and after numbers once the rerun is scored):
* entry 15 (item 3): relocalization candidates by place recognition over the whole map; reopening a closed map
  while the new one is not initialized. Alone it never fired on AQUALOC or the pool (0 relocalizations, 0 reopens).
* entry 16: `visible` was counted per search attempt instead of once per tracked frame; the found ratio test culled
  established points for ever instead of only recent ones (ORB-SLAM MapPointCulling). Also: `recent` list cleared on a
  rejected initialization; ba.py docstring.
* entry 17: COLMAP got the principal point in OpenCV's pixel convention; it uses pixel centres at +0.5 (measured).
* Also fixed on main: orientation metric on straight paths (11), RPE on KITTI (14), run_all.sh (13).
Findings not changed: OpenCV SIFT keypoints sit about +0.25 px from true (library behaviour, a quarter pixel principal
point shift for the SIFT runs); `calibration_board.py` needs `data_external/sonar_extrinsics`, absent here.

Full rerun in progress (`~/sonar_work/rerun_all.sh`, logs in `~/sonar_work/logs/jobs_rerun_*.log`): every SLAM run and
every COLMAP baseline with the merged code (`4c81dab` and later; SLAM code frozen until it ends). Pre-fix KITTI
scores are saved in `~/sonar_work/before_entry15_17/`. When it ends: `run_all.sh --skip-downloads --skip-render
--only-eval`, then CHANGELOG 15 to 17, REPORT.md (every number, plus a KITTI section), `report_summary.json` (KITTI
page), the email draft and `~/sonar_work/for_dr_negahdaripour/`.

## START HERE (next session): state at 18:10 on 2026-09-24

Greg's instruction: no errors in the code; rerun anything that failed; report results honestly (never tune on ground
truth). Nothing has been sent to Dr. Negahdaripour and nothing has been pushed.

**Done and committed on `main`** (tests: 31, all pass except the RUN_SLOW fr3 test, which passed separately):
* Code errors fixed, each with a test that fails on the old code (CHANGELOG entries still to write, numbers ready):
  15 item 3, relocalization over the whole map and reopening closed maps (`bb5c9f7`); 16 visibility counted once per
  frame and found ratio culling only for recent points (`71cfea5`); 17 COLMAP principal point + 0.5 px for COLMAP's
  pixel convention, measured (`b8e7b50`); 18 `export.git_commit` counted rewritten results as modified code
  (`55f6007`; the 48 affected run_meta.json files were corrected with a `git_dirty_note`); 19 synthetic `K.txt` was
  half a pixel off in OpenCV's convention (`0c65de5`; the K.txt files on disk were corrected, images unchanged);
  20 unscorable runs labelled TOO FEW POSED, not FAILED (`3335221`). Also `a781145`, `e9e312c` (export README states
  the pixel convention for MATLAB).
* Every SLAM run was rerun with the final code: 49 runs, all complete (`~/sonar_work/tools/audit_runs.py`).
  Before and after table: `~/sonar_work/tools/before_after_2026-09-24.md` (pre-fix KITTI metrics:
  `~/sonar_work/before_entry15_17/`). Findings to record: fr1/xyz ORB 3.89 to 1.58 cm; fr3 ORB 5.77 to 7.72 cm and SIFT
  7.28 to 5.28 cm; AQUALOC SIFT main map 502 to 758 frames (a closed map was reopened); KITTI 02 and 03 ORB now split
  in two maps. KITTI 03 was diagnosed: the refactor reproduces the old result exactly (1.2452 m); each fix alone
  causes the split; both versions fall to exactly 30 inliers (the minimum) at frame 395, the old one held one frame
  longer, i.e. a knife edge, not a code error. Item 3 fired in the rerun only in AQUALOC SIFT (1 reopen) and
  pool_width_scaled ORB (2 relocalizations).
* `calibration_board.py` (old sonar code) works once `data_external/sonar_extrinsics` is linked (done) and reproduces
  its committed results exactly.

**Still running when this was written** (background jobs of the old session; they may have stopped with it):
COLMAP reruns with the pixel convention fix, queue `~/sonar_work/rerun_colmap_b.txt` (pool_raw and
pool_width_scaled exhaustive and sequential, kitti_04, fr3 at stride 3, AQUALOC at stride 2, kitti_07), plus
tum_freiburg1_xyz. First step: `cd ~/sonar_work/sonar_sim && ../venv/bin/python ~/sonar_work/tools/audit_runs.py`.
It must print `problems: 0`; rerun whatever it lists with `~/sonar_work/runq.sh <jobs file> 1` (write job files
with bash, not zsh: zsh does not split `$var` into words), then audit again. Keep the Mac awake
(`caffeinate -i -s -t 43200`); it slept for 5 hours once and paused every job.

**Then, in order:**
1. `PYTHON=../venv/bin/python SLAM/validation/run_all.sh --skip-downloads --skip-render --only-eval` (scores
   everything, figures, summary, pool analysis, pool scale, export, PDF). It stops on the first error (`set -e`).
2. CHANGELOG entries 15 to 20 with the before and after numbers (table above; COLMAP numbers from the rerun).
3. REPORT.md: every number (conclusions, table, pool section, pool scale), a KITTI section (11 sequences, ORB and SIFT,
   COLMAP on 04 and 07; RPE over 100 m), method text (relocalization, reopening, culling, visibility), limitations
   (KITTI tracking losses and the frame 395 knife edge; no map merging; a keyframe on nearly every KITTI frame), and a
   note that K from OSCalibration.mat is used in OpenCV's pixel convention (if it came from a 1 based MATLAB toolbox the
   principal point moves by 1 px). Then check every number in REPORT.md and report_summary.json against results/.
4. `report_summary.json`: a KITTI page with `compact_table` (build_report_pdf.py supports it); rebuild the PDF.
5. Email draft `~/sonar_work/email_draft_2026-09-23.txt` (mention KITTI and the data files) and refresh
   `~/sonar_work/for_dr_negahdaripour/` (copy the new PDF and export; regenerate `reprojection_check.jpg` from the new
   `pool_reconstruction.mat` and the photos in `~/Downloads/Archive 2`, never into the repo).
6. Commit; do not push; nothing is sent without Greg.

**Tooling notes:** `~/sonar_work/runq.sh` reported rc=0 for every job until 17:48 today (fixed); jobs started with
`nohup ... &` inside a tool call do not survive (use a background tool call); the queue logs are
`~/sonar_work/logs/<jobsfile>_<n>.log`.
