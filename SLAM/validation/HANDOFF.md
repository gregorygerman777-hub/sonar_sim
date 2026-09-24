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
