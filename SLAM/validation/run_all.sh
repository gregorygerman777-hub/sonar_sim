#!/usr/bin/env bash
# Reproduce the whole ground truth validation and the pool run.
#
#   SLAM/validation/run_all.sh                 download, render, run, evaluate, plot, report
#   SLAM/validation/run_all.sh --skip-downloads
#   SLAM/validation/run_all.sh --skip-downloads --skip-render --only-eval   (re-score and re-plot)
#
# Requirements: pip install -r SLAM/validation/requirements.txt (Python 3.10+).
# The pool frames and OSCalibration.mat are not public; set POOL_DIR and POOL_CALIB to their location.
set -euo pipefail
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
V=SLAM/validation
SKIP_DL=0; SKIP_RENDER=0; ONLY_EVAL=0
for a in "$@"; do
  case $a in
    --skip-downloads) SKIP_DL=1 ;;
    --skip-render) SKIP_RENDER=1 ;;
    --only-eval) ONLY_EVAL=1 ;;
  esac
done

[ $SKIP_DL = 1 ] || bash $V/datasets/download.sh
if [ $SKIP_RENDER = 0 ]; then
  $PY $V/datasets/synthetic.py --name synthetic_pool
  $PY $V/datasets/synthetic.py --name synthetic_pool_caustics --caustics
fi

GT_SETS="synthetic_pool synthetic_pool_caustics tum_freiburg1_xyz tum_freiburg3_long_office_household aqualoc_harbor_07"
if [ $ONLY_EVAL = 0 ]; then
  for d in $GT_SETS; do
    for f in orb sift; do $PY $V/run_slam.py --dataset $d --frontend $f; done
    $PY $V/run_colmap.py --dataset $d --matcher sequential
  done
  # Pool: no ground truth. Both front ends, both intrinsics hypotheses, COLMAP sequential and exhaustive.
  for k in pool_width_scaled pool_raw; do
    for f in orb sift; do $PY $V/run_slam.py --dataset $k --frontend $f; done
  done
  $PY $V/run_colmap.py --dataset pool_width_scaled --matcher sequential
  $PY $V/run_colmap.py --dataset pool_width_scaled --matcher exhaustive
fi

for d in $GT_SETS; do
  for r in $V/results/${d}_*; do $PY $V/evaluate.py "$r"; done
done
$PY $V/summarize.py
for d in $GT_SETS; do
  for f in orb sift; do
    $PY $V/make_figures.py $V/results/${d}_$f --baseline $V/results/${d}_colmap_sequential
  done
  $PY $V/make_figures.py $V/results/${d}_colmap_sequential
done
for r in $V/results/pool_*; do $PY $V/make_figures.py "$r"; done
$PY $V/pool_analysis.py
$PY $V/build_report_pdf.py
