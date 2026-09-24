#!/usr/bin/env bash
# Reproduce the whole ground truth validation and the pool run.
#
#   SLAM/validation/run_all.sh                 download, render, run, evaluate, plot, report
#   SLAM/validation/run_all.sh --skip-downloads
#   SLAM/validation/run_all.sh --skip-downloads --skip-render --only-eval   (re-score and re-plot)
#
# Requirements: pip install -r SLAM/validation/requirements.txt (Python 3.10+).
# The pool frames and OSCalibration.mat are not public; set POOL_DIR and POOL_CALIB to their location.
# KITTI odometry (23 GB, see SLAM/optical_benchmark_20260918/README.md) is used when data_external/kitti_odometry
# (or KITTI_DIR) holds it, and skipped otherwise.
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
KITTI_SETS=""
if [ -d "${KITTI_DIR:-data_external/kitti_odometry/dataset}/sequences/00" ]; then
  KITTI_SETS="kitti_00 kitti_01 kitti_02 kitti_03 kitti_04 kitti_05 kitti_06 kitti_07 kitti_08 kitti_09 kitti_10"
else
  echo "KITTI odometry not found: skipping it"
fi

colmap_stride() {   # the reported COLMAP baselines use every 3rd frame of fr3 and every 2nd of AQUALOC (runtime)
  case $1 in
    tum_freiburg3_long_office_household) echo 3 ;;
    aqualoc_harbor_07) echo 2 ;;
    *) echo 1 ;;
  esac
}

if [ $ONLY_EVAL = 0 ]; then
  for d in $GT_SETS $KITTI_SETS; do
    for f in orb sift; do $PY $V/run_slam.py --dataset $d --frontend $f; done
  done
  for d in $GT_SETS; do
    $PY $V/run_colmap.py --dataset $d --matcher sequential --stride "$(colmap_stride $d)"
  done
  for d in kitti_04 kitti_07; do       # COLMAP reference on the shortest KITTI sequence and on the one with a loop
    case " $KITTI_SETS " in *" $d "*) $PY $V/run_colmap.py --dataset $d --matcher sequential ;; esac
  done
  # Frame spacing experiment on the synthetic scenes (REPORT.md, "a likely cause of the tracking failures").
  for n in 2 4 8 12; do $PY $V/run_slam.py --dataset synthetic_pool --frontend orb --stride $n; done
  for n in 2 4 8; do $PY $V/run_slam.py --dataset synthetic_pool_caustics --frontend orb --stride $n; done
  for n in 2 4; do $PY $V/run_slam.py --dataset synthetic_pool_caustics --frontend sift --stride $n; done
  $PY $V/run_colmap.py --dataset synthetic_pool_caustics --matcher exhaustive --stride 4
  # Pool: no ground truth. Both front ends, both intrinsics hypotheses, COLMAP sequential and exhaustive.
  # pool_raw (the K Dr. Negahdaripour confirmed) is primary; pool_width_scaled is kept as the comparison.
  for k in pool_raw pool_width_scaled; do
    for f in orb sift; do $PY $V/run_slam.py --dataset $k --frontend $f; done
    for m in sequential exhaustive; do $PY $V/run_colmap.py --dataset $k --matcher $m; done
  done
  for n in 2 3; do                     # every 2nd and every 3rd pool frame (CHANGELOG entry 10)
    for f in orb sift; do $PY $V/run_slam.py --dataset pool_raw --frontend $f --stride $n; done
  done
fi

for d in $GT_SETS $KITTI_SETS; do           # every run of the dataset, including the frame stride runs
  for r in $V/results/${d}_*/; do
    if [ -f "${r}run_meta.json" ]; then $PY $V/evaluate.py "${r%/}"; fi
  done
done
$PY $V/summarize.py
for d in $GT_SETS $KITTI_SETS; do
  for f in orb sift; do
    if [ -f $V/results/${d}_$f/run_meta.json ]; then
      $PY $V/make_figures.py $V/results/${d}_$f --baseline $V/results/${d}_colmap_sequential
    fi
  done
  if [ -f $V/results/${d}_colmap_sequential/run_meta.json ]; then
    $PY $V/make_figures.py $V/results/${d}_colmap_sequential
  fi
done
for r in $V/results/pool_*/; do $PY $V/make_figures.py "${r%/}"; done
$PY $V/spacing_experiment.py
$PY $V/pool_analysis.py
$PY $V/pool_scale.py
$PY $V/export_pool_deliverable.py
$PY $V/build_report_pdf.py
