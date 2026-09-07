#!/usr/bin/env bash
# Reproduce every result in one pass.
#
# Outputs go to results/<timestamp>/ rather than output/, so runs accumulate
# instead of overwriting each other. Each demo writes through
# python/outputs.py, which honours SONAR_OUTPUT_DIR.
#
# Stages follow the order the results are argued in: image formation first, then
# the checks against theory, then what one view cannot do and how a second sensor
# fixes it, then the raw acoustics.
set -euo pipefail

cd "$(dirname "$0")"

PYTHON="./.venv/bin/python"
[ -x "$PYTHON" ] || PYTHON="python3"

STAMP="$(date +%Y-%m-%d_%H%M%S)"
RESULTS="results/${STAMP}"
mkdir -p "$RESULTS"
export SONAR_OUTPUT_DIR="$RESULTS"

LOG="$RESULTS/run.log"
exec > >(tee "$LOG") 2>&1

echo "forward-scan sonar simulator, full reproduction"
echo "  python   $($PYTHON -V 2>&1)"
echo "  results  $RESULTS"
echo

run () {
    printf '\n----- %s -----\n' "$1"
    shift
    "$@"
}

run "build: compiled extension" $PYTHON setup.py build_ext --inplace --quiet
run "tests: analytic checks" $PYTHON tests/test_units.py

run "stage 1, image formation" true
$PYTHON python/demo1_ambiguity.py
$PYTHON python/demo2_shadow.py
$PYTHON python/demo9_target.py
$PYTHON python/demo10_motion.py
$PYTHON python/demo11_texture.py

run "stage 2, validation against theory" true
$PYTHON python/demo3_validation.py
$PYTHON python/demo4_speckle.py

run "stage 3, elevation: what one view cannot do, recovery, and its statistics" true
$PYTHON python/demo5_motion.py
$PYTHON python/demo8_optiacoustic.py
# SONAR_MC_TRIALS and SONAR_MC_CONFIDENCE override the defaults of 200 and 0.95.
$PYTHON python/demo13_montecarlo.py

run "stage 4, sea-surface multipath" true
$PYTHON python/demo6_multipath.py

run "stage 5, acoustic waveform pipeline and audio" true
$PYTHON python/demo12_chirp.py

run "stage 6, animation" true
$PYTHON python/demo7_animation.py

printf '\n----- artifacts -----\n'
ls -1 "$RESULTS"
printf '\n%s files in %s\n' "$(ls -1 "$RESULTS" | wc -l | tr -d ' ')" "$RESULTS"
