# Pairwise geometric verification and multi-view consistency, real pool imagery

[Full report (PDF)](REPORT.pdf)

Assessment of the camera front end (`SLAM/optical_benchmark_20260918/stereo_vo.py`)
on a 117-frame monocular sequence and camera calibration (`OSCalibration.mat`)
supplied by Dr. Negahdaripour. Unlike the KITTI and DFKI ARIS assessments, this
sequence has no ground-truth trajectory, so the method is an internal
consistency check rather than a scored benchmark: every one of the
C(117,2) = 6,786 possible frame pairs (not only the 116 adjacent ones) is
independently verified by essential-matrix RANSAC, and the redundancy this
creates is used to test whether the pairwise measurements agree with one
another.

## Headline result

A naive frame-to-frame chain drifts to an unphysical 500+ degrees of net
rotation. Multi-view rotation averaging over the full pairwise graph, cross
validated against an independent spectral estimator and checked for
non-uniqueness by refitting from six different starting points, finds:

- **34 of 117 frames** are internally consistent and stably determined
  regardless of solver initialization.
- **46 of 117 frames** are internally consistent with each other but have an
  unresolved orientation relative to the first 34, traced to a single graph
  edge (connecting frames 60 and 82) that is the only one of 1,564 candidate
  cross-block measurements to clear the report's normal admission bar, and
  that (being a graph bridge) cannot be checked by any amount of
  cycle-consistency filtering, however thorough.
- **37 of 117 frames** fail the initial pairwise reliability filter outright.
- Translation direction, tested separately within the certified 34-frame
  block, is internally inconsistent by a median of 40.6 degrees, about 40
  times the rotation residual, consistent with the well-known relative
  fragility of translation-direction estimates from two-view geometry
  compared to rotation.

Scene-planarity degeneracy and calibration uncertainty were tested directly
and ruled out as explanations for any of the above. Full derivations,
falsification tests (including a real implementation defect that was found,
localized to a hand-checkable 3-node example, and fixed before being
trusted), and all figures are in the [report](REPORT.pdf).

## Contents

- `REPORT.pdf`: the full write-up (16 pages).
- `figures/`: the report's figures, regenerated from the pickled intermediate
  results. One figure from the full report (Fig. 5, a case study showing two
  of the source photographs directly with matched keypoints drawn) is
  omitted here, since it displays unpublished frames from the supplied
  dataset; it is available in the non-public copy of the report.
- `scripts/`: the analysis pipeline, in the order it runs. These are a
  faithful record of what was executed in the assessment, not a
  productionized package: paths are hardcoded to the session's working
  directory and would need adjusting to rerun. The two required inputs
  (the 117-frame image sequence and `OSCalibration.mat`) are not included
  here, since they are unpublished data supplied for this assessment, not
  part of this repository's own datasets.

## Pipeline

1. `extract_features.py`: ORB features + per-frame image statistics for all
   117 frames (cached; independent of camera calibration).
2. `pairwise_verify.py`: full pairwise essential-matrix verification, all
   6,786 pairs, in parallel, plus a homography-vs-essential model-selection
   score per pair (planarity test).
3. `analyze_graph.py`: matchability decay, loop-closure candidate search,
   failure-mode correlation with image statistics, calibration sensitivity.
4. `rotation_averaging.py`: nonlinear (Huber-loss) multi-view rotation
   averaging over the verified graph, with a synthetic ground-truth
   validation before being trusted on real data.
5. `threshold_sweep.py`: self-consistency vs. graph-coverage trade-off across
   inlier thresholds.
6. `triplet_consistency.py`, `triplet_prune_and_refit.py`: three-view
   (N>2) cycle-consistency check and iterative pruning of edges that fail
   every closed triangle they participate in.
7. `spectral_sync.py`, `validate_spectral_sync.py`: an independent spectral
   rotation-synchronization estimator (Singer 2011; Arie-Nachimson et al.
   2012), including the falsification test that found and fixed a real
   defect in the rotation-extraction step, and the graph's algebraic
   connectivity (Fiedler value) as a well-posedness certificate.
8. `multi_init_robustness.py`: refits the nonlinear solution from six
   different spanning-tree initializations to test whether the "trusted
   core" reconstruction is actually unique.
9. `translation_sync.py`: linear translation-direction synchronization
   (Govindu 2001; Jiang, Cui and Tan 2013) within the certified-stable
   34-frame block, with its own synthetic rank-deficiency validation.
10. `vanishing_point_calib.py`: an attempted (inconclusive) independent
    focal-length check from the tiled floor's vanishing points.
11. `make_figures_*.py`, `make_figure_10.py`, `make_case_study.py`,
    `matlab_style.py`: figure generation.
12. `build_pdf.py`: assembles `REPORT.pdf` (set `REPORT_PUBLIC=1` to build
    the redacted copy used in this repository).

## Reproducing

Requires the 117-frame sequence and `OSCalibration.mat` (not included; obtain
from Dr. Negahdaripour), the repository's own `.venv`, plus `networkx`,
`reportlab` and `pymupdf` (not in `requirements.txt`, installed ad hoc for
this assessment). Update the hardcoded data and output paths at the top of
each script, then run in the order listed above.
