# Sonar SLAM testing

## Current conclusion
The camera front end has been assessed on real optical data with ground truth:
on KITTI odometry it reaches 1.0 to 2.1 percent translation error on ten of
eleven sequences as frame to frame stereo visual odometry, and fails only on
the highway sequence above 16 m/s. It is ready to run on an external optical
dataset given calibration. On the sonar side, ten simulated sequences with
exact ground truth are scored in KITTI odometry form. With range-compensated imagery, sonar-only odometry reaches 0.66 to
1.02 % translation error on the five forward-looking survey sequences at 0.6
to 0.9 m/s and fails at 1.26 m/s, where IMU-aided SLAM holds 4.1 %. Simulator
bearing sign is verified with isolated off-axis targets. Real-sonar accuracy
remains unvalidated: physical bearing calibration and measurement-geometry
issues are unresolved.

## Reports and results

- [Pairwise verification and multi-view consistency on real pool imagery](pool_consistency_20260919/README.md):
  the camera front end assessed on Dr. Negahdaripour's own 117-frame sequence
  (no ground truth available), using full pairwise geometric verification,
  multi-view rotation averaging cross-validated by an independent spectral
  estimator, and a translation-direction consistency check. Finds 34 of 117
  frames stably determined, 46 with an unresolved orientation traced to a
  single graph edge, and 37 failing pairwise verification outright.
  [Report](pool_consistency_20260919/REPORT.pdf).
- [Visual odometry on the KITTI odometry benchmark](optical_benchmark_20260918/REPORT.md):
  the camera front end (stereo and monocular) assessed on sequences 00 to 10
  with the devkit metric, beside published ORB SLAM2 and Stereo LSD-SLAM
  numbers. [Frozen protocol](optical_benchmark_20260918/PROTOCOL.md),
  [scores](optical_benchmark_20260918/results/metrics.json).
- [Sonar odometry on the public DFKI ARIS recordings, KITTI form](dfki_sonar_kitti_form_20260918/REPORT.md):
  the 2026-09-14 real-data trajectories rescored with the devkit metric at
  0.25 to 2 m; not validated, mirrored heading on the 22 degree passes and a
  planar model against 42 and 60 degree tilt. [Protocol](dfki_sonar_kitti_form_20260918/PROTOCOL.md),
  [scores](dfki_sonar_kitti_form_20260918/results/metrics.json).
- [KITTI-style benchmark, ten sequences, four estimators](kitti_benchmark_20260916/REPORT.md):
  KITTI pose files and metrics, the range-compensation finding, and the fusion
  weighting diagnosis. [Frozen protocol](kitti_benchmark_20260916/PROTOCOL.md),
  [scores](kitti_benchmark_20260916/results/metrics.json).
- [Latest bearing and geometry validation](geometry_validation_20260915/REPORT.md):
  off-axis target checks, fresh synthetic reference tests, and the physical-data
  calibration blocker. [Measured results](geometry_validation_20260915/results/results.json).
- [Reproduction of simulation and external benchmark](verification_20260915/REPORT.md):
  12 checks passed; eight real sequences at three frame spacings rerun.
- [External sonar research evaluation](research/README.md): protocol, source,
  trajectory plots, metrics, and limitations of the DFKI ARIS adapter.

The new synthetic tests achieved aligned position RMSE of 0.258, 0.714 and 0.858 m.
They accepted no loop closures and therefore test sonar odometry, not loop-closure
performance. These results are not physical-sonar accuracy claims.

## Reproduction

Build the repository's Python/C++ extension using the root README instructions.
External evaluation additionally uses `requirements-public-benchmark.txt`.
From the repository root:

```sh
python tests/test_slam.py
python tests/test_public_uxo.py
python SLAM/research/test_evaluation.py
python SLAM/validation/test_geometry.py
python SLAM/geometry_validation_20260915/test_admission.py
python SLAM/geometry_validation_20260915/run.py
python SLAM/kitti_benchmark_20260916/test_kitti.py
python SLAM/kitti_benchmark_20260916/test_benchmark.py
python SLAM/kitti_benchmark_20260916/run_benchmark.py
python SLAM/optical_benchmark_20260918/test_vo.py
python SLAM/optical_benchmark_20260918/run_kitti.py --workers 8
python SLAM/dfki_sonar_kitti_form_20260918/score_dfki.py
```

The optical run needs the KITTI odometry archives in `data_external/kitti_odometry`
(download commands in [optical_benchmark_20260918/README.md](optical_benchmark_20260918/README.md))
and `requirements-optical-benchmark.txt`.

`geometry_validation_20260915/run.py` writes to `output/geometry_validation_20260915`,
and a copy of its recorded results is committed beside that report under `results/`.
`kitti_benchmark_20260916/run_benchmark.py` rewrites its committed `results/` in place.
Downloaded source datasets, local environments, and compiled extensions are excluded.
