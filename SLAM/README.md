# Sonar SLAM testing

## Current conclusion
Ten simulated sequences with exact ground truth are scored in KITTI odometry
form. With range-compensated imagery, sonar-only odometry reaches 0.66 to
1.02 % translation error on the five forward-looking survey sequences at 0.6
to 0.9 m/s and fails at 1.26 m/s, where IMU-aided SLAM holds 4.1 %. Simulator
bearing sign is verified with isolated off-axis targets. Real-sonar accuracy
remains unvalidated: physical bearing calibration and measurement-geometry
issues are unresolved.

## Reports and results

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
```

`geometry_validation_20260915/run.py` writes to `output/geometry_validation_20260915`,
and a copy of its recorded results is committed beside that report under `results/`.
`kitti_benchmark_20260916/run_benchmark.py` rewrites its committed `results/` in place.
Downloaded source datasets, local environments, and compiled extensions are excluded.
