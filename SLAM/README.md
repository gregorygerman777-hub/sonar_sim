# Sonar SLAM testing

## Current conclusion
The simulated sonar SLAM demonstration reproduces successfully. Simulator bearing
sign has been verified with isolated off-axis targets, and three new planar
synthetic sequences have been tested. Real-sonar accuracy remains unvalidated:
physical bearing calibration and measurement-geometry issues are unresolved.

## Reports and results

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
```

The final command generates its results under `output/geometry_validation_20260915`.
A copy of the recorded results is committed beside the latest report under `results/`.
Downloaded source datasets, local environments, and compiled extensions are excluded.
