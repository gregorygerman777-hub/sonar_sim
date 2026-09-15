# Sonar SLAM verification — 2026-09-15

## Conclusion
The synthetic demonstration reproduces successfully. External real-sonar results remain inconsistent and do not establish readiness for an arbitrary new dataset.

## What was executed
- Six simulator/SLAM tests, four evaluation tests, and two public-data scoring tests: all 12 passed.
- The 37-ping simulated sonar/IMU experiment.
- Eight complete existing DFKI ARIS sequences at strides 1, 5, and 20: 24 configurations. These are reruns of previously evaluated data, not fresh held-out evidence.
- Existing Python/C++ implementation; MATLAB was not used. No estimator source changes were made.

Synthetic position RMSE: 0.986644 m before correction, 0.148894 m after correction. Endpoint gap: 1.153826 m to 0.001543 m. The known initial pose/velocity initialize the synthetic experiment; subsequent truth generates simulated observations and scores estimates. Exact return to the starting pose gives identical sonar geometry and an unusually easy loop closure.

## Fresh all-frame results
ATE is horizontal position RMSE after proper rigid SE(2) alignment with no scale fitting, evaluated at up to 81 graph keyframes. Stationary is the constant-position control after alignment. RPE below is pre-loop odometry translation error over approximately one second.

| Sequence | Split | Odometry ATE m | With loops ATE m | Stationary ATE m | 1 s RPE m |
|---|---|---:|---:|---:|---:|
| 2023-09-20_180722 | development | 2.888 | 2.888 | 0.762 | 1.771 |
| 2023-09-21_100927 | development | 1.277 | 1.345 | 0.735 | 0.913 |
| 2023-09-21_105856 | development | 0.105 | 0.149 | 0.213 | 0.032 |
| 2023-09-21_123500 | development | 1.391 | 1.368 | 0.758 | 1.208 |
| 2023-09-21_124735 | development | 1.687 | 1.689 | 0.581 | 1.307 |
| 2023-09-20_171105 | heldout | 0.336 | 0.345 | 0.728 | 0.100 |
| 2023-09-20_172513 | heldout | 0.602 | 0.591 | 0.760 | 0.251 |
| 2023-09-20_172851 | heldout | 0.487 | 0.416 | 0.729 | 0.094 |

## Interpretation and limitations
- Ground-truth reference data are loaded after external trajectory estimation; the metadata-whitelist test passed.
- Four of five development sequences score worse than the stationary control. All three previously held-out sequences score better, but they share the same site/object and are already consumed test data.
- The existing geometry audit flags unresolved physical bearing handedness and sonar tilt of 22–60 degrees, incompatible with a simple horizontal-plane interpretation. Those issues were not resolved by rerunning the estimator. Treat external errors as conditional baseline results.
- Loop closure can worsen error; acceptance alone does not establish a correct match.
- These tests do not validate acoustic simulator physics or real sonar/IMU fusion.

## Next step
Resolve bearing direction using a known off-axis target, then use a geometry-compatible sequence or implement a suitable tilted/3D measurement model. Validate on fresh reference data before claiming accuracy on the user's dataset.

Detailed prior audit: ../research/REPORT.md
Fresh raw metrics and trajectories: development/ and heldout/.
Synthetic plot: demo19_slam.png. Synthetic metrics: demo19_slam_metrics.json.
