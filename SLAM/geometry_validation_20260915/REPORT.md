# Bearing and geometry validation — 2026-09-15

## Status
Simulator bearing convention is verified. Three fresh geometry-compatible synthetic references were evaluated. Physical ARIS/Oculus bearing and real-data navigation accuracy remain unvalidated; the available calibration does not support that claim.

## Off-axis target experiment
Rendered an isolated 0.06 m sphere at x = -1 and +1 m, y = 4 m, z = 0 in the sensor frame. Repeated with sensor/world yaw 0 and 0.7 rad. Expected bearings were ±14.03624 degrees; observed peaks were ±13.97790 degrees. Maximum error was 0.05834 degrees, within the frozen 1-degree tolerance.

**Positive simulator bearing is sensor X-right.** No simulator sign flip is needed. This result cannot be transferred to ARIS or Oculus hardware. Hardware beam ordering needs its own independent target observation and frame calibration, not a sign chosen for the best trajectory score.

## Geometry-compatible fresh references
The protocol was written before generation/scoring, with seeds 91501, 91502 and 91503 fixed in advance. Each sequence has 81 pings along an open 270-degree horizontal arc, new sphere locations, horizontal sensor orientation, and coplanar sphere centers. Small sphere surfaces have nonzero elevation; their finite extent remains a small planar approximation. There is no exact starting-pose revisit. The unchanged sonar-only estimator sees only scans/features and timestamps, not simulator poses or an artificial IMU. All settings were retained after seeing results.

| Seed | Position ATE (m) | Stationary control (m) | Rejected pairs / 80 | Loops |
|---|---:|---:|---:|---:|
| 91501 | 0.258 | 4.597 | 0 | 0 |
| 91502 | 0.714 | 4.597 | 9 | 0 |
| 91503 | 0.858 | 4.597 | 8 | 0 |

ATE uses rigid SE(2) alignment, without scale fitting or reflection. With no accepted loops, these runs test sonar odometry rather than loop-closure effectiveness. All beat the stationary control, but the rejection counts and errors show remaining tracking limitations. These deterministic, ideal-acoustic scenes are fresh synthetic references, not fresh real measurements or validation of simulator physics. No ocean/general-dataset accuracy claim is supported.

## Physical-data admission failure
The local BlueROV dataset advertises motion-capture ground truth. Only the development line.bag topic inventory was inspected this turn; circle and L-shape images/trajectories were not opened or scored.

The published TransMatrix.yaml T_BS rotation is:

```
1  1  0
0 -1  0
0  0 -1
```

Although its determinant is +1, R-transpose times R is not identity: its Frobenius error is 1.73205. It includes shear and cannot be used as a rigid mount. The file also has duplicated T_BD keys. No typo correction or inferred mounting is applied. The separate sonar-extrinsics fixture does not have verified mounting identity with these recordings; its fitted transform cannot legitimately fill this gap. In addition, a free 3D extrinsic fit can absorb a bearing reversal through a 180-degree roll, so fitting both signs without an independently known orientation does not establish physical handedness.

The existing ARIS tilt mismatch remains unresolved. The existing SLAM/validation/geometry.py contains a 3D range/bearing model and known-map localization test; its five tests pass, but it is not an integrated unknown-map 3D SLAM pipeline. It must not be described as one.

## Implemented checks
Added admission.py to reject non-rigid, reflected and nonfinite mounting transforms instead of silently repairing them. Three admission tests passed. Existing five geometry tests passed. The simulator calibration and new reference runner are in run.py; protocol and raw results are preserved separately.

Reproduce from repository root:

```
.venv/bin/python SLAM/geometry_validation_20260915/test_admission.py
.venv/bin/python SLAM/validation/test_geometry.py
.venv/bin/python SLAM/geometry_validation_20260915/run.py
```

Recorded outputs: [results.json](results/results.json) and `results/seed_*.npz`. Reruns write to `output/geometry_validation_20260915/`.

## Required to finish physical validation
Provide an independently surveyed off-axis target recording with known sonar orientation/beam indexing and a verified sonar-to-body/mocap mounting transform for the same hardware. Establish that an unused sequence is near-horizontal with compatible reflector geometry, or integrate a 3D estimator with legitimate measured attitude/depth inputs. Freeze that adapter before opening/scoring the reserved test trajectories. Do not substitute reference attitude as an unreported sensor input.

Dataset source: https://github.com/hwgao1101/Sonar_Based_UUV_DataSet
Separate calibration source: https://github.com/hwgao1101/sonar_Extrinsics_Calibration
