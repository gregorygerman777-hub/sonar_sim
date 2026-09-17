# KITTI-style sonar SLAM benchmark protocol

Frozen before any sequence in this folder was rendered or scored.

## Purpose
Report the simulated-sonar SLAM tests in the format and with the metrics of the
KITTI odometry benchmark (Geiger, Lenz and Urtasun, CVPR 2012), so the numbers
can be checked with the official devkit or with `evo`. This is a synthetic
benchmark with exact ground truth from the simulator. It is not a physical-sonar
accuracy claim.

## Data format
- Ground truth: `dataset/poses/SS.txt`, one line per frame, twelve values, the
  3x4 row-major pose of the sonar in the frame of the first ping. The planar
  SE(2) pose is embedded with z = 0 and zero roll/pitch.
- Timestamps: `dataset/sequences/SS/times.txt`, seconds from the first ping.
- Estimates: `results/METHOD/data/SS.txt`, same layout as the ground truth.
  Every estimate is re-expressed relative to its own first pose before export.

## Sequences
Ten sequences, `00` to `09`, each an independent random scene of small spheres
(radius 0.08 to 0.16 m, reflectivity 0.5 to 1.0) in the sonar's horizontal
plane, rendered by the unchanged C++ acoustic core at 1.2 MHz, 181 beams over a
110 degree field of view, 360 range bins to 9 m, 14 degree vertical beamwidth.
Ping period 0.4 s. The sensor is level. Sequence 00 is the 37-ping closed
circle and fixed scene of `demo19_slam.py`; sequences 01 to 03 reuse the
generator and seeds 91501 to 91503 from `SLAM/geometry_validation_20260915`, so
their raw-imagery odometry ATE must reproduce that report (0.258, 0.714,
0.858 m). In 00 to 03 the sonar looks inward at a cluster of targets. In 04 to
09 the sonar looks forward along the path over a uniform random field of
targets at 0.5 per square metre covering the surveyed area plus a 5 m margin,
none within 1 m of the path (see Amendments). Seeds, shapes, frame counts and
speeds are fixed in `sequences.py` and are not changed after seeing results.

## Imaging conditions
Both conditions use the same rendered pings and the same feature extractor.
- `compensated`: each ping is multiplied by the range gain of `imaging.py`,
  which inverts the core's per-ray 1/r^4 spreading and Thorp absorption, as
  the time-varying gain of a real forward-looking sonar does before the image
  is logged. This is the primary condition.
- `raw`: the simulator output is used directly, as in every earlier report.
  This is the ablation and the regression check against those reports.

## Methods
All methods receive the same rendered images, feature sets and timestamps.
No method reads ground truth. The estimators are the existing ones, unchanged:
1. `sonar_odometry`: sequential mutual-nearest-neighbour ICP between pings
   (`SLAM/research/evaluate.py: estimate`, stride 1), before any graph.
2. `sonar_slam`: the same run after keyframe pose-graph optimisation with
   descriptor-proposed, ICP-verified loop closures. Poses between keyframes are
   propagated with the sequential odometry relative to the preceding keyframe.
3. `imu_dead_reckoning`: the synthetic planar IMU of `slam_experiment.py`
   (fixed bias plus noise, seeded per sequence), integrated from the known
   initial pose and velocity.
4. `sonar_imu_slam`: `slam_experiment.py` fusion of the IMU odometry with scan
   matching and loop closure in the C++ robust pose graph.

## Metrics
KITTI odometry errors, ported from `evaluate_odometry.cpp`:
- For every start frame (step 1) and every segment length L in
  {2, 4, 6, 8, 10, 12, 14, 16} m of ground-truth path, take the first frame at
  least L metres further along the path, form the relative ground-truth and
  estimated motions, and compute the error motion E = inv(delta_est) * delta_gt.
- Translation error = ||t(E)|| / L, reported in percent.
- Rotation error = angle(R(E)) / L, reported in degrees per metre.
- A sequence's score is the mean over all its (start, length) segments. The
  benchmark score is the mean over all segments of all sequences. Per-length
  and per-speed averages feed the KITTI error plots.
- KITTI uses 100 to 800 m because its drives are kilometres long. These
  sequences are 20 to 50 m, so the lengths are scaled by 1/50; the definition
  of the error is otherwise identical.
Also reported for continuity with the earlier reports: absolute trajectory
error (RMSE of position after rigid SE(2) alignment, no scale, no reflection)
and the stationary control.

## Rules
- Ground truth is loaded by the scorer only, after estimation.
- All ten sequences are reported, including failures. Nothing is tuned or
  rerun after reading the results.
- The frozen SHA-256 of this file is recorded in `results/metrics.json`.

## Amendments
Recorded in the order they happened; earlier outputs are described in
REPORT.md, "Development record".
1. Run 1 rendered 04 to 09 with the sonar pointing 90 degrees left of the
   direction of travel because the shape generators produced headings while
   `slam.pose_axes` places the sonar forward axis at yaw + pi/2. This
   contradicted the protocol text and was corrected in `forward_looking`.
2. Run 2 used sparse "roadside" scenes (one sphere per metre of path within
   3.5 m of it). With raw imagery the extractor kept 4 to 8 features per ping
   and 60 to 90 percent of scan pairs were rejected, on every new sequence. The
   diagnosis (the 1/r^4 dynamic range, not the estimator) led to the two
   imaging conditions above and to the uniform target field, whose density
   matches the 18-sphere interior scenes of 01 to 03. The scene change and the
   compensated condition were decided after seeing run 2, so they are not
   blind; the raw condition and sequences 00 to 03 are unchanged.

