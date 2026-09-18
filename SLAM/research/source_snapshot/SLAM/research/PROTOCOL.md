# Sonar SLAM external-data evaluation protocol

Frozen before the new dense runs and before extracting held-out recordings.

## Questions
1. Does sonar-only odometry beat a stationary estimate on real acoustic imagery?
2. Does adding the existing loop-closure front end improve or damage trajectory accuracy?
3. How sensitive are these results to frame spacing and tilted-sonar geometry?

## Data separation
The five previously evaluated DFKI sample recordings are development data, not held out.
The first three chronological recordings in the release archive, 2023-09-20_171105,
2023-09-20_172513 and 2023-09-20_172851, are prospectively selected tests. Selection
uses archive names only, before imagery or trajectory scores are examined. These
are same-site, same-object tests, NOT independent-site or independent-object validation.
They must be complete and pass published member CRCs. No replacement based on results.

## Frozen comparisons
Use all raw frames for feature extraction. Estimate at strides 1, 5, and 20, always
including the final frame. Keep existing feature and ICP thresholds (.025, 100 peaks,
.60m gate, at least 5 inliers, residual below .22m). No parameter tuning on new scores.
Compare odometry alone with existing descriptor/ICP loop closures on at most 81
keyframes (uniform frame-index subsampling). Graph edges summarize integrated odometry;
this is an explicitly documented computational adaptation to the dense C++ solver.
Use the same evaluation keyframes for paired before/after comparisons. Also report
full-rate odometry RPE at a fixed 1 second lag and temporal-spacing quantiles.
No fabricated IMU, reference gimbal orientations, or gantry positions enter estimation.

## Metrics and controls
Report SE(2)-aligned position ATE without scale or reflection, initial-pose aligned
position error, 1-second translation and rotation RPE, path-length ratio, failures,
loop reference disagreement, and compute time. A stationary baseline is scored on the
same frames and aligned the same way. Do not interpret ICP fit residual as pose accuracy.
Bootstrap entire development sequences, not correlated frames; confidence intervals
are descriptive, with only five development and three same-object test sequences.
Audit range/bearing conventions and lever-arm composition against publisher code.
Report pitch/elevation model mismatch and gantry synchronization limitations.
Perform deterministic known-transform ICP recovery and feature-extractor equivalence
checks before evaluation. Save input hashes, code snapshots, versions, and raw trajectories.

## Scope and success claims
This evaluates the repository's planar sonar SLAM pipeline on external sonar imagery.
It does not validate acoustic simulation physics, full 3-D SLAM, or sonar–IMU fusion.
No universal pass threshold is invented after seeing results. Positive transfer requires
consistent improvement over stationary and over no-loop baselines, geometrically
credible constraints, and future independent-site replication. Negative results remain
in the report. This is a rigorous diagnostic experiment, not a finished PhD validation.
