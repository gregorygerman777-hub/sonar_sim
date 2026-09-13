# Planar sonar–inertial SLAM laboratory

The SLAM addition estimates horizontal X, horizontal Y and yaw. The first pose
is fixed because a relative sensor cannot determine an absolute world origin.
The synthetic IMU integrates body-frame acceleration and yaw rate; fixed bias
and random noise make its dead-reckoned trajectory drift. Forward-scan images
are rendered by the existing C++ acoustic core, and isolated local intensity
maxima become planar range-bearing features. A coarse image descriptor proposes
a previously visited scan, then mutual-nearest-neighbour ICP must produce enough
geometric inliers before that loop closure is admitted to the graph. The robust
Gauss–Newton backend jointly adjusts every non-anchor pose and reports an
approximate marginal covariance from the inverse final information matrix.

For an edge from pose i to pose j, the measured translation is expressed in
pose i's local frame:

```text
z_ij = [ R(theta_i)^T (t_j - t_i), wrap(theta_j - theta_i) ]
e_ij = predicted_relative_pose(x_i, x_j) - z_ij
```

The optimizer minimizes the sum of uncertainty-weighted edge residuals. A
Huber loss reduces the influence of a bad scan edge, but it cannot make false
loop closures harmless; descriptor proposal, distance gating and ICP verification
remain necessary.

Run the repeatable figure and the animated console from the repository root:

```bash
.venv/bin/python python/demo19_slam.py
./launch_slam.command
```

The yellow vehicle uses the estimated pose displayed on the map, and its cyan
sector is the horizontal forward-scan field of view. The surrounding cyan
ellipse shows the axis-aligned two-standard-deviation marginal position scale.
Only diagonal covariance is exposed, so the display does not show X/Y
correlation or rotate the ellipse.

The fixed 37-ping experiment reports truth only after estimation. The current
seeded result reduces position RMSE from 0.987 m to 0.149 m and endpoint closure
error from 1.154 m to 0.0015 m. Those numbers establish internal consistency on
one synthetic scene; they are not a real-ocean accuracy claim.

## Research boundary

This is smaller than RUSSO. RUSSO combines stereo vision, an IMU and imaging
sonar in a 6-DoF underwater estimator, including operation under visual
degradation. This laboratory has no stereo feature tracks, depth, roll, pitch,
IMU bias state, pressure sensor, current model, hardware synchronization,
online calibration or pool/sea data. It demonstrates the information flow and
failure mechanisms of planar sonar-aided SLAM without claiming to reproduce the
paper.

Three defensible questions are:

1. **Why is truth shown if this is SLAM?** Truth is used only to score the final
   trajectory. Feature extraction, candidate selection, ICP and optimization do
   not read it.
2. **Why does the final loop close almost perfectly?** The last ping returns to
   the exact first simulated pose, so its noiseless sonar geometry is identical.
   Real repeat visits, dynamic clutter and speckle would make the closure less
   exact and harder to verify.
3. **Is the covariance a complete navigation uncertainty?** No. It is the local
   Gaussian marginal implied by the accepted graph and declared edge noise. It
   omits model error, data-association uncertainty and correlation from shared
   sonar returns.

The first assumption likely to break on real data is reliable scan association.
Speckle, viewpoint-dependent highlight and shadow, multipath and moving clutter
can change the FSS image enough to remove true matches or create a false loop.
