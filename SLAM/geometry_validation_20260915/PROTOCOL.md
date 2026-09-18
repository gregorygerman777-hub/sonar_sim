# Prospective geometry validation protocol

Frozen before generating or scoring the sequences below.

1. Independently render isolated spheres at sonar-local x = -1 and +1 m,
   y = 4 m, z = 0 m, radius 0.06 m. Verify positive bearing is x-right,
   using <=1 degree maximum discrepancy. Repeat at world yaw 0 and 0.7 rad.
   This certifies only this simulator and its adapter, not ARIS/Oculus hardware.
2. Admit a planar sequence only with independently established horizontal
   sensor motion and negligible reflector elevation. Reject a provided mounting
   matrix if its rotation is not orthonormal with determinant +1; do not repair
   it by fitting against test trajectories.
3. New synthetic tests: seeds 91501, 91502, 91503; 81 frames each; horizontal
   open arcs of 270 degrees and radius 4.8 m; 18 random small spheres in the
   same horizontal plane, generated independently per seed. No exact return.
   Use the existing sonar-only external estimator, unchanged, stride 1.
   No synthetic IMU or truth initialization; no ground truth enters estimation.
   Relative extraction threshold .025, 100 features, existing ICP/loop gates.
4. Score horizontal position after SE(2) alignment without scale/reflection,
   before/after loops; stationary control and rejected matches. Save all paths.
   Report all seeds, including failures; do not tune after reading results.
   These are fresh synthetic references, NOT external physical validation.
5. Keep BlueROV circle/L-shape recordings unopened pending independent mount,
   bearing and planar-geometry admission. Audit line.bag metadata only as needed.
6. Physical-data accuracy remains unvalidated until independent calibration is
   established and a geometry-compatible, untouched real sequence is scored.
