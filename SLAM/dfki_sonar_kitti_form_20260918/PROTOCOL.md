# Protocol: public sonar recordings rescored in KITTI form

Written before the rescoring script was run.

Data: the DFKI acoustic and optical UXO dataset (Dahn et al., OCEANS 2024,
Zenodo record 13778485), ARIS Explorer 3000 recordings in a test tank with
gantry ground truth. Eight complete recordings: five used for development on
2026-09-14 and three selected by date before inspection (held out). Sample
archive MD5 and member CRCs were verified in SLAM/research.

Trajectories: the per frame sonar odometry and the loop closed keyframe
trajectories written by SLAM/research/evaluate.py on 2026-09-14 at stride 1.
Nothing is re-estimated; poses between keyframes are carried by the odometry.

Metric: the KITTI devkit segment errors at lengths {0.25, 0.5, ..., 2.0} m
(the 100 to 800 m set scaled by 1/400 because the passes are 0.7 to 2.9 m),
start step one frame, using the recording timestamps; ATE after rigid SE(2)
alignment and the stationary control as before. Rotation is the yaw of the
planar estimate against the gantry yaw.

Known model violations, reported not corrected: the sonar was tilted 22 to
60 degrees below horizontal while the estimator assumes a horizontal imaging
plane, and the physical bearing direction convention has not been confirmed
with an off axis target. All eight recordings are reported.
