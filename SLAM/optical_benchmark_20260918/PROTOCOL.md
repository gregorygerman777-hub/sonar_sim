# Optical visual odometry assessment on KITTI: protocol

Frozen before any KITTI image was processed.

## Purpose
Dr. Negahdaripour asked that the code be assessed on available optical
datasets with ground truth before it is applied to his optical data. The
existing pipeline is sonar only, so this folder adds a camera front end
(features, stereo depth, robust relative pose) that feeds the same
trajectory export and KITTI scoring used for the sonar benchmark. The
assessment is on the KITTI odometry benchmark (Geiger, Lenz and Urtasun,
CVPR 2012), the reference dataset for this task, using its own metric,
lengths and start step, so the numbers are directly comparable to the
published leaderboard.

## Data
KITTI odometry, grayscale stereo (`data_odometry_gray.zip`, MD5 recorded in
`results/metrics.json`), sequences 00 to 10, which are the eleven with
public ground truth: 23,201 frames at 10 Hz, 0.4 to 5.1 km each. Camera 0
(left) and camera 1 (right), rectified, baseline from `calib.txt`. Ground
truth poses are read by the scorer only, after estimation. Sequences 11 to
21 have no public ground truth and are not used.

## Methods
1. `stereo_vo`: ORB features on the left image, spread over a grid; stereo
   correspondences found by descriptor matching on the same rectified row,
   giving metric depth from the baseline; temporal correspondences to the
   next left image by descriptor matching with a ratio test; relative motion
   by PnP with RANSAC on the previous frame's 3D points and the current
   frame's 2D features, refined on the inliers; a constant velocity guess
   initialises RANSAC. Frame to frame, no keyframes, no bundle adjustment,
   no loop closure. This is a visual odometry baseline, not a SLAM system.
2. `mono_vo_gt_scale`: the same features on the left image only; relative
   rotation and translation direction from the five point essential matrix
   with RANSAC; the translation magnitude of each step is taken from ground
   truth, as is standard for monocular KITTI evaluation, and this is stated
   wherever the numbers appear. It isolates rotation and heading accuracy
   from the scale problem that monocular vision cannot solve alone.

Parameters (features per frame, ratio, RANSAC thresholds) are fixed in
`stereo_vo.py` after a check on the first 300 frames of sequence 08 only
(the first sequence readable from the archive while it was still
downloading), before any other sequence is processed, and are then applied
unchanged to all eleven sequences. Nothing is tuned after reading full
sequence scores.

## Metrics
The KITTI devkit metric exactly as published: segment lengths 100 to 800 m,
start step 10 frames, translation error in percent and rotation error in
deg/m averaged over all segments of a sequence and over all sequences.
Also reported: absolute trajectory error (RMSE after SE(3) alignment) and
per length and per speed curves. Pose files are written in KITTI format
under `results/results/METHOD/data/SS.txt` so the official devkit or `evo`
can rescore them.

## Rules
All eleven sequences are reported, including failures. The frozen SHA 256
of this file is recorded in `results/metrics.json`.
