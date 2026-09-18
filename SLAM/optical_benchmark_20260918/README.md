# Optical visual odometry on KITTI

[Results and interpretation](REPORT.md) · [Frozen protocol](PROTOCOL.md)

Camera front end for the SLAM code, assessed on the KITTI odometry benchmark
(sequences 00 to 10, the eleven with public ground truth) with the benchmark's
own metric. Built so that the same front end can be pointed at any calibrated
monocular or stereo image sequence.

- `stereo_vo.py`: ORB features with grid bucketing, block matching stereo depth
  with a right to left consistency check, PnP with RANSAC for frame to frame
  motion, refinement against both current images, and the five point
  essential matrix path for monocular input.
- `kitti_dataset.py`: calibration, timestamps, image paths and ground truth.
- `run_kitti.py`: runs both methods on every sequence in parallel, writes
  KITTI format pose files, scores with `kitti_odometry.py` from the sonar
  benchmark folder, and draws the devkit style plots.
- `test_vo.py`: synthetic checks of the motion solvers, stereo depth and the
  pose conventions, plus the real sequence 00 calibration.

## Reproduce

```sh
pip install -r requirements-optical-benchmark.txt
mkdir -p data_external/kitti_odometry && cd data_external/kitti_odometry
for f in data_odometry_gray.zip data_odometry_poses.zip data_odometry_calib.zip; do
  curl -L -O https://s3.eu-central-1.amazonaws.com/avg-kitti/$f && unzip -q -o $f
done
cd ../..
.venv/bin/python SLAM/optical_benchmark_20260918/test_vo.py
.venv/bin/python SLAM/optical_benchmark_20260918/run_kitti.py
```

The full run processes 23,201 stereo pairs at about 160 ms each; with six
worker processes it takes about 20 minutes. Results are written to `results/`.
Pose files there can be rescored with the official devkit or with
`evo_ape kitti` and `evo_rpe kitti`.
