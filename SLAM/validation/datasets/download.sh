#!/usr/bin/env bash
# Download the ground truth datasets into data_external/ (git ignored).
#
# EuRoC MAV (first choice in the plan) is NOT downloaded: its only official host, the ETH Research
# Collection (doi:10.3929/ethz-b-000690084), answered HTTP 429 "Rate Limited" to every request on
# 2026-09-23 and the old robotics.ethz.ch server no longer answers. The adapter sequences.euroc() is
# written for the standard ASL layout; unpack MH_01_easy / V1_01_easy into data_external/euroc/ to use it.
# TUM RGB-D (motion capture ground truth, RGB stream used as a monocular camera) is used instead.
set -euo pipefail
cd "$(dirname "$0")/../../.."
D=data_external
mkdir -p $D/tum $D/aqualoc

# TUM RGB-D: fr1/xyz (short, textured desk) and fr3/long_office_household (long loop, the standard monocular test).
for s in freiburg1/rgbd_dataset_freiburg1_xyz freiburg3/rgbd_dataset_freiburg3_long_office_household; do
  name=$(basename $s)
  if [ ! -d $D/tum/$name ]; then
    curl -L -o $D/tum/$name.tgz "https://cvg.cit.tum.de/rgbd/dataset/$s.tgz"
    tar xzf $D/tum/$name.tgz -C $D/tum
  fi
done

# AQUALOC harbor sequence 07 (raw images, fisheye calibration, COLMAP ground truth rescaled in 2021).
S='https://seafile.lirmm.fr/d/79b03788f29148ca84e5/files/?dl=1&p=/Harbor_sites_sequences'
cd $D/aqualoc
for f in harbor_calibration_files/harbor_camera_calib.yaml harbor_calibration_files/harbor_imu_camera_calib.yaml \
         harbor_groundtruth_files/new_harbor_colmap_traj_sequence_07.txt; do
  [ -f $(basename $f) ] || curl -sS -L -o $(basename $f) "$S/$f"
done
if [ ! -d raw_data/harbor_images_sequence_07 ]; then
  curl -L -o harbor_sequence_07_raw_data.tar.gz "$S/harbor_sequence_07_raw_data.tar.gz"
  tar xzf harbor_sequence_07_raw_data.tar.gz
fi
