"""Repeatable planar sonar-inertial SLAM experiment used by the demo and UI."""

import math

import numpy as np

import slam
import sonar


def survey_poses(count=37, radius_m=5.2, depth_m=-1.0):
    angles = np.linspace(0.0, 2.0 * np.pi, count)
    poses = np.column_stack((radius_m * np.cos(angles),
                             radius_m * np.sin(angles),
                             angles + 0.5 * np.pi))
    poses[:, 2] = np.arctan2(np.sin(poses[:, 2]), np.cos(poses[:, 2]))
    positions = np.column_stack((poses[:, :2], np.full(count, depth_m)))
    return poses, positions


def landmark_scene():
    # Unequal locations, sizes and reflectivities make the scan descriptor
    # directional enough to reject most perceptual aliases.
    specifications = [
        (-2.3, -1.7, 0.22, 0.90), (-1.5, -0.3, 0.18, 0.55),
        (-2.0, 1.6, 0.28, 0.75), (-0.8, 2.3, 0.16, 0.95),
        (0.4, 1.1, 0.25, 0.60), (1.8, 2.0, 0.20, 0.85),
        (2.4, 0.5, 0.30, 0.50), (1.1, -0.4, 0.17, 0.90),
        (2.0, -2.0, 0.24, 0.70), (0.3, -2.5, 0.19, 0.55),
        (-0.8, -1.4, 0.26, 0.80), (-2.6, 0.4, 0.15, 0.65),
    ]
    return [sonar.make_sphere((x, y, -1.0), radius, reflectivity)
            for x, y, radius, reflectivity in specifications]


def build_simulator():
    return sonar.SonarSimulator(
        frequency_hz=1.2e6,
        num_azimuth_bins=181,
        num_range_bins=360,
        horizontal_fov_deg=110.0,
        vertical_beamwidth_deg=14.0,
        max_range_m=9.0,
        num_elevation_subrays=48,
        beam_mode="top_hat",
        num_threads=1,
    )


def render_survey(simulator, objects, poses, positions):
    images = []
    features = []
    descriptors = []
    for pose, position in zip(poses, positions):
        image = simulator.render(objects, position=position, axes=slam.pose_axes(pose))
        points, strengths = slam.extract_features(image, simulator,
                                                  relative_threshold=0.025,
                                                  max_features=100,
                                                  exclusion_bins=(2, 4))
        images.append(image)
        features.append(points)
        descriptors.append(slam.scan_descriptor(image))
    return images, features, descriptors


def integrate_imu(truth, dt=0.4, seed=20260913):
    rng = np.random.default_rng(seed)
    count = len(truth)
    unwrapped_yaw = np.unwrap(truth[:, 2])
    velocity = np.gradient(truth[:, :2], dt, axis=0)
    acceleration = np.gradient(velocity, dt, axis=0)
    yaw_rate = np.gradient(unwrapped_yaw, dt)
    imu = sonar.PlanarImu(truth[0], velocity=velocity[0])
    states = [np.asarray(truth[0], dtype=float)]
    for index in range(1, count):
        yaw_mid = unwrapped_yaw[index - 1] + 0.5 * yaw_rate[index] * dt
        c, s = np.cos(yaw_mid), np.sin(yaw_mid)
        world_to_body = np.array([[c, s], [-s, c]])
        body_acceleration = world_to_body @ acceleration[index]
        # These fixed biases emulate an uncalibrated low-cost planar IMU. They
        # create drift without giving the estimator access to the true pose.
        measured_acceleration = body_acceleration + np.array([0.010, -0.006])
        measured_acceleration += rng.normal(0.0, 0.008, 2)
        measured_yaw_rate = yaw_rate[index] + math.radians(0.10)
        measured_yaw_rate += rng.normal(0.0, math.radians(0.04))
        state, _ = imu.step(measured_acceleration, measured_yaw_rate, dt)
        states.append(np.asarray(state, dtype=float))
    return np.asarray(states)


def trajectory_rmse(estimate, truth):
    return float(np.sqrt(np.mean(np.sum((estimate[:, :2] - truth[:, :2]) ** 2, axis=1))))


def run_experiment(count=37):
    simulator = build_simulator()
    objects = landmark_scene()
    truth, positions = survey_poses(count=count)
    images, features, descriptors = render_survey(simulator, objects, truth, positions)
    imu_poses = integrate_imu(truth)

    graph = sonar.PlanarSlam(initial=truth[0])
    scan_edges = []
    for index in range(1, count):
        imu_motion = sonar.relative_pose_2d(imu_poses[index - 1], imu_poses[index])
        graph.add_odometry(imu_motion, sigma_translation=0.09,
                           sigma_yaw=math.radians(1.2))

        initial = sonar.relative_pose_2d(imu_poses[index - 1], imu_poses[index])
        measured, rmse, inliers = slam.icp_relative(
            features[index - 1], features[index], initial=initial,
            max_correspondence_m=0.60)
        if inliers >= 5 and rmse < 0.22:
            graph.add_loop_closure(index - 1, index, measured,
                                   sigma_translation=max(0.025, rmse),
                                   sigma_yaw=math.radians(0.8))
            scan_edges.append((index - 1, index, rmse, inliers, "scan"))

    loop_edges = []
    for index in range(12, count):
        candidate, similarity = slam.candidate_loop(descriptors, index,
                                                    minimum_separation=12)
        if candidate is None or similarity < 0.91:
            continue
        initial = sonar.relative_pose_2d(graph.poses()[candidate], graph.poses()[index])
        if np.linalg.norm(initial[:2]) > 1.5:
            continue
        measured, rmse, inliers = slam.icp_relative(
            features[candidate], features[index], initial=initial,
            max_correspondence_m=max(0.70, float(np.linalg.norm(initial[:2])) + 0.35))
        if inliers >= 6 and rmse < 0.18:
            graph.add_loop_closure(candidate, index, measured,
                                   sigma_translation=max(0.02, rmse),
                                   sigma_yaw=math.radians(0.6))
            loop_edges.append((candidate, index, rmse, inliers, similarity))

    before = graph.poses().copy()
    graph.optimize(iterations=40, huber_delta=2.5)
    optimized = graph.poses().copy()
    uncertainty = graph.uncertainties()
    return {
        "simulator": simulator,
        "objects": objects,
        "truth": truth,
        "positions": positions,
        "images": images,
        "features": features,
        "descriptors": descriptors,
        "imu": imu_poses,
        "before": before,
        "optimized": optimized,
        "uncertainty": uncertainty,
        "scan_edges": scan_edges,
        "loop_edges": loop_edges,
        "metrics": {
            "ping_count": count,
            "feature_count": int(sum(map(len, features))),
            "scan_constraint_count": len(scan_edges),
            "loop_closure_count": len(loop_edges),
            "imu_rmse_m": trajectory_rmse(imu_poses, truth),
            "graph_before_rmse_m": trajectory_rmse(before, truth),
            "optimized_rmse_m": trajectory_rmse(optimized, truth),
            "closure_before_m": float(np.linalg.norm(before[-1, :2] - before[0, :2])),
            "closure_after_m": float(np.linalg.norm(optimized[-1, :2] - optimized[0, :2])),
        },
    }
