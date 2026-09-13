"""Planar sonar front end for the C++ pose graph.

The backend estimates SE(2): horizontal x/y position and yaw. Elevation remains
inside the imaging model but is not estimated by this planar SLAM layer.
"""

import numpy as np


def pose_axes(pose):
    """Columns are sonar X-right, Y-forward and Z-up in world coordinates."""
    yaw = float(pose[2])
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def extract_features(image, simulator, relative_threshold=0.08, max_features=160,
                     exclusion_bins=(3, 5)):
    """Return isolated local intensity maxima as planar sonar points."""
    values = np.asarray(image, dtype=float)
    threshold = relative_threshold * float(values.max())
    candidates = []
    for bearing in range(1, values.shape[0] - 1):
        for range_bin in range(1, values.shape[1] - 1):
            value = values[bearing, range_bin]
            if value < threshold:
                continue
            neighborhood = values[bearing - 1:bearing + 2, range_bin - 1:range_bin + 2]
            if value >= float(neighborhood.max()):
                candidates.append((value, bearing, range_bin))
    candidates.sort(reverse=True)

    selected = []
    bearing_gap, range_gap = exclusion_bins
    for value, bearing, range_bin in candidates:
        if any(abs(bearing - old_bearing) <= bearing_gap and
               abs(range_bin - old_range) <= range_gap
               for _, old_bearing, old_range in selected):
            continue
        selected.append((value, bearing, range_bin))
        if len(selected) == max_features:
            break

    bearings = np.radians(simulator.azimuth_axis_deg())
    ranges = simulator.range_axis_m()
    points = np.empty((len(selected), 2), dtype=float)
    strengths = np.empty(len(selected), dtype=float)
    for index, (value, bearing, range_bin) in enumerate(selected):
        r, theta = ranges[range_bin], bearings[bearing]
        points[index] = (r * np.sin(theta), r * np.cos(theta))
        strengths[index] = value
    return points, strengths


def scan_descriptor(image, bearing_cells=24, range_cells=20):
    """Coarse normalized log-energy image used only to propose loop candidates."""
    values = np.log1p(np.asarray(image, dtype=float) / max(float(np.max(image)), 1e-30) * 100.0)
    bearing_edges = np.linspace(0, values.shape[0], bearing_cells + 1, dtype=int)
    range_edges = np.linspace(0, values.shape[1], range_cells + 1, dtype=int)
    descriptor = np.empty((bearing_cells, range_cells), dtype=float)
    for b in range(bearing_cells):
        for r in range(range_cells):
            cell = values[bearing_edges[b]:bearing_edges[b + 1],
                          range_edges[r]:range_edges[r + 1]]
            descriptor[b, r] = float(cell.max()) if cell.size else 0.0
    flat = descriptor.ravel()
    length = np.linalg.norm(flat)
    return flat / length if length > 0 else flat


def descriptor_similarity(first, second):
    return float(np.dot(first, second))


def transform_points(points, relative):
    points = np.asarray(points, dtype=float)
    c, s = np.cos(relative[2]), np.sin(relative[2])
    rotation = np.array([[c, -s], [s, c]])
    return points @ rotation.T + np.asarray(relative[:2])


def icp_relative(reference_points, current_points, initial=(0.0, 0.0, 0.0),
                 max_correspondence_m=0.65, iterations=30):
    """Estimate T_reference_current with mutual nearest-neighbor point ICP."""
    reference = np.asarray(reference_points, dtype=float)
    current = np.asarray(current_points, dtype=float)
    estimate = np.asarray(initial, dtype=float).copy()
    if len(reference) < 3 or len(current) < 3:
        return estimate, np.inf, 0

    inliers = 0
    for _ in range(iterations):
        moved = transform_points(current, estimate)
        distances = np.linalg.norm(moved[:, None, :] - reference[None, :, :], axis=2)
        current_to_reference = np.argmin(distances, axis=1)
        reference_to_current = np.argmin(distances, axis=0)
        current_indices = np.arange(len(current))
        mutual = reference_to_current[current_to_reference] == current_indices
        close = distances[current_indices, current_to_reference] < max_correspondence_m
        keep = mutual & close
        if np.count_nonzero(keep) < 3:
            return estimate, np.inf, int(np.count_nonzero(keep))

        source = current[keep]
        target = reference[current_to_reference[keep]]
        source_mean, target_mean = source.mean(axis=0), target.mean(axis=0)
        covariance = (source - source_mean).T @ (target - target_mean)
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        translation = target_mean - rotation @ source_mean
        updated = np.array([translation[0], translation[1],
                            np.arctan2(rotation[1, 0], rotation[0, 0])])
        delta = np.linalg.norm(updated[:2] - estimate[:2]) + abs(
            np.arctan2(np.sin(updated[2] - estimate[2]), np.cos(updated[2] - estimate[2])))
        estimate = updated
        inliers = int(np.count_nonzero(keep))
        if delta < 1e-7:
            break

    moved = transform_points(current, estimate)
    distances = np.linalg.norm(moved[:, None, :] - reference[None, :, :], axis=2)
    nearest = np.min(distances, axis=1)
    accepted = nearest < max_correspondence_m
    rmse = float(np.sqrt(np.mean(nearest[accepted] ** 2))) if np.any(accepted) else np.inf
    return estimate, rmse, inliers


def candidate_loop(descriptors, current_index, minimum_separation=12):
    """Return the most similar old scan without consulting ground truth."""
    latest_old = current_index - minimum_separation
    if latest_old < 0:
        return None, -1.0
    similarities = [descriptor_similarity(descriptors[i], descriptors[current_index])
                    for i in range(latest_old + 1)]
    index = int(np.argmax(similarities))
    return index, similarities[index]


def points_in_world(points, pose):
    return transform_points(points, pose)
