"""KITTI odometry benchmark: pose-file format and error metrics.

A numpy port of the reference devkit (`evaluate_odometry.cpp`, Geiger, Lenz and
Urtasun, CVPR 2012). Poses are 4x4 homogeneous matrices mapping sensor
coordinates into the coordinate frame of the first frame; a pose file stores the
first three rows of each matrix, row-major, twelve values per line.

The only deliberate departures from the devkit are parameters: the segment
lengths and the start-frame step are arguments, and segment speed uses the real
timestamps instead of the devkit's fixed 0.1 s frame period.
"""

import numpy as np

KITTI_LENGTHS_M = (100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0)
KITTI_STEP_FRAMES = 10


# --- SE(2) embedding and file format -------------------------------------

def se2_to_matrix(pose):
    """Embed a planar (x, y, yaw) pose as a 4x4 matrix with z = 0.

    Columns of the rotation are sonar X-right, Y-forward and Z-up expressed in
    world coordinates, matching `slam.pose_axes`.
    """
    x, y, yaw = (float(v) for v in pose)
    c, s = np.cos(yaw), np.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    matrix[:3, 3] = [x, y, 0.0]
    return matrix


def se2_trajectory_to_matrices(poses):
    return np.array([se2_to_matrix(pose) for pose in np.asarray(poses, dtype=float)])


def matrix_to_se2(matrix):
    return np.array([matrix[0, 3], matrix[1, 3], np.arctan2(matrix[1, 0], matrix[0, 0])])


def relative_to_first(matrices):
    """Re-express every pose in the frame of the first one, as KITTI requires."""
    matrices = np.asarray(matrices, dtype=float)
    first_inverse = np.linalg.inv(matrices[0])
    return np.einsum("ij,njk->nik", first_inverse, matrices)


def write_poses(path, matrices):
    rows = np.asarray(matrices, dtype=float)[:, :3, :].reshape(len(matrices), 12)
    np.savetxt(path, rows, fmt="%.9e")


def read_poses(path):
    rows = np.loadtxt(path, dtype=float, ndmin=2)
    if rows.shape[1] != 12:
        raise ValueError(f"{path}: expected 12 values per line, found {rows.shape[1]}")
    matrices = np.tile(np.eye(4), (len(rows), 1, 1))
    matrices[:, :3, :] = rows.reshape(len(rows), 3, 4)
    return matrices


def write_times(path, times):
    np.savetxt(path, np.asarray(times, dtype=float), fmt="%.6e")


# --- devkit metric ---------------------------------------------------------

def trajectory_distances(poses):
    """Cumulative Euclidean path length at every frame (devkit trajectoryDistances)."""
    positions = np.asarray(poses, dtype=float)[:, :3, 3]
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(steps)))


def last_frame_from_segment_length(distances, first_frame, length):
    """First frame more than `length` metres of path beyond `first_frame`, else -1."""
    target = distances[first_frame] + length
    later = np.nonzero(distances[first_frame:] > target)[0]
    return int(first_frame + later[0]) if len(later) else -1


def rotation_error(pose_error):
    trace = pose_error[0, 0] + pose_error[1, 1] + pose_error[2, 2]
    return float(np.arccos(np.clip(0.5 * (trace - 1.0), -1.0, 1.0)))


def translation_error(pose_error):
    return float(np.linalg.norm(pose_error[:3, 3]))


def sequence_errors(poses_gt, poses_result, lengths=KITTI_LENGTHS_M,
                    step=KITTI_STEP_FRAMES, times=None):
    """Segment errors for one sequence (devkit calcSequenceErrors).

    Returns a list of dicts with first_frame, last_frame, length (m), r_err in
    radians per metre, t_err as a fraction per metre, and speed in m/s.
    """
    poses_gt = np.asarray(poses_gt, dtype=float)
    poses_result = np.asarray(poses_result, dtype=float)
    if poses_gt.shape != poses_result.shape:
        raise ValueError("ground truth and result must have the same number of poses")
    distances = trajectory_distances(poses_gt)
    if times is None:
        times = 0.1 * np.arange(len(poses_gt))
    errors = []
    for first_frame in range(0, len(poses_gt), step):
        for length in lengths:
            last_frame = last_frame_from_segment_length(distances, first_frame, length)
            if last_frame == -1:
                continue
            delta_gt = np.linalg.inv(poses_gt[first_frame]) @ poses_gt[last_frame]
            delta_result = np.linalg.inv(poses_result[first_frame]) @ poses_result[last_frame]
            pose_error = np.linalg.inv(delta_result) @ delta_gt
            elapsed = float(times[last_frame] - times[first_frame])
            errors.append(dict(
                first_frame=first_frame, last_frame=last_frame, length=float(length),
                r_err=rotation_error(pose_error) / length,
                t_err=translation_error(pose_error) / length,
                speed=float(length / elapsed) if elapsed > 0 else float("nan"),
                num_frames=last_frame - first_frame + 1,
            ))
    return errors


def average_errors(errors):
    """Mean translation (%) and rotation (deg/m) over segments (devkit saveStats)."""
    if not errors:
        return dict(segments=0, t_err_percent=float("nan"), r_err_deg_per_m=float("nan"))
    t_err = np.mean([e["t_err"] for e in errors])
    r_err = np.mean([e["r_err"] for e in errors])
    return dict(segments=len(errors), t_err_percent=float(100.0 * t_err),
                r_err_deg_per_m=float(np.degrees(r_err)))


def errors_by_length(errors, lengths, minimum_segments=3):
    """Per-length averages for the KITTI error-versus-path-length plots."""
    rows = []
    for length in lengths:
        subset = [e for e in errors if e["length"] == length]
        if len(subset) < minimum_segments:
            continue
        rows.append(dict(length=float(length), **average_errors(subset)))
    return rows


def errors_by_speed(errors, bin_width=0.25, minimum_segments=3):
    """Per-speed-bin averages for the KITTI error-versus-speed plots."""
    if not errors:
        return []
    speeds = np.array([e["speed"] for e in errors])
    rows = []
    for centre in np.arange(bin_width, np.nanmax(speeds) + bin_width, bin_width):
        subset = [e for e, s in zip(errors, speeds) if abs(s - centre) < 0.5 * bin_width]
        if len(subset) < minimum_segments:
            continue
        rows.append(dict(speed=float(centre), **average_errors(subset)))
    return rows


# --- complementary absolute error, kept from the earlier reports ------------

def aligned_ate(estimate_xy, truth_xy):
    """RMSE after proper rigid SE(2) alignment: no scale, no reflection."""
    estimate_xy = np.asarray(estimate_xy, dtype=float)
    truth_xy = np.asarray(truth_xy, dtype=float)
    a = estimate_xy - estimate_xy.mean(axis=0)
    b = truth_xy - truth_xy.mean(axis=0)
    u, _, vt = np.linalg.svd(a.T @ b)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    aligned = a @ rotation.T + truth_xy.mean(axis=0)
    residual = np.linalg.norm(aligned - truth_xy, axis=1)
    return aligned, float(np.sqrt(np.mean(residual ** 2)))


def aligned_ate_se3(estimate_poses, truth_poses):
    """Position RMSE after rigid SE(3) alignment of two 4x4 pose arrays: no scale, no reflection."""
    estimate = np.asarray(estimate_poses, dtype=float)[:, :3, 3]
    truth = np.asarray(truth_poses, dtype=float)[:, :3, 3]
    a = estimate - estimate.mean(axis=0)
    b = truth - truth.mean(axis=0)
    u, _, vt = np.linalg.svd(a.T @ b)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    aligned = a @ rotation.T + truth.mean(axis=0)
    residual = np.linalg.norm(aligned - truth, axis=1)
    return aligned, float(np.sqrt(np.mean(residual ** 2)))
