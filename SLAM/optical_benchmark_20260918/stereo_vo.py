"""Stereo and monocular visual odometry front end built on OpenCV.

Conventions: a relative motion T_cur_prev is the 4x4 matrix that maps points
expressed in the previous camera frame into the current camera frame, which
is what both `cv2.solvePnP` (rvec, tvec) and `cv2.recoverPose` (R, t)
return. The trajectory is chained as T_w_cur = T_w_prev @ inv(T_cur_prev),
with the world frame equal to camera 0 at the first frame, as KITTI defines
its ground truth.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Settings:
    features: int = 4000              # ORB candidates before bucketing
    grid: tuple = (12, 4)             # bucketing cells across and down
    per_cell: int = 60                # strongest features kept per cell
    ratio: float = 0.8                # Lowe ratio for descriptor matches
    patch_half_px: int = 5            # block matching window is (2h+1) squared
    max_disparity_px: int = 128
    min_disparity_px: float = 0.5
    min_correlation: float = 0.6      # normalised cross correlation acceptance
    left_right_tolerance_px: float = 1.0
    max_depth_m: float = 80.0
    ransac_reprojection_px: float = 1.0
    ransac_iterations: int = 500
    min_inliers: int = 12
    essential_threshold_px: float = 1.0


@dataclass
class FrameFeatures:
    keypoints: np.ndarray           # (N, 2) pixel coordinates in the left image
    descriptors: np.ndarray         # (N, 32) ORB descriptors
    points3d: np.ndarray = None     # (N, 3) in the left camera frame, NaN where no stereo match
    disparity: np.ndarray = None    # (N,) left minus right column, NaN where no stereo match


def make_orb(settings):
    return cv2.ORB_create(nfeatures=settings.features, scaleFactor=1.2, nlevels=8, fastThreshold=12)


def bucket(keypoints, descriptors, shape, settings):
    """Keep the strongest features per grid cell so texture-rich regions do not dominate."""
    if not keypoints:
        return np.empty((0, 2)), np.empty((0, 32), dtype=np.uint8)
    cols, rows = settings.grid
    cell_w, cell_h = shape[1] / cols, shape[0] / rows
    order = np.argsort([-k.response for k in keypoints])
    counts = {}
    keep = []
    for index in order:
        x, y = keypoints[index].pt
        cell = (int(x // cell_w), int(y // cell_h))
        if counts.get(cell, 0) >= settings.per_cell:
            continue
        counts[cell] = counts.get(cell, 0) + 1
        keep.append(index)
    keep = np.array(keep)
    return np.array([keypoints[i].pt for i in keep], dtype=float), descriptors[keep]


def ratio_matches(matcher, desc_a, desc_b, ratio):
    """Indices (ia, ib) of matches passing Lowe's ratio test, a to b."""
    if len(desc_a) < 2 or len(desc_b) < 2:
        return np.empty((0, 2), dtype=int)
    pairs = matcher.knnMatch(desc_a, desc_b, k=2)
    out = [(m.queryIdx, m.trainIdx) for m, n in (p for p in pairs if len(p) == 2)
           if m.distance < ratio * n.distance]
    return np.array(out, dtype=int).reshape(-1, 2)


class StereoFrontEnd:
    def __init__(self, K, baseline_m, settings=None):
        self.settings = settings or Settings()
        self.K = np.asarray(K, dtype=float)
        self.baseline_m = float(baseline_m)
        self.orb = make_orb(self.settings)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def extract(self, left, right=None):
        kp_l, desc_l = self.orb.detectAndCompute(left, None)
        points_l, desc_l = bucket(kp_l, desc_l, left.shape, self.settings)
        frame = FrameFeatures(points_l, desc_l)
        if right is not None:
            frame.points3d, frame.disparity = self.stereo_depth(points_l, desc_l, right, left)
        return frame

    def stereo_depth(self, points_l, desc_l, right, left=None):
        """Metric 3D points for left features by block matching along the rectified row.

        Normalised cross correlation of a square patch over the disparity range,
        parabolic sub pixel refinement, and a right to left consistency check.
        """
        s = self.settings
        h, dmax = s.patch_half_px, s.max_disparity_px
        points3d = np.full((len(points_l), 3), np.nan)
        disparities = np.full(len(points_l), np.nan)
        if left is None or len(points_l) == 0:
            return points3d, disparities
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        height, width = left.shape
        for i, (u, v) in enumerate(points_l):
            x, y = int(round(u)), int(round(v))
            if y - h < 0 or y + h >= height or x + h >= width or x - h - 2 < 0:
                continue
            reach = min(dmax, x - h - 1)
            template = left[y - h:y + h + 1, x - h:x + h + 1]
            strip = right[y - h:y + h + 1, x - reach - h:x + h + 1]
            scores = cv2.matchTemplate(strip, template, cv2.TM_CCOEFF_NORMED).ravel()
            best = int(np.argmax(scores))
            if scores[best] < s.min_correlation:
                continue
            disparity = float(reach - best)
            if 0 < best < len(scores) - 1:
                a, b, c = scores[best - 1], scores[best], scores[best + 1]
                denominator = a - 2 * b + c
                if denominator < 0:
                    disparity -= 0.5 * (c - a) / denominator
            if disparity < s.min_disparity_px:
                continue
            # Right to left check: the right patch must match back to the left column.
            xr = x - int(round(disparity))
            if xr - h < 0:
                continue
            back_reach = min(dmax, width - 1 - h - xr)
            back_template = right[y - h:y + h + 1, xr - h:xr + h + 1]
            back_strip = left[y - h:y + h + 1, xr - h:xr + back_reach + h + 1]
            back = cv2.matchTemplate(back_strip, back_template, cv2.TM_CCOEFF_NORMED).ravel()
            if abs((xr + int(np.argmax(back))) - x) > s.left_right_tolerance_px:
                continue
            z = fx * self.baseline_m / disparity
            if z > s.max_depth_m:
                continue
            points3d[i] = ((u - cx) * z / fx, (v - cy) * z / fy, z)
            disparities[i] = disparity
        return points3d, disparities

    def temporal_matches(self, prev, cur):
        return ratio_matches(self.matcher, prev.descriptors, cur.descriptors, self.settings.ratio)

    def motion_pnp(self, prev, cur, guess=None):
        """T_cur_prev from previous 3D points and current 2D features; None if it fails."""
        s = self.settings
        matches = self.temporal_matches(prev, cur)
        if len(matches) == 0:
            return None, dict(matches=0, inliers=0)
        has_depth = np.isfinite(prev.points3d[matches[:, 0]]).all(axis=1)
        matches = matches[has_depth]
        if len(matches) < s.min_inliers:
            return None, dict(matches=int(len(matches)), inliers=0)
        objects = np.ascontiguousarray(prev.points3d[matches[:, 0]], dtype=np.float64)
        images = np.ascontiguousarray(cur.keypoints[matches[:, 1]], dtype=np.float64)
        rvec = tvec = None
        use_guess = guess is not None
        if use_guess:
            rvec, _ = cv2.Rodrigues(guess[:3, :3])
            tvec = guess[:3, 3].reshape(3, 1).copy()
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            objects, images, self.K, None, rvec=rvec, tvec=tvec, useExtrinsicGuess=use_guess,
            iterationsCount=s.ransac_iterations, reprojectionError=s.ransac_reprojection_px,
            confidence=0.999, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inliers is None or len(inliers) < s.min_inliers:
            return None, dict(matches=int(len(matches)), inliers=0 if inliers is None else int(len(inliers)))
        inliers = inliers.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(objects[inliers], images[inliers], self.K, None, rvec, tvec)
        motion = np.eye(4)
        motion[:3, :3], _ = cv2.Rodrigues(rvec)
        motion[:3, 3] = tvec.ravel()
        if cur.disparity is not None:
            motion = self.refine_stereo(motion, objects[inliers], images[inliers],
                                        cur.disparity[matches[inliers, 1]])
        return motion, dict(matches=int(len(matches)), inliers=int(len(inliers)))

    def refine_stereo(self, motion, objects, images_left, disparities):
        """Minimise reprojection error into both current images (libviso2 style), Huber weighted."""
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        has_right = np.isfinite(disparities)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        right_u = images_left[has_right, 0] - disparities[has_right]

        def residuals(params):
            R = Rotation.from_rotvec(params[:3]).as_matrix()
            moved = objects @ R.T + params[3:]
            z = np.maximum(moved[:, 2], 1e-6)
            u = fx * moved[:, 0] / z + cx
            v = fy * moved[:, 1] / z + cy
            ur = fx * (moved[has_right, 0] - self.baseline_m) / z[has_right] + cx
            return np.concatenate((u - images_left[:, 0], v - images_left[:, 1], ur - right_u))

        start = np.concatenate((Rotation.from_matrix(motion[:3, :3]).as_rotvec(), motion[:3, 3]))
        result = least_squares(residuals, start, loss="huber", f_scale=1.0, max_nfev=50)
        refined = np.eye(4)
        refined[:3, :3] = Rotation.from_rotvec(result.x[:3]).as_matrix()
        refined[:3, 3] = result.x[3:]
        return refined

    def motion_essential(self, prev, cur):
        """Rotation and unit translation direction (T_cur_prev with |t| = 1) from the 5-point solver."""
        s = self.settings
        matches = self.temporal_matches(prev, cur)
        if len(matches) < s.min_inliers:
            return None, dict(matches=int(len(matches)), inliers=0)
        a = np.ascontiguousarray(prev.keypoints[matches[:, 0]], dtype=np.float64)
        b = np.ascontiguousarray(cur.keypoints[matches[:, 1]], dtype=np.float64)
        essential, mask = cv2.findEssentialMat(a, b, self.K, method=cv2.RANSAC, prob=0.999,
                                               threshold=s.essential_threshold_px)
        if essential is None or essential.shape != (3, 3):
            return None, dict(matches=int(len(matches)), inliers=0)
        count, R, t, mask = cv2.recoverPose(essential, a, b, self.K, mask=mask)
        if count < s.min_inliers:
            return None, dict(matches=int(len(matches)), inliers=int(count))
        motion = np.eye(4)
        motion[:3, :3] = R
        motion[:3, 3] = t.ravel() / max(np.linalg.norm(t), 1e-12)
        return motion, dict(matches=int(len(matches)), inliers=int(count))


def chain(motions):
    """Absolute poses T_w_k from a list of T_k_(k-1) motions, starting at identity."""
    poses = [np.eye(4)]
    for motion in motions:
        poses.append(poses[-1] @ np.linalg.inv(motion))
    return np.array(poses)
