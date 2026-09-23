"""Keyframe based monocular SLAM: initialization, tracking against a map, keyframes,
triangulation, local and global bundle adjustment. No loop closure.

The design follows the monocular part of ORB-SLAM (Mur-Artal et al. 2015) in simplified form:

* Initialization: essential matrix and homography are both fitted; the homography is used
  when it explains the matches clearly better (planar scene, R_H > 0.45). All motion hypotheses
  are triangulated and a hypothesis is only accepted if it is clearly better than the runner up.
  Scale is fixed by setting the median scene depth of the first keyframe to 1.
* Tracking: constant velocity prediction, then projection guided search of local map points and
  robust motion only BA. If that fails, brute force descriptor matching plus PnP RANSAC. If that
  fails, relocalization against recent keyframes.
* Mapping: new keyframe when tracking weakens; new points triangulated with covisible keyframes
  under epipolar, cheirality, reprojection and parallax checks; local BA over the covisible window;
  outlier observations and weakly supported new points are culled.
* Finish: global BA over all keyframes, then every frame's pose is recomposed from its reference
  keyframe. Frames that were never tracked get one localization attempt against the final map
  and are flagged as such.

Poses are world to camera internally (see geometry.py). The world frame is the first keyframe.
"""

import time
from dataclasses import dataclass, asdict, field

import cv2
import numpy as np
from scipy.spatial import cKDTree

from . import ba
from . import features as feat
from . import geometry as geo

CHI2 = ba.CHI2_2DOF_95


@dataclass
class Settings:
    frontend: str = "orb"
    n_features: int = 2000
    grid: tuple = (8, 6)
    # initialization
    init_ratio: float = 0.9
    init_min_matches: int = 100
    init_min_points: int = 50           # points with parallax above init_min_parallax_deg
    init_min_parallax_deg: float = 1.0
    init_ambiguity: float = 0.75        # runner up hypothesis must have < this fraction of the best's points
    init_max_gap: int = 30              # frames before the reference frame is replaced
    homography_ratio: float = 0.45      # R_H above which the homography is decomposed instead of E
    # tracking
    search_radius_px: float = 15.0
    search_radius_wide_px: float = 40.0
    match_ratio: float = 0.9
    track_min_inliers: int = 30
    pnp_reprojection_px: float = 4.0
    reloc_min_inliers: int = 50
    reloc_candidates: int = 30
    new_map_after_lost: int = 10        # run_slam.py starts a new map after this many consecutive lost frames
    # keyframes and mapping
    kf_track_ratio: float = 0.9
    kf_ref_min_obs: int = 0             # 3 = ORB-SLAM rule; tried and not adopted (CHANGELOG 2)
    kf_reference: str = "points"        # "points" (every point of the last keyframe) or "tracked"; see CHANGELOG 5
    covisible_local_map: bool = True    # False restores the pre CHANGELOG-2 local map (last N keyframes)
    kf_max_gap: int = 20
    triang_neighbors: int = 6
    fuse_neighbors: int = 10            # 0 disables duplicate point fusion (pre CHANGELOG-3 behaviour)
    fuse_max_rel_distance: float = 0.1  # merge only points closer than this fraction of their depth
    triang_min_parallax_deg: float = 1.0
    local_window: int = 10
    cull_after_kfs: int = 2             # a new point seen by <= 2 keyframes after this many new keyframes is culled
    min_found_ratio: float = 0.25
    local_ba_iterations: int = 20
    local_ba_min_fixed: int = 2         # monocular gauge: fix at least two keyframes in local BA
    global_ba_iterations: int = 50


class MapPoint:
    __slots__ = ("id", "X", "desc", "obs", "first_kf", "color", "bad", "visible", "found")

    def __init__(self, pid, X, desc, first_kf, color):
        self.id, self.X, self.desc, self.first_kf, self.color = pid, X, desc, first_kf, color
        self.obs = {}
        self.bad = False
        self.visible = 1
        self.found = 1


class KeyFrame:
    def __init__(self, kid, frame, R, t, features, color_image):
        self.id, self.frame, self.R, self.t, self.f = kid, frame, R, t, features
        self.pt = np.full(len(features.uv), -1, dtype=int)
        self.color_image = color_image


@dataclass
class FrameRecord:
    index: int
    timestamp: float
    status: str = "untracked"       # tracked | keyframe | relocalized | localized_after | untracked
    ref_kf: int = -1
    R_rel: np.ndarray = None        # T_frame_kf, so that T_frame_world = T_rel o T_kf_world
    t_rel: np.ndarray = None
    inliers: int = 0


class MonoSLAM:
    def __init__(self, K, image_size, settings=None, log=print):
        self.s = settings or Settings()
        self.K = np.asarray(K, float)
        self.width, self.height = image_size
        self.extract = feat.Extractor(self.s.frontend, self.s.n_features, self.s.grid)
        self.kind = self.s.frontend
        self.log = log
        self.points = {}
        self.kfs = []
        self.frames = []
        self.next_pid = 0
        self.initialized = False
        self.init_ref = None            # (frame index, features, color image)
        self.last = None                # (R, t, features, pt_ids) of the last tracked frame
        self.velocity = None            # T_last_lastlast
        self.lost_features = {}         # features of frames that were not tracked, for the final pass
        self.stats = dict(local_ba_seconds=0.0, n_local_ba=0, reinit=0, relocalizations=0)

    # ---------------------------------------------------------------- utilities
    def _add_point(self, X, desc, kf, color):
        p = MapPoint(self.next_pid, X, desc, kf.id, color)
        self.points[p.id] = p
        self.next_pid += 1
        return p

    def _observe(self, p, kf, kp):
        """Record that keypoint kp of keyframe kf observes point p. A point has at most one keypoint per keyframe."""
        if kf.id in p.obs:
            return False
        if kf.pt[kp] >= 0 and kf.pt[kp] != p.id:
            return False
        p.obs[kf.id] = kp
        kf.pt[kp] = p.id
        p.desc = kf.f.desc[kp]
        return True

    def _erase_observation(self, p, kid):
        kf = self.kfs[kid]
        kp = p.obs.pop(kid, None)
        if kp is not None and kf.pt[kp] == p.id:
            kf.pt[kp] = -1
        if len(p.obs) < 2:
            self._set_bad(p)

    def _set_bad(self, p):
        p.bad = True
        for kid, kp in list(p.obs.items()):
            kf = self.kfs[kid]
            if kf.pt[kp] == p.id:
                kf.pt[kp] = -1
        p.obs.clear()

    def _color(self, image, uv):
        if image is None:
            return np.full((len(uv), 3), 160, np.uint8)
        x = np.clip(np.round(uv[:, 0]).astype(int), 0, image.shape[1] - 1)
        y = np.clip(np.round(uv[:, 1]).astype(int), 0, image.shape[0] - 1)
        c = image[y, x]
        if c.ndim == 1:
            c = np.repeat(c[:, None], 3, axis=1)
        return c[:, ::-1].astype(np.uint8)  # BGR to RGB

    def _local_point_ids(self, n_kfs):
        """Local map for tracking (as in ORB-SLAM): keyframes that observe the points tracked in the last
        frame, ranked by how many they share, plus their covisible neighbours and the most recent keyframes."""
        counts = {}
        if self.s.covisible_local_map and self.last is not None:
            for pid in self.last[3]:
                p = self.points.get(int(pid))
                if p is None or p.bad:
                    continue
                for kid in p.obs:
                    counts[kid] = counts.get(kid, 0) + 1
        chosen = [k for k, _ in sorted(counts.items(), key=lambda kv: -kv[1])[:n_kfs]]
        if self.s.covisible_local_map:
            for k in list(chosen[: n_kfs // 2]):
                chosen += [kf.id for kf in self._covisible(self.kfs[k], 3)]
        chosen += [kf.id for kf in self.kfs[-3 if self.s.covisible_local_map else -n_kfs:]]
        ids = set()
        for k in set(chosen):
            kf = self.kfs[k]
            ids.update(int(i) for i in kf.pt[kf.pt >= 0])
        return [i for i in ids if not self.points[i].bad]

    # ---------------------------------------------------------------- matching
    def _search_by_projection(self, f, R, t, pids, radius, tree=None):
        """Match map points to features of frame f by projecting with pose (R,t). Returns (pids, kps)."""
        if not pids:
            return np.empty(0, int), np.empty(0, int)
        X = np.array([self.points[i].X for i in pids])
        uv, z = geo.project(self.K, R, t, X)
        ok = (z > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < self.width) & (uv[:, 1] >= 0) & (uv[:, 1] < self.height)
        if not ok.any():
            return np.empty(0, int), np.empty(0, int)
        tree = tree or cKDTree(f.uv)
        idx = np.flatnonzero(ok)
        for i in idx:
            self.points[pids[i]].visible += 1
        neighbours = tree.query_ball_point(uv[idx], r=radius)
        best_for_kp = {}
        thresh = feat.LOOSE[self.kind]
        for i, cand in zip(idx, neighbours):
            if not cand:
                continue
            cand = np.asarray(cand)
            d = feat.paired_distance(np.repeat(self.points[pids[i]].desc[None], len(cand), 0), f.desc[cand], self.kind)
            order = np.argsort(d)
            best = d[order[0]]
            if best > thresh:
                continue
            if len(order) > 1 and best > self.s.match_ratio * d[order[1]]:
                continue
            kp = int(cand[order[0]])
            if kp not in best_for_kp or best < best_for_kp[kp][1]:
                best_for_kp[kp] = (pids[i], best)
        if not best_for_kp:
            return np.empty(0, int), np.empty(0, int)
        kps = np.array(list(best_for_kp.keys()), int)
        mp = np.array([v[0] for v in best_for_kp.values()], int)
        return mp, kps

    def _refine_pose(self, f, R, t, mp, kps):
        """Robust motion only BA with two rounds of outlier rejection. Returns R, t, inlier mask."""
        if len(mp) < 6:
            return R, t, np.zeros(len(mp), bool)
        X = np.array([self.points[i].X for i in mp])
        uv, sig = f.uv[kps], f.sigma[kps]
        inl = np.ones(len(mp), bool)
        for _ in range(3):
            if inl.sum() < 6:
                return R, t, np.zeros(len(mp), bool)
            R, t, _ = ba.optimize_pose(self.K, R, t, X[inl], uv[inl], sig[inl])
            proj, z = geo.project(self.K, R, t, X)
            chi2 = np.sum((proj - uv) ** 2, axis=1) / sig ** 2
            inl = (chi2 < CHI2) & (z > 0)
        return R, t, inl

    def _pnp(self, f, mp, kps):
        if len(mp) < 8:
            return None
        X = np.array([self.points[i].X for i in mp], dtype=np.float64)
        uv = np.ascontiguousarray(f.uv[kps], dtype=np.float64)
        ok, rvec, tvec, inl = cv2.solvePnPRansac(X, uv, self.K, None, iterationsCount=300,
                                                 reprojectionError=self.s.pnp_reprojection_px,
                                                 confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
        if not ok or inl is None or len(inl) < 8:
            return None
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.ravel()

    def _points_as_features(self, pids):
        desc = np.array([self.points[i].desc for i in pids])
        return feat.Features(np.zeros((len(pids), 2)), desc, np.ones(len(pids)), self.kind)

    def _track_with_guess(self, f, R, t, pids, tree, radius):
        mp, kps = self._search_by_projection(f, R, t, pids, radius, tree)
        R, t, inl = self._refine_pose(f, R, t, mp, kps)
        return R, t, mp[inl], kps[inl]

    def _track_brute_force(self, f, pids, tree):
        if len(pids) < 8:
            return None
        m = feat.ratio_match(self._points_as_features(pids), f, ratio=0.8, mutual=True)
        if len(m) < 8:
            return None
        mp, kps = np.array(pids)[m[:, 0]], m[:, 1]
        guess = self._pnp(f, mp, kps)
        if guess is None:
            return None
        R, t, inl = self._refine_pose(f, *guess, mp, kps)
        if inl.sum() < self.s.track_min_inliers:
            return None
        # With a good pose, gather everything visible from the local map.
        return self._track_with_guess(f, R, t, pids, tree, self.s.search_radius_px)

    def _relocalize(self, f, tree, candidates):
        best = None
        for kf in candidates:
            pids = [int(i) for i in kf.pt[kf.pt >= 0] if not self.points[int(i)].bad]
            got = self._track_brute_force(f, pids, tree)
            if got is None:
                continue
            R, t, mp, kps = got
            if len(mp) < self.s.reloc_min_inliers:
                continue
            # widen to the local map around this keyframe
            local = set(pids)
            for other in self.kfs[max(0, kf.id - self.s.local_window // 2): kf.id + self.s.local_window // 2 + 1]:
                local.update(int(i) for i in other.pt[other.pt >= 0])
            local = [i for i in local if not self.points[i].bad]
            R, t, mp, kps = self._track_with_guess(f, R, t, local, tree, self.s.search_radius_px)
            if len(mp) >= self.s.reloc_min_inliers and (best is None or len(mp) > len(best[2])):
                best = (R, t, mp, kps, kf)
        return best

    # ---------------------------------------------------------------- initialization
    def _try_initialize(self, index, f, color):
        ref_index, fr, ref_color = self.init_ref
        m = feat.ratio_match(fr, f, ratio=self.s.init_ratio, mutual=True)
        if len(m) < self.s.init_min_matches:
            return False
        p1, p2 = fr.uv[m[:, 0]], f.uv[m[:, 1]]
        s1, s2 = fr.sigma[m[:, 0]], f.sigma[m[:, 1]]
        E, maskE = cv2.findEssentialMat(p1, p2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        H, maskH = cv2.findHomography(p1, p2, cv2.RANSAC, 2.0)
        if E is None or E.shape != (3, 3):
            return False
        score_h = _score_homography(H, p1, p2, s2) if H is not None else 0.0
        score_f = _score_fundamental(self.K, E, p1, p2, s2)
        r_h = score_h / (score_h + score_f + 1e-12)
        if r_h > self.s.homography_ratio and H is not None:
            n, Rs, ts, _ = cv2.decomposeHomographyMat(H, self.K)
            hyps = [(Rs[i], ts[i].ravel()) for i in range(n)]
            model = "homography"
        else:
            R1, R2, tE = cv2.decomposeEssentialMat(E)
            tE = tE.ravel()
            hyps = [(R1, tE), (R1, -tE), (R2, tE), (R2, -tE)]
            model = "essential"
        results = []
        I, zero = np.eye(3), np.zeros(3)
        for R, t in hyps:
            t = t / max(np.linalg.norm(t), 1e-12)
            X = geo.triangulate(self.K, I, zero, R, t, p1, p2)
            uv1, z1 = geo.project(self.K, I, zero, X)
            uv2, z2 = geo.project(self.K, R, t, X)
            e1 = np.sum((uv1 - p1) ** 2, 1) / s1 ** 2
            e2 = np.sum((uv2 - p2) ** 2, 1) / s2 ** 2
            cosp = geo.parallax_cos(I, zero, R, t, X)
            good = np.isfinite(X).all(1) & (z1 > 0) & (z2 > 0) & (e1 < CHI2) & (e2 < CHI2) & (cosp < 0.99998)
            n_par = int(np.sum(good & (cosp < np.cos(np.radians(self.s.init_min_parallax_deg)))))
            results.append((int(good.sum()), n_par, R, t, X, good, cosp))
        results.sort(key=lambda r: -r[0])
        best = results[0]
        second = results[1][0] if len(results) > 1 else 0
        if best[1] < self.s.init_min_points or second > self.s.init_ambiguity * best[0]:
            return False
        n_good, n_par, R, t, X, good, cosp = best
        # Scale gauge: median depth of the reference view equals 1.
        med = np.median(X[good, 2])
        if not np.isfinite(med) or med <= 0:
            return False
        X, t = X / med, t / med
        kf0 = KeyFrame(0, ref_index, np.eye(3), np.zeros(3), fr, ref_color)
        kf1 = KeyFrame(1, index, R, t, f, color)
        kf1.n_tracked = int(good.sum())
        self.kfs = [kf0, kf1]
        colors = self._color(ref_color, p1)
        for j in np.flatnonzero(good):
            p = self._add_point(X[j], fr.desc[m[j, 0]], kf0, colors[j])
            self._observe(p, kf0, int(m[j, 0]))
            self._observe(p, kf1, int(m[j, 1]))
        self._bundle_adjust([0, 1], fixed={0}, iterations=self.s.global_ba_iterations, min_fixed=1)
        n_left = sum(1 for p in self.points.values() if not p.bad)
        if n_left < self.s.init_min_points:
            self.log(f"  init at {ref_index}-{index} rejected after BA ({n_left} points)")
            self.points.clear()
            self.kfs = []
            return False
        # re-normalise the scale after BA
        depths = [(kf0.R @ p.X + kf0.t)[2] for p in self.points.values() if not p.bad]
        med = np.median(depths)
        for p in self.points.values():
            p.X = p.X / med
        kf1.t = kf1.t / med
        self.initialized = True
        self.init_info = dict(ref_frame=ref_index, frame=index, model=model, r_h=float(r_h), matches=int(len(m)),
                              points=n_left, parallax_points=n_par, runner_up_points=int(second),
                              median_parallax_deg=float(np.degrees(np.arccos(np.clip(np.median(cosp[good]), -1, 1)))))
        self.log(f"  initialized with frames {ref_index} and {index}: {model}, R_H={r_h:.2f}, "
                 f"{n_left} points, runner up {second}/{n_good}")
        for rec in self.frames:
            if rec.index == ref_index:
                rec.status, rec.ref_kf, rec.R_rel, rec.t_rel = "keyframe", 0, np.eye(3), np.zeros(3)
                self.lost_features.pop(ref_index, None)
        pids1 = [int(i) for i in kf1.pt[kf1.pt >= 0]]
        self.last = (kf1.R, kf1.t, f, pids1)
        self.velocity = None
        return True

    # ---------------------------------------------------------------- main loop
    def process(self, index, timestamp, gray, color=None):
        f = self.extract(gray)
        rec = FrameRecord(index, timestamp)
        self.frames.append(rec)
        if not self.initialized:
            self.lost_features[index] = f
            if self.init_ref is None or index - self.init_ref[0] > self.s.init_max_gap:
                if len(f.uv) >= self.s.init_min_matches:
                    self.init_ref = (index, f, color)
                return rec
            if self._try_initialize(index, f, color):
                rec.status, rec.ref_kf, rec.R_rel, rec.t_rel = "keyframe", 1, np.eye(3), np.zeros(3)
                self.lost_features.pop(index, None)
            return rec
        if len(f.uv) < 10:
            self.lost_features[index] = f
            return rec
        tree = cKDTree(f.uv)
        local = self._local_point_ids(self.s.local_window)
        got = None
        if self.last is not None:
            R0, t0 = self.last[0], self.last[1]
            if self.velocity is not None:
                R0, t0 = geo.compose(*self.velocity, R0, t0)
            for radius in (self.s.search_radius_px, self.s.search_radius_wide_px):
                R, t, mp, kps = self._track_with_guess(f, R0, t0, local, tree, radius)
                if len(mp) >= self.s.track_min_inliers:
                    got = (R, t, mp, kps)
                    break
            if got is None:
                got = self._track_brute_force(f, local, tree)
                if got is not None and len(got[2]) < self.s.track_min_inliers:
                    got = None
        status = "tracked"
        if got is None:
            recent = self.kfs[::-1][: self.s.reloc_candidates]
            best = self._relocalize(f, tree, recent)
            if best is not None:
                got = best[:4]
                status = "relocalized"
                self.stats["relocalizations"] += 1
        if got is None:
            self.lost_features[index] = f
            self.last = None
            self.velocity = None
            return rec
        R, t, mp, kps = got
        for pid in mp:
            self.points[pid].found += 1
        if self.last is not None and status == "tracked":
            self.velocity = geo.compose(R, t, *geo.invert(self.last[0], self.last[1]))
        else:
            self.velocity = None
        self.last = (R, t, f, list(mp))
        last_kf = self.kfs[-1]
        if self.s.kf_reference == "tracked":
            # Compare with how many map points were tracked when the last keyframe was made. Point creation and
            # fusion in the keyframe don't raise the bar, so a keyframe is made when tracking has really weakened.
            n_ref = getattr(last_kf, "n_tracked", int(np.sum(last_kf.pt >= 0)))
        elif self.s.kf_ref_min_obs > 0:
            # ORB-SLAM: compare with the reference keyframe's points that are seen by >= 3 keyframes
            # (2 while the map has only two keyframes), not with every freshly triangulated point.
            min_obs = self.s.kf_ref_min_obs if len(self.kfs) > 2 else 2
            n_ref = sum(1 for pid in last_kf.pt[last_kf.pt >= 0] if len(self.points[int(pid)].obs) >= min_obs)
        else:
            n_ref = int(np.sum(last_kf.pt >= 0))
        need_kf = (len(mp) < self.s.kf_track_ratio * n_ref or index - last_kf.frame >= self.s.kf_max_gap
                   or len(mp) < 2 * self.s.track_min_inliers or status == "relocalized")
        if need_kf:
            kf = KeyFrame(len(self.kfs), index, R, t, f, color)
            kf.n_tracked = int(len(mp))
            self.kfs.append(kf)
            for pid, kp in zip(mp, kps):
                p = self.points[int(pid)]
                if not p.bad and kf.id not in p.obs:
                    self._observe(p, kf, int(kp))
            self._create_points(kf)
            if self.s.fuse_neighbors > 0:
                self._fuse(kf)
            self._local_mapping(kf)
            rec.status, rec.ref_kf, rec.R_rel, rec.t_rel = "keyframe", kf.id, np.eye(3), np.zeros(3)
            self.last = (kf.R, kf.t, f, [int(i) for i in kf.pt[kf.pt >= 0]])
        else:
            rec.status = status
            rec.ref_kf = last_kf.id
            rec.R_rel, rec.t_rel = geo.compose(R, t, *geo.invert(last_kf.R, last_kf.t))
        rec.inliers = int(len(mp))
        return rec

    # ---------------------------------------------------------------- mapping
    def _covisible(self, kf, n):
        counts = {}
        for pid in kf.pt[kf.pt >= 0]:
            for kid in self.points[int(pid)].obs:
                if kid != kf.id:
                    counts[kid] = counts.get(kid, 0) + 1
        ranked = sorted(counts.items(), key=lambda kv: -kv[1])
        return [self.kfs[k] for k, c in ranked[:n] if c >= 15] or [self.kfs[k] for k, _ in ranked[:2]]

    def _create_points(self, kf):
        neighbours = self._covisible(kf, self.s.triang_neighbors)
        min_cos = np.cos(np.radians(self.s.triang_min_parallax_deg))
        c_new = geo.camera_center(kf.R, kf.t)
        created = 0
        for nb in neighbours:
            baseline = np.linalg.norm(geo.camera_center(nb.R, nb.t) - c_new)
            depths = [(nb.R @ self.points[int(p)].X + nb.t)[2] for p in nb.pt[nb.pt >= 0]]
            if not depths or baseline / np.median(depths) < 0.01:
                continue
            free_a = np.flatnonzero(kf.pt < 0)
            free_b = np.flatnonzero(nb.pt < 0)
            if len(free_a) < 8 or len(free_b) < 8:
                continue
            fa = feat.Features(kf.f.uv[free_a], kf.f.desc[free_a], kf.f.sigma[free_a], self.kind)
            fb = feat.Features(nb.f.uv[free_b], nb.f.desc[free_b], nb.f.sigma[free_b], self.kind)
            m = feat.ratio_match(fa, fb, ratio=0.8, mutual=True, max_dist=feat.STRICT[self.kind] * 1.4)
            if len(m) == 0:
                continue
            ia, ib = free_a[m[:, 0]], free_b[m[:, 1]]
            ua, ub = kf.f.uv[ia], nb.f.uv[ib]
            sa, sb = kf.f.sigma[ia], nb.f.sigma[ib]
            E = geo.essential_from_poses(nb.R, nb.t, kf.R, kf.t)
            epi = geo.epipolar_sq_dist_px(self.K, E, ub, ua) / sa ** 2
            keep = epi < 3.84
            if not keep.any():
                continue
            ia, ib, ua, ub, sa, sb = ia[keep], ib[keep], ua[keep], ub[keep], sa[keep], sb[keep]
            X = geo.triangulate(self.K, kf.R, kf.t, nb.R, nb.t, ua, ub)
            pa, za = geo.project(self.K, kf.R, kf.t, X)
            pb, zb = geo.project(self.K, nb.R, nb.t, X)
            ea = np.sum((pa - ua) ** 2, 1) / sa ** 2
            eb = np.sum((pb - ub) ** 2, 1) / sb ** 2
            cosp = geo.parallax_cos(kf.R, kf.t, nb.R, nb.t, X)
            good = np.isfinite(X).all(1) & (za > 0) & (zb > 0) & (ea < CHI2) & (eb < CHI2) & (cosp < min_cos) & (cosp > 0)
            colors = self._color(kf.color_image, ua)
            for j in np.flatnonzero(good):
                if kf.pt[ia[j]] >= 0 or nb.pt[ib[j]] >= 0:
                    continue
                p = self._add_point(X[j], kf.f.desc[ia[j]], kf, colors[j])
                self._observe(p, nb, int(ib[j]))
                self._observe(p, kf, int(ia[j]))
                created += 1
        return created

    def _project_into(self, kf, pids):
        """Match points pids into keyframe kf by projection. Returns list of (pid, kp)."""
        if not pids:
            return []
        X = np.array([self.points[i].X for i in pids])
        uv, z = geo.project(self.K, kf.R, kf.t, X)
        ok = (z > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < self.width) & (uv[:, 1] >= 0) & (uv[:, 1] < self.height)
        if not ok.any():
            return []
        if getattr(kf, "tree", None) is None:
            kf.tree = cKDTree(kf.f.uv)
        out = []
        idx = np.flatnonzero(ok)
        for i, cand in zip(idx, kf.tree.query_ball_point(uv[idx], r=3.0 * np.sqrt(CHI2) * 1.2)):
            if not cand:
                continue
            cand = np.asarray(cand)
            e2 = np.sum((kf.f.uv[cand] - uv[i]) ** 2, 1) / kf.f.sigma[cand] ** 2
            cand = cand[e2 < CHI2]
            if len(cand) == 0:
                continue
            d = feat.paired_distance(np.repeat(self.points[pids[i]].desc[None], len(cand), 0), kf.f.desc[cand], self.kind)
            j = int(np.argmin(d))
            if d[j] <= feat.STRICT[self.kind]:      # ORB-SLAM Fuse uses the strict threshold (TH_LOW = 50)
                out.append((pids[i], int(cand[j])))
        return out

    def _replace(self, old, new):
        """Merge map point old into new (ORB-SLAM MapPoint::Replace)."""
        for kid, kp in list(old.obs.items()):
            kf = self.kfs[kid]
            if kf.pt[kp] == old.id:
                kf.pt[kp] = -1
            if kid not in new.obs and kf.pt[kp] < 0:
                new.obs[kid] = kp
                kf.pt[kp] = new.id
        new.visible += old.visible
        new.found += old.found
        old.obs.clear()
        old.bad = True

    def _merge_ok(self, p, q, kf):
        """Two points are only merged if they are close in 3D relative to their depth (same physical point)."""
        depth = (kf.R @ p.X + kf.t)[2]
        return depth > 0 and np.linalg.norm(p.X - q.X) < self.s.fuse_max_rel_distance * depth

    def _fuse(self, kf):
        """Fuse duplicated points between kf and its covisible keyframes (ORB-SLAM SearchInNeighbors)."""
        neighbours = self._covisible(kf, self.s.fuse_neighbors)
        own = [int(i) for i in kf.pt[kf.pt >= 0] if not self.points[int(i)].bad]
        for nb in neighbours:
            for pid, kp in self._project_into(nb, [i for i in own if nb.id not in self.points[i].obs]):
                p = self.points[pid]
                if p.bad:
                    continue
                other = nb.pt[kp]
                if other < 0:
                    self._observe(p, nb, kp)
                elif other != pid and not self.points[other].bad and self._merge_ok(p, self.points[other], nb):
                    q = self.points[other]
                    keep, drop = (p, q) if len(p.obs) >= len(q.obs) else (q, p)
                    self._replace(drop, keep)
        theirs = set()
        for nb in neighbours:
            theirs.update(int(i) for i in nb.pt[nb.pt >= 0])
        theirs = [i for i in theirs if not self.points[i].bad and kf.id not in self.points[i].obs]
        for pid, kp in self._project_into(kf, theirs):
            p = self.points[pid]
            if p.bad:
                continue
            other = kf.pt[kp]
            if other < 0:
                self._observe(p, kf, kp)
            elif other != pid and not self.points[other].bad and self._merge_ok(p, self.points[other], kf):
                q = self.points[other]
                keep, drop = (p, q) if len(p.obs) >= len(q.obs) else (q, p)
                self._replace(drop, keep)

    def _local_mapping(self, kf):
        # Cull recently created points that did not get re-observed or were rarely found when visible.
        for p in list(self.points.values()):
            if p.bad:
                continue
            age = kf.id - p.first_kf
            if age >= 2 and p.found / max(p.visible, 1) < self.s.min_found_ratio:
                self._set_bad(p)
            elif age >= self.s.cull_after_kfs and age <= self.s.cull_after_kfs + 1 and len(p.obs) <= 2:
                self._set_bad(p)
        window = [kf] + self._covisible(kf, self.s.local_window - 1)
        ids = sorted({k.id for k in window})
        started = time.perf_counter()
        self._bundle_adjust(ids, fixed={0} if 0 in ids else set(), iterations=self.s.local_ba_iterations,
                            min_fixed=self.s.local_ba_min_fixed)
        self.stats["local_ba_seconds"] += time.perf_counter() - started
        self.stats["n_local_ba"] += 1

    def _bundle_adjust(self, kf_ids, fixed=frozenset(), iterations=20, min_fixed=2):
        """BA over keyframes kf_ids and every good point they observe; other observing keyframes are fixed.

        Monocular BA has a 7 dof gauge (pose and scale). If fewer than `min_fixed` cameras are fixed, the
        oldest free keyframes are fixed as well (ORB-SLAM3 does the same in its local BA); otherwise the
        scale can wander along the nearly flat gauge direction.
        """
        kf_ids = list(kf_ids)
        pids = set()
        for k in kf_ids:
            kf = self.kfs[k]
            pids.update(int(i) for i in kf.pt[kf.pt >= 0])
        pids = sorted(i for i in pids if not self.points[i].bad)
        if not pids:
            return
        cams = set(kf_ids)
        for i in pids:
            cams.update(self.points[i].obs.keys())
        cams = sorted(cams)
        slot = {k: n for n, k in enumerate(cams)}
        pslot = {pid: n for n, pid in enumerate(pids)}
        fixed_mask = np.array([(k not in kf_ids) or (k in fixed) for k in cams])
        if fixed_mask.all():
            return
        for n in range(len(cams)):
            if fixed_mask.sum() >= min(min_fixed, len(cams) - 1):
                break
            fixed_mask[n] = True
        obs_c, obs_p, obs_uv, obs_s, obs_ref = [], [], [], [], []
        for pid in pids:
            for kid, kp in self.points[pid].obs.items():
                kf = self.kfs[kid]
                obs_c.append(slot[kid])
                obs_p.append(pslot[pid])
                obs_uv.append(kf.f.uv[kp])
                obs_s.append(kf.f.sigma[kp])
                obs_ref.append((pid, kid))
        obs_c, obs_p = np.array(obs_c), np.array(obs_p)
        obs_uv, obs_s = np.array(obs_uv), np.array(obs_s)
        rot = geo.matrix_to_rotvec(np.array([self.kfs[k].R for k in cams]))
        tr = np.array([self.kfs[k].t for k in cams])
        X = np.array([self.points[i].X for i in pids])
        for round_ in range(2):
            rot, tr, X, chi2 = ba.bundle_adjust(self.K, rot, tr, X, obs_c, obs_p, obs_uv, obs_s,
                                                fixed_cams=fixed_mask, robust=True, max_nfev=iterations)
            bad = ~(chi2 < CHI2)
            if round_ == 0 and bad.any():
                keep = ~bad
                obs_c, obs_p, obs_uv, obs_s = obs_c[keep], obs_p[keep], obs_uv[keep], obs_s[keep]
                for j in np.flatnonzero(bad):
                    pid, kid = obs_ref[j]
                    self._erase_observation(self.points[pid], kid)
                obs_ref = [r for r, k in zip(obs_ref, keep) if k]
            elif round_ == 1:
                for j in np.flatnonzero(bad):
                    pid, kid = obs_ref[j]
                    self._erase_observation(self.points[pid], kid)
        Rs = geo.rotvec_to_matrix(rot)
        for n, k in enumerate(cams):
            if not fixed_mask[n]:
                self.kfs[k].R, self.kfs[k].t = Rs[n], tr[n]
        for n, pid in enumerate(pids):
            if not self.points[pid].bad:
                self.points[pid].X = X[n]

    # ---------------------------------------------------------------- finish
    def finish(self):
        if not self.initialized:
            return
        started = time.perf_counter()
        self._bundle_adjust(range(len(self.kfs)), fixed={0}, iterations=self.s.global_ba_iterations, min_fixed=1)
        self._normalize_scale()
        self.stats["global_ba_seconds"] = time.perf_counter() - started
        # Localize frames that were never tracked (before initialization or while lost) against the final map.
        by_frame = {kf.frame: kf for kf in self.kfs}
        kf_frames = np.array(sorted(by_frame))
        for rec in self.frames:
            if rec.status != "untracked" or rec.index not in self.lost_features:
                continue
            f = self.lost_features[rec.index]
            if len(f.uv) < 10:
                continue
            nearest = kf_frames[np.argsort(np.abs(kf_frames - rec.index))[:4]]
            best = self._relocalize(f, cKDTree(f.uv), [by_frame[i] for i in nearest])
            if best is None:
                continue
            R, t, mp, kps, kf = best
            rec.status, rec.ref_kf, rec.inliers = "localized_after", kf.id, int(len(mp))
            rec.R_rel, rec.t_rel = geo.compose(R, t, *geo.invert(kf.R, kf.t))
        self.lost_features.clear()

    # ---------------------------------------------------------------- outputs
    def trajectory(self):
        """List of dicts: index, timestamp, status, R_wc, t_wc (camera to world)."""
        out = []
        for rec in self.frames:
            if rec.ref_kf < 0 or rec.R_rel is None:
                out.append(dict(index=rec.index, timestamp=rec.timestamp, status="untracked"))
                continue
            kf = self.kfs[rec.ref_kf]
            R, t = geo.compose(rec.R_rel, rec.t_rel, kf.R, kf.t)
            Rwc, twc = geo.invert(R, t)
            out.append(dict(index=rec.index, timestamp=rec.timestamp, status=rec.status, R_wc=Rwc, t_wc=twc,
                            inliers=rec.inliers))
        return out

    def map_points(self):
        good = [p for p in self.points.values() if not p.bad and len(p.obs) >= 2]
        if not good:
            return np.empty((0, 3)), np.empty((0, 3), np.uint8), np.empty(0, int)
        return (np.array([p.X for p in good]), np.array([p.color for p in good], np.uint8),
                np.array([len(p.obs) for p in good]))

    def _normalize_scale(self):
        """Rescale the whole map so that the first keyframe's median scene depth is 1 (cosmetic gauge choice)."""
        kf0 = self.kfs[0]
        X = np.array([self.points[int(i)].X for i in kf0.pt[kf0.pt >= 0]]).reshape(-1, 3)
        if len(X) == 0:
            return
        d = np.median((X @ kf0.R.T + kf0.t)[:, 2])
        if not np.isfinite(d) or d <= 0:
            return
        for p in self.points.values():
            p.X = p.X / d
        for kf in self.kfs:
            kf.t = kf.t / d
        for rec in self.frames:
            if rec.t_rel is not None:
                rec.t_rel = rec.t_rel / d

    def median_depth(self):
        """Median depth of the last keyframe's map points: a monocular scale drift diagnostic (starts at 1)."""
        if not self.kfs:
            return float("nan")
        kf = self.kfs[-1]
        X = np.array([self.points[int(i)].X for i in kf.pt[kf.pt >= 0]]).reshape(-1, 3)
        return float(np.median((X @ kf.R.T + kf.t)[:, 2])) if len(X) else float("nan")

    def reprojection_stats(self):
        """RMS and median pixel reprojection error over every keyframe observation of the final map."""
        errs = []
        for kf in self.kfs:
            idx = np.flatnonzero(kf.pt >= 0)
            if len(idx) == 0:
                continue
            X = np.array([self.points[int(kf.pt[i])].X for i in idx])
            uv, _ = geo.project(self.K, kf.R, kf.t, X)
            errs.append(np.linalg.norm(uv - kf.f.uv[idx], axis=1))
        if not errs:
            return dict(rmse_px=None, median_px=None, observations=0)
        e = np.concatenate(errs)
        return dict(rmse_px=float(np.sqrt(np.mean(e ** 2))), median_px=float(np.median(e)), observations=int(len(e)))

    def summary(self):
        statuses = [r.status for r in self.frames]
        return dict(settings=asdict(self.s), n_frames=len(self.frames), n_keyframes=len(self.kfs),
                    n_points=int(len(self.map_points()[0])),
                    status_counts={s: statuses.count(s) for s in sorted(set(statuses))},
                    init=getattr(self, "init_info", None), stats=self.stats,
                    reprojection=self.reprojection_stats())


def _score_homography(H, p1, p2, sigma):
    if H is None or abs(np.linalg.det(H)) < 1e-12:
        return 0.0
    Hinv = np.linalg.inv(H)
    h1 = np.column_stack((p1, np.ones(len(p1))))
    h2 = np.column_stack((p2, np.ones(len(p2))))
    a = h1 @ H.T
    a = a[:, :2] / a[:, 2:3]
    b = h2 @ Hinv.T
    b = b[:, :2] / b[:, 2:3]
    d1 = np.sum((a - p2) ** 2, 1) / sigma ** 2
    d2 = np.sum((b - p1) ** 2, 1) / sigma ** 2
    th = CHI2
    return float(np.sum(np.where(d1 < th, th - d1, 0)) + np.sum(np.where(d2 < th, th - d2, 0)))


def _score_fundamental(K, E, p1, p2, sigma):
    Kinv = np.linalg.inv(K)
    F = Kinv.T @ E @ Kinv
    h1 = np.column_stack((p1, np.ones(len(p1))))
    h2 = np.column_stack((p2, np.ones(len(p2))))
    l2 = h1 @ F.T
    l1 = h2 @ F
    num = np.sum(h2 * l2, 1) ** 2
    d2 = num / (l2[:, 0] ** 2 + l2[:, 1] ** 2 + 1e-18) / sigma ** 2
    d1 = num / (l1[:, 0] ** 2 + l1[:, 1] ** 2 + 1e-18) / sigma ** 2
    th, score_th = 3.84, CHI2   # ORB-SLAM: 1 dof inlier test, scored against the 2 dof constant
    return float(np.sum(np.where(d1 < th, score_th - d1, 0)) + np.sum(np.where(d2 < th, score_th - d2, 0)))
