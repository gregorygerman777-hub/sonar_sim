"""Fixed catalogue of synthetic sonar sequences with exact ground truth.

Every sequence is a planar path (x, y, yaw with yaw along the path tangent) at
a fixed 0.4 s ping period, plus a random scene of small spheres lying in the
sonar's horizontal plane (z = -1 m, the sensor depth). Seeds and shapes were
fixed in PROTOCOL.md before the first run.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "python")]

import slam_experiment  # noqa: E402
import sonar  # noqa: E402

PING_PERIOD_S = 0.4
DEPTH_M = -1.0


# --- path shapes -----------------------------------------------------------
# Shape generators return (x, y, heading) with heading the direction of travel.
# `slam.pose_axes` puts the sonar's forward axis at yaw + pi/2, so a forward-
# looking sonar has yaw = heading - pi/2 (`forward_looking`). Sequences 00 to 03
# keep their original inward-looking convention from the earlier reports.

def _tangent_heading(xy):
    """Direction of travel from central differences, unwrapped."""
    velocity = np.gradient(xy, axis=0)
    return np.unwrap(np.arctan2(velocity[:, 1], velocity[:, 0]))


def forward_looking(path):
    """Convert (x, y, heading) into sonar poses whose forward axis follows the path."""
    return np.column_stack((path[:, :2], path[:, 2] - 0.5 * np.pi))


def _resample_by_arc_length(xy, step_m):
    """Resample a densely sampled polyline at equal arc-length spacing."""
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    targets = np.arange(0.0, s[-1], step_m)
    return np.column_stack([np.interp(targets, s, xy[:, k]) for k in range(2)])


def circle(radius_m, laps, count):
    angles = np.linspace(0.0, 2.0 * np.pi * laps, count)
    return np.column_stack((radius_m * np.cos(angles), radius_m * np.sin(angles),
                            angles + 0.5 * np.pi))  # heading: counter-clockwise tangent


def arc_270(radius_m=4.8, count=81):
    # Identical to SLAM/geometry_validation_20260915/run.py.
    angles = np.linspace(0.0, 1.5 * np.pi, count)
    return np.c_[radius_m * np.cos(angles), radius_m * np.sin(angles), angles + np.pi / 2]


def lemniscate(scale_m, count):
    t = np.linspace(0.0, 2.0 * np.pi, count)
    denominator = 1.0 + np.sin(t) ** 2
    xy = np.column_stack((scale_m * np.cos(t) / denominator,
                          scale_m * np.sin(t) * np.cos(t) / denominator))
    return np.column_stack((xy, _tangent_heading(xy)))


def lawnmower(leg_length_m, spacing_m, legs, step_m):
    dense = []
    turn_radius = 0.5 * spacing_m
    for leg in range(legs):
        y = leg * spacing_m
        xs = np.linspace(0.0, leg_length_m, 400)
        if leg % 2:
            xs = xs[::-1]
        dense.append(np.column_stack((xs, np.full_like(xs, y))))
        if leg < legs - 1:
            # Semicircular turn bulging outward past the end of the leg.
            end_x = leg_length_m if leg % 2 == 0 else 0.0
            outward = 1.0 if leg % 2 == 0 else -1.0
            theta = np.linspace(0.0, np.pi, 200)[1:-1]
            dense.append(np.column_stack((end_x + outward * turn_radius * np.sin(theta),
                                          y + turn_radius - turn_radius * np.cos(theta))))
    xy = _resample_by_arc_length(np.vstack(dense), step_m)
    return np.column_stack((xy, _tangent_heading(xy)))


def random_smooth(seed, count, step_m, turn_sigma_rad):
    rng = np.random.default_rng(seed)
    turn = np.convolve(rng.normal(0.0, turn_sigma_rad, count + 8), np.ones(9) / 9.0, mode="valid")
    yaw = np.cumsum(turn)
    xy = np.cumsum(step_m * np.column_stack((np.cos(yaw), np.sin(yaw))), axis=0)
    xy -= xy[0]
    return np.column_stack((xy, yaw))


def s_curve(count, step_m, amplitude_rad, wavelength_m):
    s = step_m * np.arange(count)
    yaw = amplitude_rad * np.sin(2.0 * np.pi * s / wavelength_m)
    xy = np.cumsum(step_m * np.column_stack((np.cos(yaw), np.sin(yaw))), axis=0)
    xy -= xy[0]
    return np.column_stack((xy, yaw))


# --- scenes ----------------------------------------------------------------

def interior_scene(seed, count=18):
    # Identical draw order to SLAM/geometry_validation_20260915/run.py.
    rng = np.random.default_rng(seed)
    return [sonar.make_sphere((x, y, DEPTH_M), radius, reflectivity)
            for x, y, radius, reflectivity in zip(rng.uniform(-2, 2, count), rng.uniform(-2, 2, count),
                                                  rng.uniform(.08, .16, count), rng.uniform(.5, 1., count))]


def target_field(seed, truth, density_per_m2=0.5, margin_m=5.0, clearance_m=1.0):
    """Uniform random field of spheres over the surveyed area, like scattered
    seabed clutter, with none closer than `clearance_m` to the path."""
    rng = np.random.default_rng(seed)
    low = truth[:, :2].min(axis=0) - margin_m
    high = truth[:, :2].max(axis=0) + margin_m
    count = int(round(density_per_m2 * np.prod(high - low)))
    centres = rng.uniform(low, high, (count, 2))
    nearest = np.min(np.linalg.norm(centres[:, None, :] - truth[None, :, :2], axis=2), axis=1)
    centres = centres[nearest >= clearance_m]
    radii = rng.uniform(0.08, 0.16, len(centres))
    reflectivity = rng.uniform(0.5, 1.0, len(centres))
    return [sonar.make_sphere((x, y, DEPTH_M), r, k)
            for (x, y), r, k in zip(centres, radii, reflectivity)]


# --- catalogue ---------------------------------------------------------------

def catalogue():
    """Ordered sequence specifications; each builds (truth, objects) lazily."""
    return [
        dict(name="00", shape="closed circle, exact revisit", seed=None,
             build=lambda: (slam_experiment.survey_poses(37)[0], slam_experiment.landmark_scene())),
        dict(name="01", shape="open 270 degree arc", seed=91501,
             build=lambda: (arc_270(), interior_scene(91501))),
        dict(name="02", shape="open 270 degree arc", seed=91502,
             build=lambda: (arc_270(), interior_scene(91502))),
        dict(name="03", shape="open 270 degree arc", seed=91503,
             build=lambda: (arc_270(), interior_scene(91503))),
        dict(name="04", shape="circle, 1.25 laps, revisit at new heading", seed=20260904,
             build=lambda: _with_field(circle(4.5, 1.25, 126), 20260904)),
        dict(name="05", shape="figure eight, self crossing", seed=20260905,
             build=lambda: _with_field(lemniscate(6.0, 121), 20260905)),
        dict(name="06", shape="lawnmower survey, three 10 m legs", seed=20260906,
             build=lambda: _with_field(lawnmower(10.0, 3.0, 3, 0.30), 20260906)),
        dict(name="07", shape="random smooth heading walk", seed=20260907,
             build=lambda: _with_field(random_smooth(20260907, 101, 0.35, 0.12), 20260907)),
        dict(name="08", shape="circle, two laps, fast", seed=20260908,
             build=lambda: _with_field(circle(4.0, 2.0, 101), 20260908)),
        dict(name="09", shape="S curve, slow", seed=20260909,
             build=lambda: _with_field(s_curve(81, 0.25, np.radians(40.0), 10.0), 20260909)),
    ]


def _with_field(path, seed):
    truth = forward_looking(path)
    return truth, target_field(seed, truth)


def positions_for(truth):
    return np.column_stack((truth[:, :2], np.full(len(truth), DEPTH_M)))


def times_for(truth):
    return PING_PERIOD_S * np.arange(len(truth))
