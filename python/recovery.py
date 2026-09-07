"""Opti-acoustic elevation recovery, factored out so it can be run many times.

These three functions were written inside demo 8 and are moved here unchanged:
the detections are still made off rendered images and the fusion still walks the
ambiguity arc looking for the elevation whose projection lands on the detected
pixel. `fuse` now takes the sonar's origin and axes as arguments instead of
closing over the demo's module-level constants, which is the only edit.

The Monte Carlo layer calls exactly these, so a repeated trial is the same
pipeline as the single run, not a reimplementation of it.
"""

import sys

import numpy as np

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import scene as sc
import sonar

# The scene demo 8 argues over, kept here so the single run and the repeated
# trials cannot drift apart.
FLOOR_Z = -3.0
SONAR_POSITION = np.array([0.0, 0.0, -1.0])
CAMERA_OFFSET = np.array([0.0, 0.0, 0.30])   # a third of a metre above the head
TILT_DEG = 15.0
BEAMWIDTH_DEG = 14.0
TARGET_RANGE, TARGET_AZIMUTH, TARGET_ELEVATION = 4.0, 6.0, 3.5
TARGET_RADIUS = 0.10


def scene_axes():
    return sc.tilted_axes(TILT_DEG)


def target_centre(axes):
    return SONAR_POSITION + axes @ sc.spherical_to_world(
        TARGET_RANGE, TARGET_AZIMUTH, TARGET_ELEVATION)


def build_objects(centre):
    return [sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.06,
                             texture_amplitude=0.5, texture_scale_m=0.35),
            sonar.make_sphere(centre, TARGET_RADIUS, 0.95)]


def build_sonar():
    return sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=257,
                                num_range_bins=800, horizontal_fov_deg=30.0,
                                vertical_beamwidth_deg=BEAMWIDTH_DEG, max_range_m=8.0,
                                num_elevation_subrays=2048)


def build_camera(axes, baseline_m=0.30, attenuation_per_m=0.08):
    """The camera, mounted baseline_m above the sonar head."""
    return sonar.OpticalCamera(width=384, height=288, focal_px=300.0,
                               position=SONAR_POSITION + np.array([0.0, 0.0, baseline_m]),
                               axes=axes, attenuation_per_m=attenuation_per_m)


def sonar_detection(image, sim):
    """Intensity-weighted centroid of the brightest blob, in (range, bearing)."""
    peak = np.unravel_index(np.argmax(image), image.shape)
    mask = image > 0.25 * image[peak]
    window = np.zeros_like(mask)
    window[max(peak[0] - 12, 0):peak[0] + 13, max(peak[1] - 12, 0):peak[1] + 13] = True
    mask &= window

    weights = image * mask
    total = weights.sum()
    bearing = float((weights.sum(axis=1) * sim.azimuth_axis_deg()).sum() / total)
    range_m = float((weights.sum(axis=0) * sim.range_axis_m()).sum() / total)
    return range_m, bearing


def camera_detection(frame):
    """Centroid of the pixels well above the background, and the contrast that got them."""
    background = np.median(frame)
    contrast = (frame.max() - background) / max(background, 1e-12)
    mask = frame > background + 0.5 * (frame.max() - background)

    rows, columns = np.nonzero(mask)
    weights = frame[rows, columns] - background
    u = float((columns + 0.5) @ weights / weights.sum())
    v = float((rows + 0.5) @ weights / weights.sum())
    return (u, v), contrast, mask


def fuse(range_m, bearing_deg, pixel, camera, origin, axes, beamwidth_deg=14.0,
         samples=4001):
    """Walk the ambiguity arc and keep the elevation whose projection lands on the pixel."""
    elevations = np.linspace(-0.5 * beamwidth_deg, 0.5 * beamwidth_deg, samples)
    best = (np.inf, np.nan, None)
    track = []
    for elevation in elevations:
        point = origin + axes @ sc.spherical_to_world(range_m, bearing_deg, elevation)
        projected = camera.project(point)
        if projected is None:
            continue
        track.append(projected)
        residual = np.hypot(projected[0] - pixel[0], projected[1] - pixel[1])
        if residual < best[0]:
            best = (residual, elevation, point)
    return best, np.array(track)


# Read noise as a fraction of the target's contrast above background. A noiseless
# camera makes every trial's pixel identical and hides the half of the error
# budget that belongs to the optical sensor.
CAMERA_NOISE = 0.02


def noisy_camera_frame(camera, objects, rng, noise=CAMERA_NOISE):
    frame = camera.render(objects)
    return frame + rng.normal(0.0, noise * (frame.max() - np.median(frame)), frame.shape)


def trial(sim, objects, axes, seed, baselines, noise=CAMERA_NOISE):
    """One independent realisation, fused at each baseline.

    The sonar image does not depend on where the camera is bolted, so it is
    rendered once and shared across baselines. That makes a baseline sweep a
    paired comparison: differences between baselines are not contaminated by
    different speckle draws.
    """
    rng = np.random.default_rng(seed)
    image = sim.render(objects, position=SONAR_POSITION, axes=axes, speckle=True, seed=seed)
    detected_range, detected_bearing = sonar_detection(image, sim)

    elevations = []
    for baseline in baselines:
        camera = build_camera(axes, baseline_m=baseline)
        pixel, _, _ = camera_detection(noisy_camera_frame(camera, objects, rng, noise))
        (_, elevation, _), _ = fuse(detected_range + TARGET_RADIUS, detected_bearing, pixel,
                                    camera, SONAR_POSITION, axes, BEAMWIDTH_DEG)
        elevations.append(elevation)
    return detected_range, np.array(elevations)
