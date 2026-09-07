"""Scene construction helpers shared by the demos."""

import sys

import numpy as np

sys.path.insert(0, ".")
import sonar


def spherical_to_world(range_m, azimuth_deg, elevation_deg):
    """Place a point at (r, theta, phi) in the sonar's own frame.

    Follows the forward-scan convention with Y_s along the acoustic axis:
        P = r * [cos(phi) sin(theta), cos(phi) cos(theta), sin(phi)]
    """
    theta = np.radians(azimuth_deg)
    phi = np.radians(elevation_deg)
    return np.array([range_m * np.cos(phi) * np.sin(theta),
                     range_m * np.cos(phi) * np.cos(theta),
                     range_m * np.sin(phi)])


def to_decibels(image, floor_db=-60.0, reference=None):
    peak = reference if reference is not None else image.max()
    if peak <= 0:
        peak = 1.0
    return np.maximum(10 * np.log10(np.maximum(image, 1e-300) / peak), floor_db)


def show_image(ax, image, sim, floor_db=-60.0, reference=None, cmap="gray"):
    """Beam-bin image: range across, azimuth up."""
    azimuth = sim.azimuth_axis_deg()
    ranges = sim.range_axis_m()
    return ax.imshow(to_decibels(image, floor_db, reference), origin="lower", aspect="auto",
                     cmap=cmap, vmin=floor_db, vmax=0,
                     extent=[ranges[0], ranges[-1], azimuth[0], azimuth[-1]])


def tilted_axes(tilt_deg, roll_deg=0.0):
    """Sonar axes as columns (X_s, Y_s, Z_s), tilted nose-down and rolled.

    Y_s runs forward along the acoustic axis, Z_s is up, X_s = Y_s x Z_s.
    """
    t, r = np.radians(tilt_deg), np.radians(roll_deg)
    forward = np.array([0.0, np.cos(t), -np.sin(t)])
    up = np.array([0.0, np.sin(t), np.cos(t)])
    cross = np.cross(forward, up)
    c, s = np.cos(r), np.sin(r)
    return np.column_stack([c * cross + s * up, forward, -s * cross + c * up])
