"""Shared display pieces: the phosphor palette and the console's test scene.

Both front ends use these, so the pygame console and the matplotlib one show the
same scene through the same colour map and cannot drift apart.
"""

import sys

import numpy as np

sys.path.insert(0, ".")
import sonar

FLOOR_Z, SONAR_Z = -2.6, -0.6
CAMERA_RISE = 0.25
FLOOR_DB = -62.0
# Summing a beam-bin image over range flattens it: almost every bearing in the
# main lobe sits within a few decibels of the peak. A floor as deep as the
# image's would map all of them to the bright end and show nothing.
BEARING_FLOOR_DB = -14.0


def phosphor_table():
    """Black through saturated green to a white-hot tip, the way a CRT reads."""
    level = np.linspace(0.0, 1.0, 256)
    green = level ** 0.62
    red = np.clip((level - 0.70) / 0.30, 0.0, 1.0) ** 1.5
    blue = np.clip((level - 0.82) / 0.18, 0.0, 1.0) ** 1.8
    return (np.stack([red, green, blue], axis=1) * 255.0).astype(np.uint8)


LUT = phosphor_table()


def to_pixels(decibels, floor=FLOOR_DB):
    """dB straight to 8-bit RGB through a lookup table.

    A gather on a uint8 index array, which is far cheaper than asking a plotting
    library to normalise and colour-map floats on every frame.
    """
    index = np.clip((decibels - floor) * (255.0 / -floor), 0.0, 255.0).astype(np.uint8)
    return LUT[index]


def build_scene(tilt_deg, target_range_m, texture, cylinder):
    tilt = np.radians(tilt_deg)
    centre = np.array([0.0, target_range_m * np.cos(tilt),
                       SONAR_Z - target_range_m * np.sin(tilt)])
    floor = sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.06,
                             texture_amplitude=texture, texture_scale_m=0.30)
    if cylinder:
        yaw = np.radians(35.0)
        target = sonar.make_cylinder(centre, (np.cos(yaw), np.sin(yaw), 0.0), 0.13, 0.55,
                                     0.9, texture_amplitude=0.4 * texture,
                                     texture_scale_m=0.10)
    else:
        target = sonar.make_sphere(centre, 0.30, 0.9, texture_amplitude=0.4 * texture,
                                   texture_scale_m=0.10)
    return [floor, target], centre
