"""The README's opening image: a survey pass over a cylinder and a rock.

The sonar advances along its own boresight while the seabed texture stays put,
so the highlight and the shadow behind it sweep through the fan the way they do
on a real pass. Speckle is redrawn every frame, which is what gives the image its
boil; the texture underneath does not move, which is the distinction demo 11
measures.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import scene as sc
import sonar

FLOOR_Z, SONAR_Z, TILT_DEG = -3.0, -1.0, 22.0
FRAMES, STEP_M, LOOKS = 24, 0.050, 4

yaw = np.radians(35.0)
axes = sc.tilted_axes(TILT_DEG)
objects = [sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.05,
                            texture_amplitude=0.55, texture_scale_m=0.30),
           sonar.make_cylinder((0.30, 4.30, FLOOR_Z + 0.12), (np.cos(yaw), np.sin(yaw), 0.0),
                               0.12, 0.70, 0.85, texture_amplitude=0.25, texture_scale_m=0.10),
           sonar.make_sphere((-0.95, 4.05, FLOOR_Z + 0.05), 0.16, 0.35,
                             texture_amplitude=0.25, texture_scale_m=0.10)]

sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=289, num_range_bins=720,
                           horizontal_fov_deg=34.0, vertical_beamwidth_deg=16.0,
                           max_range_m=7.5, num_elevation_subrays=2048)

fig, ax = plt.subplots(figsize=(8.4, 4.3))
fig.patch.set_facecolor("#05080a")
ax.set_facecolor("#05080a")
ax.tick_params(colors="#8ba1a9", labelsize=8)
for spine in ax.spines.values():
    spine.set_color("#26363d")
ax.set_xlabel("range (m)", color="#8ba1a9")
ax.set_ylabel("azimuth (deg)", color="#8ba1a9")
title = ax.set_title("", color="#dbe7ea", fontsize=10)

FLOOR_DB = -50


def look(index):
    """One displayed frame: LOOKS speckle realisations averaged, as a real
    display does. Single-look speckle has contrast 1 and buries the texture."""
    position = np.array([0.0, 0.0, SONAR_Z]) + index * STEP_M * axes[:, 1]
    return np.mean([sim.render(objects, position=position, axes=axes, speckle=True,
                               seed=1000 * index + look_index) for look_index in range(LOOKS)],
                   axis=0)


artist = sc.show_image(ax, look(0), sim, floor_db=FLOOR_DB)


def frame(index):
    artist.set_data(sc.to_decibels(look(index), floor_db=FLOOR_DB))
    title.set_text(f"forward-scan sonar, survey pass   advance {index * STEP_M:.2f} m")
    return artist, title


animation = FuncAnimation(fig, frame, frames=FRAMES, blit=False)
# A documentation asset, not a run artifact, so it lives with the docs rather
# than in a timestamped results directory.
import os
os.makedirs("docs", exist_ok=True)
path = "docs/hero_survey.gif"
animation.save(path, writer=PillowWriter(fps=9), dpi=84)
print(f"wrote {path}")
