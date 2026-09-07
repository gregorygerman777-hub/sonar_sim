"""Demo 7: an animated roll sweep, written headlessly to a GIF.

Every frame comes from the C++ core. Rolling the sonar about its viewing axis
turns the mirror's elevation offset into a bearing offset, so the mirror walks
out of the object and becomes a separate blob.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import outputs
import scene as sc
import sonar

SONAR_DEPTH, TARGET_DEPTH, RANGE_M = 0.43, 0.15, 2.05
horizontal = np.sqrt(RANGE_M ** 2 - (TARGET_DEPTH - SONAR_DEPTH) ** 2)
aim_deg = np.degrees(np.arctan2(TARGET_DEPTH - SONAR_DEPTH, horizontal))
centre = np.array([0.0, horizontal, -TARGET_DEPTH])
position = np.array([0.0, 0.0, -SONAR_DEPTH])

r_direct = float(np.linalg.norm(centre - position))
r_bounced = float(np.linalg.norm(centre - np.array([0.0, 0.0, SONAR_DEPTH])))

sim = sonar.SonarSimulator(frequency_hz=1.8e6, num_azimuth_bins=97, num_range_bins=900,
                           horizontal_fov_deg=29.0, vertical_beamwidth_deg=14.0,
                           max_range_m=3.2, num_elevation_subrays=256,
                           multipath_enabled=True, surface_z=0.0)

rolls = np.arange(0.0, 91.0, 3.0)
fig, ax = plt.subplots(figsize=(8.6, 5.2))
fig.patch.set_facecolor("#0b1114")
ax.set_facecolor("#05080a")


def frame(index):
    roll = rolls[index]
    image = sim.render([sonar.make_sphere(centre, 0.05, 0.9)], position=position,
                       axes=sc.tilted_axes(aim_deg, roll))
    ax.clear()
    ax.set_facecolor("#05080a")
    sc.show_image(ax, image, sim, floor_db=-45)
    for value, colour in [(r_direct, "#35b4c4"), (0.5 * (r_direct + r_bounced), "#e8735c"),
                          (r_bounced, "#d8a53c")]:
        ax.axvline(value, color=colour, linewidth=1.0, alpha=0.9)
    ax.set(xlim=(1.98, 2.19), xlabel="range (m)", ylabel="azimuth (deg)",
           title=f"roll {roll:4.1f}°    object / ghost / mirror marked")
    ax.tick_params(colors="#8ba1a9", labelsize=8)
    ax.xaxis.label.set_color("#8ba1a9")
    ax.yaxis.label.set_color("#8ba1a9")
    ax.title.set_color("#dbe7ea")
    for spine in ax.spines.values():
        spine.set_color("#26363d")
    return []


animation = FuncAnimation(fig, frame, frames=len(rolls), blit=False)
gif_path = outputs.output_path("demo7_roll_sweep.gif")
animation.save(gif_path, writer=PillowWriter(fps=8), dpi=110)
print(f"rendered {len(rolls)} frames from roll 0 to 90 degrees")
print(f"wrote {gif_path}")
