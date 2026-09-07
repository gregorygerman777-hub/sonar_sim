"""Demo 6: sea-surface multipath, and why the mirror needs roll.

Reflection off the air-water interface adds two components. The bounced leg has
the same length and launch direction as the straight line to the sonar mirrored
through the surface, so one reflection gives both:

    object   out direct, back direct     r_d
    ghost    one leg each way            (r_d + r_m) / 2
    mirror   both legs bounced           r_m

The ghost keeps the object's bearing, so it corrupts almost every view. The
mirror arrives from the mirrored direction, which for a shallow target sits
outside the vertical beam. Rolling the sonar about its viewing axis turns that
elevation offset into a bearing offset, and the mirror appears.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import outputs
import scene as sc
import sonar

SONAR_DEPTH, TARGET_DEPTH, RANGE_M = 0.43, 0.15, 2.05
ROLLS = [0.0, 22.5, 45.0, 67.5, 90.0]

horizontal = np.sqrt(RANGE_M ** 2 - (TARGET_DEPTH - SONAR_DEPTH) ** 2)
aim_deg = np.degrees(np.arctan2(TARGET_DEPTH - SONAR_DEPTH, horizontal))
centre = np.array([0.0, horizontal, -TARGET_DEPTH])
position = np.array([0.0, 0.0, -SONAR_DEPTH])

r_direct = float(np.linalg.norm(centre - position))
r_bounced = float(np.linalg.norm(centre - np.array([0.0, 0.0, SONAR_DEPTH])))
r_ghost = 0.5 * (r_direct + r_bounced)

print(f"sonar {SONAR_DEPTH} m deep, target {TARGET_DEPTH} m deep at {RANGE_M} m, "
      f"aim {aim_deg:+.2f} deg")
print(f"  object {r_direct:.4f} m   ghost {r_ghost:.4f} m   mirror {r_bounced:.4f} m")
print(f"\n  {'roll':>6}   {'ghost/object':>13} {'mirror/object':>14}")

images, ratios = [], []
for roll in ROLLS:
    sim = sonar.SonarSimulator(frequency_hz=1.8e6, num_azimuth_bins=97, num_range_bins=900,
                               horizontal_fov_deg=29.0, vertical_beamwidth_deg=14.0,
                               max_range_m=3.2, num_elevation_subrays=256,
                               multipath_enabled=True, surface_z=0.0)
    image = sim.render([sonar.make_sphere(centre, 0.05, 0.9)], position=position,
                       axes=sc.tilted_axes(aim_deg, roll))
    images.append((roll, image, sim))

    ranges = sim.range_axis_m()
    profile = image.sum(axis=0)
    band = lambda c: profile[(ranges > c - 0.03) & (ranges < c + 0.03)].sum()
    reference = max(band(r_direct), 1e-300)
    ratios.append((band(r_ghost) / reference, band(r_bounced) / reference))
    print(f"  {roll:6.1f}   {ratios[-1][0]:13.4f} {ratios[-1][1]:14.4f}")

fig, axs = plt.subplots(2, len(ROLLS), figsize=(19, 7),
                        gridspec_kw={"height_ratios": [3, 2]})
for col, (roll, image, sim) in enumerate(images):
    ax = axs[0, col]
    sc.show_image(ax, image, sim, floor_db=-45)
    for value, colour in [(r_direct, "#35b4c4"), (r_ghost, "#e8735c"), (r_bounced, "#d8a53c")]:
        ax.axvline(value, color=colour, linewidth=0.9, alpha=0.9)
    ax.set(xlim=(1.95, 2.22), title=f"roll {roll:.1f}°")
    ax.set_xlabel("range (m)")
    if col == 0:
        ax.set_ylabel("azimuth (deg)")

    ax = axs[1, col]
    profile = image.sum(axis=0)
    ax.semilogy(sim.range_axis_m(), np.maximum(profile, 1e-30), color="#dbe7ea", linewidth=1.2)
    for value, colour in [(r_direct, "#35b4c4"), (r_ghost, "#e8735c"), (r_bounced, "#d8a53c")]:
        ax.axvline(value, color=colour, linewidth=1.0)
    ax.set(xlim=(1.95, 2.22), ylim=(profile.max() * 1e-7, profile.max() * 5),
           xlabel="range (m)")
    if col == 0:
        ax.set_ylabel("energy vs range")
    ax.grid(alpha=0.25)

fig.suptitle("Sea-surface multipath: the ghost rides on every view, the mirror only appears "
             "once roll moves it into the beam    "
             "(blue object, red ghost, amber mirror)", y=1.0)
fig.tight_layout()
figure_path = outputs.output_path("demo6_multipath.png")
fig.savefig(figure_path, dpi=135)
print(f"\nwrote {figure_path}")
