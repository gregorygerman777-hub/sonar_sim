"""Demo 5: a short motion sequence, and what it takes to break the ambiguity.

Demo 1 showed that one view cannot separate a target at +phi from one at -phi.
This is the sequel. Two targets are placed at the same (r, theta) and opposite
elevation, and the sonar is translated along a line.

Translate horizontally and the two stay locked together: only z^2 enters the
range, so a cross-range move changes both identically. Translate vertically and
they separate immediately. That asymmetry is the mechanism multi-view sonar
reconstruction relies on, and it says something practical: a survey line that
never changes depth cannot resolve elevation no matter how many pings it takes.
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

RANGE_M, AZIMUTH_DEG, ELEVATION_DEG, RADIUS_M = 6.0, 0.0, 6.0, 0.18

sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=193, num_range_bins=600,
                           horizontal_fov_deg=30.0, vertical_beamwidth_deg=16.0,
                           max_range_m=10.0, num_elevation_subrays=96)

high = sc.spherical_to_world(RANGE_M, AZIMUTH_DEG, +ELEVATION_DEG)
low = sc.spherical_to_world(RANGE_M, AZIMUTH_DEG, -ELEVATION_DEG)
targets = {"+phi": high, "-phi": low}

STEPS = [0.0, 0.6, 1.2, 1.8]
tracks = {"cross-range (x)": np.array([1.0, 0.0, 0.0]),
          "vertical (z)": np.array([0.0, 0.0, 1.0])}

lines = []
fig, axs = plt.subplots(len(tracks), len(STEPS), figsize=(17, 7.4), sharex=True, sharey=True)

for row, (label, direction) in enumerate(tracks.items()):
    lines.append(f"\n{label} translation")
    lines.append(f"  {'step':>6}  {'+phi range':>11} {'+phi az':>9}   "
                 f"{'-phi range':>11} {'-phi az':>9}   {'separation':>10}")
    for col, step in enumerate(STEPS):
        position = direction * step
        scene_objects = [sonar.make_sphere(p, RADIUS_M, 0.9) for p in targets.values()]
        image = sim.render(scene_objects, position=position)

        measurements = []
        for name, point in targets.items():
            offset = point - position
            r = float(np.linalg.norm(offset))
            az = float(np.degrees(np.arctan2(offset[0], offset[1])))
            measurements.append((r, az))

        (r_hi, az_hi), (r_lo, az_lo) = measurements
        lines.append(f"  {step:6.2f}  {r_hi:11.4f} {az_hi:9.4f}   "
                     f"{r_lo:11.4f} {az_lo:9.4f}   {abs(r_hi-r_lo):10.4f}")

        ax = axs[row, col]
        sc.show_image(ax, image, sim, floor_db=-40)
        ax.set(xlim=(5.2, 7.2), title=f"{label.split()[0]}, step {step:.1f} m")
        if col == 0:
            ax.set_ylabel("azimuth (deg)")
        if row == len(tracks) - 1:
            ax.set_xlabel("range (m)")

report = "\n".join(lines)
print(report)
table_path = outputs.output_path("demo5_motion.txt")
with open(table_path, "w") as handle:
    handle.write("Two targets at the same (r, theta), opposite elevation.\n")
    handle.write(report + "\n")

fig.suptitle("Same pair of targets, two survey lines: cross-range motion keeps them merged, "
             "vertical motion separates them", y=1.0)
fig.tight_layout()
figure_path = outputs.output_path("demo5_motion.png")
fig.savefig(figure_path, dpi=140)
print(f"\nwrote {figure_path} and {table_path}")
