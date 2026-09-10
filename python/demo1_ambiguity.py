"""Demo 1: the elevation ambiguity, proved rather than asserted.

A target at (r, theta, +phi) and the same target at (r, theta, -phi) are two
genuinely different physical configurations. A forward-scan sonar addresses its
image by range and bearing only, and its vertical beam pattern is even in phi,
so the two configurations must produce the same image. This script renders both
and checks it.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "python")
import outputs
import scene as sc
import sonar

RANGE_M, AZIMUTH_DEG, ELEVATION_DEG, RADIUS_M = 6.0, 4.0, 5.0, 0.15

sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=192, num_range_bins=480,
                           horizontal_fov_deg=30.0, vertical_beamwidth_deg=14.0,
                           max_range_m=10.0, num_elevation_subrays=64)

above = [sonar.make_sphere(sc.spherical_to_world(RANGE_M, AZIMUTH_DEG, +ELEVATION_DEG),
                           RADIUS_M, 0.8)]
below = [sonar.make_sphere(sc.spherical_to_world(RANGE_M, AZIMUTH_DEG, -ELEVATION_DEG),
                           RADIUS_M, 0.8)]

image_up = sim.render(above)
image_down = sim.render(below)

difference = np.abs(image_up - image_down)
peak = image_up.max()
relative = difference.max() / peak if peak > 0 else 0.0

# Bitwise equality is not the right test and would be wrong to claim. For the
# +phi target the sub-ray that strikes it is s; for -phi it is (n-1-s), so the
# contributions accumulate in reverse order, and floating-point addition is not
# associative. Agreement at the level of double-precision round-off is exactly
# what an exactly symmetric model should produce.
TOLERANCE = 1e-12
identical = relative <= TOLERANCE

print(f"target at r = {RANGE_M} m, theta = {AZIMUTH_DEG} deg, phi = +/-{ELEVATION_DEG} deg")
print(f"  lit bins            {int((image_up > 0).sum())} and {int((image_down > 0).sum())}")
print(f"  peak intensity      {peak:.6e} and {image_down.max():.6e}")
print(f"  max |difference|    {difference.max():.3e}")
print(f"  relative to peak    {relative:.3e}")
print(f"  tolerance           {TOLERANCE:.0e} of peak")
print(f"  equal within it     {identical}")

fig = plt.figure(figsize=(13, 9), constrained_layout=True)
for index, sign in enumerate((1, -1)):
    ax = fig.add_subplot(2, 2, index+1, projection="3d")
    centre = sc.spherical_to_world(RANGE_M, AZIMUTH_DEG, sign*ELEVATION_DEG)
    ax.scatter(*centre, s=150, c="#12707f")
    ax.plot([0,centre[0]], [0,centre[1]], [0,centre[2]], color="#12707f")
    ax.scatter(0,0,0,c="black")
    ax.set(xlabel="X starboard (m)", ylabel="Y forward (m)", zlabel="Z up (m)",
           zlim=(-1,1), title=f"Physical scene: elevation {sign*ELEVATION_DEG:+.0f} degrees")
axs = [fig.add_subplot(2,2,3), fig.add_subplot(2,2,4)]
for ax, image, title in [(axs[0], image_up, f"target at elevation +{ELEVATION_DEG:.0f}°"),
                         (axs[1], image_down, f"same target at −{ELEVATION_DEG:.0f}°")]:
    sc.show_image(ax, image, sim, reference=peak)
    ax.set(xlabel="range (m)", ylabel="azimuth (deg)", title=title, xlim=(5.2, 6.8))

fig.suptitle(f"Elevation disappears: max difference {difference.max():.2e}, relative {relative:.2e}")
figure_path = outputs.output_path("demo1_ambiguity.png")
fig.savefig(figure_path, dpi=140)
print(f"\nwrote {figure_path}")

assert identical, f"images differ by {relative:.3e} of peak, above the {TOLERANCE:.0e} tolerance"
print(f"PASS: the two renders agree to {relative:.1e} of peak, which is double-precision")
print("      round-off from summation order, not a physical difference.")
