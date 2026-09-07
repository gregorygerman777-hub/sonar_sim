"""Demo 2: occlusion. Two spheres on the same bearing, one behind the other.

Nothing here implements shadowing. Each sub-ray stops at its first surface, so
the far sphere simply receives no sub-rays over the solid angle the near one
covers, and the shadow is what is left.
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

sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=256, num_range_bins=600,
                           horizontal_fov_deg=30.0, vertical_beamwidth_deg=12.0,
                           max_range_m=10.0, num_elevation_subrays=3072)

# 3072 rather than the default 48. A compact target needs few sub-rays, but a
# seabed viewed at grazing incidence spreads one beam over hundreds of range
# bins. Measured fraction of seabed bins receiving any energy: 14.7% at 48
# sub-rays, 59.9% at 192, 99.9% from 768 up, with the bin-to-bin scatter only
# settling by about 3000. It costs 0.04 s a frame.

# The canonical case: an object resting on the seabed, viewed with the sonar
# tilted down. Sub-rays that the sphere intercepts never reach the floor behind
# it, so a dark streak trails away from the target.
FLOOR_Z, TILT_DEG = -2.0, 15.0
axes = sc.tilted_axes(TILT_DEG)

floor = sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.06)
sphere = sonar.make_sphere((0.0, 4.6, FLOOR_Z + 0.35), 0.35, 0.9)

only_far = sim.render([floor], axes=axes)
both = sim.render([floor, sphere], axes=axes)

ranges = sim.range_axis_m()
far_band = (ranges > 5.2) & (ranges < 9.0)
energy_alone = only_far[:, far_band].sum()
energy_behind = both[:, far_band].sum()

print(f"seabed alone, energy beyond the target        {energy_alone:.6e}")
print(f"same region with the sphere present           {energy_behind:.6e}")
print(f"fraction still visible                       {energy_behind/energy_alone:.4f}")
print(f"occluded                                     {100*(1-energy_behind/energy_alone):.1f}%")

fig, axs = plt.subplots(1, 3, figsize=(16, 4.6))
peak = both.max()
for ax, image, title in [(axs[0], only_far, "seabed alone"),
                         (axs[1], both, "sphere on the seabed, with its shadow")]:
    sc.show_image(ax, image, sim, reference=peak, floor_db=-50)
    ax.set(xlabel="range (m)", ylabel="azimuth (deg)", title=title, xlim=(2.5, 9.5))

profile_alone = only_far.sum(axis=0)
profile_both = both.sum(axis=0)
axs[2].semilogy(ranges, np.maximum(profile_alone, 1e-30), label="seabed alone",
                color="#12707f", linewidth=1.4)
axs[2].semilogy(ranges, np.maximum(profile_both, 1e-30), label="with occluder",
                color="#b23a2b", linewidth=1.4)
axs[2].set(xlim=(2.5, 9.5), ylim=(peak * 1e-5, peak * 30), xlabel="range (m)",
           ylabel="energy summed over azimuth", title="the far return is cut down")
axs[2].legend(fontsize=9)
axs[2].grid(alpha=0.3)

fig.suptitle("Occlusion is not implemented: it is what a sub-ray stopping at its first hit leaves behind",
             y=1.0)
fig.tight_layout()
figure_path = outputs.output_path("demo2_shadow.png")
fig.savefig(figure_path, dpi=140)
print(f"\nwrote {figure_path}")
