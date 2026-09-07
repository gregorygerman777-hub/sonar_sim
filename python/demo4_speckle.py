"""Demo 4: the same scene with speckle off and on.

Single-look sonar amplitude is Rayleigh distributed, so intensity is
exponential about its mean. Multiplying by -ln(U) with U uniform is exactly that
distribution, not an approximation to it.
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

sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=256, num_range_bins=600,
                           horizontal_fov_deg=30.0, vertical_beamwidth_deg=12.0,
                           max_range_m=10.0, num_elevation_subrays=3072)

axes = sc.tilted_axes(15.0)
objects = [sonar.make_plane((0.0, 0.0, -2.0), (0.0, 0.0, 1.0), 0.06),
           sonar.make_sphere((0.0, 4.6, -1.65), 0.35, 0.9)]

clean = sim.render(objects, axes=axes)
noisy = sim.render(objects, axes=axes, speckle=True, seed=7)

lit = clean > clean.max() * 1e-6
ratio = noisy[lit] / clean[lit]
print(f"lit cells                     {int(lit.sum())}")
print(f"speckle factor mean           {ratio.mean():.4f}   (exponential: 1.0)")
print(f"speckle factor std / mean     {ratio.std()/ratio.mean():.4f}   (exponential: 1.0)")
print(f"fraction below 0.1 of mean    {(ratio < 0.1).mean():.4f}   (1 - exp(-0.1) = 0.0952)")

fig, axs = plt.subplots(1, 3, figsize=(16.5, 4.6))
peak = clean.max()
for ax, image, title in [(axs[0], clean, "speckle off"), (axs[1], noisy, "speckle on")]:
    sc.show_image(ax, image, sim, reference=peak, floor_db=-45)
    ax.set(xlabel="range (m)", ylabel="azimuth (deg)", title=title, xlim=(2.5, 9.5))

axs[2].hist(ratio, bins=60, range=(0, 5), density=True, color="#12707f",
            alpha=0.75, label="measured")
grid = np.linspace(0, 5, 200)
axs[2].plot(grid, np.exp(-grid), color="#b23a2b", linewidth=2,
            label=r"exponential, $e^{-x}$")
axs[2].set(xlabel="observed / true intensity", ylabel="density",
           title="single-look statistics")
axs[2].legend(fontsize=9)

fig.suptitle("Speckle is not additive noise: intensity is exponentially distributed about its mean",
             y=1.0)
fig.tight_layout()
figure_path = outputs.output_path("demo4_speckle.png")
fig.savefig(figure_path, dpi=140)
print(f"\nwrote {figure_path}")
