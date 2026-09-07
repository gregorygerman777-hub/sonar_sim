"""Demo 3: validate the renderer against the analytic intensity equation.

A flat plane square to the acoustic axis is swept from 1 to 10 m. With a single
boresight sub-ray the geometry is exactly the one the equation describes, so the
measured peak must equal

    I = R * cos(incidence) / r^4 * 10^(-TL/10),  TL = 2 * alpha * (r / 1000)

with cos(incidence) = 1 and alpha from Thorp at the transmit frequency.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import outputs
import sonar

FREQUENCY_HZ, REFLECTIVITY = 600e3, 0.4

# One sub-ray on boresight: the analytic case, with no beam integration to undo.
# The azimuth bin count is odd on purpose. With an even count the bin centres
# straddle boresight, the nearest beam sits at 0.23 degrees, and the ray then
# strikes at r/cos(theta) with cos(incidence) = cos(theta). That alone shifts the
# result by 4e-5, which is exactly the discrepancy an even count produces here.
sim = sonar.SonarSimulator(frequency_hz=FREQUENCY_HZ, num_azimuth_bins=65,
                           num_range_bins=2000, horizontal_fov_deg=30.0,
                           vertical_beamwidth_deg=12.0, max_range_m=12.0,
                           num_elevation_subrays=1)

alpha = sonar.thorp_alpha(FREQUENCY_HZ)
ranges = np.arange(1.0, 10.01, 0.25)
measured, theory = [], []

for r in ranges:
    plane = sonar.make_plane((0.0, r, 0.0), (0.0, -1.0, 0.0), REFLECTIVITY)
    measured.append(sim.render([plane]).max())
    loss = sonar.two_way_loss_db(alpha, r)
    theory.append(REFLECTIVITY * 1.0 / r ** 4 * 10 ** (-loss / 10))

measured = np.array(measured)
theory = np.array(theory)
error = np.abs(measured - theory) / theory

print(f"frequency {FREQUENCY_HZ/1e3:.0f} kHz, Thorp alpha = {alpha:.4f} dB/km")
print(f"reflectivity R = {REFLECTIVITY}, single boresight sub-ray")
print(f"  worst relative error over 1 to 10 m : {error.max():.3e}")
print(f"  median relative error               : {np.median(error):.3e}")
for r, m, t in list(zip(ranges, measured, theory))[::8]:
    print(f"    r = {r:5.2f} m   measured {m:.6e}   theory {t:.6e}")

fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
axs[0].semilogy(ranges, theory, "-", color="#b23a2b", linewidth=2.4,
                label="theory  $R\\cos\\theta_i / r^4 \\cdot 10^{-TL/10}$")
axs[0].semilogy(ranges, measured, "o", color="#12707f", markersize=5,
                markerfacecolor="none", label="simulator")
axs[0].set(xlabel="range (m)", ylabel="peak intensity",
           title=f"validation at {FREQUENCY_HZ/1e3:.0f} kHz")
axs[0].legend(fontsize=9)
axs[0].grid(alpha=0.3)

axs[1].semilogy(ranges, np.maximum(error, 1e-18), "o-", color="#12707f", markersize=4)
axs[1].set(xlabel="range (m)", ylabel="relative error",
           title=f"agreement, worst {error.max():.1e}")
axs[1].grid(alpha=0.3)

fig.suptitle("The renderer reproduces the analytic intensity equation exactly", y=1.0)
fig.tight_layout()
figure_path = outputs.output_path("demo3_validation.png")
fig.savefig(figure_path, dpi=140)
print(f"\nwrote {figure_path}")

assert error.max() < 1e-9, f"worst relative error {error.max():.3e}"
print("PASS: simulator matches theory to better than 1e-9 relative")
