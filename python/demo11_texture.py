"""Demo 11: textured backscatter, and why it is not the same thing as speckle.

Uniform reflectivity gives a seabed that is a smooth ramp: bright near, dim far,
with nothing on it. Real seabed is mottled at every scale, because the local
backscatter strength varies from patch to patch. Adding a band-limited noise
field to the reflectivity is the cheap stand-in for that, and it is most of what
separates an image that reads as sonar from one that reads as a rendering.

The reason to keep the two mechanisms separate is that they behave in opposite
ways under re-imaging. Texture is attached to the ground: move the sonar and the
same patch is still bright, which is exactly the property any registration,
contour matching or patch-motion estimate has to lean on. Speckle is redrawn on
every realisation and correlates with nothing.

This measures that. The same seabed is imaged from two positions 5 cm apart
along the boresight, the second image is shifted back by the 5 bins that
corresponds to, and the residual left after dividing out the deterministic
1/r^4 trend is correlated between the two views.
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

FLOOR_Z, SONAR_Z, TILT_DEG = -3.0, -1.0, 20.0
MAX_RANGE, N_RANGE = 10.0, 1000
STEP_M = 0.05                       # 5 bins at 10 mm resolution, exactly
RANGE_RESOLUTION = MAX_RANGE / N_RANGE
SHIFT_BINS = int(round(STEP_M / RANGE_RESOLUTION))

axes = sc.tilted_axes(TILT_DEG)
boresight = axes[:, 1]
position_a = np.array([0.0, 0.0, SONAR_Z])
position_b = position_a + STEP_M * boresight

sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=257, num_range_bins=N_RANGE,
                           horizontal_fov_deg=30.0, vertical_beamwidth_deg=12.0,
                           max_range_m=MAX_RANGE, num_elevation_subrays=3072)


def seabed(textured, seed=5):
    extra = dict(texture_amplitude=0.6, texture_scale_m=0.22,
                 texture_seed=seed) if textured else {}
    return [sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.05, **extra)]


smooth = sim.render(seabed(False), position=position_a, axes=axes)
smooth_b = sim.render(seabed(False), position=position_b, axes=axes)

# Bins the seabed actually lights up. Everything outside is empty water and would
# only add zeros to the correlation.
lit = smooth > 0.02 * smooth.max()
lit &= np.roll(smooth_b, SHIFT_BINS, axis=1) > 0.02 * smooth_b.max()
print(f"seabed occupies {100 * lit.mean():.1f}% of the beam-bin grid; "
      f"shift {SHIFT_BINS} bins for a {STEP_M * 100:.0f} cm move")


def residual(image, reference):
    """Divide out the deterministic field, leaving the modulation that carries
    information. For the smooth case this is 1 everywhere, which is the point."""
    out = np.ones_like(image)
    np.divide(image, reference, out=out, where=reference > 0)
    return out


print(f"\n  {'case':>20} {'contrast':>9} {'view-to-view r':>15}")
results, panels = [], []
for name, textured, speckle in [("uniform, no speckle", False, False),
                                ("texture only", True, False),
                                ("speckle only", False, True),
                                ("texture + speckle", True, True)]:
    a = sim.render(seabed(textured), position=position_a, axes=axes,
                   speckle=speckle, seed=17)
    b = sim.render(seabed(textured), position=position_b, axes=axes,
                   speckle=speckle, seed=91)

    residual_a = residual(a, smooth)
    residual_b = np.roll(residual(b, smooth_b), SHIFT_BINS, axis=1)

    left, right = residual_a[lit], residual_b[lit]
    contrast = float(left.std() / left.mean())
    if left.std() < 1e-9 or right.std() < 1e-9:
        correlation = np.nan     # a flat field has nothing to correlate
    else:
        correlation = float(np.corrcoef(left, right)[0, 1])

    results.append((name, contrast, correlation))
    panels.append((name, a))
    print(f"  {name:>20} {contrast:9.3f} {correlation:15.3f}")

print("\nspeckle contrast for a single look should be 1.000 exactly: "
      f"measured {results[2][1]:.3f}, which is sampling scatter over "
      f"{int(lit.sum())} bins")

# Averaging L independent looks divides the speckle contrast by sqrt(L) while
# leaving the texture untouched, so the correlation the registration depends on
# should come back. This is the number that says how many pings are needed
# before two views of the same seabed can be matched at all.
# Texture and speckle multiply and are independent, so with T of contrast
# sigma_T and L-look speckle of variance 1/L the combined contrast is
# sqrt(sigma_T^2 (1 + 1/L) + 1/L), which the measured column should follow.
texture_contrast = results[1][1]
print(f"\n  {'looks':>6} {'contrast':>9} {'predicted':>10} {'view-to-view r':>15}")
looks, multilook_r, multilook_contrast = [1, 2, 4, 8, 16, 32], [], []
for count in looks:
    stack_a = np.mean([sim.render(seabed(True), position=position_a, axes=axes,
                                  speckle=True, seed=1000 + look) for look in range(count)],
                      axis=0)
    stack_b = np.mean([sim.render(seabed(True), position=position_b, axes=axes,
                                  speckle=True, seed=5000 + look) for look in range(count)],
                      axis=0)
    left = residual(stack_a, smooth)[lit]
    right = np.roll(residual(stack_b, smooth_b), SHIFT_BINS, axis=1)[lit]
    multilook_r.append(float(np.corrcoef(left, right)[0, 1]))
    multilook_contrast.append(float(left.std() / left.mean()))
    predicted = np.sqrt(texture_contrast ** 2 * (1 + 1 / count) + 1 / count)
    print(f"  {count:6d} {multilook_contrast[-1]:9.3f} {predicted:10.3f} "
          f"{multilook_r[-1]:15.3f}")

fig = plt.figure(figsize=(16.0, 9.0), constrained_layout=True)
grid = fig.add_gridspec(3, 4)

for index, (name, image) in enumerate(panels):
    ax = fig.add_subplot(grid[index // 2, 2 * (index % 2):2 * (index % 2) + 2])
    sc.show_image(ax, image, sim, floor_db=-28)
    ax.set(xlim=(2.6, 9.0), xlabel="range (m)", ylabel="azimuth (deg)", title=name)

ax = fig.add_subplot(grid[2, 0:2])
for name, image in panels[1:]:
    values = residual(image, smooth)[lit]
    ax.hist(values, bins=140, range=(0, 3.5), histtype="step", linewidth=1.5, label=name)
ax.axvline(1.0, color="#8a8a8a", linestyle=":", linewidth=1.2)
ax.set(xlabel="backscatter relative to the smooth field", ylabel="bins",
       title="what each mechanism does to the distribution")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[2, 2:4])
ax.semilogx(looks, multilook_r, "o-", color="#2a8f5f", linewidth=2.0, base=2,
            label="view-to-view correlation")
ax.semilogx(looks, multilook_contrast, "s--", color="#e8735c", linewidth=1.6, base=2,
            label="speckle contrast")
ax.semilogx(looks, [1.0 / np.sqrt(count) for count in looks], ":", color="#8a8a8a",
            linewidth=1.4, base=2, label=r"$1/\sqrt{L}$")
ax.axhline(results[1][2], color="#2a8f5f", linestyle=":", linewidth=1.2)
ax.text(1.05, results[1][2] - 0.09, f"speckle-free ceiling {results[1][2]:.2f}",
        fontsize=8, color="#2a8f5f")
ax.set(xlabel="looks averaged, L", ylabel="correlation / contrast",
       xticks=looks, ylim=(0, 1.15),
       title="how many looks before two views can be registered")
ax.set_xticklabels([str(count) for count in looks])
ax.legend(fontsize=8, loc="center right")
ax.grid(alpha=0.25, which="both")

fig.suptitle("Textured backscatter is a property of the seabed; speckle is a property "
             "of the realisation", fontsize=13)
figure_path = outputs.output_path("demo11_texture.png")
fig.savefig(figure_path, dpi=130)
print(f"\nwrote {figure_path}")
