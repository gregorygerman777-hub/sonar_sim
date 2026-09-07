"""Demo 10: what platform motion during a sweep does to the image.

A mechanically scanned head visits bearings one at a time. Bin a is formed at

    t_a = T (a + 1/2) / N_theta

and the pose used for it is the pose at that instant, so an image taken while
the vehicle surges or yaws is not a snapshot: it is a set of measurements from
N_theta slightly different places, pasted into one grid as though they came from
one. Surge slides targets in range across the fan; yaw stretches or compresses
the fan in bearing, because the sweep and the vehicle are turning at once.

What this is not: a blur. Each bearing here is a single instant, so a compact
target stays sharp and simply lands in the wrong place. Real blur needs finite
dwell inside one bin, which this does not model. The visible smear comes from
extended targets, where neighbouring bearings are displaced by different amounts
and a straight edge comes out sheared.

Every displacement below is predicted from the geometry first and then measured
off the rendered image, so the two columns are independent.
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

FLOOR_Z, SONAR_Z, TILT_DEG = -3.0, -1.0, 14.0
FOV_DEG, N_AZ = 34.0, 341
SWEEP_S = 0.40
TARGET_RANGE = 4.5
MARKER_BEARINGS = [-12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0]

position = np.array([0.0, 0.0, SONAR_Z])
axes = sc.tilted_axes(TILT_DEG)

markers = [position + axes @ sc.spherical_to_world(TARGET_RANGE, bearing, 0.0)
           for bearing in MARKER_BEARINGS]
objects = [sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.04,
                            texture_amplitude=0.45, texture_scale_m=0.4)]
objects += [sonar.make_sphere(point, 0.05, 0.95) for point in markers]
# A long bar across the fan, so the shear has something straight to act on.
objects.append(sonar.make_cylinder(position + axes @ sc.spherical_to_world(6.2, 0.0, 0.0),
                                   (1.0, 0.0, 0.0), 0.07, 1.4, 0.8))

CASES = [("static", (0.0, 0.0, 0.0), 0.0),
         ("surge 1.5 m/s", (0.0, 1.5, 0.0), 0.0),
         ("yaw 30 deg/s", (0.0, 0.0, 0.0), 30.0),
         ("surge + yaw", (0.0, 1.5, 0.0), 30.0)]


def yaw_matrix(degrees):
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def predict(point, velocity, yaw_rate_dps, sweep_s):
    """Bin the target lands in once the platform has moved, and its range there.

    Bin a is only consistent with the target if, at t_a, the target's bearing in
    the moved frame is the bearing that bin stands for. That is one equation in
    a, solved by walking the fan and taking the crossing.
    """
    bins = np.arange(N_AZ)
    times = sweep_s * (bins + 0.5) / N_AZ
    bin_bearings = -0.5 * FOV_DEG + FOV_DEG * (bins + 0.5) / N_AZ

    bearings, ranges = np.empty(N_AZ), np.empty(N_AZ)
    for index, time in enumerate(times):
        moved = position + np.array(velocity) * time
        moved_axes = yaw_matrix(yaw_rate_dps * time) @ axes
        offset = point - moved
        local = moved_axes.T @ offset
        bearings[index] = np.degrees(np.arctan2(local[0], local[1]))
        ranges[index] = np.linalg.norm(offset)

    residual = bearings - bin_bearings
    crossing = np.nonzero(np.sign(residual[:-1]) != np.sign(residual[1:]))[0]
    if len(crossing) == 0:
        return np.nan, np.nan
    index = crossing[0]
    # Linear interpolation between the two bins that bracket the crossing.
    weight = residual[index] / (residual[index] - residual[index + 1])
    return (bin_bearings[index] + weight * (bin_bearings[index + 1] - bin_bearings[index]),
            ranges[index] + weight * (ranges[index + 1] - ranges[index]))


def measure(image, sim, bearing_hint):
    """Peak of the blob nearest a bearing, centroided in both axes."""
    bearings, ranges = sim.azimuth_axis_deg(), sim.range_axis_m()
    band = np.abs(bearings - bearing_hint) < 1.5
    window = (ranges > TARGET_RANGE - 0.9) & (ranges < TARGET_RANGE + 0.9)
    patch = image[np.ix_(band, window)]

    row, column = np.unravel_index(np.argmax(patch), patch.shape)
    keep = patch > 0.3 * patch[row, column]
    weights = patch * keep
    total = weights.sum()
    bearing = float(weights.sum(axis=1) @ bearings[band] / total)
    range_m = float(weights.sum(axis=0) @ ranges[window] / total)
    return bearing, range_m


def measure(image, sim, bearing_hint):
    """Peak of the blob nearest a bearing, centroided in both axes.

    Returns None when the predicted bearing has been pushed outside the fan,
    which is itself a result: a hard enough yaw walks targets off the edge.
    """
    bearings, ranges = sim.azimuth_axis_deg(), sim.range_axis_m()
    band = np.abs(bearings - bearing_hint) < 1.5
    window = (ranges > TARGET_RANGE - 0.7) & (ranges < TARGET_RANGE + 0.7)
    if not band.any():
        return None
    patch = image[np.ix_(band, window)]
    if patch.size == 0 or patch.max() <= 0.0:
        return None

    row, column = np.unravel_index(np.argmax(patch), patch.shape)
    keep = patch > 0.3 * patch[row, column]
    weights = patch * keep
    total = weights.sum()
    bearing = float(weights.sum(axis=1) @ bearings[band] / total)
    range_m = float(weights.sum(axis=0) @ ranges[window] / total)
    return bearing, range_m


images = []
print(f"sweep {SWEEP_S} s over {FOV_DEG} deg in {N_AZ} bearings; markers on a "
      f"{TARGET_RANGE} m arc")
print("shifts are against the static image, which removes the sphere-radius bias "
      "common to both\n")

baseline = {}
for name, velocity, yaw_rate in CASES:
    moving = any(velocity) or yaw_rate != 0.0
    sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=N_AZ,
                               num_range_bins=900, horizontal_fov_deg=FOV_DEG,
                               vertical_beamwidth_deg=12.0, max_range_m=9.0,
                               num_elevation_subrays=2048,
                               platform_velocity_mps=velocity,
                               platform_yaw_rate_dps=yaw_rate,
                               sweep_duration_s=SWEEP_S if moving else 0.0)
    image = sim.render(objects, position=position, axes=axes)
    images.append((name, image, sim))

    print(name)
    print(f"  {'true brg':>9} | {'pred d.brg':>10} {'meas d.brg':>10} | "
          f"{'pred d.rng':>10} {'meas d.rng':>10}")
    apparent = []
    for hint, point in zip(MARKER_BEARINGS, markers):
        predicted = (hint, TARGET_RANGE) if not moving else predict(
            point, velocity, yaw_rate, SWEEP_S)
        measured = measure(image, sim, predicted[0]) if np.isfinite(predicted[0]) else None
        if not moving:
            baseline[hint] = measured
        reference = baseline[hint]
        if measured is None:
            apparent.append(np.nan)
            print(f"  {hint:9.2f} | {predicted[0]:10.2f} {'off fan':>10} | "
                  f"{predicted[1] - TARGET_RANGE:10.3f} {'-':>10}")
            continue
        apparent.append(measured[0])
        print(f"  {hint:9.2f} | {predicted[0] - hint:10.2f} {measured[0] - reference[0]:10.2f} | "
              f"{predicted[1] - TARGET_RANGE:10.3f} {measured[1] - reference[1]:10.3f}")

    if yaw_rate != 0.0:
        # Sweeping at FOV/T while the head turns at omega compresses or stretches
        # the fan by FOV / (FOV -+ omega T). The measured factor sits a couple of
        # percent off it because the yaw is about world Z and the head is tilted,
        # so it is not a pure bearing rotation in the sonar's own frame.
        seen = np.array(apparent, dtype=float)
        usable = np.isfinite(seen)
        measured_gain = (np.polyfit(np.array(MARKER_BEARINGS)[usable], seen[usable], 1)[0]
                         if usable.sum() > 1 else np.nan)
        analytic = FOV_DEG / (FOV_DEG - yaw_rate * SWEEP_S)
        print(f"  fan magnification: analytic {analytic:.3f}, measured {measured_gain:.3f}")
    print()

fig = plt.figure(figsize=(16.5, 8.4), constrained_layout=True)
grid = fig.add_gridspec(2, 4)

for index, (name, image, sim) in enumerate(images):
    ax = fig.add_subplot(grid[index // 2, 2 * (index % 2):2 * (index % 2) + 2])
    sc.show_image(ax, image, sim, floor_db=-40)
    for hint in MARKER_BEARINGS:
        ax.plot(TARGET_RANGE, hint, "_", color="#35b4c4", markersize=9, markeredgewidth=1.4)
    ax.set(xlim=(3.4, 7.2), xlabel="range (m)", ylabel="azimuth (deg)",
           title=f"{name}   (ticks mark where a static sonar puts them)")

fig.suptitle("Platform motion inside one sweep: surge slides the arc in range, "
             "yaw rescales the fan in bearing", fontsize=13)
figure_path = outputs.output_path("demo10_motion.png")
fig.savefig(figure_path, dpi=130)
print(f"wrote {figure_path}")
