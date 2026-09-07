"""Demo 9: the highlight-and-shadow signature of a man-made target.

A cylinder lying on the seabed is the standard stand-in for a pipe, a mine or a
length of debris, and it produces the pairing every sonar operator reads first:
a bright return off the flank turned toward the sonar, then a black wedge behind
it where the seabed gets no sub-rays at all. A rock of similar size gives a
softer, rounder highlight and a shorter shadow, which is most of why the pair is
diagnostic rather than merely visible.

The shadow is also a measurement. A sonar a height h_s above the seabed looking
at an object of height h_o whose far edge is a horizontal distance d_o away puts
the shadow's end at d_o h_s / (h_s - h_o) by similar triangles, so

    L = d_o h_o / (h_s - h_o)      and      h_o = h_s L / (d_o + L)

recovers the object's height from a length measured in the image. That is the
classical mine-hunting height estimate, and the sweep at the end checks it
against the truth rather than asserting it.
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

FLOOR_Z, SONAR_Z = -3.0, -1.0
SONAR_HEIGHT = SONAR_Z - FLOOR_Z
TILT_DEG = 22.0
RADIUS, HALF_LENGTH = 0.12, 0.70
YAW_DEG = 35.0                      # the cylinder's axis, away from broadside
CENTRE = np.array([0.30, 4.30, FLOOR_Z + RADIUS])
ROCK = np.array([-0.95, 4.05, FLOOR_Z + 0.05])

position = np.array([0.0, 0.0, SONAR_Z])
axes = sc.tilted_axes(TILT_DEG)
yaw = np.radians(YAW_DEG)
cylinder_axis = (np.cos(yaw), np.sin(yaw), 0.0)   # lying flat, yawed off broadside


def make_scene(radius, textured, seed=7):
    floor_texture = dict(texture_amplitude=0.55, texture_scale_m=0.30,
                         texture_seed=seed) if textured else {}
    body_texture = dict(texture_amplitude=0.25, texture_scale_m=0.10,
                        texture_seed=seed + 3) if textured else {}
    centre = np.array([CENTRE[0], CENTRE[1], FLOOR_Z + radius])
    return [sonar.make_plane((0.0, 0.0, FLOOR_Z), (0.0, 0.0, 1.0), 0.05, **floor_texture),
            sonar.make_cylinder(centre, cylinder_axis, radius, HALF_LENGTH, 0.85,
                                **body_texture),
            sonar.make_sphere(ROCK, 0.16, 0.35, **body_texture)]


def simulator():
    return sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=385,
                                num_range_bins=900, horizontal_fov_deg=34.0,
                                vertical_beamwidth_deg=16.0, max_range_m=9.0,
                                num_elevation_subrays=3072)


def horizontal_distance(slant_range):
    """Along-seabed distance of a bin, for a sonar SONAR_HEIGHT above the floor."""
    return np.sqrt(np.maximum(slant_range ** 2 - SONAR_HEIGHT ** 2, 0.0))


# Where the seabed is read for a reference level. It has to sit beyond the
# longest shadow any object in the sweep can throw, or the reference is taken
# from inside a shadow and the threshold collapses.
BACKGROUND_BAND = (6.6, 8.4)
GAP_BINS = 6


def measure_shadow(profile, ranges, search_from, search_to):
    """Far edge of the highlight and end of the dark run that follows it.

    The highlight is looked for in a narrow window, but the dark run is followed
    to the end of the record. Bounding the run by the same window is what clipped
    the tallest object short: its shadow ended past the window edge, so the run
    stopped at the edge rather than at the seabed coming back.
    """
    window = (ranges > search_from) & (ranges < search_to)
    peak = ranges[window][np.argmax(profile[window])]

    background = np.median(profile[(ranges > BACKGROUND_BAND[0]) &
                                   (ranges < BACKGROUND_BAND[1])])
    threshold = 0.35 * background

    after = ranges > peak
    axis = ranges[after]
    dark = profile[after] < threshold
    if not dark.any():
        return peak, peak, peak

    start = int(np.argmax(dark))
    # A single lit bin inside the shadow, off a sidelobe or a speckle draw, must
    # not end the run. It ends where the seabed stays back for GAP_BINS in a row.
    index = start
    while index < len(dark):
        if dark[index]:
            index += 1
            continue
        if (~dark[index:index + GAP_BINS]).all():
            break
        index += 1
    return peak, axis[start], axis[min(index, len(axis) - 1) - 1]


sim = simulator()
clean = sim.render(make_scene(RADIUS, textured=False), position=position, axes=axes)
rough = sim.render(make_scene(RADIUS, textured=True), position=position, axes=axes,
                   speckle=True, seed=11)

ranges = sim.range_axis_m()
bearings = sim.azimuth_axis_deg()
target_bearing = np.degrees(np.arctan2(CENTRE[0], CENTRE[1]))
column = int(np.argmin(np.abs(bearings - target_bearing)))
profile = clean[column]

peak, highlight_edge, shadow_end = measure_shadow(profile, ranges, 3.5, 5.2)
d_object = horizontal_distance(highlight_edge)
d_end = horizontal_distance(shadow_end)
shadow_length = d_end - d_object
height = SONAR_HEIGHT * shadow_length / (d_object + shadow_length)

print(f"sonar {SONAR_HEIGHT:.2f} m above the seabed, tilt {TILT_DEG}deg, "
      f"cylinder radius {RADIUS} m so its top stands {2 * RADIUS:.3f} m proud")
print(f"  highlight peak      {peak:.3f} m slant")
print(f"  highlight far edge  {highlight_edge:.3f} m slant -> {d_object:.3f} m along the floor")
print(f"  shadow ends         {shadow_end:.3f} m slant -> {d_end:.3f} m along the floor")
print(f"  shadow length       {shadow_length:.3f} m")
print(f"  height from shadow  {height:.3f} m   (true {2 * RADIUS:.3f} m, "
      f"error {100 * (height - 2 * RADIUS):+.1f} cm)")

print(f"\n  {'true height':>12} {'shadow (m)':>11} {'recovered':>10} {'error cm':>9} {'ratio':>7}")
radii, truths, recovered = [0.06, 0.09, 0.12, 0.17, 0.23], [], []
for radius in radii:
    frame = sim.render(make_scene(radius, textured=False), position=position, axes=axes)
    _, edge, end = measure_shadow(frame[column], ranges, 3.5, 5.2)
    d_o, d_e = horizontal_distance(edge), horizontal_distance(end)
    length = d_e - d_o
    estimate = SONAR_HEIGHT * length / (d_o + length)
    truths.append(2 * radius)
    recovered.append(estimate)
    print(f"  {2 * radius:12.3f} {length:11.3f} {estimate:10.3f} "
          f"{100 * (estimate - 2 * radius):9.1f} {estimate / (2 * radius):7.3f}")

# The estimate is high by a near-constant factor rather than by a random amount.
# The highlight's far edge is where the flank stops returning, which sits short of
# the top tangent that actually casts the shadow, by a fraction of the radius that
# drifts slowly with size. Removing that as a single slope leaves sub-millimetre
# residuals over a four-fold range of heights, which is what a calibrated detector
# would use and is a stronger statement than a small raw error.
slope, intercept = np.polyfit(truths, recovered, 1)
residuals = np.array(recovered) - (slope * np.array(truths) + intercept)
print(f"\n  fitted slope {slope:.4f}, intercept {1000 * intercept:+.2f} mm")
print(f"  residual about that line: max {1000 * np.abs(residuals).max():.2f} mm, "
      f"rms {1000 * np.sqrt((residuals ** 2).mean()):.2f} mm")

fig = plt.figure(figsize=(16.0, 8.6), constrained_layout=True)
grid = fig.add_gridspec(2, 3)

for index, (frame, title) in enumerate([
        (clean, "uniform reflectivity, no speckle"),
        (rough, "textured backscatter and single-look speckle")]):
    ax = fig.add_subplot(grid[index, 0:2])
    sc.show_image(ax, frame, sim, floor_db=-42)
    # The measured extent, marked on the bearing it was measured on rather than
    # banded across the whole fan.
    ax.plot([highlight_edge, shadow_end], [target_bearing, target_bearing], "-",
            color="#e8735c", linewidth=2.0, solid_capstyle="butt")
    for edge in (highlight_edge, shadow_end):
        ax.plot([edge, edge], [target_bearing - 1.4, target_bearing + 1.4], "-",
                color="#e8735c", linewidth=1.6)
    ax.annotate("highlight", xy=(peak, target_bearing), xytext=(peak - 1.0, target_bearing + 9),
                color="#f0f0f0", fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#f0f0f0", linewidth=1.0))
    ax.annotate("acoustic shadow", xy=(0.5 * (highlight_edge + shadow_end), target_bearing),
                xytext=(shadow_end + 0.35, target_bearing - 11),
                color="#e8735c", fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#e8735c", linewidth=1.0))
    ax.annotate("rock", xy=(4.4, np.degrees(np.arctan2(ROCK[0], ROCK[1]))),
                xytext=(3.0, -15.0), color="#9fd4b0", fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#9fd4b0", linewidth=1.0))
    ax.set(xlim=(3.0, 7.0), xlabel="range (m)", ylabel="azimuth (deg)", title=title)

ax = fig.add_subplot(grid[0, 2])
ax.semilogy(ranges, np.maximum(profile, 1e-30), color="#1f6f8b", linewidth=1.3,
            label="clean")
ax.semilogy(ranges, np.maximum(rough[column], 1e-30), color="#e8735c", linewidth=0.8,
            alpha=0.75, label="textured + speckle")
ax.axvspan(highlight_edge, shadow_end, color="#cccccc", alpha=0.45)
ax.set(xlim=(3.2, 6.4), ylim=(profile.max() * 1e-6, profile.max() * 6),
       xlabel="range (m)", ylabel="intensity",
       title=f"A-scan on the target bearing ({target_bearing:+.1f}$\\degree$)")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[1, 2])
ax.plot(truths, recovered, "o", color="#2a8f5f", markersize=7, label="from the shadow")
limits = [0.0, max(truths) * 1.15]
ax.plot(limits, [slope * value + intercept for value in limits], "-", color="#2a8f5f",
        linewidth=1.4,
        label=f"fit, slope {slope:.3f}, residual {1000 * np.abs(residuals).max():.1f} mm")
ax.plot(limits, limits, "--", color="#8a8a8a", linewidth=1.2, label="truth")
ax.set(xlabel="true height above the seabed (m)", ylabel="height from shadow length (m)",
       xlim=limits, ylim=limits, aspect="equal",
       title="$h_o = h_s L / (d_o + L)$, checked against truth")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

fig.suptitle("Target signature: a bright flank, a black wedge, and the height the wedge "
             "gives back", fontsize=13)
figure_path = outputs.output_path("demo9_target.png")
fig.savefig(figure_path, dpi=130)
print(f"\nwrote {figure_path}")
