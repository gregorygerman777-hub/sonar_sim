"""Demo 8: an optical camera puts back the elevation the sonar threw away.

A sonar bin is a range and a bearing. Every point on the arc

    P(phi) = C_s + R * A_s [cos(phi) sin(theta), cos(phi) cos(theta), sin(phi)]

for phi anywhere inside the vertical beamwidth produces exactly that bin, which
is the ambiguity demo 1 measures. A camera has the opposite defect: a pixel
fixes two angles and no range at all, and in turbid water it fixes nothing.

Project the sonar's arc into the camera and it becomes a curve across the image.
Where that curve meets the pixel the target was detected in, both constraints
are satisfied at once and the 3-D point is determined. The sonar supplies the
range the camera lacks, the camera supplies the elevation the sonar lacks, and
neither sensor is asked for anything it cannot measure.

The turbidity sweep at the end is the other half of the argument: the optical
term decays as exp(-2 c r) while the veiling backscatter grows as 1 - exp(-c r),
so past a few attenuation lengths the camera detection dies and the pair falls
back to the sonar's arc. That is the operating regime that makes acoustics
necessary in the first place.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import outputs
import recovery
import scene as sc
import sonar

# Scene, sonar and camera all come from recovery.py, which the Monte Carlo layer
# also uses, so a single run and a repeated trial cannot describe different scenes.
FLOOR_Z = recovery.FLOOR_Z
SONAR_POSITION = recovery.SONAR_POSITION
CAMERA_OFFSET = recovery.CAMERA_OFFSET
TILT_DEG = recovery.TILT_DEG
TARGET_RANGE = recovery.TARGET_RANGE
TARGET_AZIMUTH = recovery.TARGET_AZIMUTH
TARGET_ELEVATION = recovery.TARGET_ELEVATION
TARGET_RADIUS = recovery.TARGET_RADIUS

axes = recovery.scene_axes()
centre = recovery.target_centre(axes)
objects = recovery.build_objects(centre)
sim = recovery.build_sonar()
image = sim.render(objects, position=SONAR_POSITION, axes=axes)


detected_range, detected_bearing = recovery.sonar_detection(image, sim)
near_surface = float(np.linalg.norm(centre - SONAR_POSITION)) - TARGET_RADIUS
print(f"sonar detection   range {detected_range:.4f} m, bearing {detected_bearing:+.3f} deg")
print(f"  the sonar sees the near surface, not the centre: "
      f"|C - S| - r = {near_surface:.4f} m")
print(f"  arc length spanned by the 14 deg beam at that range: "
      f"{detected_range * np.radians(14.0):.3f} m  <- what one bin alone leaves undetermined")

clear = recovery.build_camera(axes, baseline_m=CAMERA_OFFSET[2], attenuation_per_m=0.08)
frame = clear.render(objects)
pixel, contrast, mask = recovery.camera_detection(frame)
print(f"camera detection  pixel ({pixel[0]:.2f}, {pixel[1]:.2f}), contrast {contrast:.2f}")

# The sonar measures the near surface while the camera centroids the whole disc,
# so the two detections are one radius apart along the line of sight. Adding the
# known radius back is the same correction a detector applies once it has decided
# what it is looking at; both numbers are reported so the bias is visible.
for label, arc_range in (("raw", detected_range), ("radius corrected",
                                                    detected_range + TARGET_RADIUS)):
    (residual, elevation, point), track = recovery.fuse(arc_range, detected_bearing, pixel, clear, SONAR_POSITION, axes)
    error = float(np.linalg.norm(point - centre))
    print(f"fused, {label:16} elevation {elevation:+.3f} deg "
          f"(true {TARGET_ELEVATION:+.3f}), pixel residual {residual:.2f} px, "
          f"3-D error {100 * error:.1f} cm")

(residual, elevation, point), track = recovery.fuse(detected_range + TARGET_RADIUS,
                                           detected_bearing, pixel, clear,
                                           SONAR_POSITION, axes)

print(f"\n  {'c (1/m)':>8} {'contrast':>9} {'detected':>9} {'elevation':>10} {'error deg':>10}")
turbidities, errors, frames = [0.02, 0.08, 0.2, 0.4, 0.7, 1.1], [], []
for c in turbidities:
    camera = recovery.build_camera(axes, baseline_m=CAMERA_OFFSET[2], attenuation_per_m=c)
    murky = camera.render(objects)
    frames.append(murky)
    spot, murky_contrast, _ = recovery.camera_detection(murky)
    # Below a few percent of contrast the brightest blob is no longer the target,
    # it is whatever the veiling glow happened to land on.
    found = murky_contrast > 0.05
    if found:
        (_, murky_elevation, _), _ = recovery.fuse(detected_range + TARGET_RADIUS,
                                          detected_bearing, spot, camera,
                                          SONAR_POSITION, axes)
        errors.append(abs(murky_elevation - TARGET_ELEVATION))
    else:
        murky_elevation, _ = np.nan, None
        errors.append(np.nan)
    print(f"  {c:8.2f} {murky_contrast:9.3f} {str(found):>9} {murky_elevation:10.3f} "
          f"{errors[-1]:10.3f}")

arc_length_cm = 100.0 * (detected_range + TARGET_RADIUS) * np.radians(14.0)
uncertainty = []
for c, error_deg in zip(turbidities, errors):
    # Detected: the residual localisation error. Lost: the whole arc is still open,
    # which is exactly where a sonar-only system sits at every range and bearing.
    uncertainty.append(arc_length_cm if np.isnan(error_deg)
                       else 100.0 * np.radians(error_deg) * (detected_range + TARGET_RADIUS))

fig = plt.figure(figsize=(16.0, 8.8), constrained_layout=True)
grid = fig.add_gridspec(2, 6)

ax = fig.add_subplot(grid[0, 0:2])
sc.show_image(ax, image, sim, floor_db=-45)
ax.plot(detected_range, detected_bearing, "o", markerfacecolor="none",
        markeredgecolor="#e8735c", markersize=13)
ax.set(xlim=(2.5, 6.0), xlabel="range (m)", ylabel="azimuth (deg)",
       title=f"sonar: one bin, {detected_range:.2f} m at {detected_bearing:+.2f}$\\degree$")

ax = fig.add_subplot(grid[0, 2:4])
ax.imshow(frame ** 0.45, cmap="bone", origin="upper")
ax.plot(track[:, 0], track[:, 1], "-", color="#35b4c4", linewidth=2.0,
        label="sonar ambiguity arc")
ax.plot(*pixel, "+", color="#e8735c", markersize=16, markeredgewidth=2.2,
        label="camera detection")
ax.plot(*clear.project(point), "o", markerfacecolor="none", markeredgecolor="#c8a020",
        markersize=15, markeredgewidth=1.8, label="intersection")
ax.legend(loc="lower left", fontsize=8, framealpha=0.75)
ax.set(title="camera, c = 0.08 /m, with the arc drawn in", xticks=[], yticks=[])

ax = fig.add_subplot(grid[0, 4:6])
elevations = np.linspace(-7.0, 7.0, 400)
arc = np.array([SONAR_POSITION + axes @ sc.spherical_to_world(
    detected_range + TARGET_RADIUS, detected_bearing, e) for e in elevations])
ax.plot(arc[:, 1], arc[:, 2], color="#35b4c4", linewidth=2.5,
        label=f"consistent with the bin ({arc_length_cm:.0f} cm of arc)")
ax.plot(centre[1], centre[2], "o", color="#e8735c", markersize=9, label="truth")
ax.plot(point[1], point[2], "x", color="#c8a020", markersize=13, markeredgewidth=2.4,
        label=f"fused ({100 * float(np.linalg.norm(point - centre)):.1f} cm out)")
ax.set(xlabel="forward Y (m)", ylabel="height Z (m)", aspect="equal",
       title="side view of the ambiguity the bin leaves open")
ax.legend(fontsize=8, loc="lower left")
ax.grid(alpha=0.25)

for column, index in enumerate((1, 3, 5)):
    ax = fig.add_subplot(grid[1, column])
    ax.imshow(frames[index] ** 0.45, cmap="bone", origin="upper")
    ax.set(title=f"c = {turbidities[index]} /m", xticks=[], yticks=[])

ax = fig.add_subplot(grid[1, 3:6])
detected_mask = ~np.isnan(np.array(errors, dtype=float))
ax.semilogy(np.array(turbidities)[detected_mask], np.array(uncertainty)[detected_mask],
            "o-", color="#2a8f5f", linewidth=2.0, label="fused, camera holds the target")
ax.semilogy(np.array(turbidities)[~detected_mask], np.array(uncertainty)[~detected_mask],
            "s--", color="#e8735c", linewidth=2.0, label="camera lost, sonar arc is all there is")
ax.axhline(arc_length_cm, color="#8a8a8a", linestyle=":", linewidth=1.4)
ax.text(0.02, arc_length_cm * 1.15, f"sonar alone: {arc_length_cm:.0f} cm", fontsize=9,
        color="#555555")
ax.set(xlabel="beam attenuation c (1/m)", ylabel="localisation error (cm)",
       title="what the pair knows about the target's height")
ax.legend(fontsize=8, loc="center left")
ax.grid(alpha=0.25, which="both")

fig.suptitle("Opti-acoustic fusion: the sonar's range crossed with the camera's elevation",
             fontsize=13)
figure_path = outputs.output_path("demo8_optiacoustic.png")
fig.savefig(figure_path, dpi=130)
print(f"\nwrote {figure_path}")
