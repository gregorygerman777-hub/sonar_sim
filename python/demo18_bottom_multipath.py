"""Demo 18: seabed multipath and the critical grazing angle.

The surface path treats the boundary as an ideal pressure-release mirror
(amplitude -1, roughness aside). A seabed is a real fluid, so its coefficient
depends on the density and sound-speed contrast, and below a critical grazing
angle theta_c = acos(c_water / c_bottom) the boundary reflects totally: the
same cutoff BELLHOP's ray trace shows for a fast bottom. See the model in
core/physics.h (bottom_reflection_coefficient) and its use in
core/simulator.cpp (deposit_boundary), which is what gives the seabed its own
ghost/mirror pair alongside the sea-surface one.

This sweeps the *sediment sound speed* rather than the geometry, with the
sonar, target and boundary all fixed. That keeps every geometric factor in
the rendered ghost (range, spreading, absorption) exactly constant across the
sweep, so the measured energy is a clean, quantitative stand-in for the
reflection coefficient squared: dividing by the deep-total-reflection value
should reproduce the closed-form curve, not just resemble it, the same
standard the plane-intensity and roughness checks elsewhere hold to.

The target is a plane facing the sonar, not a sphere: a single boresight ray
(num_elevation_subrays=1, num_azimuth_bins=1) then hits it at an exact,
hand-computable point, so the grazing angle used for the "theory" curve below
is not an approximation of what the renderer actually sees.
"""
import sys, json
sys.path[:0] = [".", "python"]
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import sonar, outputs

WATER_SPEED, WATER_RHO = 1500.0, 1000.0
BOTTOM_RHO = 1900.0

POSITION = (0.0, 0.0, -1.2)
WALL_Y = 4.0
BOTTOM_Z = -3.0
BASE = dict(num_azimuth_bins=1, num_range_bins=600, max_range_m=8.0,
           num_elevation_subrays=1, horizontal_fov_deg=2.0, vertical_beamwidth_deg=2.0,
           direct_enabled=False, bottom_mirror_enabled=False, num_threads=1,
           bottom_z=BOTTOM_Z, water_density_kgm3=WATER_RHO, bottom_density_kgm3=BOTTOM_RHO,
           speed_of_sound_mps=WATER_SPEED)
target_objects = [sonar.make_plane((0, WALL_Y, 0), (0, -1, 0), 0.85)]

# The boresight ray from POSITION runs along +y at fixed x=0, z=POSITION[2],
# so it hits the wall at exactly (0, WALL_Y, POSITION[2]) -- no curvature or
# beam-averaging to approximate away, unlike a sphere target would need.
point = np.array([0.0, WALL_Y, POSITION[2]])
position = np.array(POSITION)
mirrored_sonar = np.array([position[0], position[1], 2 * BOTTOM_Z - position[2]])
bounced = float(np.linalg.norm(mirrored_sonar - point))
rise = abs(2 * BOTTOM_Z - point[2] - position[2])
grazing_rad = np.arcsin(rise / bounced)
critical_speed = WATER_SPEED / np.cos(grazing_rad)
print(f"fixed grazing angle: {np.degrees(grazing_rad):.3f} deg "
      f"-> critical bottom speed {critical_speed:.1f} m/s")

bottom_speeds = np.linspace(critical_speed - 260, critical_speed + 260, 45)
measured, theory = [], []
for c2 in bottom_speeds:
    sim = sonar.SonarSimulator(**BASE, bottom_enabled=True, bottom_speed_mps=float(c2))
    measured.append(float(sim.render(target_objects, POSITION).sum()))
    theory.append(sonar.bottom_reflection(grazing_rad, WATER_SPEED, float(c2), WATER_RHO, BOTTOM_RHO) ** 2)
measured, theory = np.array(measured), np.array(theory)

# Both curves saturate at 1 (total reflection) once the bottom is enough
# faster than water to pass critical -- cos(theta2) = (c2/c1)cos(grazing) >= 1
# -- so that plateau (c2 above critical, not below) is the reference level for
# both, with no free scale factor.
deep = bottom_speeds > critical_speed + 180
measured_norm = measured / measured[deep].mean()
above_floor = theory > 1e-6
relative_error = float(np.max(np.abs(measured_norm - theory)[above_floor] / theory[above_floor]))
print(f"max relative error, measured vs. closed-form R^2: {relative_error:.3e}")

fig, ax = plt.subplots(figsize=(7.5, 4.5), constrained_layout=True)
ax.plot(bottom_speeds, theory, "-", color="#2c7fb8", linewidth=2, label="theory: R(grazing)^2")
ax.plot(bottom_speeds, measured_norm, "o", color="#e8735c", ms=4, label="measured: rendered ghost energy")
ax.axvline(critical_speed, color="#444", linestyle="--", linewidth=1,
          label=f"critical speed {critical_speed:.0f} m/s")
ax.set(xlabel="bottom compressional speed (m/s)", ylabel="reflected energy / total-reflection energy",
      title=f"Seabed critical angle at fixed grazing = {np.degrees(grazing_rad):.2f} deg")
ax.legend(fontsize=9)
ax.grid(alpha=0.25)
figure_path = outputs.output_path("demo18_bottom_multipath.png")
fig.savefig(figure_path, dpi=140)

report = dict(grazing_deg=float(np.degrees(grazing_rad)), critical_speed_mps=float(critical_speed),
             bottom_speeds_mps=bottom_speeds.tolist(), measured_normalized=measured_norm.tolist(),
             theory_r_squared=theory.tolist(), max_relative_error=relative_error,
             note="Bottom speed swept at fixed geometry, so geometric spreading/absorption are "
                  "constant and the ratio isolates the reflection-coefficient model.")
with open(outputs.output_path("demo18_metrics.json"), "w") as f:
    json.dump(report, f, indent=2)
print(f"wrote {figure_path}")

assert relative_error < 0.01, f"measured seabed reflection does not track theory: {relative_error}"
print(json.dumps(dict(grazing_deg=report["grazing_deg"], critical_speed_mps=report["critical_speed_mps"],
                      max_relative_error=relative_error), indent=2))
