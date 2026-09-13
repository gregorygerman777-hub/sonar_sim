"""Unit tests against analytic values worked out by hand.

Run directly; exits non-zero on any failure.
"""

import math
import sys

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import acoustics
import sonar

failures = []


def check(name, got, want, tolerance):
    ok = abs(got - want) <= tolerance
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<46} got {got:.9g}  want {want:.9g}")
    if not ok:
        failures.append(name)


print("beam pattern B(phi)")
LAMBDA, ELEMENTS = 1.0, 64
spacing = 0.5 * LAMBDA
check("B(0) = 1 on boresight", sonar.array_beam_pattern(0.0, ELEMENTS, spacing, LAMBDA), 1.0, 1e-12)

# First null where N*d*sin(phi)/lambda = 1, so sin(phi) = lambda/(N*d) = 2/64.
null_phi = math.asin(LAMBDA / (ELEMENTS * spacing))
check("B = 0 at the first null", sonar.array_beam_pattern(null_phi, ELEMENTS, spacing, LAMBDA),
      0.0, 1e-16)
check("B is even in phi",
      sonar.array_beam_pattern(0.03, ELEMENTS, spacing, LAMBDA) -
      sonar.array_beam_pattern(-0.03, ELEMENTS, spacing, LAMBDA), 0.0, 1e-15)

print("\nThorp absorption, dB/km")
# f = 100 kHz: 0.11*1e4/1.0001e4 + 44*1e4/1.41e4 + 2.75e-4*1e4 + 0.003
check("alpha at 100 kHz", sonar.thorp_alpha(100e3), 34.0686628, 1e-6)
check("alpha at 10 kHz", sonar.thorp_alpha(10e3), 1.1870299, 1e-6)
check("alpha at 1.8 MHz", sonar.thorp_alpha(1.8e6),
      0.11 * 1800**2 / (1 + 1800**2) + 44 * 1800**2 / (4100 + 1800**2)
      + 2.75e-4 * 1800**2 + 0.003, 1e-9)

print("\ntwo-way transmission loss, dB")
check("TL at 100 m for alpha = 34.0687", sonar.two_way_loss_db(34.0686628, 100.0),
      2 * 34.0686628 * 0.1, 1e-12)

print("\nray-sphere intersection")
check("sphere at y = 5 radius 1, ray along +y",
      sonar.ray_sphere((0, 5, 0), 1.0, 0.5, (0, 0, 0), (0, 1, 0)), 4.0, 1e-12)
check("same sphere, ray along -y misses",
      -1.0 if sonar.ray_sphere((0, 5, 0), 1.0, 0.5, (0, 0, 0), (0, -1, 0)) is None else 0.0,
      -1.0, 1e-12)
# Offset ray: closest approach 0.6, half-chord sqrt(1 - 0.36) = 0.8, so t = 5 - 0.8.
check("offset ray, t = 5 - sqrt(1 - 0.6^2)",
      sonar.ray_sphere((0, 5, 0), 1.0, 0.5, (0.6, 0, 0), (0, 1, 0)),
      5.0 - math.sqrt(1 - 0.36), 1e-12)

print("\nray-plane intersection")
check("plane z = -2, ray straight down",
      sonar.ray_plane((0, 0, -2), (0, 0, 1), 0.05, (0, 0, 0), (0, 0, -1)), 2.0, 1e-12)
check("plane y = 10 facing the sonar, ray along +y",
      sonar.ray_plane((0, 10, 0), (0, -1, 0), 0.05, (0, 0, 0), (0, 1, 0)), 10.0, 1e-12)
# 45 degrees down onto a plane 3 m below: slant range 3*sqrt(2).
check("plane z = -3, ray at 45 degrees",
      sonar.ray_plane((0, 0, -3), (0, 0, 1), 0.05, (0, 0, 0),
                      (0, math.sqrt(0.5), -math.sqrt(0.5))), 3 * math.sqrt(2), 1e-12)
check("plane behind the ray is not hit",
      -1.0 if sonar.ray_plane((0, 0, 2), (0, 0, 1), 0.05, (0, 0, 0), (0, 0, -1)) is None else 0.0,
      -1.0, 1e-12)

print("\nray-cylinder intersection")
# Axis across the line of sight: the wall is a circle of radius 1 about y = 5 at
# every station inside the length, so the range does not depend on the offset.
check("cylinder broadside, ray down the middle",
      sonar.ray_cylinder((0, 5, 0), (1, 0, 0), 1.0, 2.0, 0.8, (0, 0, 0), (0, 1, 0)),
      4.0, 1e-12)
check("cylinder broadside, ray 1.5 m along the axis",
      sonar.ray_cylinder((0, 5, 0), (1, 0, 0), 1.0, 2.0, 0.8, (1.5, 0, 0), (0, 1, 0)),
      4.0, 1e-12)
check("ray past the end of the cylinder misses",
      -1.0 if sonar.ray_cylinder((0, 5, 0), (1, 0, 0), 1.0, 2.0, 0.8,
                                 (2.5, 0, 0), (0, 1, 0)) is None else 0.0, -1.0, 1e-12)
# Axis pointing at the sonar: the near end cap sits at y = 5 - 1.
check("end cap of an end-on cylinder",
      sonar.ray_cylinder((0, 5, 0), (0, 1, 0), 0.5, 1.0, 0.8, (0, 0, 0), (0, 1, 0)),
      4.0, 1e-12)
check("end cap hit 0.3 m off the axis",
      sonar.ray_cylinder((0, 5, 0), (0, 1, 0), 0.5, 1.0, 0.8, (0.3, 0, 0), (0, 1, 0)),
      4.0, 1e-12)
check("ray outside the cap radius misses",
      -1.0 if sonar.ray_cylinder((0, 5, 0), (0, 1, 0), 0.5, 1.0, 0.8,
                                 (0.7, 0, 0), (0, 1, 0)) is None else 0.0, -1.0, 1e-12)
# Vertical axis, ray horizontal: side wall again, radius 1 at y = 5.
check("upright cylinder, horizontal ray",
      sonar.ray_cylinder((0, 5, 0), (0, 0, 1), 1.0, 10.0, 0.8, (0, 0, 0), (0, 1, 0)),
      4.0, 1e-12)

print("\nbackscatter texture")
check("amplitude 0 leaves reflectivity untouched",
      sonar.surface_texture((0.3, 1.7, -2.1), 0.0), 1.0, 0.0)
check("the same point gives the same value",
      sonar.surface_texture((0.3, 1.7, -2.1), 0.6) -
      sonar.surface_texture((0.3, 1.7, -2.1), 0.6), 0.0, 0.0)
samples = [sonar.surface_texture((0.137 * i, -0.211 * i, 0.317 * i), 0.6, 0.25, 4)
           for i in range(4000)]
check("mean is 1 over many points", sum(samples) / len(samples), 1.0, 0.02)
check("stays inside 1 +- amplitude",
      1.0 if all(0.4 - 1e-12 <= v <= 1.6 + 1e-12 for v in samples) else 0.0, 1.0, 0.0)

print("\nbottom reflection coefficient (Rayleigh two-fluid)")


def rayleigh_r(grazing_rad, c1, c2, rho1, rho2):
    """Independent re-derivation of physics.h's formula, worked by hand from
    Snell's law in grazing form (cos t1 / c1 = cos t2 / c2) and impedance
    Z_i = rho_i c_i."""
    cos_t2 = (c2 / c1) * math.cos(grazing_rad)
    if cos_t2 >= 1.0:
        return 1.0
    sin_t2 = math.sqrt(1.0 - cos_t2 ** 2)
    z1, z2 = rho1 * c1, rho2 * c2
    return (z2 * math.sin(grazing_rad) - z1 * sin_t2) / (z2 * math.sin(grazing_rad) + z1 * sin_t2)


C1, RHO1 = 1500.0, 1000.0
C2, RHO2 = 1650.0, 1900.0  # sand: faster and denser than water
check("normal incidence (grazing 90 deg) matches (Z2-Z1)/(Z2+Z1)",
      sonar.bottom_reflection(math.pi / 2, C1, C2, RHO1, RHO2),
      (RHO2 * C2 - RHO1 * C1) / (RHO2 * C2 + RHO1 * C1), 1e-12)
check("matches an independent re-derivation at 35 deg grazing",
      sonar.bottom_reflection(math.radians(35), C1, C2, RHO1, RHO2),
      rayleigh_r(math.radians(35), C1, C2, RHO1, RHO2), 1e-12)

critical_deg = math.degrees(math.acos(C1 / C2))
check("below the critical grazing angle: total reflection",
      sonar.bottom_reflection(math.radians(critical_deg - 5.0), C1, C2, RHO1, RHO2), 1.0, 1e-12)
check("just above the critical grazing angle: still close to 1 (continuity)",
      sonar.bottom_reflection(math.radians(critical_deg + 0.5), C1, C2, RHO1, RHO2), 1.0, 0.05)
check("well above critical: partial reflection, |R| < 1",
      1.0 if abs(sonar.bottom_reflection(math.radians(80.0), C1, C2, RHO1, RHO2)) < 1.0 else 0.0,
      1.0, 0.0)

# A bottom slower than water (soft mud) has no critical angle: Snell's law
# always has a real solution, so the coefficient should never saturate at 1.
check("slower bottom (no critical angle) stays a genuine partial reflection",
      1.0 if abs(sonar.bottom_reflection(math.radians(5.0), C1, 1400.0, RHO1, 1400.0)) < 1.0 else 0.0,
      1.0, 0.0)

print("\nroughness coherence factor")
K = 2 * math.pi / 0.05  # wavenumber at 3 cm wavelength
check("flat interface (sigma=0) leaves it at 1", sonar.roughness_factor(K, 0.0, 0.7), 1.0, 1e-15)
sigma, sin_g = 0.002, 0.8
expected_gamma = math.exp(-0.5 * (2 * K * sigma * sin_g) ** 2)
check("matches exp(-Ra^2/2) worked by hand", sonar.roughness_factor(K, sigma, sin_g),
      expected_gamma, 1e-12)

print("\nreflected-leg occlusion")
# A target sits above a wall; the straight line up to the mirrored sonar (i.e.
# the folded path back down through the wall) has to cross the wall, so the
# segment from the target to a point above the wall must register as blocked,
# while a segment that stays on the near side must not.
wall = [sonar.make_plane((0, 3, 0.0), (0, -1, 0), 0.05)]  # a wall facing -y at y=3
check("segment straight through the wall is blocked",
      1.0 if sonar.segment_blocked(wall, (0, 1, 0), (0, 5, 0)) else 0.0, 1.0, 0.0)
check("segment that never reaches the wall is not blocked",
      1.0 if sonar.segment_blocked(wall, (0, 1, 0), (0, 2, 0)) else 0.0, 0.0, 0.0)
check("segment starting exactly on the wall does not occlude itself",
      1.0 if sonar.segment_blocked(wall, (0, 3, 0), (0, 5, 0)) else 0.0, 0.0, 0.0)

print("\npinhole camera")
camera = sonar.OpticalCamera(width=320, height=240, focal_px=200.0, position=(0, 0, 0))
check("a point on the optical axis lands at the centre",
      camera.project((0, 10, 0))[0] - 160.0, 0.0, 1e-12)
# u = w/2 + f x / y = 160 + 200 * 1 / 10.
check("1 m to starboard at 10 m is 20 px right", camera.project((1, 10, 0))[0], 180.0, 1e-12)
# v = h/2 - f z / y = 120 - 200 * 2 / 10, rows counting downward.
check("2 m up at 10 m is 40 px up", camera.project((0, 10, 2))[1], 80.0, 1e-12)
check("a point behind the pinhole does not project",
      -1.0 if camera.project((0, -10, 0)) is None else 0.0, -1.0, 1e-12)
# Back-project a pixel, walk 7 m down the ray, project it again.
direction = camera.ray(230.0, 60.0)
round_trip = camera.project(tuple(7.0 * component for component in direction))
check("pixel -> ray -> pixel round trip, u", round_trip[0], 230.0, 1e-9)
check("pixel -> ray -> pixel round trip, v", round_trip[1], 60.0, 1e-9)

print("\nbearing threading")
# The threaded render splits bearings across workers, and with multipath on a
# bearing can deposit into a neighbour's column. Both cases have to come back
# bit for bit identical to the single-threaded result or the parallelism is
# quietly changing the physics.
scene_objects = [sonar.make_plane((0, 0, -2.0), (0, 0, 1), 0.06,
                                  texture_amplitude=0.5, texture_scale_m=0.3),
                 sonar.make_sphere((0.2, 4.0, -1.4), 0.25, 0.9),
                 sonar.make_cylinder((-0.6, 5.0, -1.85), (1, 0.4, 0), 0.15, 0.6, 0.85)]
for multipath in (False, True):
    frames = [sonar.SonarSimulator(num_azimuth_bins=97, num_range_bins=300, max_range_m=8.0,
                                   num_elevation_subrays=257, multipath_enabled=multipath,
                                   surface_z=0.0, num_threads=threads)
              .render(scene_objects, position=(0, 0, -0.5))
              for threads in (1, 8)]
    difference = float(abs(frames[0] - frames[1]).max())
    check(f"1 thread == 8 threads, multipath {multipath}", difference, 0.0, 0.0)

# Same check with the bottom boundary in play too: it deposits into neighbour
# columns exactly like the surface path, so it needs the same private-
# accumulator treatment, and this is what would catch it if it didn't.
for bottom in (False, True):
    frames = [sonar.SonarSimulator(num_azimuth_bins=97, num_range_bins=300, max_range_m=8.0,
                                   num_elevation_subrays=257, multipath_enabled=True,
                                   bottom_enabled=bottom, surface_z=0.0, bottom_z=-3.0,
                                   num_threads=threads)
              .render(scene_objects, position=(0, 0, -0.5))
              for threads in (1, 8)]
    difference = float(abs(frames[0] - frames[1]).max())
    check(f"1 thread == 8 threads, bottom multipath {bottom}", difference, 0.0, 0.0)

print("\nbottom multipath: enabled adds energy, and occlusion removes it")
sphere_objects = [sonar.make_sphere((0, 4.0, -1.0), 0.3, 0.85)]
base = dict(num_azimuth_bins=61, num_range_bins=400, max_range_m=8.0,
           num_elevation_subrays=193, horizontal_fov_deg=30.0, vertical_beamwidth_deg=20.0,
           direct_enabled=False, num_threads=1, bottom_z=-3.0, bottom_rms_height_m=0.0)
position = (0, 0, -1.2)

off = sonar.SonarSimulator(**base, bottom_enabled=False).render(sphere_objects, position)
on = sonar.SonarSimulator(**base, bottom_enabled=True).render(sphere_objects, position)
check("bottom multipath off carries no energy with direct_enabled=False",
      float(off.sum()), 0.0, 0.0)
check("bottom multipath on carries some energy", 1.0 if on.sum() > 0.0 else 0.0, 1.0, 0.0)

# A wide floor plane between the sphere and the bottom boundary blocks the
# reflected leg for every sub-ray: the same scene, same boundary, but now the
# path down to z = -3 has to pass through solid ground at z = -2 first.
floor_objects = sphere_objects + [sonar.make_plane((0, 0, -2.0), (0, 0, 1), 0.05)]
blocked = sonar.SonarSimulator(**base, bottom_enabled=True).render(floor_objects, position)
check("a floor between the target and the bottom boundary blocks the reflection",
      float(blocked.sum()), 0.0, 0.0)

print("\nchirp generation")
import numpy as np

chirp_sonar = sonar.ChirpSonar(frequency_hz=300e3, chirp_bandwidth_hz=60e3,
                               chirp_duration_s=2e-3, sample_rate_hz=1.2e6)
sweep = chirp_sonar.transmit(swept=True)
check("chirp starts at cos(0)", sweep[0], 1.0, 0.0)
check("chirp length is T * fs", len(sweep), 2400, 0)

# s(t) = cos(2 pi (f0 t + B/(2T) t^2)), evaluated here independently of the C++.
times = np.array([137, 900, 2399]) / 1.2e6
closed_form = np.cos(2 * math.pi * (300e3 * times + (60e3 / (2 * 2e-3)) * times ** 2))
check("chirp matches the closed form", float(np.abs(sweep[[137, 900, 2399]] - closed_form).max()),
      0.0, 1e-12)

tone = chirp_sonar.transmit(swept=False)
check("tone matches a fixed frequency",
      float(np.abs(tone[[137, 900, 2399]] - np.cos(2 * math.pi * 300e3 * times)).max()),
      0.0, 1e-12)

print("\nmatched filter")
# Correlating the transmitted copy with itself peaks at zero lag with sum(s^2).
self_correlation = chirp_sonar.compress(sweep, sweep)
check("autocorrelation peaks at zero lag", int(np.argmax(self_correlation)), 0, 0)
check("peak equals sum of squares", self_correlation[0], float((sweep ** 2).sum()), 1e-6)

# A single reflector at a known range, noise free: the peak must land within half
# a sample, which is c / (4 fs) = 0.31 mm.
TRUE_RANGE = 4.0
record = chirp_sonar.receive([(TRUE_RANGE, 1.0, 1.0)], sweep, 0.010)
shape = acoustics.envelope(chirp_sonar.compress(record, sweep))
ranges = chirp_sonar.range_axis_m(len(shape))
recovered = ranges[int(np.argmax(shape))]
check("recovered range within half a sample", recovered, TRUE_RANGE,
      0.5 * 1500.0 / (2 * 1.2e6))
check("recovered delay within half a sample", 2 * recovered / 1500.0, 2 * TRUE_RANGE / 1500.0,
      0.5 / 1.2e6)

# Two reflectors four resolution cells apart must come back as two peaks, and one
# cell apart must not: a filter that resolves better than c / (2 B) is wrong.
for gap, expected in ((4 * chirp_sonar.range_resolution_m, 2),
                      (0.5 * chirp_sonar.range_resolution_m, 1)):
    pair = chirp_sonar.receive([(4.0, 1.0, 1.0), (4.0 + gap, 1.0, 1.0)], sweep, 0.010)
    shape_pair = acoustics.envelope(chirp_sonar.compress(pair, sweep))
    step = ranges[1] - ranges[0]
    separation = max(int(0.6 * chirp_sonar.range_resolution_m / step), 3)
    kept = acoustics.peak_indices(shape_pair, 0.4 * shape_pair.max(), separation)
    check(f"{1000 * gap:.1f} mm apart resolves into {expected}", len(kept), expected, 0)

print(f"\n{'all tests passed' if not failures else str(len(failures)) + ' FAILED: ' + ', '.join(failures)}")
sys.exit(1 if failures else 0)
