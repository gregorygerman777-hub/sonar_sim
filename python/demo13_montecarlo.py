"""Demo 13: the noisy results as distributions rather than single runs.

Every number in this project that depends on a noise draw was previously
reported from one realisation. That is an anecdote: speckle has contrast 1, so a
single run can land anywhere. This reruns the same pipelines, seeded
independently each time, and reports means with bootstrap confidence intervals.

Nothing here reimplements the recovery. The detections and the arc-ray fusion
come from recovery.py, which demo 8 also calls, so the single run and the
repeated trials are the same code on the same scene.

    SONAR_MC_TRIALS=50 .venv/bin/python python/demo13_montecarlo.py
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import acoustics
import montecarlo
import outputs
import recovery
import scene as sc
import sonar

TRIALS = int(os.environ.get("SONAR_MC_TRIALS", "200"))
CONFIDENCE = float(os.environ.get("SONAR_MC_CONFIDENCE", "0.95"))
BASELINES = [0.05, 0.10, 0.20, 0.30, 0.45, 0.60]

print(f"{TRIALS} trials, {100 * CONFIDENCE:.0f}% intervals, "
      f"bootstrap over 10000 resamples\n")

axes = recovery.scene_axes()
centre = recovery.target_centre(axes)
objects = recovery.build_objects(centre)
sim = recovery.build_sonar()

print("A. elevation recovery at the nominal 0.30 m baseline")
sweep_errors = np.empty((TRIALS, len(BASELINES)))
ranges = np.empty(TRIALS)
for trial in range(TRIALS):
    ranges[trial], recovered = recovery.trial(sim, objects, axes, 9000 + trial,
                                              BASELINES)
    sweep_errors[trial] = recovered - recovery.TARGET_ELEVATION

nominal = BASELINES.index(0.30)
elevation_stats = montecarlo.summarise(sweep_errors[:, nominal], CONFIDENCE, seed=1)
print("  " + montecarlo.format_summary("elevation error", elevation_stats, "degrees"))
print(f"  elevation error: {elevation_stats['mean']:+.4f} plus or minus "
      f"{elevation_stats['std']:.4f} degrees across {TRIALS} trials")

# What the error means in metres, and what one bin alone leaves open.
arc_m = np.radians(recovery.BEAMWIDTH_DEG) * (ranges.mean() + recovery.TARGET_RADIUS)
position_stats = montecarlo.summarise(
    np.abs(np.radians(sweep_errors[:, nominal])) * (ranges.mean() + recovery.TARGET_RADIUS),
    CONFIDENCE, seed=2)
print(f"  as a height error: {100 * position_stats['mean']:.2f} cm mean, "
      f"{100 * position_stats['low']:.2f} to {100 * position_stats['high']:.2f} cm "
      f"({100 * CONFIDENCE:.0f}% CI), against {100 * arc_m:.0f} cm of arc "
      f"left open by the sonar alone")

print("\nB. baseline sweep")
print(f"  {'baseline m':>11} {'mean err':>9} {'std':>8} {'CI low':>9} {'CI high':>9}")
sweep_stats = []
for index, baseline in enumerate(BASELINES):
    stats = montecarlo.summarise(sweep_errors[:, index], CONFIDENCE, seed=10 + index)
    sweep_stats.append(stats)
    print(f"  {baseline:11.2f} {stats['mean']:+9.4f} {stats['std']:8.4f} "
          f"{stats['low']:+9.4f} {stats['high']:+9.4f}")

print("\nC. texture against speckle, view to view")
STEP_M = 0.05
texture_sim = sonar.SonarSimulator(frequency_hz=1.2e6, num_azimuth_bins=257,
                                   num_range_bins=1000, horizontal_fov_deg=30.0,
                                   vertical_beamwidth_deg=12.0, max_range_m=10.0,
                                   num_elevation_subrays=3072)
texture_axes = sc.tilted_axes(20.0)
position_a = np.array([0.0, 0.0, -1.0])
position_b = position_a + STEP_M * texture_axes[:, 1]
seabed = [sonar.make_plane((0.0, 0.0, -3.0), (0.0, 0.0, 1.0), 0.05,
                           texture_amplitude=0.6, texture_scale_m=0.22, texture_seed=5)]
smooth_a = texture_sim.render([sonar.make_plane((0.0, 0.0, -3.0), (0.0, 0.0, 1.0), 0.05)],
                              position=position_a, axes=texture_axes)
smooth_b = texture_sim.render([sonar.make_plane((0.0, 0.0, -3.0), (0.0, 0.0, 1.0), 0.05)],
                              position=position_b, axes=texture_axes)
shift = int(round(STEP_M / (10.0 / 1000)))
lit = (smooth_a > 0.02 * smooth_a.max())
lit &= np.roll(smooth_b, shift, axis=1) > 0.02 * smooth_b.max()


def residual(image, reference):
    out = np.ones_like(image)
    np.divide(image, reference, out=out, where=reference > 0)
    return out


correlations = []
for trial in range(TRIALS):
    left = residual(texture_sim.render(seabed, position=position_a, axes=texture_axes,
                                       speckle=True, seed=20000 + 2 * trial), smooth_a)[lit]
    right = np.roll(residual(texture_sim.render(seabed, position=position_b, axes=texture_axes,
                                                speckle=True, seed=20001 + 2 * trial),
                             smooth_b), shift, axis=1)[lit]
    correlations.append(float(np.corrcoef(left, right)[0, 1]))
correlation_stats = montecarlo.summarise(correlations, CONFIDENCE, seed=3)
print("  " + montecarlo.format_summary("single-look view-to-view correlation",
                                       correlation_stats, ""))

print("\nD. chirp range recovery under noise")
chirp_sonar = sonar.ChirpSonar(frequency_hz=300e3, chirp_bandwidth_hz=60e3,
                               chirp_duration_s=2e-3, sample_rate_hz=1.2e6)
TRUE_RANGES = (4.000, 4.050)
sweep_waveform = chirp_sonar.transmit(swept=True)
clean = chirp_sonar.receive([(r, 1.0, 1.0) for r in TRUE_RANGES], sweep_waveform, 0.010)
chirp_ranges = chirp_sonar.range_axis_m(len(clean))
bin_m = chirp_ranges[1] - chirp_ranges[0]

chirp_errors, resolved = [], 0
for trial in range(TRIALS):
    shape = acoustics.envelope(
        chirp_sonar.compress(chirp_sonar.add_noise(clean, 0.15, seed=30000 + trial),
                             sweep_waveform))
    quiet = (chirp_ranges > 1.0) & (chirp_ranges < 3.0)
    window = (chirp_ranges > 3.5) & (chirp_ranges < 4.6)
    peaks = [p for p in acoustics.peak_indices(shape, 6.0 * np.median(shape[quiet]),
                                               max(int(0.6 * chirp_sonar.range_resolution_m
                                                       / bin_m), 3)) if window[p]]
    if len(peaks) == 2:
        resolved += 1
        for index, truth in zip(peaks, TRUE_RANGES):
            chirp_errors.append(1000.0 * (chirp_ranges[index] - truth))
chirp_stats = montecarlo.summarise(chirp_errors, CONFIDENCE, seed=4)
print(f"  both targets resolved in {resolved}/{TRIALS} trials "
      f"({100 * resolved / TRIALS:.1f}%)")
print("  " + montecarlo.format_summary("range error", chirp_stats, "mm"))

fig = plt.figure(figsize=(15.5, 8.4), constrained_layout=True)
grid = fig.add_gridspec(2, 2)

ax = fig.add_subplot(grid[0, 0])
ax.hist(sweep_errors[:, nominal], bins=max(TRIALS // 8, 10), color="#8fbfd6",
        edgecolor="#1f6f8b")
ax.axvline(elevation_stats["mean"], color="#e8735c", linewidth=2.0,
           label=f"mean {elevation_stats['mean']:+.4f}$\\degree$")
ax.axvspan(elevation_stats["low"], elevation_stats["high"], color="#e8735c", alpha=0.25,
           label=f"{100 * CONFIDENCE:.0f}% CI on the mean")
ax.set(xlabel="elevation error (deg)", ylabel="trials",
       title=f"A. recovery at a 0.30 m baseline, {TRIALS} trials\n"
             f"{elevation_stats['mean']:+.4f} $\\pm$ {elevation_stats['std']:.4f} deg")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[0, 1])
means = np.array([s["mean"] for s in sweep_stats])
lows = np.array([s["low"] for s in sweep_stats])
highs = np.array([s["high"] for s in sweep_stats])
spread = np.array([s["std"] for s in sweep_stats])
ax.plot(BASELINES, means, "o-", color="#1f6f8b", linewidth=1.8, label="mean error")
ax.fill_between(BASELINES, lows, highs, color="#1f6f8b", alpha=0.25,
                label=f"{100 * CONFIDENCE:.0f}% CI on the mean")
ax.fill_between(BASELINES, means - spread, means + spread, color="#1f6f8b", alpha=0.10,
                label="$\\pm$ 1 sd of a single trial")
ax.axhline(0.0, color="#8a8a8a", linestyle=":", linewidth=1.2)
ax.set(xlabel="camera baseline above the sonar (m)", ylabel="elevation error (deg)",
       title="B. how the mounting baseline changes the recovery")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[1, 0])
ax.hist(correlations, bins=max(TRIALS // 8, 10), color="#9fd4b0", edgecolor="#2a8f5f")
ax.axvline(correlation_stats["mean"], color="#e8735c", linewidth=2.0,
           label=f"mean {correlation_stats['mean']:.4f}")
ax.axvspan(correlation_stats["low"], correlation_stats["high"], color="#e8735c", alpha=0.25,
           label=f"{100 * CONFIDENCE:.0f}% CI")
ax.set(xlabel="view-to-view correlation, single look", ylabel="trials",
       title="C. what one look of speckle leaves of the texture\n"
             "(speckle-free ceiling is 0.996)")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[1, 1])
ax.hist(chirp_errors, bins=max(TRIALS // 6, 10), color="#f0c9a0", edgecolor="#c8781a")
ax.axvline(chirp_stats["mean"], color="#1f6f8b", linewidth=2.0,
           label=f"mean {chirp_stats['mean']:+.3f} mm")
ax.axvspan(chirp_stats["low"], chirp_stats["high"], color="#1f6f8b", alpha=0.25,
           label=f"{100 * CONFIDENCE:.0f}% CI")
for value in (-0.5 * 1000 * bin_m, 0.5 * 1000 * bin_m):
    ax.axvline(value, color="#8a8a8a", linestyle=":", linewidth=1.2)
ax.set(xlabel="chirp range error (mm)", ylabel="detections",
       title=f"D. pulse compression under noise, {resolved}/{TRIALS} trials resolved\n"
             "dotted lines are half a sample")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

fig.suptitle(f"Monte Carlo: every noisy result as a distribution, {TRIALS} independently "
             f"seeded trials", fontsize=13)
figure_path = outputs.output_path("demo13_montecarlo.png")
fig.savefig(figure_path, dpi=130)
print(f"\nwrote {figure_path}")
