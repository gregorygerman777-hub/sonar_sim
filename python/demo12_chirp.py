"""Demo 12: pulse compression, from a raw waveform to a resolved pair of targets.

Two point reflectors 50 mm apart. A plain pulse of duration T cannot separate
echoes closer than c T / 2, which here is 1.5 m, so it sees one blob. A linear FM
chirp sweeping B hertz over the same T seconds compresses in the matched filter
to a pulse of width about 1/B, giving

    delta_r = c / (2 B) = 12.5 mm

so the same energy, in the same time, resolves the pair. The improvement is the
time-bandwidth product B T. Urick, Principles of Underwater Sound, 3rd ed.,
ch. 2 and 9.

The echoes are put a long way below the noise floor on purpose. The matched
filter's processing gain is what digs them out, which is the other half of why
pulse compression is used.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import acoustics
import outputs
import sonar

RANGES = (4.000, 4.050)
REFLECTIVITY = 1.0
NOISE_SIGMA = 0.15
RECORD_S = 0.010
AUDIO_DIVIDE = 600.0

chirp_sonar = sonar.ChirpSonar(frequency_hz=300e3, chirp_bandwidth_hz=60e3,
                               chirp_duration_s=2e-3, sample_rate_hz=1.2e6)
reflectors = [(r, REFLECTIVITY, 1.0) for r in RANGES]

print(f"f0 = 300 kHz, B = 60 kHz, T = 2 ms, fs = 1.2 MHz, BT = "
      f"{chirp_sonar.time_bandwidth_product:.0f}")
print(f"  chirp resolution  c / (2 B) = {1000 * chirp_sonar.range_resolution_m:.2f} mm")
print(f"  plain pulse       c T / 2   = {chirp_sonar.uncompressed_resolution_m:.3f} m")
print(f"  targets {RANGES[0]:.3f} m and {RANGES[1]:.3f} m, "
      f"{1000 * (RANGES[1] - RANGES[0]):.0f} mm apart\n")

# Resolution first, on one reflector with no noise, because a width measured on
# two overlapping compressed pulses is measuring their interference as much as
# the filter.
solo = chirp_sonar.transmit(swept=True)
solo_shape = acoustics.envelope(
    chirp_sonar.compress(chirp_sonar.receive([(RANGES[0], REFLECTIVITY, 1.0)], solo, RECORD_S),
                         solo))
solo_ranges = chirp_sonar.range_axis_m(len(solo_shape))
bin_m = solo_ranges[1] - solo_ranges[0]
solo_peak = int(np.argmax(solo_shape))
solo_width = acoustics.half_power_width(solo_shape, solo_peak) * bin_m
# A matched-filtered LFM has a sinc envelope. Its half-power width is 0.886 / B
# in time; the Rayleigh figure c / (2 B) is the peak-to-null spacing, so the two
# differ by exactly that factor and both are quoted rather than one being fudged
# into the other.
print(f"single target, no noise: peak at {solo_ranges[solo_peak]:.4f} m "
      f"(truth {RANGES[0]:.3f})")
print(f"  compressed half-power width {1000 * solo_width:.2f} mm  against "
      f"0.886 c / (2 B) = {1000 * 0.886 * chirp_sonar.range_resolution_m:.2f} mm  "
      f"({100 * solo_width / (0.886 * chirp_sonar.range_resolution_m) - 100:+.1f}%)\n")

records, results = {}, {}
for name, swept in (("chirp", True), ("plain pulse", False)):
    transmit = chirp_sonar.transmit(swept=swept)
    clean = chirp_sonar.receive(reflectors, transmit, RECORD_S)
    received = chirp_sonar.add_noise(clean, NOISE_SIGMA, seed=7)
    compressed = chirp_sonar.compress(received, transmit)
    shape = acoustics.envelope(compressed)
    ranges = chirp_sonar.range_axis_m(len(shape))

    # Threshold referred to the noise, not to the peak. The envelope of Gaussian
    # noise through a matched filter is Rayleigh, whose median is 0.83 sigma, so
    # six medians is about 5 sigma: over the twelve thousand lags in this record
    # the largest noise excursion is around 4.3 sigma, which this clears.
    quiet = (ranges > 1.0) & (ranges < 3.0)
    noise_level = np.median(shape[quiet])
    threshold = 6.0 * noise_level
    window = (ranges > 3.5) & (ranges < 4.6)

    # A waveform is not allowed to claim two targets closer together than its own
    # resolution. Enforcing that is what makes the plain pulse report one target
    # instead of a row of noise ripples riding on its merged blob.
    resolution = (chirp_sonar.range_resolution_m if swept
                  else chirp_sonar.uncompressed_resolution_m)
    separation = max(int(0.6 * resolution / bin_m), 3)
    peaks = [p for p in acoustics.peak_indices(shape, threshold, separation) if window[p]]

    records[name] = (transmit, clean, received, compressed, shape, ranges)
    results[name] = peaks

    echo_peak = np.abs(clean).max()
    print(f"{name}")
    print(f"  echo amplitude {echo_peak:.4f} against noise sigma {NOISE_SIGMA}: "
          f"raw SNR {20 * np.log10(echo_peak / NOISE_SIGMA):+.1f} dB, "
          f"compressed peak {shape[window].max() / noise_level:.0f}x the noise")
    print(f"  peaks above threshold in 3.5 to 4.6 m: {len(peaks)}")
    for index in peaks:
        nearest = min(RANGES, key=lambda value: abs(value - ranges[index]))
        print(f"    range {ranges[index]:.4f} m   delay "
              f"{1e6 * 2.0 * ranges[index] / 1500.0:9.3f} us   truth {nearest:.3f} m "
              f"{1e6 * 2.0 * nearest / 1500.0:9.3f} us   "
              f"error {1000 * (ranges[index] - nearest):+.2f} mm")
    if len(peaks) < 2:
        print(f"    the pair is not separated: c T / 2 = "
              f"{chirp_sonar.uncompressed_resolution_m:.2f} m against a "
              f"{1000 * (RANGES[1] - RANGES[0]):.0f} mm gap")
    print()

audio_files = []
for label, signal in (("transmit", records["chirp"][0]),
                      ("received", records["chirp"][2]),
                      ("compressed", records["chirp"][3])):
    audible = acoustics.to_audio(signal, chirp_sonar.sample_rate_hz, divide=AUDIO_DIVIDE)
    path = outputs.output_path(f"demo12_chirp_{label}.wav")
    seconds = acoustics.write_wav(path, audible)
    audio_files.append((path, seconds))
    print(f"wrote {path}  ({seconds:.2f} s, carrier {300e3 / AUDIO_DIVIDE:.0f} Hz)")

fig = plt.figure(figsize=(15.5, 8.6), constrained_layout=True)
grid = fig.add_gridspec(3, 2)

transmit, clean, received, compressed, shape, ranges = records["chirp"]
time_us = 1e6 * np.arange(len(transmit)) / chirp_sonar.sample_rate_hz

ax = fig.add_subplot(grid[0, 0])
# The waveform itself is only four samples a cycle at this carrier, so plotting it
# raw looks like a triangle wave. The instantaneous frequency is the informative
# view and doubles as a check that the sweep really is linear.
measured = acoustics.instantaneous_frequency(transmit, chirp_sonar.sample_rate_hz) / 1e3
trim = slice(30, -30)
ax.plot(time_us[trim], measured[trim], color="#1f6f8b", linewidth=1.2, label="measured")
ax.plot(time_us, (300e3 + (60e3 / 2e-3) * (time_us * 1e-6)) / 1e3, "--", color="#c8a020",
        linewidth=1.2, label="f0 + (B/T) t")
ax.set(xlabel="time (us)", ylabel="instantaneous frequency (kHz)",
       title="the sweep: 300 to 360 kHz across 2 ms")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[0, 1])
record_us = 1e6 * np.arange(len(received)) / chirp_sonar.sample_rate_hz
ax.plot(record_us, received, color="#8a8a8a", linewidth=0.4)
ax.plot(record_us, clean, color="#e8735c", linewidth=0.6, label="echoes without noise")
ax.set(xlim=(5200, 5600), xlabel="time (us)", ylabel="amplitude",
       title="received record: the echoes are under the noise")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[1, :])
for name, colour in (("chirp", "#2a8f5f"), ("plain pulse", "#e8735c")):
    _, _, _, _, shape_n, ranges_n = records[name]
    ax.plot(ranges_n, shape_n / shape_n.max(), color=colour, linewidth=1.4,
            label=f"{name}, {len(results[name])} peak(s)")
for value in RANGES:
    ax.axvline(value, color="#8a8a8a", linestyle=":", linewidth=1.2)
ax.set(xlim=(3.6, 4.5), xlabel="range (m)", ylabel="matched filter envelope, normalised",
       title="the same two targets, the same pulse length: only the chirp separates them")
ax.legend(fontsize=9)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[2, 0])
ax.plot(ranges, shape / shape.max(), color="#2a8f5f", linewidth=1.4)
for value in RANGES:
    ax.axvline(value, color="#8a8a8a", linestyle=":", linewidth=1.2)
ax.axhline(1 / np.sqrt(2), color="#c8a020", linestyle="--", linewidth=1.0,
           label="half power")
ax.set(xlim=(3.95, 4.10), xlabel="range (m)", ylabel="envelope",
       title="compressed pulses, close up")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

ax = fig.add_subplot(grid[2, 1])
frequency = np.fft.rfftfreq(len(transmit), 1.0 / chirp_sonar.sample_rate_hz) / 1e3
for name, colour in (("chirp", "#2a8f5f"), ("plain pulse", "#e8735c")):
    spectrum = np.abs(np.fft.rfft(records[name][0]))
    ax.plot(frequency, 20 * np.log10(np.maximum(spectrum / spectrum.max(), 1e-6)),
            color=colour, linewidth=1.0, label=name)
ax.set(xlim=(250, 400), ylim=(-60, 3), xlabel="frequency (kHz)", ylabel="dB",
       title="where the resolution comes from: occupied bandwidth")
ax.legend(fontsize=8)
ax.grid(alpha=0.25)

fig.suptitle("Pulse compression: a linear FM chirp and its matched filter", fontsize=13)
figure_path = outputs.output_path("demo12_chirp.png")
fig.savefig(figure_path, dpi=130)
print(f"\nwrote {figure_path}")
