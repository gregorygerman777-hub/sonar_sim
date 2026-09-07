"""Waveform analysis helpers: envelope, peak picking and WAV export.

The heavy correlation is in C++. What is left here is analysis, which is where
numpy belongs.
"""

import wave

import numpy as np


def analytic_signal(signal):
    """Hilbert transform in its FFT form: keep the positive frequencies, double them."""
    count = len(signal)
    mask = np.zeros(count)
    mask[0] = 1.0
    mask[1:(count + 1) // 2] = 2.0
    if count % 2 == 0:
        mask[count // 2] = 1.0
    return np.fft.ifft(np.fft.fft(signal) * mask)


def envelope(signal):
    """Magnitude of the analytic signal.

    The matched filter of a real chirp oscillates at the carrier, so its peak
    height and width are only meaningful on the envelope.
    """
    return np.abs(analytic_signal(signal))


def instantaneous_frequency(signal, sample_rate_hz):
    """Derivative of the analytic phase, which for an LFM chirp is a straight line."""
    phase = np.unwrap(np.angle(analytic_signal(signal)))
    return np.gradient(phase) * sample_rate_hz / (2.0 * np.pi)


def peak_indices(values, threshold, separation):
    """Local maxima above a threshold, keeping the strongest within a separation."""
    candidates = np.nonzero((values[1:-1] > values[:-2]) & (values[1:-1] >= values[2:]) &
                            (values[1:-1] > threshold))[0] + 1
    kept = []
    for index in candidates[np.argsort(values[candidates])[::-1]]:
        if all(abs(index - other) >= separation for other in kept):
            kept.append(int(index))
    return sorted(kept)


def half_power_width(values, peak):
    """Width of a peak at half its power, in samples, by linear interpolation."""
    half = values[peak] / np.sqrt(2.0)
    left = peak
    while left > 0 and values[left] > half:
        left -= 1
    right = peak
    while right < len(values) - 1 and values[right] > half:
        right += 1
    if left == peak or right == peak:
        return 0.0
    lower = left + (half - values[left]) / (values[left + 1] - values[left])
    upper = right - (half - values[right]) / (values[right - 1] - values[right])
    return upper - lower


def to_audio(signal, sample_rate_hz, divide=600.0, audio_rate=44100):
    """Frequency-divide into the audible band by stretching time.

    Resampling the record onto a time base `divide` times longer divides every
    frequency in it by the same factor and leaves the waveform's shape alone. At
    300 kHz and a factor of 600 the carrier lands at 500 Hz.
    """
    duration = len(signal) / sample_rate_hz
    count = int(round(duration * divide * audio_rate))
    source = np.arange(len(signal)) / sample_rate_hz
    return np.interp(np.linspace(0.0, duration, count, endpoint=False), source, signal)


def write_wav(path, signal, audio_rate=44100):
    peak = np.abs(signal).max()
    scaled = (signal / peak * 32000.0).astype("<i2") if peak > 0 else signal.astype("<i2")
    with wave.open(path, "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(audio_rate)
        handle.writeframes(scaled.tobytes())
    return len(scaled) / audio_rate
