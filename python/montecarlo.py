"""Repeated trials and their statistics.

A stochastic sensor model reported as a single run is an anecdote. Every number
here that depends on a noise draw is measured over many independently seeded
runs and reported as a mean with an interval.

The interval is a percentile bootstrap rather than a t interval. It assumes
nothing about the shape of the error distribution, which matters because these
errors are not obviously Gaussian: they come from a centroid on a thresholded
blob, which is a nonlinear function of the noise.
"""

import numpy as np


def bootstrap_interval(values, confidence=0.95, resamples=10000, seed=0):
    """Percentile bootstrap confidence interval for the mean."""
    rng = np.random.default_rng(seed)
    values = np.asarray(values, dtype=float)
    means = rng.choice(values, size=(resamples, values.size), replace=True).mean(axis=1)
    tail = 0.5 * (1.0 - confidence)
    return float(np.quantile(means, tail)), float(np.quantile(means, 1.0 - tail))


def summarise(values, confidence=0.95, resamples=10000, seed=0):
    values = np.asarray(values, dtype=float)
    low, high = bootstrap_interval(values, confidence, resamples, seed)
    return {"n": int(values.size), "mean": float(values.mean()),
            "std": float(values.std(ddof=1)), "low": low, "high": high,
            "confidence": confidence}


def format_summary(name, stats, unit):
    return (f"{name}: {stats['mean']:+.4f} plus or minus {stats['std']:.4f} {unit} "
            f"across {stats['n']} trials "
            f"({100 * stats['confidence']:.0f}% CI on the mean "
            f"[{stats['low']:+.4f}, {stats['high']:+.4f}])")
