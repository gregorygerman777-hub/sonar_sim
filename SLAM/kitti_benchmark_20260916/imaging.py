"""Range compensation for rendered pings, the time-varying gain of real sonars.

The C++ core attenuates each return by 1/r^4 (two-way spherical spreading of
intensity) and by Thorp absorption, 10^(-2 alpha r / 10000) with alpha in dB/km.
Every commercial forward-looking sonar undoes this in hardware before the image
is displayed or logged, so scan matching on raw simulator output is a harder
problem than scan matching on real imagery. The gain below inverts the core's
per-ray loss (the 40 log r plus absorption law used for point targets) and is
normalised to one at maximum range. The peak of an extended sphere still falls
slowly with range after compensation because a nearer sphere spans more
elevation sub-rays that sum into the same pixel; the residual is a few times
over the 2 to 8 m span, against more than a hundred times uncompensated.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "python")]

import sonar  # noqa: E402


def range_compensation(simulator, frequency_hz):
    ranges = np.asarray(simulator.range_axis_m(), dtype=float)
    alpha_db_per_km = sonar.thorp_alpha(frequency_hz)
    absorption_per_m = alpha_db_per_km * np.log(10.0) / 5000.0
    gain = ranges ** 4 * np.exp(absorption_per_m * ranges)
    return gain / gain[-1]
