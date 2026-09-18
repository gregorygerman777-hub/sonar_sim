"""Run the external Fortran BELLHOP solver and validate its parsed products."""

import json
from pathlib import Path
import sys
import tempfile

sys.path[:0] = [".", "python"]

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import bellhop
import outputs


environment = bellhop.BellhopEnvironment(
    frequency_hz=30000.0,
    sound_speed_surface_mps=1500.0,
    sound_speed_bottom_mps=1500.0,
)
work = Path(tempfile.mkdtemp(prefix="sonar_bellhop_demo_"))
rays, arrivals, metadata = bellhop.solve(environment, work)
far_record = arrivals["records"][-1]
far_arrivals = far_record["arrivals"]

direct = [arrival for arrival in far_arrivals
          if arrival["top_bounces"] == 0 and arrival["bottom_bounces"] == 0]
surface = [arrival for arrival in far_arrivals if arrival["top_bounces"] > 0]
bottom = [arrival for arrival in far_arrivals if arrival["bottom_bounces"] > 0]
direct_delay = min(arrival["delay_s"] for arrival in direct)
analytic_delay = environment.max_range_m / 1500.0
relative_delay_error = abs(direct_delay - analytic_delay) / analytic_delay

figure, axes = plt.subplots(1, 2, figsize=(15, 6), constrained_layout=True)
colours = {"direct": "#22d3ee", "surface": "#f43f5e", "bottom": "#facc15",
           "combined": "#e2e8f0"}
for ray in rays["rays"]:
    if ray["top_bounces"] and ray["bottom_bounces"]:
        kind = "combined"
    elif ray["top_bounces"]:
        kind = "surface"
    elif ray["bottom_bounces"]:
        kind = "bottom"
    else:
        kind = "direct"
    points = ray["points_m"]
    axes[0].plot(points[:, 0], points[:, 1], color=colours[kind], alpha=0.32, linewidth=0.7)
axes[0].invert_yaxis()
axes[0].set(xlabel="range (m)", ylabel="depth (m)",
            title=f"Actual BELLHOP ray file: {len(rays['rays'])} launch rays")
axes[0].grid(alpha=0.2)

for arrival in far_arrivals:
    if arrival["top_bounces"] and arrival["bottom_bounces"]:
        kind = "combined"
    elif arrival["top_bounces"]:
        kind = "surface"
    elif arrival["bottom_bounces"]:
        kind = "bottom"
    else:
        kind = "direct"
    level_db = 20.0 * np.log10(max(arrival["amplitude"], 1e-15))
    delay_ms = 1000.0 * arrival["delay_s"]
    axes[1].vlines(delay_ms, -80.0, level_db, color=colours[kind], linewidth=2.0)
    axes[1].scatter(delay_ms, level_db, color=colours[kind], s=24)
axes[1].set(xlabel="one-way arrival time (ms)", ylabel="relative amplitude (dB)",
            title=f"Eigenray arrivals at {environment.max_range_m:.0f} m")
axes[1].grid(alpha=0.2)

figure.suptitle("Acoustics Toolbox integration — external GPL BELLHOP process, parsed by the MIT simulator")
figure.savefig(outputs.output_path("demo20_bellhop.png"), dpi=150)

report = {
    "solver": metadata["executable"],
    "ray_count": len(rays["rays"]),
    "far_receiver_arrival_count": len(far_arrivals),
    "direct_arrivals": len(direct),
    "surface_arrivals": len(surface),
    "bottom_arrivals": len(bottom),
    "direct_delay_s": direct_delay,
    "analytic_straight_path_delay_s": analytic_delay,
    "relative_delay_error": relative_delay_error,
    "work_directory": str(work),
    "boundary": ("BELLHOP is a separately installed GPL executable. The simulator writes environment "
                 "files and parses ASCII outputs; no Acoustics Toolbox or PYAT source is copied here."),
}
with open(outputs.output_path("demo20_bellhop_metrics.json"), "w") as handle:
    json.dump(report, handle, indent=2)
print(json.dumps(report, indent=2))

assert len(rays["rays"]) == environment.ray_count
assert direct and surface and bottom
assert relative_delay_error < 1e-5
