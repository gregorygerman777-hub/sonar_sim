"""Couple a geometric forward-scan image to actual BELLHOP eigenray arrivals."""

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
import scene
import sonar


SOURCE_DEPTH_M = 8.0
TARGET_DEPTH_M = 8.0
TARGET_RANGE_M = 80.0

environment = bellhop.BellhopEnvironment(
    frequency_hz=30000.0,
    water_depth_m=30.0,
    source_depth_m=SOURCE_DEPTH_M,
    receiver_depth_m=TARGET_DEPTH_M,
    max_range_m=100.0,
    sound_speed_surface_mps=1500.0,
    sound_speed_bottom_mps=1500.0,
    launch_min_deg=-60.0,
    launch_max_deg=60.0,
    ray_count=601,
    receiver_count=101,
)

simulator = sonar.SonarSimulator(
    frequency_hz=30000.0,
    num_azimuth_bins=97,
    num_range_bins=1200,
    horizontal_fov_deg=30.0,
    vertical_beamwidth_deg=20.0,
    max_range_m=120.0,
    num_elevation_subrays=128,
    beam_mode="top_hat",
)
target = sonar.make_sphere((0.0, TARGET_RANGE_M, -TARGET_DEPTH_M), 1.2, 0.9)
direct_image = simulator.render([target], position=(0.0, 0.0, -SOURCE_DEPTH_M))

workspace = Path(tempfile.mkdtemp(prefix="sonar_bellhop_fss_"))
rays, arrivals, metadata = bellhop.solve(environment, workspace)
coupled_image = bellhop.apply_two_way_multipath(
    direct_image, simulator.range_axis_m(), arrivals,
    source_depth_m=SOURCE_DEPTH_M, target_depth_m=TARGET_DEPTH_M,
)

receiver_index = int(np.argmin(np.abs(arrivals["receiver_ranges_m"] - TARGET_RANGE_M)))
paths = bellhop.two_way_paths(arrivals["records"][receiver_index]["arrivals"])
object_paths = [path for path in paths
                if path["top_bounces"] == 0 and path["bottom_bounces"] == 0]
ghost_paths = [path for path in paths if path["mixed"]]
mirror_paths = [path for path in paths
                if not path["mixed"] and path not in object_paths]

ranges = simulator.range_axis_m()
direct_profile = direct_image.sum(axis=0)
coupled_profile = coupled_image.sum(axis=0)
reference = max(float(coupled_image.max()), 1e-300)

figure, axes = plt.subplots(2, 2, figsize=(15, 9), constrained_layout=True)
scene.show_image(axes[0, 0], direct_image, simulator, floor_db=-55)
axes[0, 0].set(title="Geometric C++ FSS: direct return", xlim=(74, 105),
               xlabel="apparent range (m)", ylabel="bearing (deg)")
scene.show_image(axes[0, 1], coupled_image, simulator, floor_db=-55,
                 reference=reference, cmap="magma")
axes[0, 1].set(title="FSS remapped by actual BELLHOP eigenrays", xlim=(74, 105),
               xlabel="apparent range (m)", ylabel="bearing (deg)")

axes[1, 0].plot(ranges, direct_profile / max(direct_profile.max(), 1e-300),
                color="#22d3ee", label="direct geometric image")
axes[1, 0].plot(ranges, coupled_profile / max(coupled_profile.max(), 1e-300),
                color="#f43f5e", label="BELLHOP-coupled image")
axes[1, 0].set(xlim=(74, 105), ylim=(0, 1.05), xlabel="apparent range (m)",
               ylabel="normalized bearing-summed intensity",
               title="Object, ghost and mirror energy separate in range")
axes[1, 0].grid(alpha=0.2)
axes[1, 0].legend()

colours = []
labels = []
for path in paths:
    if path in object_paths:
        colours.append("#22d3ee"); labels.append("object")
    elif path["mixed"]:
        colours.append("#f43f5e"); labels.append("ghost")
    else:
        colours.append("#facc15"); labels.append("mirror")
path_ranges = np.array([path["apparent_range_m"] for path in paths])
path_levels = 10.0 * np.log10(np.maximum(
    [path["relative_intensity"] for path in paths], 1e-15))
axes[1, 1].scatter(path_ranges, path_levels, c=colours, s=20, alpha=0.75)
axes[1, 1].set(xlim=(74, 105), ylim=(-65, 5), xlabel="apparent range (m)",
               ylabel="relative path intensity (dB)",
               title="Reciprocal eigenray pairs at the target")
axes[1, 1].grid(alpha=0.2)
for name, colour in (("object", "#22d3ee"), ("ghost", "#f43f5e"),
                     ("mirror", "#facc15")):
    axes[1, 1].scatter([], [], c=colour, label=name)
axes[1, 1].legend()

figure.suptitle("Two-way BELLHOP/FSS coupling: direct/direct, mixed, and reflected/reflected paths")
figure_path = outputs.output_path("demo21_bellhop_fss.png")
figure.savefig(figure_path, dpi=150)

report = {
    "solver": metadata["executable"],
    "target_range_m": TARGET_RANGE_M,
    "one_way_arrivals": len(arrivals["records"][receiver_index]["arrivals"]),
    "two_way_paths": len(paths),
    "object_paths": len(object_paths),
    "ghost_paths": len(ghost_paths),
    "mirror_paths": len(mirror_paths),
    "first_object_range_m": min(path["apparent_range_m"] for path in object_paths),
    "first_ghost_range_m": min(path["apparent_range_m"] for path in ghost_paths),
    "first_mirror_range_m": min(path["apparent_range_m"] for path in mirror_paths),
    "coupling_assumption": ("All image returns are assigned the configured target depth; "
                            "reciprocal path pairs are summed as intensity without coherent phase."),
}
with open(outputs.output_path("demo21_bellhop_fss_metrics.json"), "w") as handle:
    json.dump(report, handle, indent=2)
print(json.dumps(report, indent=2))
print(f"wrote {figure_path}")

assert object_paths and ghost_paths and mirror_paths
assert report["first_object_range_m"] < report["first_ghost_range_m"]
assert coupled_profile[ranges > TARGET_RANGE_M + 0.5].sum() > 0.0
