"""Planar sonar-inertial SLAM with image features, loop closure and uncertainty."""

import json
import sys

sys.path[:0] = [".", "python"]

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

import outputs
import scene
import slam
import slam_experiment


result = slam_experiment.run_experiment()
truth = result["truth"]
before = result["before"]
optimized = result["optimized"]
uncertainty = result["uncertainty"]

figure, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
sample = len(result["images"]) // 4
scene.show_image(axes[0, 0], result["images"][sample], result["simulator"], floor_db=-42)
points = result["features"][sample]
ranges = np.linalg.norm(points, axis=1)
bearings = np.degrees(np.arctan2(points[:, 0], points[:, 1]))
axes[0, 0].scatter(ranges, bearings, s=18, facecolors="none", edgecolors="#ffbf47")
axes[0, 0].set(title=f"Ping {sample}: local maxima passed to scan matching",
               xlabel="range (m)", ylabel="bearing (deg)")

trajectory = axes[0, 1]
trajectory.plot(*truth[:, :2].T, color="#cbd5e1", linewidth=2.5, label="known truth (evaluation)")
trajectory.plot(*before[:, :2].T, color="#f97316", linestyle="--", label="IMU dead reckoning")
trajectory.plot(*optimized[:, :2].T, color="#22d3ee", linewidth=2.2, label="optimized SLAM")
for index in range(4, len(optimized), 4):
    sigma_x, sigma_y = uncertainty[index, :2]
    trajectory.add_patch(Ellipse(optimized[index, :2], 4 * sigma_x, 4 * sigma_y,
                                 fill=False, edgecolor="#22d3ee", alpha=0.45))
for first, second, *_ in result["loop_edges"]:
    trajectory.plot([optimized[first, 0], optimized[second, 0]],
                    [optimized[first, 1], optimized[second, 1]], color="#f43f5e",
                    linewidth=2.0, label="accepted loop closure")
trajectory.axis("equal")
trajectory.grid(alpha=0.2)
handles, labels = trajectory.get_legend_handles_labels()
unique = dict(zip(labels, handles))
trajectory.legend(unique.values(), unique.keys(), fontsize=8)
trajectory.set(title="SE(2) trajectory and two-standard-deviation marginal ellipses",
               xlabel="world X (m)", ylabel="world Y (m)")

map_axis = axes[1, 0]
for index, local_points in enumerate(result["features"]):
    dead_map = slam.points_in_world(local_points, before[index])
    optimized_map = slam.points_in_world(local_points, optimized[index])
    map_axis.scatter(*dead_map.T, s=3, color="#f97316", alpha=0.12)
    map_axis.scatter(*optimized_map.T, s=4, color="#22d3ee", alpha=0.22)
map_axis.scatter([], [], s=18, color="#f97316", alpha=0.7, label="dead-reckoned map")
map_axis.scatter([], [], s=18, color="#22d3ee", alpha=0.7, label="optimized map")
map_axis.axis("equal")
map_axis.grid(alpha=0.2)
map_axis.legend(fontsize=8)
map_axis.set(title="Accumulated sonar feature map", xlabel="world X (m)", ylabel="world Y (m)")

error_axis = axes[1, 1]
dead_error = np.linalg.norm(before[:, :2] - truth[:, :2], axis=1)
optimized_error = np.linalg.norm(optimized[:, :2] - truth[:, :2], axis=1)
error_axis.plot(dead_error, color="#f97316", label="dead reckoning")
error_axis.plot(optimized_error, color="#22d3ee", label="optimized")
error_axis.fill_between(np.arange(len(uncertainty)), 0.0,
                        2.0 * np.linalg.norm(uncertainty[:, :2], axis=1),
                        color="#22d3ee", alpha=0.12, label="2σ marginal scale")
error_axis.grid(alpha=0.2)
error_axis.legend(fontsize=8)
error_axis.set(title="Position error uses truth only after estimation",
               xlabel="ping", ylabel="position error (m)")

metrics = result["metrics"]
figure.suptitle(
    "Planar sonar–IMU SLAM: "
    f"RMSE {metrics['graph_before_rmse_m']:.3f} → {metrics['optimized_rmse_m']:.3f} m; "
    f"closure {metrics['closure_before_m']:.3f} → {metrics['closure_after_m']:.4f} m",
    fontsize=14,
)
figure.savefig(outputs.output_path("demo19_slam.png"), dpi=150)

report = dict(metrics=metrics, scan_edges=result["scan_edges"], loop_edges=result["loop_edges"],
              model=("Planar SE(2); rendered sonar maxima and descriptor/ICP loop closure; "
                     "C++ robust pose graph and marginal covariance; synthetic IMU."),
              limitations=("Known initial gauge and flat motion; no depth/roll/pitch, IMU bias state, "
                           "hardware timing, water-current model, coherent acoustic phase, or real-data calibration."))
with open(outputs.output_path("demo19_slam_metrics.json"), "w") as handle:
    json.dump(report, handle, indent=2)
print(json.dumps(report, indent=2))

assert metrics["loop_closure_count"] >= 1
assert metrics["optimized_rmse_m"] < metrics["graph_before_rmse_m"] * 0.25
assert metrics["closure_after_m"] < 0.01
