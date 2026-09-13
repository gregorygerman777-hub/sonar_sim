"""Adapter for an external Acoustics Toolbox BELLHOP executable.

The GPL solver remains a separate process. This module writes its documented
environment format and reads ASCII ray and arrival products.
"""

from dataclasses import dataclass
from pathlib import Path
import os
import shutil
import subprocess

import numpy as np


DEFAULT_EXECUTABLES = (
    Path.home() / ".local/opt/acoustics-toolbox-2020/Bellhop/bellhop.exe",
    Path.home() / ".local/bin/bellhop",
)


@dataclass
class BellhopEnvironment:
    frequency_hz: float = 30000.0
    water_depth_m: float = 30.0
    source_depth_m: float = 5.0
    receiver_depth_m: float = 5.0
    max_range_m: float = 120.0
    sound_speed_surface_mps: float = 1490.0
    sound_speed_bottom_mps: float = 1520.0
    bottom_sound_speed_mps: float = 1700.0
    bottom_density_gcm3: float = 1.8
    bottom_attenuation_db_wavelength: float = 0.5
    launch_min_deg: float = -35.0
    launch_max_deg: float = 35.0
    ray_count: int = 241
    receiver_count: int = 61


def executable_path():
    configured = os.environ.get("BELLHOP_EXECUTABLE")
    candidates = ([Path(configured)] if configured else []) + list(DEFAULT_EXECUTABLES)
    path_entry = shutil.which("bellhop") or shutil.which("bellhop.exe")
    if path_entry:
        candidates.append(Path(path_entry))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    raise FileNotFoundError("BELLHOP executable not found; set BELLHOP_EXECUTABLE")


def write_environment(path, environment, run_type):
    path = Path(path)
    profile_depths = np.linspace(0.0, environment.water_depth_m, 21)
    profile_speeds = np.linspace(environment.sound_speed_surface_mps,
                                 environment.sound_speed_bottom_mps, 21)
    lines = [
        "'Forward-scan propagation laboratory'",
        f"{environment.frequency_hz:.9g}",
        "1",
        "'CVW'",
        f"51 0.0 {environment.water_depth_m:.9g}",
    ]
    lines.extend(f"{depth:.9g} {speed:.9g} /"
                 for depth, speed in zip(profile_depths, profile_speeds))
    lines.extend([
        "'A' 0.0",
        (f"{environment.water_depth_m:.9g} {environment.bottom_sound_speed_mps:.9g} "
         f"0.0 {environment.bottom_density_gcm3:.9g} "
         f"{environment.bottom_attenuation_db_wavelength:.9g} 0.0 /"),
        "1",
        f"{environment.source_depth_m:.9g} /",
        "1",
        f"{environment.receiver_depth_m:.9g} /",
        str(environment.receiver_count),
        f"0.001 {environment.max_range_m / 1000.0:.9g} /",
        f"'{run_type}'",
        str(environment.ray_count),
        f"{environment.launch_min_deg:.9g} {environment.launch_max_deg:.9g} /",
        f"0.0 {environment.water_depth_m + 1.0:.9g} {environment.max_range_m / 1000.0 + 0.001:.9g}",
        "",
    ])
    path.write_text("\n".join(lines))
    return path


def run(environment, directory, run_type="R", stem="propagation"):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    write_environment(directory / f"{stem}.env", environment, run_type)
    completed = subprocess.run(
        [str(executable_path()), stem], cwd=directory, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30, check=False,
    )
    if completed.returncode:
        raise RuntimeError(f"BELLHOP failed ({completed.returncode}):\n{completed.stdout}")
    return completed.stdout


def read_rays(path):
    lines = Path(path).read_text().splitlines()
    frequency_hz = float(lines[1])
    source_shape = [int(value) for value in lines[2].split()]
    beam_shape = [int(value) for value in lines[3].split()]
    top_depth_m = float(lines[4])
    bottom_depth_m = float(lines[5])
    index = 7
    rays = []
    expected = int(np.prod(source_shape)) * int(np.prod(beam_shape))
    for _ in range(expected):
        if index >= len(lines) or not lines[index].strip():
            break
        launch_angle_deg = float(lines[index])
        index += 1
        count, top_bounces, bottom_bounces = map(int, lines[index].split()[:3])
        index += 1
        points = np.array([[float(value) for value in lines[index + step].split()[:2]]
                           for step in range(count)])
        index += count
        rays.append({"launch_angle_deg": launch_angle_deg,
                     "top_bounces": top_bounces,
                     "bottom_bounces": bottom_bounces,
                     "points_m": points})
    return {"frequency_hz": frequency_hz, "top_depth_m": top_depth_m,
            "bottom_depth_m": bottom_depth_m, "rays": rays}


def read_arrivals(path):
    lines = Path(path).read_text().splitlines()
    if "2D" not in lines[0]:
        raise ValueError("only BELLHOP 2-D ASCII arrivals are supported")
    frequency_hz = float(lines[1])

    def counted_values(line):
        values = line.split()
        count = int(values[0])
        return np.array([float(value) for value in values[1:1 + count]])

    source_depths = counted_values(lines[2])
    receiver_depths = counted_values(lines[3])
    receiver_ranges = counted_values(lines[4])
    index = 5
    records = []
    for source_index in range(len(source_depths)):
        maximum_arrivals = int(lines[index])
        index += 1
        for depth_index in range(len(receiver_depths)):
            for range_index in range(len(receiver_ranges)):
                count = int(lines[index])
                index += 1
                arrivals = []
                for _ in range(count):
                    fields = lines[index].split()
                    index += 1
                    arrivals.append({
                        "amplitude": float(fields[0]),
                        "phase_deg": float(fields[1]),
                        "delay_s": float(fields[2]),
                        "attenuation_delay_s": float(fields[3]),
                        "source_angle_deg": float(fields[4]),
                        "receiver_angle_deg": float(fields[5]),
                        "top_bounces": int(fields[6]),
                        "bottom_bounces": int(fields[7]),
                    })
                records.append({"source_index": source_index,
                                "receiver_depth_index": depth_index,
                                "receiver_range_index": range_index,
                                "maximum_arrivals": maximum_arrivals,
                                "arrivals": arrivals})
    return {"frequency_hz": frequency_hz, "source_depths_m": source_depths,
            "receiver_depths_m": receiver_depths,
            "receiver_ranges_m": receiver_ranges, "records": records}


def solve(environment, directory):
    directory = Path(directory)
    ray_log = run(environment, directory, "R", "rays")
    arrival_log = run(environment, directory, "A", "arrivals")
    return read_rays(directory / "rays.ray"), read_arrivals(directory / "arrivals.arr"), {
        "ray_log": ray_log, "arrival_log": arrival_log,
        "executable": str(executable_path()),
    }
