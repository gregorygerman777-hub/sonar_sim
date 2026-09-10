#!/bin/bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
repo_dir="$(cd "$project_dir/../.." && pwd)"
result_dir="$project_dir/Saved/checks/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$result_dir"
c++ -std=c++17 -O2 -pthread -I"$repo_dir/core" -I"$project_dir/Source/AbyssSonar" \
 "$project_dir/Tests/bridge_checks.cpp" "$repo_dir/core/geometry.cpp" "$repo_dir/core/mesh.cpp" \
 "$repo_dir/core/physics.cpp" "$repo_dir/core/simulator.cpp" "$repo_dir/core/reconstruction.cpp" \
 -o "$result_dir/bridge_checks"
"$result_dir/bridge_checks" "$project_dir/Content/Targets" | tee "$result_dir/results.txt"
