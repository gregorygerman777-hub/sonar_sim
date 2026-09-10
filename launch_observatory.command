#!/bin/bash
set -euo pipefail
repo_dir="$(cd "$(dirname "$0")" && pwd)"
cd "$repo_dir"
checked_core="${SONAR_CORE_DIR:-$repo_dir/../sonar_sim_mac3d_audit_20260909}"
python_bin="${SONAR_PYTHON:-$checked_core/.venv/bin/python}"
if [[ ! -x "$python_bin" ]]; then
  echo 'Set SONAR_PYTHON to your Python interpreter and SONAR_CORE_DIR to the directory containing a freshly built sonar extension.' >&2
  exit 2
fi
# The local clean-build environment is deliberately separate from the linked legacy .venv.
for source_file in "$repo_dir"/core/*; do
  if [[ -f "$source_file" && "$checked_core" != "$repo_dir" ]]; then
    cmp -s "$source_file" "$checked_core/core/$(basename "$source_file")" || {
      echo 'The core changed since the checked build. Rebuild the extension before launching this preview.' >&2
      exit 2
    }
  fi
done
export SONAR_CORE_DIR="$checked_core"
exec "$python_bin" python/observatory_3d.py "$@"
