#!/usr/bin/env bash
set -euo pipefail

archive="${1:-}"
if [[ -z "$archive" || ! -f "$archive" ]]; then
    echo "usage: $0 /path/to/atWin10_2020_11_4.zip" >&2
    exit 2
fi
if ! command -v gfortran >/dev/null; then
    echo "gfortran is required" >&2
    exit 1
fi

destination="${BELLHOP_INSTALL_DIR:-$HOME/.local/opt/acoustics-toolbox-2020}"
if [[ -e "$destination" ]]; then
    echo "installation already exists: $destination"
    echo "BELLHOP executable: $destination/Bellhop/bellhop.exe"
    exit 0
fi

parent="$(dirname "$destination")"
mkdir -p "$parent"
stage="$(mktemp -d "$parent/acoustics-toolbox-stage.XXXXXX")"
unzip -q "$archive" 'atWin10_2020_11_4/*' -d "$stage"
mv "$stage/atWin10_2020_11_4" "$destination"
rmdir "$stage"

flags='-std=gnu -O2 -ffast-math -funroll-all-loops -fomit-frame-pointer -I../misc -I../tslib'
make -C "$destination/misc" -k all FC=gfortran "FFLAGS=$flags"
make -C "$destination/Bellhop" -k all FC=gfortran "FFLAGS=$flags"

"$destination/Bellhop/bellhop.exe" 2>&1 | head -1 || true
echo "BELLHOP executable: $destination/Bellhop/bellhop.exe"
