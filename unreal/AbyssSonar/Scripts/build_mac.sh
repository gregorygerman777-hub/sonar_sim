#!/bin/bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
engine_root="${UNREAL_ENGINE_ROOT:-}"
if [[ -z "$engine_root" ]]; then
  echo 'Set UNREAL_ENGINE_ROOT to the installed Unreal Engine directory, such as /Users/Shared/Epic Games/UE_5.8.' >&2
  exit 2
fi
build_script="$engine_root/Engine/Build/BatchFiles/Mac/Build.sh"
editor="$engine_root/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"
if [[ ! -f "$build_script" || ! -x "$editor" ]]; then
  echo 'Unreal Engine build tools were not found at UNREAL_ENGINE_ROOT.' >&2
  exit 2
fi
if [[ "$(xcode-select -p)" == *CommandLineTools* ]]; then
  echo 'Full Xcode is required. Select its developer directory before building Unreal.' >&2
  exit 2
fi
/bin/bash "$build_script" AbyssSonarEditor Mac Development "$project_dir/AbyssSonar.uproject" -WaitMutex
"$editor" "$project_dir/AbyssSonar.uproject" -ExecutePythonScript="$project_dir/Scripts/bootstrap.py"
