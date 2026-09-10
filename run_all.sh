#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
PYTHON="${SONAR_PYTHON:-./.venv/bin/python}"
exec "$PYTHON" python/reproduce.py
