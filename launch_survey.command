#!/bin/zsh
repo_dir="$(cd "$(dirname "$0")" && pwd)"
cd "$repo_dir" || exit 1
exec "${SONAR_PYTHON:-.venv/bin/python}" python/survey_lab.py "$@"
