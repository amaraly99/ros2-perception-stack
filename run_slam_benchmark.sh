#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# run_slam_benchmark.sh — THE way to run the HIL SLAM benchmark (unified method).
#
#   ./run_slam_benchmark.sh <slam> <tag>               # e.g. orbslam2 unified_orb2_mono_N10 (all = every SLAM)
#   ./run_slam_benchmark.sh --list                     # the SLAM names
#   ./run_slam_benchmark.sh --tag <name>               # all other settings from config/hil/slam_bench.yaml
#   ./run_slam_benchmark.sh --tag t --dry-run          # checks only (Wi-Fi, MATLAB, container, model)
#   ./run_slam_benchmark.sh --tag t --arms orbslam2 --runs 3   # a command-line option wins over the file
#   ./run_slam_benchmark.sh --help                     # every option
#
# Before: in MATLAB on Windows ->  cd <repo>\matlab ; hil_session_start_slam('mono')
# Guide:  benchmarks/SLAM_BENCHMARK.md
# ─────────────────────────────────────────────────────────────────────────────
VENV_PY="${HILRUN_PYTHON:-$HOME/.venvs/hilrun/bin/python}"
[[ -x "$VENV_PY" ]] || { echo "missing $VENV_PY — run: python3 -m venv ~/.venvs/hilrun && ~/.venvs/hilrun/bin/pip install rich pyyaml" >&2; exit 1; }
exec "$VENV_PY" "$(cd "$(dirname "$0")" && pwd)/benchmarks/slam_runner.py" "$@"
