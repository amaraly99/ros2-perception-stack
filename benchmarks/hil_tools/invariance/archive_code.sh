#!/usr/bin/env bash
# archive_code.sh <commit> <dest dir>: the run code (benchmarks/hil_runs, benchmarks/inloop_accuracy.py) of one commit, via git archive,
# so check_run.py of that commit can be run against stored run folders (its hilpaths resolves the archive as the repo root).
set -eu
C="$1"; D="$2"; ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
mkdir -p "$D"
git -C "$ROOT" archive "$C" benchmarks/hil_runs benchmarks/inloop_accuracy.py | tar -x -C "$D"
echo "$C" > "$D/COMMIT"; echo "archived $(git -C "$ROOT" rev-parse --short "$C") into $D"
