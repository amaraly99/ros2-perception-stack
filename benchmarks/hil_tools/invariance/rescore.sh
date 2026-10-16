#!/usr/bin/env bash
# rescore.sh <out root> <tag> <check_run.py> <batches.tsv> [run list]: re-run check_run.py with the arguments run_one.sh passes
# (check_run.py <run>/stack <label> --pre-resume-dir <run> --out-dir <out>) on every run of the listed batches, into
# <out root>/<tag>/<batch basename>/<run>/. Never into run folders. batches.tsv lines: "<session> <batch folder>" under
# $HIL_RUNS_ROOT (default /mnt/usb/hil_runs). A run list file ("<batch folder> <run dir>" per line) restricts the set.
# Generalised from stage6/prescore_16A/rescore_16A.sh (16A). The HIL_OURS, HIL_GEOM and HIL_HELD_REF variables are unset for the
# call, so an archived check_run.py uses the archive's own scorer and config even when hil_env.sh was sourced in this shell.
set -u
OUT="$1"; TAG="$2"; CR="$3"; BATCHES="$4"; LIST="${5:-}"
H="${HIL_RUNS_ROOT:-/mnt/usb/hil_runs}"; PY="${HIL_MCAP_PY:-$HOME/venvs/mcap/bin/python}"
run_one() {  # run_one <batch folder> <run dir name>
  local b="$1" r="$2" rd="$H/$1/$2" out="$OUT/$TAG/$(basename "$1")/$2" label="level2_${2#*_}"
  [ -d "$rd/approach_bag" ] || return 0
  mkdir -p "$out"
  env -u HIL_OURS -u HIL_GEOM -u HIL_HELD_REF "$PY" "$CR" "$rd/stack" "$label" --pre-resume-dir "$rd" --out-dir "$out" > "$out/${label}_check.txt" 2>&1
  echo "$b $r rc=$?" >> "$OUT/$TAG/progress.txt"
}
[ -e "$OUT/$TAG" ] && { echo "rescore: REFUSE: $OUT/$TAG exists (every re-score writes into a new tree)"; exit 20; }
mkdir -p "$OUT/$TAG"; : > "$OUT/$TAG/progress.txt"
if [ -n "$LIST" ]; then
  while read -r b r; do [ -n "$b" ] && run_one "$b" "$r"; done < "$LIST"
else
  grep -vE '^\s*(#|$)' "$BATCHES" | while read -r s b _; do
    for rd in "$H/$b"/0[0-9][0-9]_*; do [ -d "$rd" ] && run_one "$b" "$(basename "$rd")"; done
  done
fi
echo DONE >> "$OUT/$TAG/progress.txt"; echo "rescore $TAG: $(grep -c 'rc=' "$OUT/$TAG/progress.txt") runs into $OUT/$TAG"
