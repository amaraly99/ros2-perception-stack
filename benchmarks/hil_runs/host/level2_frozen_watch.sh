#!/usr/bin/env bash
# level2_frozen_watch.sh — DRAFT. Early frozen-camera detection from host files only
# (no docker exec, no ros2). Started in the background by level2_one_run.sh.
#
# usage: level2_frozen_watch.sh <out_dir> <tag> [action=kill|mark] [stamps=/tmp/sim_cam_stamps.csv]
#
# Signal: sim_camera_bridge writes /tmp/sim_cam_stamps.csv (host /tmp is mounted in the
# container) with t_sim_stamp_ns = MATLAB's sim clock stamped on each frame, flushed every
# 100 frames (5-10 s at 10-20 Hz). In a frozen run the sim clock does not advance (it stayed at
# 0.0 s for the whole run in all 4 frozen pi_det01 runs); in a live run it advances ~18 s per
# 20 s of wall time. Verdict once >=100 rows spanning >=5 s of wall time are on disk:
#   FROZEN if the sim clock advanced < 1.0 s over those rows (counted only after the last clock reset,
#   >=60 rows / >=4 s; stale pre-restart frames are excluded).
# The first verdict normally comes 5-10 s after frames start flowing (well inside 20 s).
set -u
OUT="$1"; TAG="$2"; ACTION="${3:-kill}"; F="${4:-/tmp/sim_cam_stamps.csv}"
T0=$(date +%s); LIMIT=180          # give up (no verdict) after 3 min
# run_hil_experiments.sh deletes the old stamps file before each run; wait for a fresh one
while :; do
  [ -f "$F" ] && [ "$(stat -c %Y "$F")" -ge "$T0" ] && break
  [ $(( $(date +%s) - T0 )) -ge "$LIMIT" ] && exit 0
  sleep 2
done
while :; do
  v=$(python3 - "$F" <<'PY'
import csv, sys
try:
    r = [x for x in csv.DictReader(open(sys.argv[1])) if x.get("t_sim_stamp_ns") and x.get("t_recv_mono_ns")]
except Exception:
    r = []
if len(r) < 100: print("WAIT", len(r)); sys.exit()
r = r[:300]                                    # judge the start of the run only
# Only the segment after the LAST sim-clock reset counts: frames re-sent from before the restart carry the old
# stamp (e.g. 75.4 s), and max-min across that reset would make a frozen restart (clock stuck at 0) look live.
cut = 0
for i in range(1, len(r)):
    if int(r[i]["t_sim_stamp_ns"]) < int(r[i-1]["t_sim_stamp_ns"]) - 5e8: cut = i
r = r[cut:]
if len(r) < 60: print("WAIT", len(r)); sys.exit()
wall = (int(r[-1]["t_recv_mono_ns"]) - int(r[0]["t_recv_mono_ns"])) / 1e9
sims = [int(x["t_sim_stamp_ns"]) for x in r]
adv = (max(sims) - min(sims)) / 1e9
if wall < 4: print("WAIT", len(r)); sys.exit()
print("FROZEN" if adv < 1.0 else "LIVE", len(r), f"{wall:.1f}", f"{adv:.2f}")
PY
)
  set -- $v
  case "$1" in
    LIVE)   echo "$(date -u +%FT%TZ) $TAG early check LIVE: rows=$2 wall=${3}s sim_advance=${4}s" >> "$OUT/runner.log"; exit 0 ;;
    FROZEN)
      cp -f "$F" "$OUT/${TAG}_early_cam_stamps.csv" 2>/dev/null
      {
        echo "EARLY_FROZEN $(date -u +%FT%TZ) tag=$TAG"
        echo "  sim clock advanced ${4}s over $2 frames / ${3}s wall (threshold 1.0 s)"
        echo "  evidence: $OUT/${TAG}_early_cam_stamps.csv"
        echo "  action: $ACTION"
      } | tee "$OUT/${TAG}_EARLY_FROZEN.txt" >> "$OUT/runner.log"
      if [ "$ACTION" = kill ]; then
        # anchored to the run script's exec form (/bin/bash <path>/run_hil_experiments.sh[...] ... --label level2_<tag>),
        # so a shell whose command line merely mentions the label is never matched
        pkill -TERM -f "^/bin/bash [^ ]*run_hil_experiments\.sh[^ ]* .*--label level2_${TAG}( |$)" 2>/dev/null
        sleep 3
        # 5Q: the pinned stack worktree's launcher (was our repo's run_stack_hil.sh, which no longer starts the stack)
        (cd "${HIL_WT:-$HOME/amar_4490660}" && ./run_stack_hil.sh stop >> "$OUT/${TAG}_early_stop.log" 2>&1)
      fi
      exit 3 ;;
  esac
  [ $(( $(date +%s) - T0 )) -ge "$LIMIT" ] && { echo "$(date -u +%FT%TZ) $TAG early check: no verdict in ${LIMIT}s" >> "$OUT/runner.log"; exit 0; }
  sleep 2
done
