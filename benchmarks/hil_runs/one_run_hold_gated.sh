#!/usr/bin/env bash
# usage: one_run_hold_gated.sh <model> <k>   (one_run_hold.sh + the HOLD_GATE_CMD hook, 2026-10-04)     e.g. one_run_hold.sh yolo26n 1
# Copy of ~/reproduction/level2_one_run_v3.sh (the per-run script of the 89 hold runs, DEVIATIONS 17.18).
# Only change: backend, placement and detector cores are env-overridable. Defaults = the original (cpu, B, 0,1).
#   NPU (DEVIATIONS 17.50 A3): L2_BACKEND=npu L2_PLACEMENT=A L2_DETECTOR_CORE= (empty = no --detector-core flag,
#   so the placement preset decides).
# 5Q (benchmarks/hil_runs): paths from hil_env.sh; L2_OUT is required (the run's own folder) and is the working directory;
# the held state must report the fixed hold sim time (config/held_reference.json hold_sim_t) or the run stops (exit 7).
set -u
M="$1"; K="$2"
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
OUT="${L2_OUT:?set L2_OUT to the run folder}"; mkdir -p "$OUT"
L2_BACKEND="${L2_BACKEND:-cpu}"; L2_PLACEMENT="${L2_PLACEMENT:-B}"
DC="${L2_DETECTOR_CORE-0,1}"; DCARG=(); [ -n "$DC" ] && DCARG=(--detector-core "$DC")
TAG="${M}_${L2_BACKEND}_${L2_PLACEMENT}_r${K}"
LOG="$OUT/runner.log"
cd "$OUT" || exit 1

# Cooldown between runs. Level 2 used a <50 C gate (Level 1 protocol), but this Pi idles
# at 50-52 C, so the gate spent 90-270 s waiting out the last 1-2 C -- while the reference
# sweep had no gate at all and ran back to back. Replaced 2026-09-22 (design review F1,
# approved) with a fixed cooldown. L2_TEMP_GATE=1 restores the old behaviour exactly.
w=0
if [ "${L2_TEMP_GATE:-0}" = 1 ]; then
  while :; do
    t=$(vcgencmd measure_temp | sed 's/[^0-9.]//g')
    awk -v a="$t" 'BEGIN{exit !(a<50)}' && break
    [ "$w" -ge 600 ] && break
    sleep 15; w=$((w+15))
  done
else
  w="${L2_COOLDOWN_S:-60}"
  [ "$w" -gt 0 ] && sleep "$w"
fi
t=$(vcgencmd measure_temp | sed 's/[^0-9.]//g')
{ echo "=== $TAG ==="; echo "start_utc: $(date -u +%FT%TZ)"
  echo "start_temp_c: $t"; echo "temp_wait_s: $w"
  echo "load1: $(cut -d' ' -f1 /proc/loadavg)"; echo "top5:"
  ps -eo pcpu,pmem,comm --sort=-pcpu | head -6; } >> "$LOG"

# thermal/clock/throttle every 1 s
bash "$HIL_HOST/pi_telemetry.sh" "$OUT/${TAG}_thermal.csv" 1 &
TP=$!
# memory every 5 s: MemAvailable and swap used, in kB
( echo "utc,memavailable_kb,swap_used_kb"
  while :; do
    ma=$(awk '/^MemAvailable:/{print $2}' /proc/meminfo)
    su=$(awk '/^SwapTotal:/{t=$2} /^SwapFree:/{f=$2} END{print t-f}' /proc/meminfo)
    echo "$(date -u +%FT%TZ),$ma,$su"
    sleep 5
  done ) > "$OUT/${TAG}_mem.csv" &
MP=$!
# per-core utilization every 1 s + top 8 threads per 15 s (deviation from plan 1a; pinned to core 0)
taskset -c 0 python3 "$HIL_HOST/level2_coresampler.py" "$OUT/${TAG}_cores.csv" "$OUT/${TAG}_topthreads.csv" &
CP=$!

# ── MATLAB orchestration v3: HOLD at the initial condition (DEVIATIONS 17.18) ──
#   pub_start 20 -> start_hold -> poll for state=held -> PRE-RUN IC GUARD on the
#   held pose -> boot the stack while still held -> `resume` the instant the stack
#   log says "HIL stack is running." -> run -> stop -> pub_stop.
#   The drone therefore never free-runs unsupervised: it waits at its IC until the
#   controller exists. Requires supervisor v3 (start_hold/resume/pose in status).
MLOG="$OUT/${TAG}_matlab.log"
STACK_LOG_PATH="$OUT/level2_${TAG}_run1_stack.log"
BEAR="$HIL_RUNS/bearing_from_status.py"
ORCH_EXTRA="--no-matlab-ctl"

{ echo "orchestrate: full_hold (v3)"; echo "status_before: $(hil status)"; } >> "$MLOG"
r=$(hil pub_start "${L2_PUB_HZ:-20}"); echo "pub_start ${L2_PUB_HZ:-20}: $r" >> "$MLOG"
if [ "$r" != ok ]; then
  echo "ORCH_FAILED pub_start reply '$r'" | tee "$OUT/${TAG}_ORCH_FAILED.txt" >> "$LOG"
  kill "$TP" "$MP" "$CP" 2>/dev/null; wait "$TP" "$MP" "$CP" 2>/dev/null; exit 4
fi
echo "start_hold: $(hil start_hold)" >> "$MLOG"

# wait for the supervisor to pause the model at its IC
HELD_LINE=""; 
for i in $(seq 1 "${L2_HOLD_WAIT_S:-90}"); do
  st=$(hil status)
  case "$st" in *state=held*) HELD_LINE="$st"; break ;; esac
  sleep 1
done
echo "after_start_hold (${i} s): ${HELD_LINE:-$(hil status)}" >> "$MLOG"

if [ -z "$HELD_LINE" ]; then
  echo "ORCH_FAILED never reached state=held" | tee "$OUT/${TAG}_ORCH_FAILED.txt" >> "$LOG"
  hil stop >/dev/null; hil pub_stop >/dev/null
  kill "$TP" "$MP" "$CP" 2>/dev/null; wait "$TP" "$MP" "$CP" 2>/dev/null; exit 4
fi
echo "$HELD_LINE" > "$OUT/held_status.txt"

# ── 5Q: FIXED HOLD. The supervisor must report the fixed pause time at state=held ──────────────
WANT_T=$(python3 -c "import json,sys; print(f\"{float(json.load(open(sys.argv[1]))['hold_sim_t']):.2f}\")" "$HIL_HELD_REF")
GOT_T=$(echo "$HELD_LINE" | grep -oP '\bt=\K[0-9.]+')
echo "hold_sim_t: got=${GOT_T:-none} want=$WANT_T" >> "$MLOG"
if [ "${GOT_T:-x}" != "$WANT_T" ]; then
  echo "hold: FAIL sim time at hold t=${GOT_T:-none}, expected $WANT_T -> not resuming" >> "$LOG"
  hil stop >/dev/null; hil pub_stop >/dev/null
  kill "$TP" "$MP" "$CP" 2>/dev/null; wait "$TP" "$MP" "$CP" 2>/dev/null
  exit 7   # 7 = held at the wrong sim time
fi

# ── PRE-RUN IC GUARD: bearing error at the held pose, before anything moves ────
PRE=$(python3 "$BEAR" "$HELD_LINE"); PRC=$?
PRE_ERR=$(echo "$PRE" | cut -d' ' -f1); PRE_YAW=$(echo "$PRE" | cut -d' ' -f2)
echo "pre_ic_guard: err=${PRE_ERR}deg yaw=${PRE_YAW} rc=$PRC" >> "$MLOG"
echo "$(date -u +%FT%TZ) PRE_IC ${TAG} rc=$PRC err=${PRE_ERR} yaw=${PRE_YAW} raw='${PRE}'" >> "$OUT/pre_ic_guard.log"
if [ "$PRC" -ne 0 ]; then
  echo "pre_ic_guard: FAIL (err=${PRE_ERR} deg) -> aborting run before it costs anything" >> "$LOG"
  hil stop >/dev/null; hil pub_stop >/dev/null
  kill "$TP" "$MP" "$CP" 2>/dev/null; wait "$TP" "$MP" "$CP" 2>/dev/null
  exit 5   # 5 = pre-run IC fault
fi

# ── PRE-RESUME GATE (2026-10-04, prompt 5D). Only change from one_run_hold.sh. ──────────
# Runs while held and BEFORE the stack boots, so the gate has the NPU and CPUs to itself and the
# frozen-camera watcher never sees a long paused clock. HOLD_GATE_CMD empty = original behaviour.
if [ -n "${HOLD_GATE_CMD:-}" ]; then
  echo "$(date -u +%FT%TZ) hold_gate: start" >> "$MLOG"
  bash -c "$HOLD_GATE_CMD" >> "$MLOG" 2>&1; HGRC=$?
  echo "$(date -u +%FT%TZ) hold_gate: rc=$HGRC" >> "$MLOG"
  if [ "$HGRC" -ne 0 ]; then
    echo "hold_gate: FAIL (rc=$HGRC) -> not resuming" >> "$LOG"
    hil stop >/dev/null; hil pub_stop >/dev/null
    kill "$TP" "$MP" "$CP" 2>/dev/null; wait "$TP" "$MP" "$CP" 2>/dev/null
    exit 6   # 6 = pre-resume gate failed
  fi
fi

# ── resume watcher: release the hold the moment the stack is up ────────────────
rm -f "$STACK_LOG_PATH"
(
  for _ in $(seq 1 "${L2_RESUME_WAIT_S:-180}"); do
    if [ -f "$STACK_LOG_PATH" ] && grep -q "HIL stack is running" "$STACK_LOG_PATH" 2>/dev/null; then
      for _t in 1 2 3 4 5; do
        rr=$(hil resume)
        echo "$(date -u +%FT%TZ) resume -> $rr" >> "$MLOG"
        s2=$(hil status)
        echo "$(date -u +%FT%TZ) status after resume: $s2" >> "$MLOG"
        case "$s2" in *sim=running*) exit 0 ;; esac
        sleep 1
      done
      exit 0
    fi
    sleep 1
  done
  echo "$(date -u +%FT%TZ) resume watcher TIMED OUT; resuming anyway" >> "$MLOG"
  hil resume >> "$MLOG" 2>&1
) &
RESUME_PID=$!

# early frozen-camera watcher (host files only; kills this run if the sim clock does not advance).
# Exits after its verdict (~10-15 s after frames flow). L2_EARLY_ACTION=mark only records it.
bash "$HIL_HOST/level2_frozen_watch.sh" "$OUT" "$TAG" "${L2_EARLY_ACTION:-kill}" &
WP=$!

# L2_RUNNER: alternative run script (default: the repo's); L2_EXTRA: extra flags, e.g. --bag
MATLAB_HOST_IP="$HIL_CTL_HOST" "${L2_RUNNER:-$HIL_RUNS/runner_adapter_hold.sh}" ${L2_EXTRA:-} ${ORCH_EXTRA} \
    --detector "${L2_DETECTOR:-yolo}" --model "$M" --backend "$L2_BACKEND" \
    --placement "$L2_PLACEMENT" "${DCARG[@]}" --slam-mode fast \
    --duration "${L2_DURATION:-60}" --runs 1 \
    --label "level2_${TAG}" \
    --output-dir "$OUT/"
RC=$?
kill "$RESUME_PID" 2>/dev/null; wait "$RESUME_PID" 2>/dev/null
echo "stop: $(hil stop)" >> "$MLOG"
echo "pub_stop: $(hil pub_stop)" >> "$MLOG"
echo "status_after: $(hil status)" >> "$MLOG"
wait "$WP" 2>/dev/null; WRC=$?

kill "$TP" "$MP" "$CP" 2>/dev/null; wait "$TP" "$MP" "$CP" 2>/dev/null
{ echo "exit_rc: $RC"; echo "early_watch_rc: $WRC"; echo "end_utc: $(date -u +%FT%TZ)"; echo; } >> "$LOG"

# ── POST-RUN IC GUARD (17.14) + pre/post pose agreement check ─────────────────
GUARD_INT="${IC_GUARD_PYTHON:-$HIL_MCAP_PY}"
GUARD_PY="$HIL_RUNS/ic_guard.py"
GRC=2
if [ -f "$GUARD_PY" ] && [ -x "$GUARD_INT" ]; then
  GLINE=$("$GUARD_INT" "$GUARD_PY" "$OUT/level2_${TAG}_run1" 2>&1); GRC=$?
  GJSON=$("$GUARD_INT" "$GUARD_PY" --json "$OUT/level2_${TAG}_run1" 2>/dev/null)
  echo "$(date -u +%FT%TZ) $GLINE" >> "$OUT/ic_guard.log"
  [ -n "$GJSON" ] && echo "$GJSON" >> "$OUT/ic_guard.jsonl"
  echo "ic_guard: $GLINE" >> "$LOG"
  # STEP 1 check: does the first RECORDED pose match the pre-run HELD pose?
  POSTYAW=$(echo "$GJSON" | python3 -c "import sys,json;print(f\"{json.load(sys.stdin)['yaw_deg']:.2f}\")" 2>/dev/null)
  if [ -n "$POSTYAW" ] && [ -n "${PRE_YAW:-}" ]; then
    DY=$(python3 -c "print(f'{abs((($POSTYAW)-($PRE_YAW)+180)%360-180):.2f}')" 2>/dev/null)
    echo "pose_match: pre_yaw=$PRE_YAW post_yaw=$POSTYAW dyaw=${DY}deg" | tee -a "$LOG" >> "$OUT/pose_match.log"
  fi
else
  echo "ic_guard: SKIPPED" >> "$LOG"
fi
echo >> "$LOG"
if [ "$RC" -ne 0 ]; then exit "$RC"; fi
exit "$GRC"
