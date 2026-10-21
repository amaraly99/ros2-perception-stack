#!/usr/bin/env bash
# usage: one_run_hold_gated.sh <model> <k>   (one_run_hold.sh + the HOLD_GATE_CMD hook, 2026-10-04)     e.g. one_run_hold.sh yolo26n 1
# Copy of ~/reproduction/level2_one_run_v3.sh (the per-run script of the 89 hold runs, DEVIATIONS 17.18).
# Only change: backend, placement and detector cores are env-overridable. Defaults = the original (cpu, B, 0,1).
#   NPU (DEVIATIONS 17.50 A3): L2_BACKEND=npu L2_PLACEMENT=A L2_DETECTOR_CORE= (empty = no --detector-core flag,
#   so the placement preset decides).
# 5Q (benchmarks/hil_runs): paths from hil_env.sh; L2_OUT is required (the run's own folder) and is the working directory;
# the held state must report the fixed hold sim time (config/held_reference.json hold_sim_t) or the run stops (exit 7).
# 8I/8J: the hold is released only once the detector has produced a target box on the held frame AND the controller's
# sim-health latch is engaged; neither within L2_DET_WAIT_S (default 30 s) marks the run INVALID and stops the batch
# (exit 8). The gate writes the exact resume wall ns to RUN_DIR/t_resume_ns.txt. Post-run IC guard: run_one.sh.
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
# 20B: per-process memory (PSS, RSS, peak RSS from /proc) and the eth0 byte counters, every 2 s,
# pinned to core 0 like the core sampler. The host-wide ${TAG}_mem.csv above stays exactly as it is,
# so every stored run's memory record keeps its meaning; this adds the per-component figure the
# Level 2 table needs. Only started when the caller asks (L2_PROCMEM=1), so a detector-sweep rerun
# writes the same set of files as before.
if [ "${L2_PROCMEM:-0}" = 1 ]; then
  taskset -c 0 python3 "$HIL_HOST/level2_procmem.py" "$OUT/${TAG}_procmem.csv" "$OUT/${TAG}_link.csv" 2 "${L2_IFACE:-eth0}" &
  PMP=$!
fi

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
  kill "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; wait "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; exit 4
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
  kill "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; wait "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; exit 4
fi
echo "$HELD_LINE" > "$OUT/held_status.txt"

# ── 5Q: FIXED HOLD. The supervisor must report the fixed pause time at state=held ──────────────
WANT_T=$(python3 -c "import json,sys; print(f\"{float(json.load(open(sys.argv[1]))['hold_sim_t']):.2f}\")" "$HIL_HELD_REF")
GOT_T=$(echo "$HELD_LINE" | grep -oP '\bt=\K[0-9.]+')
echo "hold_sim_t: got=${GOT_T:-none} want=$WANT_T" >> "$MLOG"
if [ "${GOT_T:-x}" != "$WANT_T" ]; then
  echo "hold: FAIL sim time at hold t=${GOT_T:-none}, expected $WANT_T -> not resuming" >> "$LOG"
  hil stop >/dev/null; hil pub_stop >/dev/null
  kill "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; wait "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null
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
  kill "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; wait "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null
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
    kill "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; wait "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null
    exit 6   # 6 = pre-resume gate failed
  fi
fi

# ── resume watcher: release the hold only after the detector's FIRST target box (8I, DEVIATIONS 17.67) ──
# Until 8I the hold was released the moment the stack was up. Before its first detection the FSM sits in
# SEARCHING/FULL_ROTATE and commands search_spin_speed (0.5 rad/s), and the supervisor applies the last
# /cmd_vel it received during the hold, so a detector that needs seconds per frame (CPU m-models: about
# 2.0 s) starts the mission already turning away from the sign. Run 006 of 2026-10-08 lost 53.00 deg of
# heading that way. The gate costs paused seconds only: detector start-up now falls outside the measured
# window, which is the usual convention for latency work.
# Source of truth for "first target detection": the sidecar's detection logger (/tmp/r20_dets.log; host
# wall ns in field 1, count of stop-sign boxes at conf >= 0.20 in field 3), the same file the 10 s stop
# rule reads. The host needs no ROS for it, and its class and confidence match the controller's
# target_class / min_confidence. On timeout the run is INVALID and the batch stops: the session gate
# already proved every configuration detects the held frame, so no detection here is plumbing, not a result.
# 8J (B): the gate waits for TWO conditions, so every run starts in the same state.
#   (i)  a target detection on the held frame, and
#   (ii) the controller's sim-health latch engaged ("Sim health LOST: SIM_PAUSED", cmd_vel held at 0).
# Why (ii). The latch needs sim_unhealthy_entry_sec = 1.0 s of observed PAUSED, which accrues in wall time while the
# model is held, so whether it was engaged at resume used to depend on how long the controller had been up: session 1
# resumed 001, 003, 004 latched (motionless, first non-zero /cmd_vel at +0.63, +0.60, +0.51 s) and 002, 005, 006
# unlatched with a live command already in force (002 vx 1.488, 005 vx 1.481, 006 wz +0.500). That split lines up with
# CPU against NPU, which is exactly the confound session 1 was discarded for. Holding until the latch is engaged makes
# it the same for every run: motionless at resume, commands from about +0.5 s when the 0.5 s recovery debounce clears.
# The latch is deterministic without touching the controller, because waiting costs only paused wall time and the
# condition is a pure function of it (DEVIATIONS 17.67; VERIFIED on 001, 003, 004: one LOST line each, no RECOVERED
# before resume). The log line is RCLCPP_ERROR, unthrottled, emitted once per transition, and lands in the same stack
# log the "HIL stack is running" wait already reads, so no fixed-wait fallback is needed.
# NOTE the gate does NOT wait for APPROACHING. Lock-on needs consecutive_dets_ >= lockon_consec (2) while
# watchdog_tick zeroes that counter whenever a detection is older than 2 x watchdog_period_sec (0.4 s), so a CPU
# m-model at about 0.5 Hz may never lock on; gating on APPROACHING would time out and stop the batch on its first
# such run. See DEVIATIONS 17.67 (e).
# 9B (item 6): the latch line is read from the LIVE launch log, not from the stack log. The stack log gets
# node output only when run_hil_experiments.sh's save_logs() appends /tmp/hil_launch.log at teardown, so the
# 8J gate was polling a file that could not hold the line until after the run was over (9A: the gate waited
# its full 30 s while the latch had engaged 30 s earlier). ros2 launch writes node output to
# /tmp/hil_launch.log (worktree run_stack_hil.sh:300) and /tmp is bind mounted, so the host reads it live.
# Only THIS run's line may satisfy the gate, enforced two ways, both needed:
#   (i)  the file is emptied before the stack starts. The worktree runner already removes it at the top of
#        every run, which is why no Oct 8 stack log holds a previous run's node lines (9B item 1, VERIFIED:
#        0 cross-run lines in all six). We empty it here as well, from the host with sudo because container
#        root owns it through the bind mount, so the gate does not depend on the runner's ordering.
#   (ii) the latch stamp must be at or after this run's stack start, captured here before the runner is
#        called. Second granularity is enough and exact: the stack takes 60 to 90 s to boot, so a stale line
#        is minutes old, while a 19-digit ns value would lose precision in awk's doubles (256 ns at 1.8e18).
# Not changed: node output may still be block buffered by ros2 launch's Python stdout, which cannot be made
# line buffered without editing the pinned worktree. Measured bound (9B item 3): in 9A run 001 the latch line
# sat at byte offset 3672, inside the first 8192-byte block, and 8192 bytes were produced 8.263 s after the
# latch stamp, so the worst case is well inside L2_DET_WAIT_S. The gate records what it actually cost in
# t_latch_gate.txt (latch stamp, when the gate saw it, stack start) so the real delay is measured per run.
DETLOG="${L2_DETLOG:-/tmp/r20_dets.log}"
LAUNCHLOG="${L2_LAUNCHLOG:-/tmp/hil_launch.log}"
RUN_ROOT="$(dirname "$OUT")"
rm -f "$STACK_LOG_PATH"
sudo rm -f "$LAUNCHLOG" 2>/dev/null || true
T_STACK_START_NS=$(date +%s%N); T_STACK_START_S=${T_STACK_START_NS:0:10}
echo "$T_STACK_START_NS" > "$RUN_ROOT/t_stack_start_ns.txt"
echo "$(date -u +%FT%TZ) resume gate: stack start $T_STACK_START_NS, $LAUNCHLOG emptied; only latch lines stamped >= ${T_STACK_START_S} count" >> "$MLOG"
(
  for _ in $(seq 1 "${L2_RESUME_WAIT_S:-180}"); do
    if [ -f "$STACK_LOG_PATH" ] && grep -q "HIL stack is running" "$STACK_LOG_PATH" 2>/dev/null; then
      DT=""; LATCHED=""; LSEEN=""
      for _d in $(seq 1 $(( ${L2_DET_WAIT_S:-30} * 4 ))); do
        [ -z "$DT" ] && DT=$(awk '$3 > 0 { print $1; exit }' "$DETLOG" 2>/dev/null)
        if [ -z "$LATCHED" ]; then
          # this run's latch stamp only: at or after the stack start captured above
          LATCHED=$(awk -v t0="$T_STACK_START_S" '
            index($0, "Sim health LOST") > 0 {
              if (match($0, /\[[0-9]{10}\.[0-9]{9}\]/)) {
                s = substr($0, RSTART + 1, RLENGTH - 2); split(s, a, ".")
                if (a[1] + 0 >= t0 + 0) { print s; exit }
              }
            }' "$LAUNCHLOG" 2>/dev/null)
          [ -n "$LATCHED" ] && LSEEN=$(date +%s%N)
        fi
        [ -n "$DT" ] && [ -n "$LATCHED" ] && break
        sleep 0.25
      done
      if [ -z "$DT" ] || [ -z "$LATCHED" ]; then
        GR="no detection on held frame"
        if [ -n "$DT" ]; then
          GR="sim-health latch not engaged (cmd_vel not held at 0)"
          # distinguish "never engaged" from "only a previous run's line is present", which the stamp
          # guard rejects on purpose: the two need different fixes, so say which one happened.
          grep -q "Sim health LOST" "$LAUNCHLOG" 2>/dev/null && \
            GR="sim-health latch line present but stamped before this run's stack start (stale, rejected)"
        fi
        [ -z "$DT" ] && [ -z "$LATCHED" ] && GR="no detection on held frame and sim-health latch not engaged"
        echo "$(date -u +%FT%TZ) resume gate: $GR within ${L2_DET_WAIT_S:-30} s (det=${DT:-none} latched=${LATCHED:-no}, stack_start=${T_STACK_START_S}) -> not resuming" >> "$MLOG"
        echo "$GR" > "$RUN_ROOT/NO_DET_ON_HELD_FRAME.txt"
        hil stop >> "$MLOG" 2>&1
        (cd "$HIL_WT" && ./run_stack_hil.sh stop) >> "$MLOG" 2>&1
        exit 8
      fi
      echo "$DT" > "$RUN_ROOT/t_first_det_ns.txt"
      # 9B: what the latch cost this run. Three fields: the latch's own ROS stamp, the wall ns at which the
      # gate first saw it, and this run's stack start. seen minus stamp is the gate delay (launch stdout
      # buffering plus the 0.25 s poll); stamp minus stack start is how long the stack took to latch.
      printf 'latch_stamp=%s gate_saw_ns=%s stack_start_ns=%s\n' "$LATCHED" "$LSEEN" "$T_STACK_START_NS" \
        > "$RUN_ROOT/t_latch_gate.txt"
      echo "$(date -u +%FT%TZ) resume gate: first target detection on the held frame at t_first_det_ns=$DT, sim-health latch engaged, latch_stamp=$LATCHED gate_saw_ns=$LSEEN stack_start_ns=$T_STACK_START_NS" >> "$MLOG"
      for _t in 1 2 3 4 5; do
        # 8J (A): the EXACT resume time. This wall ns is taken immediately before the resume command is sent and is
        # what every check uses (t_resume_ns.txt). The sidecar's value, which it derives by polling the matlab log
        # every 0.25 s, lags this by up to that poll interval and now goes to t_resume_ns_sidecar.txt for comparison
        # only. The lag mattered once the gate made every run resume with a detection already in hand: session 1's
        # run 005 resumed in APPROACHING and its "pre-resume" pose was already 0.0894 m along, against start_tol_m
        # 0.1 m. t_resume_ns.txt stays one field so its readers (ic_guard.py, check_run.py) need no format change.
        TRW=$(date +%s%N)
        rr=$(hil resume)
        [ -f "$RUN_ROOT/t_resume_ns.txt" ] || echo "$TRW" > "$RUN_ROOT/t_resume_ns.txt"
        echo "$(date -u +%FT%TZ) resume -> $rr" >> "$MLOG"
        echo "$(date -u +%FT%TZ) resume gate: resume sent at wall ns $TRW, $(python3 -c "print(f'{($TRW - $DT) / 1e9:.3f}')") s after the first detection" >> "$MLOG"
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

kill "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null; wait "$TP" "$MP" "$CP" ${PMP:-} 2>/dev/null
{ echo "exit_rc: $RC"; echo "early_watch_rc: $WRC"; echo "end_utc: $(date -u +%FT%TZ)"; echo; } >> "$LOG"

# ── 8I: the post-run IC guard (17.14) now runs in run_one.sh, not here. It reads the LAST pre-resume
# /sim/drone_pose from RUN_DIR/approach_bag, and that bag is still the live recorder's /tmp/r20_bag at
# this point: run_one.sh stops the recorder and moves it into RUN_DIR after this script returns. A bad
# initial condition still stops the batch, through check_run.py's start_pose_expected on that same
# pre-resume pose; what changes is that smoke_rc now means only "did the stack run".
echo >> "$LOG"
if [ -f "$RUN_ROOT/NO_DET_ON_HELD_FRAME.txt" ]; then
  echo "resume gate: no target detection on the held frame -> run INVALID" >> "$LOG"
  exit 8   # 8 = resume gate: no detection on the held frame
fi
exit "$RC"
