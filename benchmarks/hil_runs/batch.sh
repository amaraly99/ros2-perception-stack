#!/usr/bin/env bash
# batch.sh <plan.tsv> <batch_dir>: run a list of fixed-hold runs unattended, one after another (5Q).
#   Plan lines (tab separated, '#' comments): key  model  backend  role      (role: row | sentinel | pilot | expect_stop)
#   14A: an optional 5th column names the stack config (config/hil/stack/<name>.yaml in $HIL_WT) for that run; it must be
#   ours_fx554_<placement>_ibvs or start with ours_fx554_<placement>_ibvs_ and exist. A 4-column line runs exactly as before.
#   16A (DEVIATIONS 17.77): on an ablation row (key starting with "lw", session 5a, never pooled) the two geometry-gate checks are
#   recorded in the log instead of stopping the batch; sentinel and main-table keys (sen*, c*, x*) stay gated.
#   expect_stop (SSD from 20 m, which sees no sign: 5M): the 10 s stop rule is expected to end the run. Accepted only if
#   STOP_RULE.txt exists, no guard exit fired (run_one rc not in 4 5 6 7 10-14 20) and the checks that do not need the
#   approach pass (start pose, interface, RMW, fx, SLAM off, clock offset, held render); rates, scorer and distance are not judged.
#   e.g.  s1n01  yolo26n  npu  sentinel
#   NPU runs: placement A, stack config ours_fx554_A_ibvs; CPU runs: placement B, ours_fx554_B_ibvs (the backend is set by the
#   stack config only; run_one.sh refuses a mismatch). The mission window comes from config/protocol_fx554.json.
#   Needs SIM_MODEL_SHA256 exported (the sha256 apply_sim_params.m printed for the running model).
#   25C (R4, R5, DEVIATIONS 17.86): HIL_SIM_SETUP (hil_env.sh, default frozen) names the simulator setup. frozen: the model
#   hash check below, unchanged. sim25w (25W's wrapper, mono or stereo in HIL_SIM_MODE): SIM_MODEL_SHA256 is not used; the
#   supervisor's model_info reply must equal the reply pinned for that mode in config/sim_setups.json (sim_setup.py), before
#   the session gate and before every run. Such a batch also needs REC_HELD_ONLY=1 (rec.sh: no render saved after the hold
#   gate, one terminal frame), L2_PROCMEM=1 (check_run.py's SLAM co-load rule reads procmem.csv), SIM_MODEL_SHA256 unset
#   (no hash it never checked goes into a pin record), HIL_SIM_SETUPS at the committed config/sim_setups.json, and every
#   plan line's stack config must have camera.stereo true exactly when HIL_SIM_MODE is stereo.
# Order of work:
#   0. refuse an existing batch_dir; our repo must be clean, and its HEAD is pinned for the whole batch (HIL_OURS_PIN);
#      SIM_MODEL_SHA256 must equal the model in sim_geometry.json and held_reference.json; runtime files must verify.
#      8A (Ahmed): a power marker is needed. After a full power removal Ahmed runs power_marker.sh, which writes
#      $HIL_POWER_DIR/power_cycled_<YYYYMMDD_HHMM>; the batch refuses to start unless one exists that is newer than this boot
#      (/proc/stat btime) and not yet consumed, and consumes it (writes <marker>.consumed with the batch id). So the NPU open
#      budget (opens since boot) only restarts after a confirmed power-on: a soft reboot gives no new marker, so no batch.
#   1. session gate, once: a held start (hold_once.sh): hold time, reference render, the 11 CPU configurations. 5R (Ahmed): no
#      NPU configuration here; bursts of single-image NPU processes are when the Hailo-10H failed (5Q, 5R soft-reboot test).
#      Then the NPU reference check (5R): yolo26n NPU on the held reference fixture must give the box and conf of
#      held_reference.json npu_reference (1 open); repeated at the end of the session (also after a stop); a mismatch stops.
#   2. per run, before it: temperature (wait while above 70 C, give up after 15 min), live throttle bits, supervisor
#      sim=stopped, free disk, no container up, no /tmp leftovers, our repo still clean at the pin, and for an NPU run the
#      NPU budget (/dev/hailo0 present, no "Device disconnected" since boot, no HailoRT driver/open failure in hailort.log
#      since boot (8A), opens since boot + 2 <= budget). The same check comes before every NPU open of the batch, so after an
#      NPU failure nothing opens the NPU again until a power-on (and the batch has stopped: the run is INVALID, run_one.sh).
#   3. the run: run_one.sh into <batch_dir>/<NNN>_<label>/ (fixed hold, own-config gate, stack, resume, teardown, check_run).
#   4. per run, after it: the stack run rc 0 (smoke_rc), every PLUMBING check of check_run passes (16A: on lw keys the two
#      geometry-gate checks are recorded instead, DEVIATIONS 17.77) (5R, Ahmed: the outcome
#      check final_dist_in_window_provisional is reported, never a stop: a run that does not reach is data for Reach),
#      teardown (no container, supervisor sim=stopped); for a sentinel, Det Hz,
#      /cmd_vel Hz and Det latency within 5 % of the A3 means.
# The batch STOPS at the first failed check: it writes STOPPED.txt (reason) and remaining.tsv (runs not started), and never
# retries a run. A failed run stays in its folder and is listed in batch_state.tsv; Ahmed decides what happens next.
set -u; set -o noclobber
PLAN="$(readlink -f "$1")"; BD="$(readlink -m "$2")"     # 5R fix: absolute, before the cd below (a relative plan was not found)
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
PROTO="$HIL_RUNS/config/protocol_fx554.json"
[ -e "$BD" ] && { echo "batch: REFUSE: $BD exists (every batch writes into a new folder)"; exit 20; }
[ -f "$PLAN" ] || { echo "batch: REFUSE: no plan $PLAN"; exit 20; }
mkdir -p "$(dirname "$BD")"; mkdir "$BD" || exit 20
cd "$BD" || exit 20
BATCH_ID="$(basename "$BD")"; export BATCH_ID
pj() { python3 -c "import json,sys; j=json.load(open(sys.argv[1])); exec(sys.argv[2])" "$PROTO" "$1"; }
log() { echo "$(date -u +%FT%TZ) $*" | tee -a "$BD/batch.log"; }
grep -vE '^\s*(#|$)' "$PLAN" > "$BD/plan.tsv"; cp "$PLAN" "$BD/plan_source.tsv"
[ -s "$BD/plan.tsv" ] || { echo "batch: REFUSE: no runs in $PLAN"; exit 20; }   # 5R fix: an empty plan is refused, not "done"
printf 'n\tkey\tmodel\tbackend\trole\tdir\trun_rc\tcheck_rc\tstart_utc\tend_utc\tnpu_opens_before\tnpu_opens_after\n' > "$BD/batch_state.tsv"
REF_START=0
stop() {  # stop <reason> <first not-started line number (1-based in plan.tsv)>
  log "STOP: $1"; echo "$1" > "$BD/STOPPED.txt"
  tail -n "+$2" "$BD/plan.tsv" > "$BD/remaining.tsv"
  log "remaining runs not started: $(wc -l < "$BD/remaining.tsv") (remaining.tsv)"
  if [ "$REF_START" = 1 ]; then REF_START=0; log "end-of-session $(ref_check end 2>&1 | tail -1)"; fi
  exit 1
}

# ── 0. batch pre-checks ──────────────────────────────────────────────────────
[ "$HIL_SIM_SETUP" = frozen ] && : "${SIM_MODEL_SHA256:?export SIM_MODEL_SHA256 first}"
gs=$(git -C "$HIL_OURS" status --porcelain) || stop "git status failed in $HIL_OURS" 1
[ -z "$gs" ] || stop "our repo is not clean: runs must come from committed code" 1
HIL_OURS_PIN=$(git -C "$HIL_OURS" rev-parse HEAD); export HIL_OURS_PIN
log "batch $BATCH_ID: plan $PLAN ($(wc -l < "$BD/plan.tsv") runs), ours $HIL_OURS_PIN, stack $HIL_AMAR_PIN, patch ${HIL_PATCH_SHA:0:8}"
if [ "$HIL_SIM_SETUP" = frozen ]; then
for f in "$HIL_GEOM" "$HIL_HELD_REF"; do
  want=$(python3 -c "import json,sys; j=json.load(open(sys.argv[1])); print(j.get('model_sha256') or j.get('sim_model_sha256'))" "$f")
  [ "$want" = "$SIM_MODEL_SHA256" ] || stop "SIM_MODEL_SHA256 ${SIM_MODEL_SHA256:0:8} != ${want:0:8} in $f" 1
done
else                                                            # 25C R4: the model_info check of the named setup and mode
  [ "$HIL_SIM_SETUPS" = "$HIL_RUNS/config/sim_setups.json" ] || stop "HIL_SIM_SETUPS must be the committed $HIL_RUNS/config/sim_setups.json" 1
  r=$(python3 "$HIL_RUNS/sim_setup.py" check "$HIL_SIM_SETUP" "${HIL_SIM_MODE:-}" 2>&1) || stop "$r" 1; log "$r"
  [ "${REC_HELD_ONLY:-}" = 1 ] || stop "REC_HELD_ONLY=1 is required for a $HIL_SIM_SETUP batch (R5: no render saved after the hold)" 1
  [ "${L2_PROCMEM:-0}" = 1 ] || stop "L2_PROCMEM=1 is required for a $HIL_SIM_SETUP batch (R3: the co-load rule reads procmem.csv)" 1
  [ -z "${SIM_MODEL_SHA256:-}" ] || stop "SIM_MODEL_SHA256 must be unset for a $HIL_SIM_SETUP batch (its model is checked by model_info, not a hash)" 1
  log "renders saved per run: until the hold gate has read the final held render, then only the terminal frame (REC_HELD_ONLY=1); L2_PROCMEM=1"
  while IFS=$'\t' read -r _k _m BK0 _r CF0; do               # every plan line: camera.stereo true exactly in a stereo session
    case "$BK0" in npu) P0=A ;; cpu) P0=B ;; *) continue ;; esac   # a bad backend is refused per line below, as before
    CF0="${CF0:-ours_fx554_${P0}_ibvs}"
    ST=$(python3 "$HIL_WT/scripts/parse_stack.py" "$HIL_WT/config/hil/stack/$CF0.yaml" --get camera.stereo 2>/dev/null)
    { [ "$ST" = True ] && [ "$HIL_SIM_MODE" = stereo ]; } || { [ "$ST" != True ] && [ "$HIL_SIM_MODE" = mono ]; } \
      || stop "plan: stack config $CF0 (camera.stereo ${ST:-unset}) does not match HIL_SIM_MODE $HIL_SIM_MODE" 1
  done < "$BD/plan.tsv"
fi
(cd "$HIL_RUNTIME" && sha256sum --quiet -c SHA256SUMS >/dev/null 2>&1) || stop "runtime files missing or changed" 1
[ "$(sha256sum "$HIL_PATCH" | cut -d' ' -f1)" = "$HIL_PATCH_SHA" ] || stop "patch file hash differs from HIL_PATCH_SHA" 1
if [ -n "${HIL_CODE_COMMIT:-}" ]; then    # 5R: the run code at HEAD must be the recorded code commit (hil_env.sh aside)
  git -C "$HIL_OURS" diff --quiet "$HIL_CODE_COMMIT" HEAD -- benchmarks/hil_runs benchmarks/inloop_accuracy.py integration \
    ':(exclude)benchmarks/hil_runs/hil_env.sh' || stop "run code at HEAD differs from HIL_CODE_COMMIT $HIL_CODE_COMMIT" 1
  log "run code = HIL_CODE_COMMIT ${HIL_CODE_COMMIT:0:7} (hil_env.sh aside)"
fi
BT=$(awk '/^btime/ {print $2}' /proc/stat)                     # 8A: power marker, newer than this boot and not consumed
PM=$(for f in "$HIL_POWER_DIR"/power_cycled_*; do
       [[ "$f" =~ /power_cycled_[0-9]{8}_[0-9]{4}$ ]] && [ -f "$f" ] && [ ! -e "$f.consumed" ] && [ "$(stat -c %Y "$f")" -gt "$BT" ] && echo "$f"
     done | sort | tail -1)
[ -n "$PM" ] || stop "no unconsumed power marker newer than this boot ($(date -d "@$BT" '+%F %T')) in $HIL_POWER_DIR: run power_marker.sh after a full power removal" 1
echo "consumed by batch $BATCH_ID at $(date -u +%FT%TZ), boot $(date -d "@$BT" '+%F %T')" > "$PM.consumed" || stop "cannot write $PM.consumed" 1
log "power marker $(basename "$PM") consumed (boot $(date -d "@$BT" '+%F %T')): NPU open budget counts from this power-on"
if [ "$HIL_SIM_SETUP" = frozen ]; then log "sim model ${SIM_MODEL_SHA256:0:8}, $(vcgencmd get_throttled), $(vcgencmd measure_temp)"
else log "sim setup $HIL_SIM_SETUP mode $HIL_SIM_MODE, $(vcgencmd get_throttled), $(vcgencmd measure_temp)"; fi

npu_opens() {  # VDevice creates since boot (HailoRT logs local time, as uptime -s)
  local b; b=$(uptime -s)
  awk -v b="[$b" '/Creating vdevice/ { s=$1" "$2; if (substr(s,1,length(b)) >= b) n++ } END { print n+0 }' "$HOME/.hailo/hailort/hailort.log" 2>/dev/null
}
npu_fails() {  # 8A: HailoRT driver/open failures since boot (the 5R ENXIO: "Failed to open device file /dev/hailo0 with error 6")
  local b; b=$(uptime -s)
  awk -v b="[$b" '/Failed to open device file \/dev\/hailo|HAILO_DRIVER_OPERATION_FAILED/ { s=$1" "$2; if (substr(s,1,length(b)) >= b) n++ } END { print n+0 }' "$HOME/.hailo/hailort/hailort.log" 2>/dev/null
}
npu_ok() {  # npu_ok <opens needed>
  [ -e /dev/hailo0 ] || { echo "/dev/hailo0 missing"; return 1; }
  sudo dmesg | grep -q 'Device disconnected' && { echo "dmesg: hailo 'Device disconnected' since boot (power cycle needed)"; return 1; }
  local f; f=$(npu_fails); [ "${f:-0}" -eq 0 ] || { echo "hailort.log: $f driver/open failures since boot (power cycle needed)"; return 1; }
  local n budget; n=$(npu_opens); budget=$(pj 'print(j["between_runs"]["npu_open_budget_per_boot"])')
  [ $((n + $1)) -le "$budget" ] || { echo "NPU budget: $n opens since boot + $1 > $budget (power cycle before more NPU work)"; return 1; }
  echo "NPU ok: $n opens since boot"
}
pre_run_checks() {  # pre_run_checks <backend>
  local t lim poll maxw w=0 th st free min
  lim=$(pj 'print(j["between_runs"]["temp_wait_above_c"])'); poll=$(pj 'print(j["between_runs"]["temp_poll_s"])')
  maxw=$(pj 'print(j["between_runs"]["temp_max_wait_s"])'); min=$(pj 'print(j["between_runs"]["disk_min_gb"])')
  while :; do
    t=$(vcgencmd measure_temp | sed 's/[^0-9.]//g')
    awk -v a="$t" -v l="$lim" 'BEGIN{exit !(a<=l)}' && break
    [ "$w" -ge "$maxw" ] && { echo "temperature $t C still above $lim C after ${w} s"; return 1; }
    sleep "$poll"; w=$((w + poll))
  done
  echo "temp $t C (waited ${w} s)"
  th=$(vcgencmd get_throttled | cut -d= -f2); [ $(( th & 0xF )) -eq 0 ] || { echo "live throttle bits set: $th"; return 1; }
  # 8I: a freshly started supervisor replies sim=stopped with state=idle; a torn-down one replies state=stopped.
  # Both mean "the model is not running and nothing is held", so both are accepted and no manual `hil stop` is needed
  # before a session (session 1 of 2026-10-08 needed one; DEVIATIONS 17.67).
  st=$(hil status)
  case "$st" in
    *sim=stopped*state=stopped*|*sim=stopped*state=idle*) ;;
    *) echo "supervisor not stopped: $st"; return 1 ;;
  esac
  if [ "$HIL_SIM_SETUP" != frozen ]; then                         # 25C R4: the same setup and mode before every run
    local ms; ms=$(python3 "$HIL_RUNS/sim_setup.py" check "$HIL_SIM_SETUP" "$HIL_SIM_MODE" 2>&1) || { echo "$ms"; return 1; }
  fi
  local d; for d in "$BD" "$HIL_WT"; do                              # run folders, and the stack's bags in the worktree
    free=$(df -BG --output=avail "$d" | tail -1 | tr -dc 0-9); [ "$free" -ge "$min" ] || { echo "free disk ${free} GB < ${min} GB at $d"; return 1; }
  done
  [ -z "$(sudo docker ps -q)" ] || { echo "a container is up: $(sudo docker ps --format '{{.Names}}' | tr '\n' ' ')"; return 1; }
  ls -d /tmp/r20_* >/dev/null 2>&1 && { echo "/tmp holds r20_* files from an earlier run"; return 1; }
  local gs; gs=$(git -C "$HIL_OURS" status --porcelain) || { echo "git status failed in $HIL_OURS"; return 1; }
  [ "$(git -C "$HIL_OURS" rev-parse HEAD)" = "$HIL_OURS_PIN" ] && [ -z "$gs" ] || { echo "our repo moved or is dirty"; return 1; }
  [ "$1" = npu ] && { npu_ok 2 || return 1; }
  echo "throttle $th, supervisor stopped, disk ${free} GB free"
}

ref_check() {  # ref_check <start|end>: yolo26n NPU on the held reference fixture must give npu_reference (1 NPU open)
  local o="$BD/ref_$1" r fx
  mkdir "$o" || { echo "cannot create $o"; return 1; }
  r=$(npu_ok 1) || { echo "$r"; return 1; }
  fx=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['fixture_png'])" "$HIL_HELD_REF")
  (cd "$o" && python3 "$HIL_RUNS/gate_render.py" "$HIL_RUNS/$fx" "$o/ref.json" --configs yolo26n_npu < /dev/null > "$o/ref.out" 2> "$o/ref.err")
  python3 - "$o/ref.json" "$HIL_HELD_REF" <<'REF'
import json, sys
try:
    g = json.load(open(sys.argv[1]))
except (OSError, ValueError) as e:
    print(f"NPU reference: no result ({e})"); sys.exit(1)
ref = json.load(open(sys.argv[2]))["npu_reference"]
got = (g.get("all_dets") or {}).get(ref["config"])
ok = got == ref["dets"]
print(f"NPU reference {'ok' if ok else 'MISMATCH'}: {ref['config']} {got} (expected {ref['dets']})"); sys.exit(0 if ok else 1)
REF
}

# ── 1. session gate ──────────────────────────────────────────────────────────
r=$(npu_ok 2) || stop "before the session gate: $r" 1; log "$r"
r=$(pre_run_checks cpu) || stop "before the session gate: $r" 1; log "$r"
bash "$HIL_RUNS/hold_once.sh" "$BD/000_session_gate" --gate yolo26n_cpu yolo26s_cpu yolo26m_cpu yolov8n_cpu yolov8s_cpu yolov8m_cpu yolov11n_cpu yolov11s_cpu yolov11m_cpu centernet_resnet_v1_18_cpu ssd_mobilenet_v2_cpu < /dev/null > "$BD/session_gate.out" 2>&1; GRC=$?
log "session gate rc=$GRC: $(grep -E 'hold ok|GATE|FAIL' "$BD/000_session_gate/hold.log" | tail -2 | cut -d' ' -f2- | tr '\n' ' ')"
[ "$GRC" -eq 0 ] || stop "session gate failed (rc $GRC, see 000_session_gate/)" 1
r=$(ref_check start) || stop "start-of-session $r" 1; log "start-of-session $r"; REF_START=1

# ── 2-4. runs ────────────────────────────────────────────────────────────────
N=0
while IFS=$'\t' read -r -u 3 KEY MDL BK ROLE CFG5; do         # plan on fd 3: nothing in a run can read it
  N=$((N + 1)); NNN=$(printf '%03d' "$N")
  case "$BK" in npu) P=A ;; cpu) P=B ;; *) stop "plan line $N: backend '$BK' is not npu or cpu" "$N" ;; esac
  CFG="ours_fx554_${P}_ibvs"; RD="$BD/${NNN}_${MDL}_${BK}_${P}_r${KEY}"
  if [ -n "${CFG5:-}" ]; then                                    # 14A: per-run stack config (5th column)
    # 20B: Level 2 varies the controller and the SLAM arm, so a placement $P config is no longer
    # always an ibvs one. The admissible names stay an explicit list (no wildcard over controllers):
    # the Level 2 controllers are ibvs, proportional and h_vs, each optionally with a suffix naming
    # the SLAM arm (e.g. ours_fx554_A_ibvs_stereo061_accurate). Nothing else is accepted, and the
    # file must exist. This is a plan-admissibility check on the name, not a validity check on data:
    # no run already scored changes, because every stored run used ours_fx554_<P>_ibvs.
    case "$CFG5" in
      "ours_fx554_${P}_ibvs"|"ours_fx554_${P}_ibvs_"*) ;;
      "ours_fx554_${P}_proportional"|"ours_fx554_${P}_proportional_"*) ;;
      "ours_fx554_${P}_h_vs"|"ours_fx554_${P}_h_vs_"*) ;;
      *) stop "plan line $N: stack config '$CFG5' is not a placement $P Level 2 config" "$N" ;;
    esac
    [ -f "$HIL_WT/config/hil/stack/$CFG5.yaml" ] || stop "plan line $N: no stack config $CFG5.yaml in $HIL_WT" "$N"
    CFG="$CFG5"
  fi
  DUR=$(pj "w=j['mission_window_s']; print(w.get('${MDL}_${BK}', w['default']))")
  r=$(pre_run_checks "$BK") || stop "before run $NNN ($KEY $MDL $BK): $r" "$N"; log "pre $NNN: $r"
  OB=$(npu_opens); T0=$(date -u +%FT%TZ); log "run $NNN: key $KEY $MDL $BK role $ROLE $CFG window ${DUR}s -> $(basename "$RD")"
  RUN_DIR="$RD" RUN_KEY="$KEY" MODEL="$MDL" BACKEND="$BK" PLACEMENT=$P STACK_CFG="$CFG" DURATION="$DUR" \
    bash "$HIL_RUNS/run_one.sh" < /dev/null > "$BD/${NNN}_run_one.out" 2>&1; RRC=$?
  CRC=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['check_rc'])" "$RD/run_result.json" 2>/dev/null || echo none)
  OA=$(npu_opens)
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$NNN" "$KEY" "$MDL" "$BK" "$ROLE" "$(basename "$RD")" "$RRC" "$CRC" "$T0" "$(date -u +%FT%TZ)" "$OB" "$OA" >> "$BD/batch_state.tsv"
  log "run $NNN: run rc=$RRC check rc=$CRC, NPU opens $OB -> $OA"
  if [ "$ROLE" = expect_stop ]; then
    r=$(python3 - "$RD" "$RRC" <<'EXPECT'
import glob, json, os, sys
rd, rrc = sys.argv[1], int(sys.argv[2])
if not os.path.exists(os.path.join(rd, "STOP_RULE.txt")):
    print("expect_stop FAIL: no STOP_RULE.txt (the stop rule did not fire)"); sys.exit(1)
if rrc in (4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 20):   # 8I: 8 = resume gate, no detection on the held frame
    print(f"expect_stop FAIL: guard exit {rrc}"); sys.exit(1)
c = json.load(open(glob.glob(os.path.join(rd, "check", "*_check.json"))[0]))["checks"]
need = ["pi_interface_eth0", "rmw_fastrtps", "cam_fx_expected", "slam_pose_off", "start_pose_expected",
        "clock_offset_recorded", "held_render_is_reference"]
bad = [k for k in need if not c.get(k)]
print("expect_stop " + (f"FAIL: {bad}" if bad else f"ok: stop rule fired, run rc {rrc}, pre-stop checks pass")); sys.exit(1 if bad else 0)
EXPECT
) || stop "run $NNN ($KEY): $r" "$((N + 1))"
    log "run $NNN: $r"
  else
  SRC=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['smoke_rc'])" "$RD/run_result.json" 2>/dev/null || echo "$RRC")
  [ "$SRC" = 0 ] || stop "run $NNN ($KEY) failed: stack run rc $SRC, run_one rc $RRC (see $(basename "$RD")/run.log and ${NNN}_run_one.out)" "$((N + 1))"
  r=$(python3 - "$RD" "$KEY" <<'PLUMB'
import glob, json, os, sys
# Reported per run, never a stop: these describe what the mission DID, not whether the rig was wired right.
# 5R: final_dist_in_window_provisional (a run that does not reach is data for Reach).
# 9B (item 8): det_hz_gt_0 and cmd_within_5pct_of_det join it, because a CPU run that never locks on stays
#   in SEARCHING and fails them structurally, not through any fault of the rig. Oct 8 run 006 is the
#   precedent: every plumbing check passed (held render, IC guard pose, start pose, fx, transport) while
#   cmd_within_5pct_of_det read +822 pct, cmd_vel at the watchdog's 4.51 Hz against 0.489 Hz of detections.
#   det_hz_gt_0 is safe to demote because the resume gate already refuses to start a run with no detection
#   on the held frame, and first_det_before_resume still marks INVALID and stops, so "the detector is dead"
#   is caught earlier and harder than here.
# NOT demoted (on every key except lw*, 16A below): scorer_runs, geometry_gate_is_protocol and geometry_gate_capture_pass still stop the batch.
#   9C (17.69): they no longer fail structurally on a run that never locks on. geometry_gate_is_protocol is
#   configuration-only in check_run.py, and the scorer measures its gate on frames with |yaw rate| at or
#   below 0.05 rad/s (the held frame always qualifies), so a run that never locks on passes the gate on its
#   zero-yaw frames and is a result (recorded below with reason "no lock-on"), while a run whose geometry is
#   wrong on those frames still stops the batch. See DEVIATIONS 17.68 (e) and 17.69.
OUTCOME = {"final_dist_in_window_provisional", "det_hz_gt_0", "cmd_within_5pct_of_det"}
# 17D/18A/18B (DEVIATIONS 17.81, 17.82, 17.83): first_motion_ok and slam_coload_alive (check_run.py) are VALIDITY checks: they stay out
#   of OUTCOME and out of RECORDED on every key, so a run whose commands the simulator never applied, or whose SLAM co-load
#   never ran, is INVALID and stops the batch like every other plumbing check (the two faults of 17.79 and REPORT_S5 attempt 1).
# 16A (DEVIATIONS 17.77, Ahmed): on an ABLATION row (plan key starting with "lw": session 5a, never pooled into Tables 13/14)
#   scorer_runs and geometry_gate_capture_pass are RECORDED, not stopping. The gate still runs on every row and its value is
#   logged here; the sentinel (sen*) and every main-table key (c*, x*) stay gated, so a rig-level geometry fault still stops
#   the batch at its first row. Reason: after an overshoot a slow row can re-find the sign from the side, where the single-point
#   gate is off by the target's along-axis uncertainty (17.76 d); on an ablation row that is an outcome, not a plumbing fault.
ABLATION = len(sys.argv) > 2 and sys.argv[2].startswith("lw")
RECORDED = {"scorer_runs", "geometry_gate_capture_pass"} if ABLATION else set()
try:
    c = json.load(open(glob.glob(os.path.join(sys.argv[1], "check", "*_check.json"))[0]))
except (IndexError, OSError, ValueError) as e:
    print(f"no check_run result ({e})"); sys.exit(1)
bad = [k for k, v in c["checks"].items() if not v and k not in OUTCOME and k not in RECORDED]
out = [k for k, v in c["checks"].items() if not v and k in OUTCOME]
rec = [k for k, v in c["checks"].items() if not v and k in RECORDED]
i = c["info"]
# the gate value when the scorer ran; when it refused (the usual way the pair fails) check.json holds null and the RMS sits
# only in the scorer's refusal line (info.scorer_tail), so that line is printed instead
_gv = (i.get("geometry_gate") or {}).get("frozen_target_reprojection_rms_px")
gate_text = (f"gate {_gv:.1f} px" if isinstance(_gv, (int, float)) else
             next((l.strip() for l in (i.get("scorer_tail") or []) if "refusing to score" in l), "gate value not in check.json"))
# 9C (item 9, 17.69): a run that never left SEARCHING is recorded with reason "no lock-on"; the batch continues.
nolock = (i.get("final_state") == "SEARCHING" and not i.get("reached")
          and not i.get("n_lost") and not i.get("n_reacquire"))
print(("plumbing FAIL: " + ", ".join(bad)) if bad else
      "plumbing ok; outcome: " + ("no lock-on; " if nolock else "")
      + f"reached {i.get('reached')} final {i.get('final_dist_frozen_sign_m')} m" + (f" (outside window: {out})" if out else "")
      + (f" (ablation row: geometry gate recorded, not stopping: {rec}; {gate_text})" if rec else ""))
sys.exit(1 if bad else 0)
PLUMB
) || stop "run $NNN ($KEY): check_run $r (see $(basename "$RD")/check/)" "$((N + 1))"
  log "run $NNN: $r"
  fi
  [ -z "$(sudo docker ps -q)" ] || stop "after run $NNN: a container is still up (teardown incomplete)" "$((N + 1))"
  v=$(python3 -c "import json,sys; j=json.load(open(sys.argv[1])); print('ok' if j['valid'] else 'INVALID: ' + j['invalid_reason'])" "$RD/run_result.json" 2>&1)
  [ "$v" = ok ] || stop "run $NNN ($KEY): $v" "$((N + 1))"
  st=$(hil status); case "$st" in *sim=stopped*) ;; *) stop "after run $NNN: supervisor not stopped: $st" "$((N + 1))" ;; esac
  if [ "$ROLE" = sentinel ]; then
    r=$(python3 - "$RD/stack" "$PROTO" <<'PY'
import glob, json, sys
s = json.load(open(sys.argv[2]))["sentinel"]; tol = s["tol_frac"]
st = json.load(open(glob.glob(f"{sys.argv[1]}/*_run1_stats.json")[0]))
sm = json.load(open(glob.glob(f"{sys.argv[1]}/*_summary.json")[0]))
got = {"det_hz": st["det"]["hz_mean"], "cmd_vel_hz": st["cmd_vel"]["hz_mean"],
       "det_latency_ms": sm["detector_latency_ms"]["det_path"]["mean"]}
bad = [f"{k} {v:.3f} vs {s[k]} ({100 * (v - s[k]) / s[k]:+.2f} %)" for k, v in got.items() if abs(v - s[k]) > tol * s[k]]
print("sentinel " + ("FAIL: " + "; ".join(bad) if bad else "ok: " + ", ".join(f"{k} {v:.3f}" for k, v in got.items())))
sys.exit(1 if bad else 0)
PY
) || stop "run $NNN: $r" "$((N + 1))"
    log "run $NNN: $r"
  fi
done 3< "$BD/plan.tsv"
REF_START=0; r=$(ref_check end) || stop "end-of-session $r" "$((N + 1))"; log "end-of-session $r"
log "batch done: $N runs, all checks passed; NPU opens since boot $(npu_opens)"
echo "done" > "$BD/DONE.txt"
