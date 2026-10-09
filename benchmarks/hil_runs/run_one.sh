#!/usr/bin/env bash
# run_one.sh: one fixed-hold run with the pre-resume gate (from stage6/r20.sh, 5D Part B / 5J / 5M; generalised in 5Q).
# 5J standing rule: every run writes into a NEW folder and nothing is overwritten. noclobber is on, so a '>' never overwrites.
# Inputs (environment):
#   RUN_DIR      new folder for this run (must not exist; created here)
#   RUN_KEY      run label suffix, unique within the batch (label = level2_<model>_<backend>_<placement>_r<RUN_KEY>)
#   MODEL BACKEND [PLACEMENT=A] [STACK_CFG=ours_fx554_A_ibvs] [DURATION=60]   (the stack config sets the backend: refused if it
#   is not BACKEND, so a run labelled cpu can never run on the NPU)
#   SIM_MODEL_SHA256   the sha256 apply_sim_params.m printed for the running Simulink model
#   HIL_OURS_PIN       our repo commit this run must come from (default: HEAD, if the repo is clean)
#   [GATE_CONFIGS]     pre-resume gate set (default: <model>_<backend>, the run's own configuration)   [BATCH_ID]
# Layout: RUN_DIR/ run.log, pin_record.json, run_result.json, hold gate outputs, approach_bag/, recorder logs, sidecar files
#         (8I: t_first_det_ns.txt from the resume gate, and NO_DET_ON_HELD_FRAME.txt if that gate timed out);
#         RUN_DIR/stack/ the stack run (smoke_run_hold.sh); RUN_DIR/check/ check_run.py outputs.
# Refuses (exit 20) if RUN_DIR exists, if /tmp holds files from an earlier run, or if a runtime file is missing or changed.
set -u; set -o noclobber
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
: "${RUN_DIR:?set RUN_DIR to a NEW run folder}"; : "${RUN_KEY:?set RUN_KEY}"; : "${MODEL:?}"; : "${BACKEND:?}"
: "${SIM_MODEL_SHA256:?export SIM_MODEL_SHA256 first}"
P="${PLACEMENT:-A}"; CFG="${STACK_CFG:-ours_fx554_A_ibvs}"; DUR="${DURATION:-60}"
O="$RUN_DIR"; S="$O/stack"; M="$MODEL"; B="$BACKEND"; K="$RUN_KEY"
L="level2_${M}_${B}_${P}_r${K}"; TAG="${M}_${B}_${P}_r${K}"
export GATE_CONFIGS="${GATE_CONFIGS:-${M}_${B}}"
if [ -z "${HIL_OURS_PIN:-}" ]; then
  gs=$(git -C "$HIL_OURS" status --porcelain) || { echo "run_one: REFUSE: git status failed in $HIL_OURS"; exit 20; }
  [ -z "$gs" ] || { echo "run_one: REFUSE: our repo is not clean (runs come from committed code)"; exit 20; }
  HIL_OURS_PIN=$(git -C "$HIL_OURS" rev-parse HEAD)
fi
export HIL_OURS_PIN
[ -e "$O" ] && { echo "run_one: REFUSE: $O exists (every run writes into a new folder)"; exit 20; }
for f in /tmp/r20_bag /tmp/r20_bag_rec.log /tmp/r20_hash.log /tmp/r20_hash.err /tmp/r20_renders /tmp/r20_dets.log; do
  [ -e "$f" ] && { echo "run_one: REFUSE: $f is left from an earlier run; move it aside first"; exit 20; }
done
(cd "$HIL_RUNTIME" 2>/dev/null && sha256sum --quiet -c SHA256SUMS >/dev/null 2>&1) || { echo "run_one: REFUSE: runtime files in $HIL_RUNTIME missing or changed"; exit 20; }
CB=$(python3 "$HIL_WT/scripts/parse_stack.py" "$HIL_WT/config/hil/stack/$CFG.yaml" --get detector.yolo.backend 2>/dev/null)
[ "$CB" = "$B" ] || { echo "run_one: REFUSE: stack config $CFG sets backend '${CB:-none}', the run asks for $B"; exit 20; }
mkdir -p "$(dirname "$O")"; mkdir "$O" || { echo "run_one: REFUSE: cannot create $O"; exit 20; }
mkdir "$S" "$O/check"; cd "$O" || exit 20
echo "run: $L cfg=$CFG duration=${DUR}s gate=$GATE_CONFIGS ours=$HIL_OURS_PIN batch=${BATCH_ID:-none} start_utc=$(date -u +%FT%TZ)" | tee "$O/run.log"
# 5R: SoC temperature and LIVE throttle bits (0xF) at the start and the end of the run; the peak comes from the 1 Hz telemetry
export RUN_T_START=$(vcgencmd measure_temp | sed 's/[^0-9.]//g') RUN_TH_START=$(vcgencmd get_throttled | cut -d= -f2)
echo "soc_start: ${RUN_T_START} C throttled ${RUN_TH_START} (live bits $(( RUN_TH_START & 0xF )))" | tee -a "$O/run.log"
# 8A: an NPU failure during the run (a new kernel "Device disconnected", or a HailoRT driver/open failure in hailort.log since
# the run started) marks the run INVALID; batch.sh then stops and opens the NPU no more (no retry anywhere)
HLOG="$HOME/.hailo/hailort/hailort.log"
export RUN_HDISC_START=$(sudo dmesg | grep -c 'Device disconnected') RUN_LT_START=$(date '+%F %T')
echo "status_pre: $(hil status)" | tee -a "$O/run.log"
sed 's/^/runtime: /' "$HIL_RUNTIME/SHA256SUMS" | tee -a "$O/run.log"
python3 -c "import time,json;t=time.time_ns();m=time.monotonic_ns();print(json.dumps({'utc_ns':t,'monotonic_ns':m,'realtime_minus_monotonic_ns':t-m}))" > "$S/${L}_clock_offset.json"
bash "$HIL_RUNS/rec.sh" start hash | tee -a "$O/run.log"                # render logger up before pub_start / start_hold
bash "$HIL_RUNS/sidecar_stoprule.sh" "$S/${TAG}_matlab.log" "$O" > "$O/sidecar.out" 2>&1 & SC=$!
SMOKE_OUT="$S" HOLD_SCRIPT="$HIL_RUNS/one_run_hold_gated.sh" HOLD_GATE_CMD="bash $HIL_RUNS/hold_gate.sh $O" HOLD_DURATION="$DUR" \
  HOLD_BACKEND="$B" HOLD_PLACEMENT="$P" HOLD_K="$K" bash "$HIL_RUNS/smoke_run_hold.sh" "$L" "$CFG" "$M"; RC=$?
echo "smoke_rc=$RC" | tee -a "$O/run.log"
bash "$HIL_RUNS/rec.sh" stop bag | tee -a "$O/run.log"; bash "$HIL_RUNS/rec.sh" stop hash | tee -a "$O/run.log"
kill "$SC" 2>/dev/null; wait "$SC" 2>/dev/null
# the recorders run as root: sudo for the move (5I: a plain mv copied the bag but could not remove /tmp/r20_bag)
[ -d /tmp/r20_bag ] && sudo mv /tmp/r20_bag "$O/approach_bag"
for f in /tmp/r20_hash.log /tmp/r20_bag_rec.log /tmp/r20_dets.log /tmp/r20_hash.err /tmp/r20_renders; do
  [ -e "$f" ] && sudo mv "$f" "$O/"
done
sudo chown -R "$(id -u):$(id -g)" "$O"
# 8J (A): both resume times, for comparison. t_resume_ns.txt is the gate's exact wall ns taken immediately before it
# sent `hil resume` and is what every check uses; t_resume_ns_sidecar.txt is the sidecar's polled value and lags it.
echo "resume_times: gate=$(cat "$O/t_resume_ns.txt" 2>/dev/null || echo none) sidecar=$(cat "$O/t_resume_ns_sidecar.txt" 2>/dev/null || echo none) first_det=$(cat "$O/t_first_det_ns.txt" 2>/dev/null || echo none)" | tee -a "$O/run.log"
python3 - "$O" <<'RT' | tee -a "$O/run.log"
import os, sys
O = sys.argv[1]
def rd(n):
    try: return int(open(os.path.join(O, n)).read().split()[0])
    except (OSError, ValueError, IndexError): return None
g, sc, fd = rd("t_resume_ns.txt"), rd("t_resume_ns_sidecar.txt"), rd("t_first_det_ns.txt")
print(f"resume_lag: sidecar - gate = {(sc - g) / 1e9:+.3f} s" if g and sc else "resume_lag: not available")
print(f"resume_gate: resume sent {(g - fd) / 1e9:+.3f} s after the first target box" if g and fd else "resume_gate: not available")
RT
# 8I: post-run IC guard (DEVIATIONS 17.14, 17.67). It runs here and not in one_run_hold_gated.sh because it reads
# the LAST /sim/drone_pose before t_resume_ns from "$O/approach_bag", which exists only once the recorder is stopped
# and the bag is moved (the two lines above). pose_match keeps the cross-check of that pose against the held
# supervisor status line, which is an independent source for the same yaw. The batch still stops on a bad initial
# condition through check_run.py's start_pose_expected, which reads the same pre-resume pose with a tighter tolerance
# (0.1 m and 0.5 deg, against the IC guard's 5 deg of bearing); ic_guard_rc is recorded here and in ic_guard.log.
GLINE=$("$HIL_MCAP_PY" "$HIL_RUNS/ic_guard.py" "$S/${L}_run1" --pre-resume-dir "$O" 2>&1); GRC=$?
GJSON=$("$HIL_MCAP_PY" "$HIL_RUNS/ic_guard.py" --json "$S/${L}_run1" --pre-resume-dir "$O" 2>/dev/null)
echo "$(date -u +%FT%TZ) $GLINE" >> "$S/ic_guard.log"
[ -n "$GJSON" ] && echo "$GJSON" >> "$S/ic_guard.jsonl"
echo "ic_guard_rc=$GRC: $GLINE" | tee -a "$O/run.log"
PRE_YAW=$(grep -oP '\byaw=\K[0-9.]+' "$S/pre_ic_guard.log" 2>/dev/null | tail -1)
POSTYAW=$(echo "$GJSON" | python3 -c "import sys,json;print(f\"{json.load(sys.stdin)['yaw_deg']:.2f}\")" 2>/dev/null)
if [ -n "$POSTYAW" ] && [ -n "$PRE_YAW" ]; then
  DY=$(python3 -c "print(f'{abs((($POSTYAW)-($PRE_YAW)+180)%360-180):.2f}')" 2>/dev/null)
  echo "pose_match: pre_yaw=$PRE_YAW post_yaw=$POSTYAW dyaw=${DY}deg" | tee -a "$O/run.log" >> "$S/pose_match.log"
fi
"$HIL_MCAP_PY" "$HIL_RUNS/check_run.py" "$S" "$L" --pre-resume-dir "$O" --out-dir "$O/check" > "$O/check/${L}_check.txt" 2>&1; CRC=$?
echo "check_rc=$CRC" | tee -a "$O/run.log"
# 5R: Table 14 from the approach bag and from the stack bag over the window both cover; per bin the values must match.
# Information for the report (consistency/consistency.json); it does not change the run result.
"$HIL_MCAP_PY" "$HIL_RUNS/tools/scorer_consistency.py" "$(ls "$O"/approach_bag/*.mcap 2>/dev/null | head -1)" \
  "$(ls "$S/${L}_run1_bag"/*.mcap 2>/dev/null | head -1)" "$S/${L}_run1" "$S/${L}_clock_offset.json" "$O/consistency" \
  > "$O/consistency.txt" 2>&1; echo "consistency_rc=$? ($(tail -1 "$O/consistency.txt"))" | tee -a "$O/run.log"
# pin record: the stack's pin (smoke_run_hold.sh) plus what this run adds; a new file, the stack's pin is not rewritten
export RUN_T_END=$(vcgencmd measure_temp | sed 's/[^0-9.]//g') RUN_TH_END=$(vcgencmd get_throttled | cut -d= -f2)
echo "soc_end: ${RUN_T_END} C throttled ${RUN_TH_END} (live bits $(( RUN_TH_END & 0xF )))" | tee -a "$O/run.log"
export RUN_HDISC_END=$(sudo dmesg | grep -c 'Device disconnected') \
  RUN_HFAIL=$(awk -v b="[$RUN_LT_START" '/Failed to open device file \/dev\/hailo|HAILO_DRIVER_OPERATION_FAILED/ { s=$1" "$2; if (substr(s,1,length(b)) >= b) n++ } END { print n+0 }' "$HLOG" 2>/dev/null)
echo "npu_health: kernel 'Device disconnected' ${RUN_HDISC_START} -> ${RUN_HDISC_END}, HailoRT driver/open failures during the run ${RUN_HFAIL:-0}" | tee -a "$O/run.log"
python3 - "$O" "$S/${L}_pin.json" "$S/${L}_clock_offset.json" "$L" "$RC" "$CRC" <<'PY' | tee -a "$O/run.log"
import hashlib, json, os, re, subprocess, sys
O, pin_p, co_p, label, rc, crc = sys.argv[1:7]
E = os.environ
def sh(c): return subprocess.run(c, shell=True, capture_output=True, text=True).stdout.strip()
def sha(p):
    try: return hashlib.sha256(open(p, "rb").read()).hexdigest()
    except OSError: return None
def rd(p):
    try: return open(p).read().strip()
    except OSError: return None
import csv, glob
def live(h):
    try: return int(h, 16) & 0xF
    except (TypeError, ValueError): return None
rows = []
for f in glob.glob(os.path.join(O, "stack", f"{label}_thermal.csv")):
    rows = [r for r in csv.DictReader(open(f))]
temps = [float(r["temp_c"]) for r in rows if r.get("temp_c")]
flags = [int(r["cur_flags"]) for r in rows if r.get("cur_flags") not in (None, "")]
ts, te = float(E["RUN_T_START"]), float(E["RUN_T_END"])
lb, la = live(E["RUN_TH_START"]), live(E["RUN_TH_END"])
during = sum(1 for x in flags if x != 0)
hd = int(E["RUN_HDISC_END"] or 0) - int(E["RUN_HDISC_START"] or 0); hf = int(E.get("RUN_HFAIL") or 0)   # 8A
gate_fail = rd(os.path.join(O, "NO_DET_ON_HELD_FRAME.txt"))     # 8I/8J: the resume gate timed out
why = [w for w, bad in (("live throttle bits before the run", lb != 0), ("live throttle bits after the run", la != 0),
                        (f"throttle during the run ({during} telemetry rows)", during > 0), ("no run telemetry", not rows),
                        (f"NPU failure during the run ({hd} kernel 'Device disconnected', {hf} HailoRT driver/open failures)",
                         hd > 0 or hf > 0),
                        (f"resume gate: {gate_fail}", bool(gate_fail))) if bad]
thermal = {"soc_temp_start_c": ts, "soc_temp_peak_c": max(temps + [ts, te]), "soc_temp_end_c": te,
           "telemetry_rows": len(rows), "telemetry_source": f"stack/{label}_thermal.csv (1 Hz, smoke_run_hold.sh)",
           "throttled_word_start": E["RUN_TH_START"], "throttled_word_end": E["RUN_TH_END"],
           "live_bits_start": lb, "live_bits_end": la, "throttle_rows_during": during,
           "npu_disconnects_during": hd, "npu_driver_failures_during": hf,
           "valid": not why, "invalid_reason": "; ".join(why) or None}
try: pin = json.load(open(pin_p))
except (OSError, ValueError) as e: pin = {"stack_pin_error": str(e)}
held = rd(os.path.join(O, "stack", "held_status.txt")) or ""
m = re.search(r"\bt=([0-9.]+)", held)
ref = json.load(open(E["HIL_HELD_REF"]))
final = rd(os.path.join(O, "held_final_sha1.txt"))
try: gate = json.load(open(os.path.join(O, "gate.json")))
except (OSError, ValueError): gate = {}
rec = {
  "label": label, "batch_id": E.get("BATCH_ID"), "run_dir": O,
  "ours_repo_commit": E["HIL_OURS_PIN"],
  "hil_runs_tree": sh(f"git -C {E['HIL_OURS']} rev-parse {E['HIL_OURS_PIN']}:benchmarks/hil_runs"),
  "pinned_stack": {"worktree": E["HIL_WT"], "commit": E["HIL_AMAR_PIN"], "patch": E["HIL_PATCH"],
                   "patch_sha256_expected": E["HIL_PATCH_SHA"], "patch_sha256_file": sha(E["HIL_PATCH"])},
  "sim_model_sha256": E.get("SIM_MODEL_SHA256"),
  "detector_model_artifact": pin.get("model_artifact"),
  "hold": {"status_line": held, "sim_t": float(m.group(1)) if m else None, "expected_sim_t": ref["hold_sim_t"],
           "final_render_sha1": final, "reference_render_sha1": ref["render_sha1"], "is_reference": final == ref["render_sha1"]},
  "gate": {"configs": gate.get("configs_gated"), "pass": gate.get("pass"), "pixels": gate.get("pixels"), "frame_hash": gate.get("frame_hash")},
  "config_sha256": {k: sha(p) for k, p in (("sim_geometry", E["HIL_GEOM"]), ("held_reference", E["HIL_HELD_REF"]),
                    ("protocol", os.path.join(E["HIL_RUNS"], "config", "protocol_fx554.json")),
                    ("runtime_SHA256SUMS", os.path.join(E["HIL_RUNTIME"], "SHA256SUMS")))},
  "clock_offset_pre_run": json.load(open(co_p)),
  "smoke_rc": int(rc), "check_rc": int(crc),
  "thermal": thermal,
  "stack_pin": pin,
}
with open(os.path.join(O, "pin_record.json"), "x") as f: json.dump(rec, f, indent=2)
with open(os.path.join(O, "run_result.json"), "x") as f:
    json.dump({"label": label, "smoke_rc": int(rc), "check_rc": int(crc), "hold_sim_t": rec["hold"]["sim_t"],
               "valid": thermal["valid"], "invalid_reason": thermal["invalid_reason"],
               "held_is_reference": rec["hold"]["is_reference"]}, f, indent=1)
print(f"pin record: {O}/pin_record.json ours {rec['ours_repo_commit'][:7]} stack {rec['pinned_stack']['commit'][:7]} "
      f"patch {str(rec['pinned_stack']['patch_sha256_file'])[:8]} sim {str(rec['sim_model_sha256'])[:8]} hold t={rec['hold']['sim_t']}")
print(f"thermal: start {thermal['soc_temp_start_c']} peak {thermal['soc_temp_peak_c']} end {thermal['soc_temp_end_c']} C; "
      f"run {'VALID' if thermal['valid'] else 'INVALID: ' + thermal['invalid_reason']}")
PY
echo "end_utc=$(date -u +%FT%TZ)" | tee -a "$O/run.log"
[ "$RC" -ne 0 ] && exit "$RC"
exit "$CRC"
