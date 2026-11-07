#!/bin/bash
# run_hil_experiments.sh — Ahmed's detector-run loop (exp/detector-accuracy), driving
# THIS tree's run_stack_hil.sh with a stack config instead of the old flag launcher.
#
# Ported from exp/detector-accuracy:benchmarks/run_hil_experiments.sh. What changed:
#   - the switch: --stack-config <name> launches ./run_stack_hil.sh --config <name>
#     --mode record (bag on) [--model <id>]; placement, backend, cores, controller,
#     cam_fx and SLAM all come from config/hil/stack/<name>.yaml;
#   - mission metrics come from /bench/state in the run's bag
#     (benchmarks/mission_from_bench_state.py), not from the visp CSV, which the shared
#     FSM does not write;
#   - the container-side probe and the stage report are OUR versions, copied from the
#     pinned exp/detector-accuracy commit to /tmp/hil_tools (the container mounts /tmp),
#     so this tree's own benchmarks/e2e_latency_probe.py is neither used nor touched;
#   - the MATLAB supervisor gets pub_start / start / stop / pub_stop, each retried once;
#     no reply after the retry ends the run with exit 7.
#
# Usage:
#   ./benchmarks/run_hil_experiments.sh --stack-config ours_fx1200_A_ibvs \
#       [--model yolo26n] --duration 60 --warmup 5 --runs 1 \
#       --label smoke_yolo26n_npu_A_ibvs --output-dir <dir>
#
# Per-run outputs in <dir>: <label>_run<N>{.csv,_camera.csv,_stats.json,_telemetry.csv,
#   _cam_stamps.csv,_stages.csv,_stack.log,_matlab.log,_params.yaml,_slam_state.txt,
#   _mission.json,_run_config.yaml}, <label>_run<N>_bag/ (the .mcap), <label>_summary.json.
# Exit: 0 ok, 1 all runs failed / bad args, 7 supervisor did not answer after one retry.

set -u
cd "$(dirname "$0")/.."
WS="$(pwd)"

STACK_CONFIG=""; MODEL=""
DURATION=60; WARMUP=35; RUNS=1; LABEL=""; OUTPUT_DIR="benchmarks/results/hil"
MATLAB_CTL=true; PUB_HZ="${PUB_HZ:-20}"
OURS_REPO="${OURS_REPO:-/home/ahmed/ROS2-PROJECT-SPRING2026}"
MCAP_PY="${MCAP_PY:-/home/ahmed/venvs/mcap/bin/python}"
MATLAB_HOST_IP="${MATLAB_HOST_IP:-192.168.56.1}"
MATLAB_CTL_PORT="${MATLAB_CTL_PORT:-55556}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --stack-config) STACK_CONFIG="$2"; shift 2 ;;
        --model)        MODEL="$2"; shift 2 ;;
        --duration)     DURATION="$2"; shift 2 ;;
        --warmup)       WARMUP="$2"; shift 2 ;;
        --runs)         RUNS="$2"; shift 2 ;;
        --label)        LABEL="$2"; shift 2 ;;
        --output-dir)   OUTPUT_DIR="$2"; shift 2 ;;
        --no-matlab-ctl) MATLAB_CTL=false; shift ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

log() { echo "[hil-exp] $*"; }
die() { echo "[hil-exp] ERROR: $*" >&2; exit 1; }

[[ -n "$LABEL" ]]        || die "--label is required"
[[ -n "$STACK_CONFIG" ]] || die "--stack-config is required (config/hil/stack/<name>.yaml)"
CFG="${WS}/config/hil/stack/${STACK_CONFIG}.yaml"
[[ -f "$CFG" ]] || die "stack config not found: $CFG"
mkdir -p "$OUTPUT_DIR"

cfg_get() { python3 "${WS}/scripts/parse_stack.py" "$CFG" --get "$1"; }
CONTROLLER=$(cfg_get controller.type)
DDS=$(cfg_get network.dds); DDS="${DDS:-fastrtps}"
ROS_DOMAIN_ID=$(cfg_get network.ros_domain_id); ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
TELEM_CSV=$(cfg_get detector.yolo.telemetry_csv); TELEM_CSV="${TELEM_CSV:-/tmp/yolo_telemetry.csv}"
STAMP_CSV=$(cfg_get camera.stamp_log)
BACKEND=$(cfg_get detector.yolo.backend)
[[ -n "$MODEL" ]] || MODEL=$(cfg_get detector.yolo.model)
case "$CONTROLLER" in
    ibvs)         CTRL_NODE=/visp_servo_node ;;
    proportional) CTRL_NODE=/hil_servo_node ;;
    *)            CTRL_NODE="" ;;
esac
case "$DDS" in
    cyclonedds) RMW="rmw_cyclonedds_cpp"; DDS_ENV="CYCLONEDDS_URI=file:///tmp/cyclonedds_hil.resolved.xml" ;;
    fastrtps)   RMW="rmw_fastrtps_cpp";   DDS_ENV="FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_hil.resolved.xml" ;;
    *) die "unknown DDS '$DDS'" ;;
esac

# ── our tools, from the pinned commit of exp/detector-accuracy, to /tmp ─────────
TOOLS=/tmp/hil_tools
OURS_HEAD=$(git -C "$OURS_REPO" rev-parse HEAD 2>/dev/null) || die "cannot read $OURS_REPO"
mkdir -p "$TOOLS"
for t in e2e_latency_probe.py latency_stage_report.py; do
    git -C "$OURS_REPO" show "${OURS_HEAD}:benchmarks/${t}" > "${TOOLS}/${t}" \
        || die "cannot copy benchmarks/${t} from ${OURS_HEAD}"
done
log "tools: ${TOOLS} from exp/detector-accuracy ${OURS_HEAD}"

in_container() {  # run a command inside the stack container with ROS2 sourced
    sudo docker exec ros2_perception_stack bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=${RMW}
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
        export ${DDS_ENV}
        $1
    "
}

sup() {  # sup <logfile> <words...>: one supervisor command, retried once; echoes the reply
    local f="$1"; shift
    [[ "$MATLAB_CTL" == "true" ]] || { echo "skipped"; return 0; }
    local r try
    for try in 1 2; do
        r=$(echo "$*" | timeout 5 nc -w 3 "$MATLAB_HOST_IP" "$MATLAB_CTL_PORT" 2>/dev/null | tr -d '\r')
        [[ -n "$r" ]] && break
        sleep 2
    done
    echo "$(date -u +%FT%TZ) $* -> ${r:-NO_REPLY}" >> "$f"
    [[ -n "$r" ]] || { echo "NO_REPLY"; return 1; }
    echo "$r"
}

wait_for_frames() {  # poll the camera SHM write_seq until frames flow (or timeout)
    python3 - "$1" <<'PYEOF'
import mmap, struct, sys, time
deadline = time.monotonic() + float(sys.argv[1])
last = -1
while time.monotonic() < deadline:
    try:
        with open("/dev/shm/ovcam_frames", "rb") as f:
            mm = mmap.mmap(f.fileno(), 128, mmap.MAP_SHARED, mmap.PROT_READ)
            seq = struct.unpack_from("<Q", mm, 64)[0]
            mm.close()
        if last >= 0 and seq > last:
            sys.exit(0)
        last = seq
    except (FileNotFoundError, ValueError):
        pass
    time.sleep(1.0)
sys.exit(1)
PYEOF
}

save_logs() {  # save_logs <stack_log>: append the /tmp launch, producer and SLAM logs; never fails
    { echo "=== /tmp/hil_launch.log"; cat /tmp/hil_launch.log
      echo "=== /tmp/yolo_producer.log"; cat /tmp/yolo_producer.log
      echo "=== /tmp/slam_sidecar_slam_ov2slam.log (tail 200)"; tail -200 /tmp/slam_sidecar_slam_ov2slam.log
      # 27A: the ORB-SLAM3 RGB-D container's console too, so check_run.py's library-error test sees it; written only
      # when that container ran this run (removed at run start below), so every other stack log is unchanged
      [[ -f /tmp/slam_sidecar_slam_orbslam3.log ]] && { echo "=== /tmp/slam_sidecar_slam_orbslam3.log (tail 200)"
          tail -200 /tmp/slam_sidecar_slam_orbslam3.log; }
    } >> "$1" 2>/dev/null || true
}

teardown() {  # teardown <stack_log> <matlab_log>; exit 7 if the supervisor stopped answering
    local rc=0
    sup "$2" stop >/dev/null || rc=1
    ./run_stack_hil.sh stop >> "$1" 2>&1
    sup "$2" pub_stop >/dev/null || rc=1
    [[ $rc -eq 0 ]] || { log "SUPERVISOR_NO_REPLY (stop/pub_stop) — stack is down"; exit 7; }
}

STACK_ARGS=(--config "$STACK_CONFIG" --mode record)
[[ -n "$MODEL" ]] && STACK_ARGS+=(--model "$MODEL")
log "config: ${STACK_CONFIG} model=${MODEL:-config} backend=${BACKEND} controller=${CONTROLLER}"
log "        duration=${DURATION}s warmup=${WARMUP}s runs=${RUNS} label=${LABEL} → ${OUTPUT_DIR}"

OK_RUNS=0
for N in $(seq 1 "$RUNS"); do
    log "════ run $N/$RUNS ════"
    RUN_PREFIX="${OUTPUT_DIR}/${LABEL}_run${N}"
    STACK_LOG="${RUN_PREFIX}_stack.log"
    MLOG="${RUN_PREFIX}_matlab.log"
    PROBE_CSV="/tmp/hil_probe_${LABEL}_run${N}.csv"
    sudo rm -f "$TELEM_CSV" ${STAMP_CSV:+"$STAMP_CSV"} /tmp/hil_launch.log /tmp/yolo_producer.log \
        /tmp/slam_sidecar_slam_ov2slam.log /tmp/slam_sidecar_slam_orbslam3.log "$PROBE_CSV" 2>/dev/null || true

    # 1. simulation first (a restart resets the drone pose), frames on
    echo "status_before: $(sup "$MLOG" status)" >> "$MLOG"
    r=$(sup "$MLOG" pub_start "$PUB_HZ") || { log "SUPERVISOR_NO_REPLY (pub_start)"; exit 7; }
    [[ "$r" == ok || "$r" == skipped ]] || log "WARN pub_start reply '$r'"
    r=$(sup "$MLOG" start) || { log "SUPERVISOR_NO_REPLY (start)"; sup "$MLOG" pub_stop >/dev/null; exit 7; }
    [[ "$r" == ok || "$r" == skipped ]] || log "WARN start reply '$r'"

    # 2. stack (container, bridges, producer, SLAM sidecar, bag, detector + controller)
    ./run_stack_hil.sh "${STACK_ARGS[@]}" --run-tag "${LABEL}_run${N}" > "$STACK_LOG" 2>&1
    if [[ $? -ne 0 ]]; then
        log "STARTUP_FAILED run $N (stack launch — see $STACK_LOG)"
        save_logs "$STACK_LOG"; teardown "$STACK_LOG" "$MLOG"; continue
    fi
    RUNREL=$(grep -m1 -oP 'Recording \(mode=\w+\) → \K\S+(?=/bag)' "$STACK_LOG")

    # 3. frames must flow before the clock starts
    if ! wait_for_frames 45; then
        log "STARTUP_FAILED run $N (no camera frames within 45 s — is MATLAB publishing?)"
        save_logs "$STACK_LOG"; teardown "$STACK_LOG" "$MLOG"; continue
    fi

    # 3b. /cmd_vel liveness probe, in parallel with the measurement window (DDS discovery)
    CMDVEL_TIMEOUT=$((WARMUP + 45))
    in_container "timeout ${CMDVEL_TIMEOUT} ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist > /dev/null" 2>> "$STACK_LOG" &
    CMDVEL_PID=$!

    # 4. measure (OUR probe, from /tmp/hil_tools)
    log "camera frames flowing — collecting ${DURATION}s (+${WARMUP}s warmup, overlapped)..."
    in_container "python3 ${TOOLS}/e2e_latency_probe.py \
        --duration $((WARMUP + DURATION)) --skip-first-s ${WARMUP} \
        --output ${PROBE_CSV}" >> "$STACK_LOG" 2>&1

    if ! wait "$CMDVEL_PID"; then
        log "STARTUP_FAILED run $N (no /cmd_vel within ${CMDVEL_TIMEOUT} s)"
        save_logs "$STACK_LOG"; teardown "$STACK_LOG" "$MLOG"; continue
    fi

    # 5. what actually ran: controller parameters, SLAM tracking state, supervisor pose
    [[ -n "$CTRL_NODE" ]] && in_container "ros2 param dump ${CTRL_NODE}" > "${RUN_PREFIX}_params.yaml" 2>> "$STACK_LOG"
    in_container "timeout 5 ros2 topic echo --once /slam/tracking_state" > "${RUN_PREFIX}_slam_state.txt" 2>&1 || true
    echo "status_end: $(sup "$MLOG" status)" >> "$MLOG"

    # 6. tear down (stop sends SIGINT to the bag recorder first, so the .mcap is closed)
    teardown "$STACK_LOG" "$MLOG"
    echo "status_after: $(sup "$MLOG" status)" >> "$MLOG"
    sleep 3

    # 7. collect
    BASE="${PROBE_CSV%.csv}"
    cp -f "$PROBE_CSV"         "${RUN_PREFIX}.csv"            2>/dev/null || true
    cp -f "${BASE}_camera.csv" "${RUN_PREFIX}_camera.csv"     2>/dev/null || true
    cp -f "${BASE}_stats.json" "${RUN_PREFIX}_stats.json"     2>/dev/null || true
    cp -f "$TELEM_CSV"         "${RUN_PREFIX}_telemetry.csv"  2>/dev/null || true
    [[ -n "$STAMP_CSV" ]] && cp -f "$STAMP_CSV" "${RUN_PREFIX}_cam_stamps.csv" 2>/dev/null || true
    if [[ -n "$RUNREL" && -d "${WS}/${RUNREL}" ]]; then
        sudo cp -r "${WS}/${RUNREL}/bag" "${RUN_PREFIX}_bag" 2>/dev/null || true
        sudo cp -f "${WS}/${RUNREL}/run_config.yaml" "${RUN_PREFIX}_run_config.yaml" 2>/dev/null || true
        sudo chown -R "$(id -u):$(id -g)" "${RUN_PREFIX}_bag" "${RUN_PREFIX}_run_config.yaml" 2>/dev/null || true
    fi
    save_logs "$STACK_LOG"

    # 8. mission metrics from /bench/state in the bag
    MCAP=$(ls "${RUN_PREFIX}_bag"/*.mcap 2>/dev/null | head -1)
    if [[ -n "$MCAP" ]]; then
        "$MCAP_PY" benchmarks/mission_from_bench_state.py --bag "$MCAP" \
            --output "${RUN_PREFIX}_mission.json" >> "$STACK_LOG" 2>&1 \
            || log "WARN run $N: mission_from_bench_state failed"
    else
        log "WARN run $N: no .mcap found (RUNREL='${RUNREL}')"
    fi

    if [[ ! -s "${RUN_PREFIX}.csv" ]]; then
        log "RUN_FAILED run $N (probe produced no data)"
        continue
    fi

    # 9. per-run stage table (OUR stage report, from /tmp/hil_tools)
    python3 "${TOOLS}/latency_stage_report.py" \
        --telemetry "${RUN_PREFIX}_telemetry.csv" \
        --probe "${RUN_PREFIX}.csv" \
        --bridge-log "${RUN_PREFIX}_cam_stamps.csv" \
        --out "${RUN_PREFIX}_stages.csv" || true

    OK_RUNS=$((OK_RUNS + 1))
    log "run $N complete"
done

# ── aggregate (same summary schema as the detector sweep) ─────────────────────
log "aggregating ${OK_RUNS}/${RUNS} successful runs..."
python3 - "$OUTPUT_DIR" "$LABEL" "$RUNS" "$MODEL" "$BACKEND" "$STACK_CONFIG" "$CONTROLLER" <<'PYEOF'
import csv, glob, json, math, os, sys
outdir, label, runs, model, backend, stack_config, controller = sys.argv[1:8]
runs = int(runs)

stats = []
for p in sorted(glob.glob(os.path.join(outdir, f"{label}_run*_stats.json"))):
    with open(p) as f:
        stats.append(json.load(f))

def ms(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return {"mean": None, "std": None, "n": 0}
    m = sum(vals) / len(vals)
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals)) if len(vals) > 1 else 0.0
    return {"mean": round(m, 3), "std": round(sd, 3), "n": len(vals)}

def agg(fn):
    out = []
    for s in stats:
        try:
            out.append(fn(s))
        except (KeyError, TypeError):
            pass
    return ms([v for v in out if v])

def frozen_ratio(p):
    if not os.path.exists(p):
        return None
    with open(p) as f:
        rows = list(csv.DictReader(f))
    if not rows or "frame_hash" not in rows[0]:
        return None
    h = [r["frame_hash"] for r in rows if r.get("frame_hash")]
    return len(set(h)) / len(h) if h else None

fr = [r for r in (frozen_ratio(os.path.join(outdir, f"{label}_run{n}_telemetry.csv"))
                  for n in range(1, runs + 1)) if r is not None]
fr_mean = round(sum(fr) / len(fr), 4) if fr else None

STAGES = {"S3": "s3_frame_age", "S4": "s4_preprocess", "S5": "s5_inference",
          "S6": "s6_nms", "S7": "s7_shm_write", "S3-S8": "det_path"}
st = {k: [] for k in STAGES.values()}; st["detector_compute"] = []
for n in range(1, runs + 1):
    sp = os.path.join(outdir, f"{label}_run{n}_stages.csv")
    if not os.path.exists(sp):
        continue
    by = {}
    with open(sp) as f:
        for row in csv.DictReader(f):
            try:
                by[row["stage"]] = float(row["mean_ms"])
            except (KeyError, ValueError):
                pass
    for c, k in STAGES.items():
        if c in by:
            st[k].append(by[c])
    comp = [by.get(c) for c in ("S4", "S5", "S6", "S7")]
    if all(c is not None for c in comp):
        st["detector_compute"].append(sum(comp))

missions = []
for n in range(1, runs + 1):
    mp = os.path.join(outdir, f"{label}_run{n}_mission.json")
    if os.path.exists(mp):
        try:
            with open(mp) as f:
                missions.append(json.load(f))
        except (ValueError, OSError):
            pass
mission = {"runs": len(missions), "reached_success_rate": None}
if missions:
    mission = {
        "runs": len(missions),
        "reached_success_rate": round(sum(1 for m in missions if m.get("reached")) / len(missions), 3),
        "time_to_reached_sim_s":   ms([m.get("time_to_reached_sim_s") for m in missions]),
        "approach_duration_sim_s": ms([m.get("approach_duration_sim_s") for m in missions]),
        "sim_rate":                ms([m.get("sim_rate") for m in missions]),
        "time_to_reached_s":       ms([m.get("time_to_reached_s") for m in missions]),
        "approach_duration_s":     ms([m.get("approach_duration_s") for m in missions]),
        "time_to_first_approach_s": ms([m.get("time_to_first_approach_s") for m in missions]),
        "n_reacquire":             ms([m.get("n_reacquire") for m in missions]),
        "n_lost":                  ms([m.get("n_lost") for m in missions]),
        "final_dist_frozen_sign_m": ms([m.get("final_dist_frozen_sign_m") for m in missions]),
    }

summary = {
    "label": label, "stack_config": stack_config, "controller": controller,
    "detector": "yolo", "model": model, "backend": backend,
    "runs_requested": runs, "runs_ok": len(stats),
    "cmd_vel_hz":  agg(lambda s: s["cmd_vel"]["hz_mean"]),
    "det_hz":      agg(lambda s: s["det"]["hz_mean"]),
    "camera_hz":   agg(lambda s: s["sim_camera"]["hz_mean"]),
    "ovcam_hz":    agg(lambda s: s["ovcam"]["hz_mean"]),
    "e2e_det_to_cmd_ms_mean": agg(lambda s: s["e2e_det_to_cmd_ms"]["mean"]),
    "e2e_det_to_cmd_ms_p95":  agg(lambda s: s["e2e_det_to_cmd_ms"]["p95"]),
    "cam_to_det_ms_mean":     agg(lambda s: s["cam_to_det_ms"]["mean"]),
    "cam_to_det_ms_p95":      agg(lambda s: s["cam_to_det_ms"]["p95"]),
    "detector_latency_ms": {k: ms(v) for k, v in st.items()},
    "mission": mission,
    "unique_frame_ratio": fr_mean,
    "frozen_camera_flag": fr_mean is not None and fr_mean < 0.05,
}
out = os.path.join(outdir, f"{label}_summary.json")
with open(out, "w") as f:
    json.dump(summary, f, indent=2)
print(json.dumps(summary, indent=2))
print(f"→ {out}")
PYEOF

[[ "$OK_RUNS" -gt 0 ]] || die "all $RUNS runs failed"
log "done: $OK_RUNS/$RUNS runs ok"
