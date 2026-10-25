#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# slam_hil_bench.sh — ONE flight of ONE SLAM arm. Runs on the Pi HOST (not in the container).
#
#   ./benchmarks/slam_hil_bench.sh <arm> <run_num>
#
# It boots the stack with run_stack_hil.sh (mono, stereo) or run_stack_hil_rgbd.sh (rgbd),
# waits for the record gate, writes meta.txt, then stays alive until SIGINT or SIGTERM and
# stops the stack. Every decision (readiness, tracking, arrival, valid or not) is made by
# the Python runner (benchmarks/hilbench/kinds/slam.py). This script sends no MATLAB command.
#
# Required env:
#   MATLAB_HOST_IP   Windows host IP; must equal network.matlab_host_ip of the stack config
# Optional env:
#   CAMERA_MODE      mono | stereo | rgbd (default mono): picks the stack config of the arm file
#   DDS              fastrtps | cyclonedds (default fastrtps); must equal network.dds of the stack config
#   ROS_DOMAIN_ID    default 0
#   RUN_TAG          session tag, part of the run folder name
#   RECORD_GATE      path of a file; recording starts only once that file exists
#   RECORD_GATE_TIMEOUT  seconds to wait for the gate (default 120)
#
# Lines the runner reads: "Run dir: bags/...", "waiting for record gate", "[bench] recording".
# ─────────────────────────────────────────────────────────────────
set -u
cd "$(dirname "$0")/.."
WS="$(pwd)"

log() { echo "[bench] $*"; }
die() { echo "[bench] ERROR: $*" >&2; exit 1; }

STACK_PID=""
STACK_SCRIPT=""
STACK_LOG=""
STOP_REQUESTED=0
RUNREL=""

# ── 1. parse_args ─────────────────────────────────────────────────
parse_args() {
    ARM="${1:-}"
    RUN_NUM="${2:-1}"
    [[ -n "$ARM" ]] || die "usage: $0 <arm> [run_num]"
    [[ "$ARM" =~ ^[A-Za-z0-9_]+$ ]] || die "arm name must be letters, digits, underscore (got '$ARM')"
    [[ "$RUN_NUM" =~ ^[0-9]+$ ]] || die "run_num must be a positive integer (got '$RUN_NUM')"
    CAMERA_MODE="${CAMERA_MODE:-mono}"
    case "$CAMERA_MODE" in
        mono|stereo|rgbd) ;;
        *) die "CAMERA_MODE must be mono, stereo or rgbd (got '$CAMERA_MODE')" ;;
    esac
    ARM_FILE="$WS/config/hil/slam_arms/${ARM}.yaml"
    [[ -f "$ARM_FILE" ]] || die "arm file not found: $ARM_FILE"
    STACK="$(python3 - "$ARM_FILE" "$CAMERA_MODE" <<'PY'
import sys, yaml
arm = yaml.safe_load(open(sys.argv[1])) or {}
print((arm.get("stack") or {}).get(sys.argv[2], ""))
PY
)"
    [[ -n "$STACK" ]] || die "arm '$ARM' has no stack config for camera mode '$CAMERA_MODE'"
    STACK_CFG="$WS/config/hil/stack/${STACK}.yaml"
    [[ -f "$STACK_CFG" ]] || die "stack config not found: $STACK_CFG"
    if [[ "$CAMERA_MODE" == "rgbd" ]]; then
        STACK_SCRIPT="$WS/run_stack_hil_rgbd.sh"
    else
        STACK_SCRIPT="$WS/run_stack_hil.sh"
    fi
    [[ -x "$STACK_SCRIPT" ]] || die "not executable: $STACK_SCRIPT"
    log "arm=$ARM run=$RUN_NUM camera_mode=$CAMERA_MODE stack=$STACK"
}

# ── 2. check_arm_file — the two gates of preflight() in scripts/hil_matrix/run_matrix.py ──
check_arm_file() {
    # init_gate.enabled=true would override --hold-fsm and launch the controller.
    local gate delay
    gate="$(awk '/^init_gate:/{f=1} f&&/^  enabled:/{print $2; exit}' "$STACK_CFG")"
    [[ "$gate" == "false" ]] || die "$STACK: init_gate.enabled='$gate', must be false"
    # The startup delay must be the same for every arm, or the comparison is unfair.
    delay="$(grep -E '^ *startup_delay_sec:' "$STACK_CFG" | grep -oE '[0-9]+' | head -1)"
    [[ "$delay" == "2" ]] || die "$STACK: slam.startup_delay_sec='$delay', must be 2"
    log "stack config ok: init_gate false, startup_delay_sec 2"
}

# ── 3. resolve_env — the DDS and host settings must agree with the stack config ──
resolve_env() {
    : "${MATLAB_HOST_IP:?set MATLAB_HOST_IP to your Windows host IP}"
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    DDS="${DDS:-fastrtps}"
    RUN_TAG="${RUN_TAG:-}"
    case "$DDS" in
        cyclonedds|fastrtps) ;;
        *) die "DDS must be 'cyclonedds' or 'fastrtps' (got '$DDS')" ;;
    esac
    command -v envsubst >/dev/null || die "'envsubst' missing - apt install gettext-base"
    # run_stack_hil.sh takes these two values from the stack config, not from the env.
    # A different value here means MATLAB and the Pi would not find each other.
    local cfg_dds cfg_ip
    cfg_dds="$(awk '/^network:/{f=1} f&&/^  dds:/{print $2; exit}' "$STACK_CFG")"
    cfg_ip="$(awk '/^network:/{f=1} f&&/^  matlab_host_ip:/{print $2; exit}' "$STACK_CFG")"
    [[ "$cfg_dds" == "$DDS" ]] || die "$STACK: network.dds='$cfg_dds' but DDS='$DDS'"
    [[ "$cfg_ip" == "$MATLAB_HOST_IP" ]] || die "$STACK: network.matlab_host_ip='$cfg_ip' but MATLAB_HOST_IP='$MATLAB_HOST_IP'"
    local iface
    iface="$(ip route get "$MATLAB_HOST_IP" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)"
    [[ -n "$iface" ]] || die "no route to $MATLAB_HOST_IP"
    export MATLAB_HOST_IP ROS_DOMAIN_ID DDS RUN_TAG
    log "env: dds=$DDS domain=$ROS_DOMAIN_ID matlab=$MATLAB_HOST_IP iface=$iface"
}

# ── 4. boot_stack — run_stack_hil.sh (or the rgbd copy), recording waits for the gate ──
boot_stack() {
    local tag="${RUN_TAG:+${RUN_TAG}_}${ARM}_N${RUN_NUM}"
    STACK_LOG="/tmp/slam_hil_bench_${ARM}_N${RUN_NUM}.log"
    : > "$STACK_LOG"
    log "starting $(basename "$STACK_SCRIPT") --config $STACK --mode benchmark --hold-fsm --run-tag $tag"
    # The stack script prints "Run dir:" and "waiting for record gate"; tee passes them on.
    "$STACK_SCRIPT" --config "$STACK" --mode benchmark --hold-fsm --run-tag "$tag" \
        > >(tee "$STACK_LOG") 2>&1 &
    STACK_PID=$!
}

# ── 5. wait_gate — the stack script returns after the recorder has started ──
wait_gate() {
    local rc=0
    wait "$STACK_PID" || rc=$?
    if [[ $STOP_REQUESTED -eq 1 ]]; then
        return 0
    fi
    [[ $rc -eq 0 ]] || die "$(basename "$STACK_SCRIPT") exited with code $rc (see $STACK_LOG)"
    RUNREL="$(grep -m1 -oE 'Run dir: bags/[^ ]+' "$STACK_LOG" | sed 's/^Run dir: //')"
    [[ -n "$RUNREL" ]] || die "no 'Run dir:' line from the stack script"
    log "stack up, run dir $RUNREL"
}

# ── 6. start_samplers — the stack script starts them in benchmark mode; check, do not duplicate ──
start_samplers() {
    local f
    for f in slam_process_cpu.csv slam_thread_cpu.csv; do
        if [[ -e "$WS/$RUNREL/$f" ]]; then
            log "sampler file present: $f"
        else
            log "WARN sampler file missing (yet): $f"
        fi
    done
}

# ── 7. write_meta — append the facts of this flight to meta.txt ──
write_meta() {
    {
        echo "slam_arm=$ARM"
        echo "camera_mode=$CAMERA_MODE"
        echo "run_num=$RUN_NUM"
        echo "stack_config=$STACK"
        echo "stack_script=$(basename "$STACK_SCRIPT")"
        echo "bench_script=slam_hil_bench.sh"
        echo "record_gate=${RECORD_GATE:-}"
    } >> "$WS/$RUNREL/meta.txt"
}

# ── 8. hold_until_stop — the runner owns the time ──
hold_until_stop() {
    log "running until SIGINT or SIGTERM ..."
    while [[ $STOP_REQUESTED -eq 0 ]]; do
        sleep 0.5
    done
}

# ── 9. stop_stack — also called by the EXIT trap ──
STOPPED=0
stop_stack() {
    [[ $STOPPED -eq 0 ]] || return 0
    STOPPED=1
    log "Stopping..."
    [[ -n "$STACK_PID" ]] && kill "$STACK_PID" 2>/dev/null
    if [[ -n "$STACK_SCRIPT" ]]; then
        "$STACK_SCRIPT" stop || true
    fi
    log "stack stopped"
}

on_signal() { STOP_REQUESTED=1; }
trap on_signal INT TERM
trap stop_stack EXIT

parse_args "$@"
check_arm_file
resolve_env
boot_stack
wait_gate
if [[ $STOP_REQUESTED -eq 0 ]]; then
    start_samplers
    write_meta
    hold_until_stop
fi
exit 0
