#!/usr/bin/env bash
# smoke_run_hold.sh <label=level2_<model>_<backend>_<placement>_r<HOLD_K>> <stack_config> <model> — HELD-START arm: one_run_hold.sh (unmodified) via an adapter
# Pre/post guards: throttle 0x0, our repo clean at its pinned HEAD, worktree diff == the saved patch.
# Then the worktree runner (one run, warmup 5 s, 60 s window), throttle telemetry, a live process
# sampler (Cpus_allowed_list of detector + controller, OpenCV libs mapped into ov2slam_node), pin record.
# Exit: runner rc, or 10 throttle before, 11 tracked change before, 12 throttle during, 13 tracked change after,
#       14 sim model sha256 missing or not the expected one.
# Patch: ours_on_4490660_2026-10-04b.patch (e337034b...), kept in patches/. The 2026-10-03 patch (2e8e354e...) is SUPERSEDED.
# 5Q (benchmarks/hil_runs): pins and paths from hil_env.sh. Our repo must be clean at HIL_OURS_PIN (the batch sets it to HEAD
# at batch start, so every run of a batch comes from one commit); outputs go to SMOKE_OUT (the run's stack/ folder).
set -u
L="$1"; CFG="$2"; M="$3"
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
OUT="${SMOKE_OUT:?set SMOKE_OUT to the stack folder of the run}"; mkdir -p "$OUT"
OURS="$HIL_OURS"; WT="$HIL_WT"; PATCH="$HIL_PATCH"; PATCH_SHA="$HIL_PATCH_SHA"; AMAR_PIN="$HIL_AMAR_PIN"
OURS_PIN="${HIL_OURS_PIN:?set HIL_OURS_PIN to the commit of our repo this run must come from}"
# Sim model check (2026-10-04). The Pi cannot read the Windows file, so the operator exports
# SIM_MODEL_SHA256 = the sha256 apply_sim_params.m printed for the model now running; it must equal
# model_sha256 in sim_geometry.json. Until 4C fills that in (PENDING_4C), every run stops here.
# 25C (R4, DEVIATIONS 17.86): that is the check of the frozen setup (HIL_SIM_SETUP=frozen, the default) and is unchanged. For
# another setup the hash is not used: sim_setup.py compares the supervisor's model_info reply with the pinned reply of
# HIL_SIM_MODE (exit 14 on a mismatch), and the pin records the setup instead of the sim_geometry.json hash.
# 25C (R3): sample_procs waits for ov2slam_node only when the stack config enables SLAM, so a SLAM-off run is sampled too.
GEOM="$HIL_GEOM"
WIN_MODEL_SHA=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['model_sha256'])" "$GEOM")
say() { echo "$(date -u +%FT%TZ) [$L] $*" | tee -a "$OUT/stage3.log"; }

tracked_ok() {  # our repo clean at pin; worktree HEAD at pin and its full diff == the saved patch
    [ "$(git -C "$OURS" rev-parse HEAD)" = "$OURS_PIN" ] || { echo "ours HEAD moved"; return 1; }
    [ -z "$(git -C "$OURS" status --porcelain)" ] || { echo "ours dirty"; return 1; }
    [ "$(git -C "$WT" rev-parse HEAD)" = "$AMAR_PIN" ] || { echo "worktree HEAD moved"; return 1; }
    local idx; idx=$(mktemp); rm -f "$idx"
    local s; s=$(cd "$WT" && GIT_INDEX_FILE="$idx" git read-tree "$AMAR_PIN" && GIT_INDEX_FILE="$idx" git add -A \
        && GIT_INDEX_FILE="$idx" git diff --cached --binary --full-index "$AMAR_PIN" | sha256sum | cut -d' ' -f1)
    rm -f "$idx"
    [ "$s" = "$PATCH_SHA" ] || { echo "worktree diff sha $s != patch"; return 1; }
}

sample_procs() {  # live: wait for the stack, then record affinity + mapped OpenCV libs (host /proc)
    local f="$OUT/${L}_procs.txt" ctrl det ov slam_on
    slam_on=$(python3 "$WT/scripts/parse_stack.py" "$WT/config/hil/stack/$CFG.yaml" --get slam.enabled 2>/dev/null)
    for _ in $(seq 1 90); do
        ctrl=$(pgrep -f '/(visp_servo_node|hil_servo_node)( |$)' | head -1)
        det=$(pgrep -f 'src/yolo_producer/.*producer\.py' | head -1)
        ov=$(pgrep -f '/ov2slam_node( |$)' | head -1)
        [ -n "$ctrl" ] && [ -n "$det" ] && { [ -n "$ov" ] || [ "$slam_on" = False ]; } && break
        sleep 2
    done
    sleep 15   # let SLAM settle past its startup delay
    {
        echo "sampled_utc: $(date -u +%FT%TZ)"
        for name in detector_producer yolo_bridge_node controller ov2slam_node; do
            case $name in
                detector_producer) p=$(pgrep -f 'src/yolo_producer/.*producer\.py' | head -1) ;;
                yolo_bridge_node)  p=$(pgrep -f '/yolo_bridge_node( |$)' | head -1) ;;
                controller)        p=$(pgrep -f '/(visp_servo_node|hil_servo_node)( |$)' | head -1) ;;
                ov2slam_node)      p=$(pgrep -f '/ov2slam_node( |$)' | head -1) ;;
            esac
            if [ -n "$p" ]; then
                echo "$name pid=$p comm=$(cat /proc/$p/comm 2>/dev/null) $(sudo grep Cpus_allowed_list /proc/$p/status | tr -s '\t' ' ')"
            else
                echo "$name NOT_FOUND"
            fi
        done
        if [ -n "${ov:-}" ] && [ -d "/proc/$ov" ]; then
            echo "ov2slam_node_mapped_opencv:"
            sudo awk '$6 ~ /libopencv/ {print $6}' "/proc/$ov/maps" | sort -u | sed 's/^/  /'
        fi
    } > "$f" 2>&1
}

th=$(vcgencmd get_throttled | cut -d= -f2)
[ $(( th & 0xF )) -eq 0 ] || { say "STOP throttle before run (live bits): $th"; exit 10; }; say "throttle word before run: $th (5F: guard on live bits only; sticky bits 16-19 recorded, not blocking)"
w=$(tracked_ok) || { say "STOP tracked change before run: $w"; exit 11; }
if [ "$HIL_SIM_SETUP" = frozen ]; then
case "$WIN_MODEL_SHA" in [0-9a-f]*) ;; *) say "STOP sim model sha256 not set in $GEOM ($WIN_MODEL_SHA)"; exit 14 ;; esac
[ "${#WIN_MODEL_SHA}" -eq 64 ] || { say "STOP sim model sha256 in $GEOM is not 64 hex chars"; exit 14; }
[ "${SIM_MODEL_SHA256:-}" = "$WIN_MODEL_SHA" ] || { say "STOP sim model sha256 '${SIM_MODEL_SHA256:-unset}' != expected $WIN_MODEL_SHA"; exit 14; }
else
SS=$(python3 "$HIL_RUNS/sim_setup.py" check "$HIL_SIM_SETUP" "$HIL_SIM_MODE" 2>&1) || { say "STOP $SS"; exit 14; }; say "$SS"
WIN_MODEL_SHA=""                                                # not the running model's: the pin records the setup instead
fi

bash "$HIL_HOST/pi_telemetry.sh" "$OUT/${L}_thermal.csv" 1 & TP=$!
sample_procs & SP=$!
say "RUN cfg=$CFG model=$M temp=$(vcgencmd measure_temp)"
( L2_OUT="$OUT" L2_BACKEND="${HOLD_BACKEND:?}" L2_PLACEMENT="${HOLD_PLACEMENT:?}" L2_RUNNER="$HIL_RUNS/runner_adapter_hold.sh" ADAPTER_STACK_CONFIG="$CFG" L2_EXTRA="--warmup 5" L2_DURATION="${HOLD_DURATION:-60}" L2_EARLY_ACTION=mark L2_PUB_HZ=20 bash "${HOLD_SCRIPT:-$HIL_RUNS/one_run_hold_gated.sh}" "$M" "${HOLD_K:?}" \
    ) > "$OUT/${L}_holdwrap.log" 2>&1
RC=$?
kill "$TP" 2>/dev/null; wait "$TP" 2>/dev/null
kill "$SP" 2>/dev/null; wait "$SP" 2>/dev/null
thr=$(awk -F, 'NR>1 && $5!="0"{n++} END{print n+0}' "$OUT/${L}_thermal.csv")   # 5F: live bits (cur_flags), not the sticky word
tmax=$(awk -F, 'NR>1 && $2+0>m{m=$2+0} END{print m}' "$OUT/${L}_thermal.csv")
w2=$(tracked_ok); TRC=$?

ART=$(grep -oP '^\s*artifact: "\K[^"]+' "$OUT/${L}_run1_run_config.yaml" 2>/dev/null)
python3 - "$OUT/${L}_pin.json" "$OUT/${L}_procs.txt" <<PYEOF
import json, subprocess, sys
def sh(c): return subprocess.run(c, shell=True, capture_output=True, text=True).stdout.strip()
art = "${ART}"
try:
    procs = open(sys.argv[2]).read().splitlines()
except OSError:
    procs = ["not sampled"]
json.dump({
  "label": "${L}", "stack_config": "${CFG}", "model": "${M}",
  "ours_repo": {"path": "${OURS}", "head": sh("git -C ${OURS} rev-parse HEAD"),
                "status_porcelain": sh("git -C ${OURS} status --porcelain")},
  "amar_worktree": {"path": "${WT}", "head": sh("git -C ${WT} rev-parse HEAD"),
                    "status_porcelain": sh("git --no-optional-locks -C ${WT} status --porcelain")},  # 5Q: no index refresh in the pinned worktree
  "patch": {"path": "${PATCH}", "sha256": sh("sha256sum ${PATCH} | cut -d' ' -f1"),
            "supersedes": ["e337034b4c2bbb44fe4c33240c06864a8c22a3acb693f8e11bec4f2bf2125af9",
                           "2e8e354e8069ea0f594cecc01d9a6438739dc7bef18fea66604e8c1108281904"]},
  "image_id": sh("sudo docker image inspect ros2_perception_stack --format '{{.Id}}'"),
  "model_artifact": {"path": art, "sha256": sh(f"sha256sum '{art}' | cut -d' ' -f1") if art else ""},
  "windows_simulink_model_sha256": "${WIN_MODEL_SHA}" or None,
  **({"sim_setup": "${HIL_SIM_SETUP}/${HIL_SIM_MODE}"} if "${HIL_SIM_SETUP}" != "frozen" else {}),
  "cpu_pin_note": "detector.cpu and controller.cpu = \"0-3\": taskset -c 0-3, all four Pi 5 cores, equivalent to unpinned",
  "live_processes": procs,
  "runner_rc": ${RC}, "throttle_rows_nonzero": ${thr}, "temp_max_c": "${tmax}",
  "tracked_check_after": "${w2:-ok}",
}, open(sys.argv[1], "w"), indent=2)
PYEOF
say "END rc=$RC throttle_rows=$thr tmax=$tmax tracked_after=${w2:-ok}"
[ "$thr" -gt 0 ] && exit 12
[ $TRC -ne 0 ] && exit 13
exit $RC
