#!/usr/bin/env bash
# sidecar_stoprule.sh <mlog> <outdir> — 5D: the 4D sidecar without its in-container frame recorder (rec.sh owns recording now). Outside the pinned worktree; writes only to /tmp and <outdir>.
# 1) when the stack container is up: start a frame recorder (/sim/camera/image_raw + /sim/drone_pose, /tmp) and a detection logger
# 5M: the detection logger and FastRTPS profile come from the kept folder stage6/runtime, not /tmp. The stack container mounts
#     only the workspace and /tmp (run_stack_hil.sh, pinned), so both files are docker-cp'd into the container's own filesystem
#     (/r20_runtime, gone with the container) and checked against runtime/SHA256SUMS there. The stack's own /tmp profile is
#     compared with the kept copy and the result logged (a mismatch is logged, not acted on).
# 2) at "resume -> ok": 10 s stop rule (no stop-sign box in 10 s -> hil stop + stack stop), then stop the recorder at resume + 50 s
MLOG="$1"; O="$2"; DL=/tmp/r20_dets.log
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
say() { echo "$(date -u +%FT%T.%3NZ) $*" | tee -a "$O/sidecar.log"; }
RT="$HIL_RUNTIME"; CR=/r20_runtime
ENV="source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 FASTRTPS_DEFAULT_PROFILES_FILE=$CR/fastrtps_hil.resolved.xml"
for _ in $(seq 1 1200); do  # 5F: 600 s (was 300 s), the pre-resume gate runs before the stack boots
  sudo docker ps --format '{{.Names}}' | grep -qx ros2_perception_stack && break; sleep 0.5
done
say "container up"
sudo docker exec ros2_perception_stack mkdir -p "$CR"
for f in r554s1_detlog.py fastrtps_hil.resolved.xml; do sudo docker cp "$RT/$f" "ros2_perception_stack:$CR/$f" >/dev/null; done
WANT=$(grep -E ' (r554s1_detlog.py|fastrtps_hil.resolved.xml)$' "$RT/SHA256SUMS")
if printf '%s\n' "$WANT" | sudo docker exec -i -w "$CR" ros2_perception_stack sha256sum -c --quiet - >/dev/null 2>&1; then
  say "runtime files copied into the container ($CR), sha256 ok"
else
  say "RUNTIME FILES NOT VERIFIED in the container ($CR): detection logger not started"; echo "runtime_files_unverified" > "$O/RUNTIME_UNVERIFIED.txt"
fi
if sudo docker exec ros2_perception_stack cmp -s /tmp/fastrtps_hil.resolved.xml "$CR/fastrtps_hil.resolved.xml"; then
  say "stack /tmp/fastrtps_hil.resolved.xml == kept profile"
else
  say "PROFILE MISMATCH: stack /tmp/fastrtps_hil.resolved.xml differs from the kept profile (logged only)"
fi
if [ ! -e "$O/RUNTIME_UNVERIFIED.txt" ]; then
  sudo docker exec -d ros2_perception_stack bash -lc "$ENV; python3 $CR/r554s1_detlog.py $DL > /tmp/r554s1_detlog.err 2>&1"
  say "detection logger started"
fi
for _ in $(seq 1 1200); do grep -q 'resume -> ok' "$MLOG" 2>/dev/null && break; sleep 0.25; done
TR=$(date +%s%N); say "resume seen (t_resume_ns=$TR)"; echo "$TR" > "$O/t_resume_ns.txt"
sleep 10
N=$(awk -v a="$TR" -v b="$((TR + 10000000000))" '$1>=a && $1<=b && $3>0 {n++} END {print n+0}' "$DL" 2>/dev/null)
M=$(awk -v a="$TR" -v b="$((TR + 10000000000))" '$1>=a && $1<=b {n++} END {print n+0}' "$DL" 2>/dev/null)
say "first 10 s after resume: $M detection messages, $N with a stop-sign box"
if [ "${M:-0}" -eq 0 ]; then
  say "detection logger saw NO messages: logger fault, not a detector result; stop rule not applied live (checked from the run bag after)"
  echo "logger_no_messages" > "$O/STOP_RULE_NOT_EVALUATED.txt"
elif [ "${N:-0}" -eq 0 ]; then
  say "STOP RULE: no stop-sign box in 10 s after resume -> ending the run"; echo "stop_rule_fired" > "$O/STOP_RULE.txt"
  hil stop | tee -a "$O/sidecar.log"
  (cd "$HIL_WT" && ./run_stack_hil.sh stop) >> "$O/sidecar.log" 2>&1
  exit 3
fi
say "stop rule passed: stop-sign boxes seen in the first 10 s"
