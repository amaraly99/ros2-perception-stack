#!/usr/bin/env bash
# rec.sh start|stop <name> [extra]: 5D recorder fix. Each recorder runs in ITS OWN container (--init, so tini is PID 1 and
# forwards signals), with the process exec'd so SIGINT reaches it. stop = SIGINT, wait up to 30 s for a clean exit, check
# the closing line; only then force-remove (and say so). Writes to /tmp only.
# 5M: inputs (bag QoS, render logger, FastRTPS profile) come from the kept folder stage6/runtime (mounted read-only into
# each recorder container), not /tmp; start refuses (exit 2) if a runtime file is missing or differs from runtime/SHA256SUMS.
#   bag:  rec.sh start bag   -> /tmp/r20_bag  (/sim/drone_pose /cmd_vel /yolo/detections /bench/state), log /tmp/r20_bag_rec.log
#   5Q: no camera images (1.4 GB per run in 5I; the held render is kept by the hash logger). /yolo/detections is added because
#   this bag is up before resume and Table 14 is scored from it (the stack's bag starts 1.6 s after resume, 5I). Its type
#   (yolo_msgs) comes from the pinned worktree's install/, mounted READ-ONLY at /workspace as the stack mounts it.
#   hash: rec.sh start hash  -> render_hash_logger.py, log /tmp/r20_hash.log, renders /tmp/r20_renders/
#   25C (R5, DEVIATIONS 17.86): REC_HELD_ONLY=1 (unset: every distinct render, as before) hands the logger the marker
#   /tmp/r20_hold_done, which hold_gate.sh writes once it has read the final held render: no render is saved after it,
#   and the last frame is saved at exit as terminal_<sha1>.raw (the one flight frame kept, for the terminal-state figure).
set -u; set -o noclobber; ACT="$1"; K="$2"; C="r20_$K"
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
# 5J standing rule: never overwrite or append to an earlier run's files. start refuses (exit 2) while its outputs exist;
# r20.sh moves them into the run folder after each run.
refuse_if_exists() { for f in "$@"; do [ -e "$f" ] && { echo "rec: REFUSE: $f exists (left from an earlier run; move it aside)"; exit 2; }; done; }
RT="$HIL_RUNTIME"
check_runtime() { (cd "$RT" 2>/dev/null && sha256sum --quiet -c SHA256SUMS >/dev/null 2>&1) || { echo "rec: REFUSE: runtime files in $RT missing or changed (sha256sum -c SHA256SUMS)"; exit 2; }; }
ENV="source /opt/ros/jazzy/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 FASTRTPS_DEFAULT_PROFILES_FILE=$RT/fastrtps_hil.resolved.xml"
case "$ACT:$K" in
  start:bag)
    refuse_if_exists /tmp/r20_bag /tmp/r20_bag_rec.log; check_runtime
    [ -f "$HIL_WT/install/setup.bash" ] || { echo "rec: REFUSE: $HIL_WT/install/setup.bash missing (yolo_msgs type support)"; exit 2; }
    sudo docker run -d --rm --init --name "$C" --entrypoint "" --net=host --ipc=host -v /tmp:/tmp -v "$RT:$RT:ro" -v "$HIL_WT:/workspace:ro" ros2_perception_stack \
      bash -lc "$ENV; source /workspace/install/setup.bash; exec ros2 bag record --storage mcap -o /tmp/r20_bag --qos-profile-overrides-path $RT/r554_bag_qos.yaml /sim/drone_pose /cmd_vel /yolo/detections /bench/state > /tmp/r20_bag_rec.log 2>&1" >/dev/null
    for _ in $(seq 1 60); do grep -q "Subscribed to topic '/sim/drone_pose'" /tmp/r20_bag_rec.log 2>/dev/null && { echo "rec bag: up"; exit 0; }; sleep 0.5; done
    echo "rec bag: NOT subscribed after 30 s"; exit 3 ;;
  start:hash)
    refuse_if_exists /tmp/r20_hash.log /tmp/r20_hash.err /tmp/r20_renders /tmp/r20_hold_done; check_runtime
    case "${REC_HELD_ONLY:-}" in "") HD= ;; 1) HD=/tmp/r20_hold_done ;; *) echo "rec: REFUSE: REC_HELD_ONLY '${REC_HELD_ONLY}' is not empty or 1"; exit 2 ;; esac
    sudo docker run -d --rm --init --name "$C" --entrypoint "" --net=host --ipc=host -v /tmp:/tmp -v "$RT:$RT:ro" ros2_perception_stack \
      bash -lc "$ENV; exec python3 $RT/r20_render_hash_logger.py /tmp/r20_hash.log /tmp/r20_renders $HD > /tmp/r20_hash.err 2>&1" >/dev/null
    echo "rec hash: started${HD:+ (renders saved until $HD exists, then only the terminal frame)}" ;;
  stop:*)
    sudo docker kill -s INT "$C" >/dev/null 2>&1
    if timeout 30 sudo docker wait "$C" >/dev/null 2>&1 || ! sudo docker ps -q --filter "name=^$C\$" | grep -q .; then
      echo "rec $K: exited cleanly after SIGINT"
    else
      echo "rec $K: still running 30 s after SIGINT -> force remove"; sudo docker rm -f "$C" >/dev/null 2>&1
    fi
    [ "$K" = bag ] && { grep -q 'Recording stopped' /tmp/r20_bag_rec.log && echo "rec bag: 'Recording stopped' logged" || echo "rec bag: NO closing line"; ls /tmp/r20_bag; } ;;
esac
