#!/usr/bin/env python3
"""slam_replay.py — replay one recorded flight into SLAM backends offline (identical images for every backend).

    python3 benchmarks/slam_replay.py <run_dir> --stacks unified_ov2slam_accurate_mono unified_orbslam2_mono ...
    python3 benchmarks/slam_replay.py <run_dir> --stacks ... --start-s 9.5   # skip the init leg (no init gate)

Each backend runs exactly as its stack config starts it live (same image, command, remaps, overlay, library
path), on ROS_DOMAIN_ID 99 (isolated from the live HIL on 0). `ros2 bag play` publishes the recorded SLAM
inputs with their original stamps. A logger writes the backend's pose topic to CSV.
Output: <run_dir>/replay/<stack>[_from<s>]/ (poses.csv, slam.log, play.log). Score with slam_replay_score.py.
"""
import argparse
import shlex
import subprocess
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent
PERCEPTION = "ros2_perception_stack"


def stack_env(name):
    out = subprocess.run([sys.executable, str(WS / "scripts" / "parse_stack.py"), str(WS / "config/hil/stack" / f"{name}.yaml"),
                          "--emit-env"], capture_output=True, text=True, check=True).stdout
    env = {}
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            try:
                env[k] = shlex.split(v)[0] if v else ""
            except ValueError:
                env[k] = v
    return env


def input_topics(name, env):
    t = ["/ovcam/image_raw"]
    if env.get("STEREO_ON") == "true":
        t.append("/ovcam/right/image_raw")
    if "rgbd" in name:
        t.append("/sim/camera/depth/image_raw")
    return t


def docker(*args, check=False):
    return subprocess.run(["sudo", "-n", "docker", *args], capture_output=True, text=True, check=check)


def replay_one(run_dir, name, start_s, warmup_s):
    env = stack_env(name)
    tag = name + (f"_from{start_s:g}" if start_s else "")
    out = run_dir / "replay" / tag
    out.mkdir(parents=True, exist_ok=True)
    for c in ("replay_slam", "replay_log", "replay_play"):
        docker("rm", "-f", c)
    common = ["--net=host", "--ipc=host", "-v", f"{WS}:/workspace", "-v", f"{out}:/out"]
    ros = ("source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; export ROS_DOMAIN_ID=99; "
           "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI; ")
    overlay = f"source {env['SLAM_SETUP_OVERLAY']}; " if env.get("SLAM_SETUP_OVERLAY") else ""
    ld = f"export LD_LIBRARY_PATH={env['SLAM_LD_PREFIX']}:/workspace/opencv/build/lib:${{LD_LIBRARY_PATH:-}}; "
    remaps = env.get("SLAM_REMAPS", "")
    slam_cmd = (ros + overlay + ld + "export ORB_BENCH_TIMING_CSV=/out/timing_events.csv OV2_BENCH_TIMING_CSV=/out/ov2slam_timing_events.csv; "
                f"exec stdbuf -oL -eL {env['SLAM_COMMAND']} --ros-args {remaps} > /out/slam.log 2>&1")
    docker("run", "-d", "--name", "replay_slam", "--entrypoint", "bash", *common, env["SLAM_IMAGE"], "-c", slam_cmd, check=True)
    pose_topic = env.get("SLAM_POSE_OUT") or "/slam/pose"
    docker("run", "-d", "--name", "replay_log", "--entrypoint", "bash", *common, PERCEPTION, "-c",
           ros + f"exec python3 /workspace/benchmarks/slam_replay_pose_logger.py {pose_topic} /out/poses.csv", check=True)
    # wait until the backend says it is ready (vocabulary loaded, handshake done), then a margin
    ready = ("ORB Extractor Parameters", "handshake ACKed", "ready to process Keyframes", "Odometry", "rtabmap")
    t_w = time.monotonic()
    while time.monotonic() - t_w < 180:
        txt = (out / "slam.log").read_text(errors="replace") if (out / "slam.log").exists() else ""
        if any(k in txt for k in ready) and time.monotonic() - t_w >= 5:
            break
        time.sleep(1.0)
    time.sleep(max(3.0, warmup_s - (time.monotonic() - t_w)) if warmup_s > 0 else 3.0)
    topics = " ".join(input_topics(name, env))
    bag = (run_dir / "bag").relative_to(WS)
    play = ros + f"ros2 bag play /workspace/{bag} --topics {topics}" + (f" --start-offset {start_s:g}" if start_s else "") + " > /out/play.log 2>&1"
    t0 = time.monotonic()
    docker("run", "--rm", "--name", "replay_play", "--entrypoint", "bash", *common, PERCEPTION, "-c", play)
    time.sleep(6.0)                            # let the backend finish the last frames
    for c in ("replay_log", "replay_slam"):
        docker("stop", "-t", "8", c)
        docker("rm", "-f", c)
    subprocess.run(["sudo", "-n", "chown", "-R", "amaraly:amaraly", str(out)])
    n = sum(1 for line in open(out / "poses.csv") if line[:1].isdigit()) if (out / "poses.csv").exists() else 0
    print(f"{tag}: {n} poses in {time.monotonic() - t0:.0f} s -> {out}", flush=True)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--stacks", nargs="+", required=True)
    ap.add_argument("--start-s", type=float, default=0.0, help="start the replay this far into the bag")
    ap.add_argument("--warmup-s", type=float, default=25.0)
    a = ap.parse_args()
    run_dir = Path(a.run_dir).resolve()
    for name in a.stacks:
        try:
            replay_one(run_dir, name, a.start_s, a.warmup_s)
        except subprocess.CalledProcessError as e:
            print(f"{name}: FAILED to start ({e.stderr.strip()[:200]})", flush=True)


if __name__ == "__main__":
    main()
