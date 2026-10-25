#!/usr/bin/env python3
"""slam_record_flight.py — fly the system-like approach once and record every camera stream SLAM uses.

    ~/.venvs/hilrun/bin/python benchmarks/slam_record_flight.py --mode stereo --init 1 --tag straight_stereo061_init
    ~/.venvs/hilrun/bin/python benchmarks/slam_record_flight.py --mode rgbd   --init 0 --tag straight_rgbd_noinit

MATLAB must run hil_session_start_slam(<mode>, ...) (the waiting loop). The flight is matlab/slam_straight_approach.m:
optional 3 m sideways init leg, then straight to the controllers' stop point, camera facing the sign.
The bag (bags/run_..._<tag>_N1_*/bag) holds GT poses and the SLAM inputs; benchmarks/slam_replay.py replays it
into every backend offline, so all backends get identical images. The live SLAM sidecar of --arm also runs
(its result is in the same bag) but is not the point.
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hilbench.container import BenchProcess  # noqa: E402
from hilbench.gate import RecordGate  # noqa: E402
from hilbench.supervisor import Supervisor, SupervisorError  # noqa: E402

WS = Path(__file__).resolve().parent.parent
TOPICS = {"mono": "/ovcam/image_raw",
          "stereo": "/ovcam/image_raw /ovcam/right/image_raw",
          "rgbd": "/ovcam/image_raw /sim/camera/depth/image_raw"}
ARM = {"mono": "ov2_accurate", "stereo": "ov2_accurate", "rgbd": "orbslam2"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=sorted(TOPICS), required=True)
    ap.add_argument("--init", type=int, choices=(0, 1), default=1, help="1 = 3 m sideways init leg first")
    ap.add_argument("--vmax", type=float, default=1.0)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--arm", help="arm whose stack boots the camera bridge (default per mode)")
    ap.add_argument("--host", default="192.168.1.201")
    ap.add_argument("--port", type=int, default=55556)
    a = ap.parse_args()
    arm = a.arm or ARM[a.mode]

    sup = Supervisor(a.host, a.port)
    try:
        sup.send("probe_init")
        sys.exit("MATLAB accepted a bare probe_init: old supervisor. Paste hil_session_start_slam(...).")
    except SupervisorError as e:
        if "bad_args" not in str(e):
            sys.exit(f"MATLAB is not in the waiting loop ({e}). Paste hil_session_start_slam(...).")
    info = sup.model_info()
    print(f"MATLAB model {info.get('model')}  right_cam={info.get('right_cam')}  depth_port={info.get('depth_port')}")
    sup.pub_on(20)
    st = sup.hold(60.0)
    print(f"held at x={st.get('x')} y={st.get('y')} z={st.get('z')} yaw={st.get('yaw')}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    gate = RecordGate(WS, f"rec_{a.tag}_{stamp}", 1, 4)
    gate.arm()
    env = {"MATLAB_HOST_IP": a.host, "DDS": "fastrtps", "ROS_DOMAIN_ID": "0", "RUN_TAG": a.tag,
           "RECORD_GATE": str(gate.host_path), "RECORD_GATE_TIMEOUT": "600", "CAMERA_MODE": a.mode,
           "EXTRA_BAG_TOPICS": TOPICS[a.mode]}
    log = WS / "bags" / f".rec_{a.tag}_{stamp}_console.log"
    bench = BenchProcess([f"./benchmarks/slam_hil_bench.sh", arm, "1"], log, lambda line, b: None,
                         in_container=False, env=env)
    t_end = time.monotonic() + 120
    while time.monotonic() < t_end and not bench.gate_wait and bench.alive():
        time.sleep(0.5)
    if not bench.gate_wait:
        bench.stop(timeout=90, signal_cmd=["pkill", "-TERM", "-f", "slam_hil_bench.sh"])
        sys.exit(f"bench did not reach the gate: {bench.error or (list(bench.lines)[-3:])}")
    run_dir = WS / bench.run_rel
    print(f"run dir {run_dir}")
    try:
        sup.resume()
        # frames must flow before recording (the bridge counter climbs)
        hil = Path("/tmp/hil_launch.log")
        def frames():
            try:
                return hil.read_text(errors="replace").count("sim_camera_bridge: frame")
            except OSError:
                return 0
        a0, t_end = frames(), time.monotonic() + 120
        while frames() <= a0 + 5 and time.monotonic() < t_end:
            time.sleep(1.0)
        time.sleep(3.0)
        gate.open()
        n = gate.wait_recorder(run_dir, timeout=20.0)
        print(f"recorder subscribed to {n} topics")
        reply = sup.send(f"probe_straight {a.vmax:g} {a.init} 3.15 0")
        print(f"probe_straight -> {reply}")
        t0 = time.monotonic()
        while True:
            time.sleep(2.0)
            s = sup.send("probe_status")
            if "state=done" in s:
                break
            if time.monotonic() - t0 > 180:
                print("approach did not finish in 180 s")
                break
        print(s)
        time.sleep(2.0)
    finally:
        bench.stop(timeout=90, signal_cmd=["pkill", "-TERM", "-f", "slam_hil_bench.sh"])
        gate.close()
        sup.stop()
        subprocess.run(["sudo", "-n", "chown", "-R", f"{os.getuid()}:{os.getgid()}", str(run_dir)], timeout=60)
    with open(run_dir / "flight.txt", "w") as f:
        f.write(f"mode={a.mode}\ninit={a.init}\nvmax={a.vmax}\nmodel={info.get('model')}\nprobe_status={s}\n")
    print(f"recorded: {run_dir}")


if __name__ == "__main__":
    main()
