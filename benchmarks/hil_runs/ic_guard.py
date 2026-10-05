#!/usr/bin/env python3
"""IC guard: does a recorded run START with the frozen target inside the FOV?

Reads the FIRST /sim/drone_pose sample of a run's bag and reports the bearing
error from the drone's heading to the frozen target (DEVIATIONS 17.1).

PASS  |bearing error| <= 5 deg   (the tolerance Ahmed fixed, 2026-09-23)
FAIL  otherwise -> the run is an IC-fault run: recorded by name with its yaw,
      never counted toward a row, never pooled (17.13).

Why this runs after the run and not after `start`: the Pi has no host-side ROS
(no rclpy, no /opt/ros) and no container is up between runs, so /sim/drone_pose
cannot be read before the stack boots without starting one -- and reading it
once the stack IS up would mean a ros2 CLI call inside a timed run, which the
standing rules forbid. The bag's first pose is the same quantity, exactly, just
available two minutes later. Recorded as a deviation in DEVIATIONS 17.14.

usage: ic_guard.py <run_prefix>            # e.g. .../level2_yolo26n_cpu_B_r1_run1
       ic_guard.py --json <run_prefix>
exit:  0 = pass, 1 = fail (IC fault), 2 = cannot tell (no bag / no poses)
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hilpaths  # noqa: E402
sys.path.insert(0, hilpaths.BENCH)
import inloop_accuracy as IA  # noqa: E402

# Target, tolerance and fx come from one file, not from constants here (2026-10-04).
GEOM = json.load(open(hilpaths.GEOM))
TARGET = GEOM["target_xyz"]
TOL_DEG = GEOM["start_bearing_tol_deg"]


def run_fx(prefix):
    """fx for this run: the cam_fx the controller ran with (<run>_params.yaml), else the
    current geometry file. The scorer's in-view test depends on it (half-FOV 14.9 deg at
    1200, 30.0 deg at 554); the bearing check does not."""
    try:
        for line in open(f"{prefix}_params.yaml"):
            k, _, v = line.strip().partition(":")
            if k == "cam_fx":
                return float(v), "params.yaml"
    except (OSError, ValueError):
        pass
    return float(GEOM["fx"]), "sim_geometry.json"


def check(prefix):
    bags = sorted(glob.glob(f"{prefix}_bag/*.mcap"))
    if not bags:
        return {"status": "unknown", "reason": f"no .mcap under {prefix}_bag/"}
    try:
        poses, dets, _ = IA.read_bag(bags[0], os.path.join(os.path.dirname(prefix), ".inloop_cache"))
    except Exception as e:
        return {"status": "unknown", "reason": f"bag unreadable: {e}"}
    if not poses:
        return {"status": "unknown", "reason": "no /sim/drone_pose in bag"}

    fx, fx_src = run_fx(prefix)
    IA.FX = IA.FY = fx
    p0 = poses[0]
    _, x, y, z, yaw = p0
    bearing = math.atan2(TARGET[1] - y, TARGET[0] - x)
    err = math.degrees((bearing - yaw + math.pi) % (2 * math.pi) - math.pi)

    uv = IA.project(p0, TARGET)
    in_view0 = bool(IA.in_view(uv))
    inview_frac = sum(1 for p in poses if IA.in_view(IA.project(p, TARGET))) / len(poses)

    return {
        "status": "pass" if abs(err) <= TOL_DEG else "fail",
        "yaw_deg": math.degrees(yaw),
        "bearing_err_deg": err,
        "tol_deg": TOL_DEG,
        "start_xyz": [x, y, z],
        "start_offset_m": math.dist((x, y, z), GEOM["start_xyz_yaw"][:3]),
        "target_in_view_at_start": in_view0,
        "target_held": inview_frac,
        "n_poses": len(poses),
        "n_det_msgs": len(dets),
        "fx": fx,
        "fx_source": fx_src,
    }


def main():
    args = [a for a in sys.argv[1:] if a != "--json"]
    as_json = "--json" in sys.argv
    if not args:
        sys.exit(__doc__)
    r = check(args[0])
    r["run"] = os.path.basename(args[0])
    if as_json:
        print(json.dumps(r))
    else:
        if r["status"] == "unknown":
            print(f"IC_GUARD UNKNOWN {r['run']}: {r['reason']}")
        else:
            print(f"IC_GUARD {r['status'].upper()} {r['run']}: "
                  f"yaw={r['yaw_deg']:.2f}deg bearing_err={r['bearing_err_deg']:+.2f}deg "
                  f"(tol +/-{TOL_DEG:.0f}) start=({r['start_xyz'][0]:.2f},{r['start_xyz'][1]:.2f},"
                  f"{r['start_xyz'][2]:.2f}) held={r['target_held']:.2f}")
    return {"pass": 0, "fail": 1, "unknown": 2}[r["status"]]


if __name__ == "__main__":
    sys.exit(main())
