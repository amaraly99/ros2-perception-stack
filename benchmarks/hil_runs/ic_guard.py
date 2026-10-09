#!/usr/bin/env python3
"""IC guard: does a recorded run START with the frozen target inside the FOV?

Reads the LAST /sim/drone_pose recorded BEFORE resume, which is the initial condition the
sim actually resumed from, and reports the bearing error from the drone's heading to the
frozen target (DEVIATIONS 17.1).

PASS  |bearing error| <= 5 deg   (the tolerance Ahmed fixed, 2026-09-23)
FAIL  otherwise -> the run is an IC-fault run: recorded by name with its yaw,
      never counted toward a row, never pooled (17.13).

Which pose, and why it changed (8I, 2026-10-08, DEVIATIONS 17.67). Until 8I this read
poses[0] of the stack's own bag, and this docstring claimed that pose was "the same
quantity, exactly, just available two minutes later". That claim was false. The stack bag
starts 0.84 to 2.70 s AFTER resume (measured on all six runs of session 1, 2026-10-08), so
whatever the controller commanded in between had already moved the drone before the first
recorded pose. Run 006 of that session read yaw 410.36 deg against a held 357.36 and the
53.00 deg difference was the drone's own commanded search spin, not a simulator fault. The
initial condition now comes from <pre-resume dir>/approach_bag/*.mcap together with
<pre-resume dir>/t_resume_ns.txt, which is exactly the source check_run.py's
start_pose_expected has used since 5J.

The Pi still has no host-side ROS (no rclpy, no /opt/ros) and no container is up between
runs, so /sim/drone_pose cannot be read live without a ros2 CLI call inside a timed run,
which the standing rules forbid. The recorded pre-resume pose is the same quantity and it
is the one the sim resumed from. Recorded as a deviation in DEVIATIONS 17.14 and 17.67.

usage: ic_guard.py [--json] <run_prefix> [--pre-resume-dir DIR]
       # e.g. ic_guard.py .../stack/level2_yolo26n_cpu_B_r1_run1
       # DIR holds t_resume_ns.txt and approach_bag/; it defaults to the parent of the
       # prefix's own directory, which is run_one.sh's layout (RUN_DIR/stack/<prefix>).
exit:  0 = pass, 1 = fail (IC fault), 2 = cannot tell (no bag, no resume time, no pre-resume pose)
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


def check(prefix, pre_dir):
    """Bearing error at the pose the sim resumed from: the last approach-bag /sim/drone_pose
    before t_resume_ns. read_bag runs with no cache on purpose: this bag carries no camera
    images (rec.sh, 5Q), so it reads in about 0.2 s and the guard writes nothing into the
    run folder it is checking."""
    tr_p = os.path.join(pre_dir, "t_resume_ns.txt")
    bags = sorted(glob.glob(os.path.join(pre_dir, "approach_bag", "*.mcap")))
    if not os.path.exists(tr_p):
        return {"status": "unknown", "reason": f"missing {tr_p}"}
    if not bags:
        return {"status": "unknown", "reason": f"no approach bag in {pre_dir}/approach_bag"}
    try:
        tr = int(open(tr_p).read().split()[0])   # 8J: field 1 is the gate's exact resume wall ns
    except (OSError, ValueError) as e:
        return {"status": "unknown", "reason": f"unreadable {tr_p}: {e}"}
    try:
        poses, dets, _ = IA.read_bag(bags[0], None)
    except Exception as e:
        return {"status": "unknown", "reason": f"bag unreadable: {e}"}
    pre = [p for p in poses if p[0] < tr]
    if not pre:
        return {"status": "unknown", "reason": f"no /sim/drone_pose before resume in {bags[0]}"}

    fx, fx_src = run_fx(prefix)
    IA.FX = IA.FY = fx
    p0 = pre[-1]
    _, x, y, z, yaw = p0
    bearing = math.atan2(TARGET[1] - y, TARGET[0] - x)
    err = math.degrees((bearing - yaw + math.pi) % (2 * math.pi) - math.pi)

    uv = IA.project(p0, TARGET)
    in_view0 = bool(IA.in_view(uv))
    run_poses = [p for p in poses if p[0] >= tr]          # 8I: the run is what follows resume
    inview_frac = (sum(1 for p in run_poses if IA.in_view(IA.project(p, TARGET))) / len(run_poses)
                   if run_poses else 0.0)

    return {
        "status": "pass" if abs(err) <= TOL_DEG else "fail",
        "yaw_deg": math.degrees(yaw),
        "bearing_err_deg": err,
        "tol_deg": TOL_DEG,
        "start_xyz": [x, y, z],
        "start_offset_m": math.dist((x, y, z), GEOM["start_xyz_yaw"][:3]),
        "target_in_view_at_start": in_view0,
        "target_held": inview_frac,
        "n_poses": len(run_poses),
        "n_pre_resume_poses": len(pre),
        "start_pose_t_before_resume_s": round((tr - p0[0]) / 1e9, 3),
        "pre_resume_pose_spread_m": round(max(math.dist(p[1:4], p0[1:4]) for p in pre), 4),
        "pose_source": f"last pre-resume /sim/drone_pose of {len(pre)}, {bags[0]}",
        "n_det_msgs": len(dets),
        "fx": fx,
        "fx_source": fx_src,
    }


def main():
    argv = sys.argv[1:]
    as_json = "--json" in argv
    pre_dir = argv[argv.index("--pre-resume-dir") + 1] if "--pre-resume-dir" in argv else None
    args = [a for a in argv if a != "--json"]
    if "--pre-resume-dir" in args:
        i = args.index("--pre-resume-dir"); del args[i:i + 2]
    if not args:
        sys.exit(__doc__)
    prefix = args[0]
    # default: RUN_DIR, the parent of the stack dir the prefix lives in (run_one.sh layout)
    pre_dir = pre_dir or os.path.dirname(os.path.dirname(os.path.abspath(prefix)))
    r = check(prefix, pre_dir)
    r["run"] = os.path.basename(prefix)
    r["pre_resume_dir"] = pre_dir
    if as_json:
        print(json.dumps(r))
    else:
        if r["status"] == "unknown":
            print(f"IC_GUARD UNKNOWN {r['run']}: {r['reason']}")
        else:
            print(f"IC_GUARD {r['status'].upper()} {r['run']}: "
                  f"yaw={r['yaw_deg']:.2f}deg bearing_err={r['bearing_err_deg']:+.2f}deg "
                  f"(tol +/-{TOL_DEG:.0f}) start=({r['start_xyz'][0]:.2f},{r['start_xyz'][1]:.2f},"
                  f"{r['start_xyz'][2]:.2f}) held={r['target_held']:.2f} "
                  f"pose={r['start_pose_t_before_resume_s']:.3f}s before resume")
    return {"pass": 0, "fail": 1, "unknown": 2}[r["status"]]


if __name__ == "__main__":
    sys.exit(main())
