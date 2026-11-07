#!/usr/bin/env python3
"""
mission_from_bench_state.py — mission-state metrics from /bench/state in a run's bag.

The shared ServoFsmNode publishes /bench/state on every /cmd_vel tick and on every
diag tick as "<STATE>,<sim_time>,<controller>,<dist_to_target>" (servo_fsm_node.cpp,
publish_bench_state). It is in every recorded bag (run_stack_hil.sh BAG_TOPICS), so
the mission timeline is read back from the bag after the run, independent of when any
live monitor attaches. Same metric schema as the visp-CSV path of the detector sweep
(mission_from_visp_csv.py), plus sim-time metrics taken from the message itself.

The dist field is measured against /sim/target_pose, the hardcoded constant
(35.5, 23.7), not the sign the drone actually approaches. The final distance to the
frozen sign (35.1, 2.92) is therefore computed here from the last /sim/drone_pose.

Needs mcap_ros2 (the host scorer's venv: ~/venvs/mcap/bin/python).

CLI:
  python3 mission_from_bench_state.py --bag <run>/bag/<file>.mcap --output <mission.json>
"""
import argparse
import glob
import json
import math
import os
import sys

STATES = ("SEARCHING", "APPROACHING", "REACQUIRE", "REACHED")
DEBOUNCE_S = 1.0
FROZEN_SIGN = (35.1, 2.92)     # frozen target, /sim/drone_pose frame (DEVIATIONS 17.1)


def read_bag(path):
    from mcap_ros2.reader import read_ros2_messages
    states, poses = [], []
    for m in read_ros2_messages(path, topics=["/bench/state", "/sim/drone_pose"]):
        if m.channel.topic == "/bench/state":
            f = m.ros_msg.data.split(",")
            if len(f) < 4:
                continue
            try:
                states.append((m.log_time_ns, f[0].strip(), float(f[1]), f[2].strip(), float(f[3])))
            except ValueError:
                continue
        else:
            d = list(m.ros_msg.data)
            if len(d) >= 5:
                poses.append((m.log_time_ns, d))
    return states, poses


def segments(rows):
    """rows: [(t_s, state)] -> [[state, t_start, t_end], ...] (maximal equal-state runs)."""
    segs = []
    for t, s in rows:
        if not segs or segs[-1][0] != s:
            if segs:
                segs[-1][2] = t
            segs.append([s, t, t])
        else:
            segs[-1][2] = t
    return segs


def metrics(states, poses, debounce_s=DEBOUNCE_S):
    if not states:
        return None
    t0 = states[0][0]
    rel = [((t - t0) / 1e9, s) for t, s, *_ in states]
    segs = segments(rel)

    def first(state):
        for st, ts, _ in segs:
            if st == state:
                return ts
        return None

    t_reach = first("REACHED")
    t_first_app = first("APPROACHING")
    t_app = None
    if t_reach is not None:
        for st, ts, _ in segs:
            if ts >= t_reach:
                break
            if st == "APPROACHING":
                t_app = ts
    approach = round(t_reach - t_app, 3) if (t_reach is not None and t_app is not None) else None

    n_reacq = sum(1 for st, ts, te in segs if st == "REACQUIRE" and (te - ts) >= debounce_s)
    n_lost, seen_app = 0, False
    for st, ts, te in segs:
        if st == "APPROACHING":
            seen_app = True
        elif st == "SEARCHING" and seen_app and (te - ts) >= debounce_s:
            n_lost += 1
    dwell = {s: 0.0 for s in STATES}
    for st, ts, te in segs:
        if st in dwell:
            dwell[st] += te - ts

    # sim time straight from the message (last_hb_sim_); first-edge values.
    def first_sim(state):
        for _, s, sim, *_ in states:
            if s == state:
                return sim
        return None
    sim_reach = first_sim("REACHED")
    sim_app = None
    if sim_reach is not None:
        for _, s, sim, *_ in states:
            if sim >= sim_reach:
                break
            if s == "APPROACHING" and sim_app is None:
                sim_app = sim
            elif s != "APPROACHING":
                sim_app = None
    wall = (states[-1][0] - t0) / 1e9
    sim_span = states[-1][2] - states[0][2]
    sim_rate = round(sim_span / wall, 3) if wall > 0 and sim_span > 0 else None

    final = {}
    if poses:
        _, d = poses[-1]
        final = {
            "final_pose_xyz_yaw": [round(d[0], 3), round(d[1], 3), round(d[2], 3), round(d[4], 4)],
            "final_dist_frozen_sign_m": round(math.dist((d[0], d[1]), FROZEN_SIGN), 3),
        }
        if t_reach is not None:
            t_reach_ns = t0 + int(t_reach * 1e9)
            at = min(poses, key=lambda p: abs(p[0] - t_reach_ns))[1]
            final["reached_dist_frozen_sign_m"] = round(math.dist((at[0], at[1]), FROZEN_SIGN), 3)

    return {
        "reached": t_reach is not None,
        "time_to_reached_s": round(t_reach, 3) if t_reach is not None else None,
        "approach_duration_s": approach,
        "time_to_first_approach_s": round(t_first_app, 3) if t_first_app is not None else None,
        "time_to_reached_sim_s": round(sim_reach, 3) if sim_reach is not None else None,
        "approach_duration_sim_s": (round(sim_reach - sim_app, 3)
                                    if (sim_reach is not None and sim_app is not None) else None),
        "sim_rate": sim_rate,
        "n_reacquire": n_reacq,
        "n_lost": n_lost,
        "time_in_state_s": {s: round(v, 3) for s, v in dwell.items()},
        "initial_state": segs[0][0],
        "final_state": segs[-1][0],
        "controller": states[0][3],
        "n_transitions": len(segs) - 1,
        "transitions": [[round(ts, 3), st] for st, ts, _ in segs[1:]],
        **final,
        "source": "bench_state",
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--bag", required=True, help=".mcap file, or the bag directory")
    ap.add_argument("--output", required=True)
    a = ap.parse_args()
    bag = a.bag
    if os.path.isdir(bag):
        found = sorted(glob.glob(os.path.join(bag, "*.mcap")))
        if not found:
            sys.exit(f"no .mcap in {bag}")
        bag = found[0]
    states, poses = read_bag(bag)
    m = metrics(states, poses)
    if m is None:
        sys.exit(f"no /bench/state messages in {bag}")
    m["bag"] = os.path.abspath(bag)
    with open(a.output, "w") as f:
        json.dump(m, f, indent=2)
    print(json.dumps({k: m[k] for k in ("reached", "time_to_reached_sim_s", "final_state",
                                        "final_dist_frozen_sign_m") if k in m}))


if __name__ == "__main__":
    main()
