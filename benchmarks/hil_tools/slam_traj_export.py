#!/usr/bin/env python3
"""slam_traj_export.py (20C, read-only, stage 1 of 2 of the SLAM accuracy scorer; proposed/07_ate_scorer.md):
export a run's ground-truth and SLAM trajectories from its STACK bag to plain TSV, so stage 2 can score them
under an interpreter that has numpy. Two stages because no interpreter on Pi-3 has both readers (VERIFIED:
$HIL_MCAP_PY has mcap_ros2 and no numpy; the system python has numpy and no mcap_ros2).

Run this one under $HIL_MCAP_PY (~/venvs/mcap/bin/python).

Reads <run>/stack/*_run1_bag/*.mcap:
  /sim/drone_pose    std_msgs/Float64MultiArray, data = [x, y, z, _, yaw]   -> gt.tsv
  /slam/pose         geometry_msgs/PoseStamped                              -> slam.tsv
  /slam/tracking_state (when the bag has it, REC_TRACKING_STATE=1)          -> tracking_state.tsv

Time base: the BAG LOG TIME of every message, in ns, for both topics. That is the only clock the two share.
/slam/pose's own header.stamp is not a time on this path (VERIFIED on a stored run: sec=0, nanosec=1174, a
counter), and /sim/drone_pose is a bare Float64MultiArray with no header at all. Both topics are written by
the same recorder in the same container, so their log times are on one monotonic source and differences
between them are meaningful.

Exact-zero poses are dropped, as Amar's aggregate_matrix.py does (its lines 63 to 65), and counted in the
header so the drop is never silent.

Usage: $HIL_MCAP_PY slam_traj_export.py <run dir> --out <dir>"""
import argparse, glob, os, sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    mc = sorted(glob.glob(os.path.join(a.run, "stack", "*_run1_bag", "*.mcap")))
    if not mc:
        sys.exit(f"slam_traj_export: no stack bag under {a.run}/stack/*_run1_bag/")
    from mcap_ros2.reader import read_ros2_messages
    os.makedirs(a.out, exist_ok=True)
    gt, sl, ts = [], [], []
    n_zero_gt = n_zero_sl = n_short = 0
    for m in read_ros2_messages(mc[0], topics=["/sim/drone_pose", "/slam/pose", "/slam/tracking_state"]):
        t, top, r = m.log_time_ns, m.channel.topic, m.ros_msg
        if top == "/sim/drone_pose":
            d = list(getattr(r, "data", []))
            if len(d) < 3:
                n_short += 1
                continue
            if d[0] == 0.0 and d[1] == 0.0 and d[2] == 0.0:
                n_zero_gt += 1
                continue
            gt.append((t, d[0], d[1], d[2], d[4] if len(d) >= 5 else float("nan")))
        elif top == "/slam/pose":
            p = r.pose.position
            if p.x == 0.0 and p.y == 0.0 and p.z == 0.0:
                n_zero_sl += 1
                continue
            sl.append((t, p.x, p.y, p.z))
        else:
            ts.append((t, getattr(r, "data", getattr(r, "tracking_state", ""))))

    def w(name, header, rows, fmt):
        with open(os.path.join(a.out, name), "w") as f:
            f.write(f"# {os.path.basename(a.run)}  {mc[0]}\n")
            f.write(f"# bag log time in ns; {len(rows)} rows; dropped exact zeros: gt {n_zero_gt}, slam {n_zero_sl};"
                    f" short gt messages {n_short}\n")
            f.write(header + "\n")
            for r in rows:
                f.write(fmt(r) + "\n")
        return len(rows)

    n1 = w("gt.tsv", "t_ns\tx\ty\tz\tyaw_rad", gt,
           lambda r: "%d\t%.9f\t%.9f\t%.9f\t%.9f" % r)
    n2 = w("slam.tsv", "t_ns\tx\ty\tz", sl, lambda r: "%d\t%.9f\t%.9f\t%.9f" % r)
    n3 = w("tracking_state.tsv", "t_ns\tstate", ts, lambda r: "%d\t%s" % r) if ts else 0
    if not ts:
        with open(os.path.join(a.out, "tracking_state.tsv"), "w") as f:
            f.write("# no /slam/tracking_state in this bag (REC_TRACKING_STATE was not set for the run)\n")
            f.write("t_ns\tstate\n")
    print(f"{os.path.basename(a.run)}: gt {n1}, slam {n2}, tracking_state {n3} rows -> {a.out} "
          f"(dropped zeros: gt {n_zero_gt}, slam {n_zero_sl})")


if __name__ == "__main__":
    main()
