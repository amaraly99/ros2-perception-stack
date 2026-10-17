#!/usr/bin/env python3
"""yaw_hold_check.py (16F, class A, read-only): does the controller command yaw while APPROACHING? Per run folder it reads the
approach bag (/cmd_vel, /sim/drone_pose, /bench/state) and prints, for the messages published while the latest /bench/state
is APPROACHING and while it is SEARCHING: how many /cmd_vel messages carry a non-zero angular.z and a non-zero linear.y and
the largest |angular.z|; and for the poses in APPROACHING the yaw range (deg), the number of distinct yaw values, and the
largest lateral share over those poses, max |sin(yaw - 360 deg)| x 100 pct (the body-forward component of a world-y lateral
command; the simulator integrates cmd_vel along the world axes, SIM_PARAMS.md). Arguments that are not run folders are
skipped with a note (a shell glob may match stray files). Usage: ~/venvs/mcap/bin/python yaw_hold_check.py <run folder> ..."""
import math, os, sys


def main():
    from mcap_ros2.reader import read_ros2_messages
    for rd in sys.argv[1:]:
        bag = os.path.join(rd, "approach_bag", "r20_bag_0.mcap")
        if not os.path.isdir(rd) or not os.path.exists(bag):
            print(f"skipped (not a run folder with an approach bag): {os.path.basename(rd)}"); continue
        poses, cmds, states = [], [], []
        for m in read_ros2_messages(bag):
            t = m.log_time_ns; top = m.channel.topic
            if top == "/sim/drone_pose":
                poses.append((t, m.ros_msg.data[4]))
            elif top == "/cmd_vel":
                cmds.append((t, m.ros_msg.linear.x, m.ros_msg.linear.y, m.ros_msg.angular.z))
            elif top == "/bench/state":
                states.append((t, m.ros_msg.data.split(",")[0]))
        states.sort(); poses.sort(); cmds.sort()
        def st(t):
            s = "SEARCHING"
            for tt, v in states:
                if tt <= t:
                    s = v
                else:
                    break
            return s
        ca = [c for c in cmds if st(c[0]) == "APPROACHING"]; cs = [c for c in cmds if st(c[0]) == "SEARCHING"]
        ya = [math.degrees(p[1]) for p in poses if st(p[0]) == "APPROACHING"]
        share = max((abs(math.sin(math.radians(y))) for y in ya), default=0.0) * 100   # sin(y) = sin(y - 360 deg)
        print(f"{os.path.basename(rd)}: cmd_vel in APPROACHING {len(ca)}: |wz|>1e-6 in {sum(1 for c in ca if abs(c[3]) > 1e-6)}, |vy|>1e-6 in {sum(1 for c in ca if abs(c[2]) > 1e-6)}, "
              f"max|wz| {max((abs(c[3]) for c in ca), default=0):.3f}; in SEARCHING {len(cs)}: |wz|>1e-6 in {sum(1 for c in cs if abs(c[3]) > 1e-6)}, max|wz| {max((abs(c[3]) for c in cs), default=0):.3f}; "
              f"poses in APPROACHING {len(ya)}: yaw {min(ya, default=0):.3f} to {max(ya, default=0):.3f} deg, distinct {len(set(round(y, 4) for y in ya))}, max lateral share over poses {share:.1f} pct")
    return 0


if __name__ == "__main__":
    sys.exit(main())
