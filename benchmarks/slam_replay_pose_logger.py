#!/usr/bin/env python3
"""Log every message of one pose topic to CSV (stamp_s, x, y, z, qx, qy, qz, qw), whatever its type
(PoseStamped, PoseWithCovarianceStamped, Odometry, TransformStamped). Runs inside a ROS 2 container.
    python3 slam_replay_pose_logger.py /slam/pose /out/poses.csv
Also logs the arrival time and stamp of every /ovcam/image_raw frame to images.csv: some backends write a
wrong pose stamp (OV2SLAM: about 1e-9 x the true value), so the scorer pairs each pose with the last image
that arrived before it (recv_wall), not with its own stamp.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message


def pose_of(m):
    for path in ("pose.pose", "pose", "transform"):
        o = m
        try:
            for part in path.split("."):
                o = getattr(o, part)
        except AttributeError:
            continue
        if hasattr(o, "position"):
            return o.position, o.orientation
        if hasattr(o, "translation"):
            return o.translation, o.rotation
    raise ValueError(f"no pose in {type(m).__name__}")


def main():
    topic, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Node("replay_pose_logger")
    f = open(out, "w", buffering=1)
    f.write("recv_wall,stamp_s,x,y,z,qx,qy,qz,qw\n")
    fi = open(out.replace("poses.csv", "images.csv"), "w", buffering=1)
    fi.write("recv_wall,stamp_s\n")
    from sensor_msgs.msg import Image

    def img_cb(m):
        fi.write(f"{time.time():.6f},{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.6f}\n")

    qos_i = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(Image, "/ovcam/image_raw", img_cb, qos_i)
    t_end = time.monotonic() + 600
    mtype = None
    while mtype is None and time.monotonic() < t_end:
        for name, types in node.get_topic_names_and_types():
            if name == topic:
                mtype = types[0]
        rclpy.spin_once(node, timeout_sec=0.5)
    if mtype is None:
        f.write("# topic never appeared\n")
        return
    cls = get_message(mtype)

    def cb(m):
        try:
            p, q = pose_of(m)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            f.write(f"{time.time():.6f},{st:.6f},{p.x:.6f},{p.y:.6f},{p.z:.6f},{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}\n")
        except Exception as e:  # noqa: BLE001
            f.write(f"# {e}\n")

    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(cls, topic, cb, qos)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):  # noqa: BLE001
        pass
    f.close()


if __name__ == "__main__":
    main()
