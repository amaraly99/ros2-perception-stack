# Logs every /yolo/detections message: host wall time (ns), header stamp (ns), count of stop-sign boxes (conf >= 0.20), classes.
import sys, time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from yolo_msgs.msg import DetectionArray
out = open(sys.argv[1], "a", buffering=1)
def cb(m):
    n = sum(1 for d in m.detections if d.class_name == "stop sign" and d.confidence >= 0.20)
    out.write(f"{time.time_ns()} {m.header.stamp.sec * 10**9 + m.header.stamp.nanosec} {n} "
              + ";".join(f"{d.class_name}:{d.confidence:.2f}" for d in m.detections) + "\n")
rclpy.init(); node = rclpy.create_node("r554s1_detlog")
node.create_subscription(DetectionArray, "/yolo/detections", cb,
                         QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10))
rclpy.spin(node)
