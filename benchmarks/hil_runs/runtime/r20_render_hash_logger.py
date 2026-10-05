"""render_hash_logger.py <log> <render_dir>  — one line per /sim/camera/image_raw message: wall_ns sha1 encoding h w.
Each new render (new sha1) is also saved once as <render_dir>/<sha1>.raw. Used by the pre-resume gate (5D) to count
distinct renders during the hold and identical copies of the final one. Runs in its own container."""
import hashlib, os, sys, time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
log, rdir = open(sys.argv[1], "a", buffering=1), sys.argv[2]; os.makedirs(rdir, exist_ok=True)
def cb(m):
    b = bytes(m.data); h = hashlib.sha1(b).hexdigest(); p = f"{rdir}/{h}.raw"
    if not os.path.exists(p): open(p, "wb").write(b)
    log.write(f"{time.time_ns()} {h} {m.encoding} {m.height} {m.width}\n")
rclpy.init(); n = rclpy.create_node("r20_render_hash")
n.create_subscription(Image, "/sim/camera/image_raw", cb,
                      QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5))
rclpy.spin(n)
