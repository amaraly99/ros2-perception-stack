"""render_hash_logger.py <log> <render_dir> [hold_done_file]: one line per /sim/camera/image_raw message: wall_ns sha1 encoding h w.
Each new render (new sha1) is also saved once as <render_dir>/<sha1>.raw. Used by the pre-resume gate (5D) to count
distinct renders during the hold and identical copies of the final one. Runs in its own container.
25C (R5, DEVIATIONS 17.86): with hold_done_file, no new render is saved once that file exists (hold_gate.sh writes it
after it has read the final held render, so every render a check reads is saved and no render after the hold is), and
the last frame received is saved once at exit as <render_dir>/terminal_<sha1>.raw with <render_dir>/terminal.txt
(wall_ns sha1 encoding h w): the one flight frame kept, for the terminal-state figure (Ahmed, 2026-10-25). Without
hold_done_file every distinct render is saved and nothing else, as before. The log line is the same either way."""
import hashlib, os, sys, time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
log, rdir = open(sys.argv[1], "a", buffering=1), sys.argv[2]; os.makedirs(rdir, exist_ok=True)
done = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] else None
last = None
def cb(m):
    global last
    b = bytes(m.data); h = hashlib.sha1(b).hexdigest(); p = f"{rdir}/{h}.raw"
    if not os.path.exists(p) and not (done and os.path.exists(done)): open(p, "wb").write(b)
    t = time.time_ns()
    if done: last = (t, h, m.encoding, m.height, m.width, b)
    log.write(f"{t} {h} {m.encoding} {m.height} {m.width}\n")
rclpy.init(); n = rclpy.create_node("r20_render_hash")
n.create_subscription(Image, "/sim/camera/image_raw", cb,
                      QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5))
try:
    rclpy.spin(n)
finally:
    if last is not None:
        open(f"{rdir}/terminal_{last[1]}.raw", "wb").write(last[5])
        open(f"{rdir}/terminal.txt", "w").write(" ".join(str(v) for v in last[:5]) + "\n")
