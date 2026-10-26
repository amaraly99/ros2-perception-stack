#!/usr/bin/env python3
# level2_procmem.py (prompt 20C, DEVIATIONS 17.85): per-process memory and eth0 link counters.
# usage: level2_procmem.py <procmem.csv> <link.csv> [interval_s] [iface]
#
# Why: the Pi kernel boots with cgroup_disable=memory (VERIFIED in /proc/cmdline on Pi-3), so
# `docker stats` reports 0 for every container's memory and anything built on cgroup memory reads 0.
# Amar's benchmarks/slam_cpu_sampler.sh at 1beef72 solves this with sudo docker stats + sudo docker
# top + /proc, three sudo docker calls every 2 s. This reads /proc only, resolves the processes by
# the same command-line patterns smoke_run_hold.sh (sample_procs) already uses with pgrep -f, and
# spawns nothing per sample, so it adds no measurable load to the run it is measuring.
#
# procmem.csv: utc, name, pid, pss_mb, rss_mb, hwm_mb      (one row per named process per sample)
#   pss  = /proc/<pid>/smaps_rollup Pss          shared pages split between processes
#   rss  = /proc/<pid>/status VmRSS
#   hwm  = /proc/<pid>/status VmHWM              peak RSS, so peaks between samples still count
#   slam_container (25A): pid = the SLAM process, pss/rss/hwm = sums over EVERY process in that
#   process's cgroup (its container), the definition of Amar's benchmarks/slam_cpu_sampler.sh:16-19
#   (sum of PSS over `docker top`); the runtime table takes the per-run max of this sum.
# link.csv:    utc, rx_bytes, tx_bytes, rx_MBps, tx_MBps   (<iface> counters, default eth0)
#
# The processes are the four the run already pins and records (smoke_run_hold.sh's _procs.txt), an ORB-SLAM3
# node, the two camera bridges and (25A, 25C) their right-eye instances, plus one slam_container row per
# sample (the sums above). A name that is not running is written with empty fields, never skipped, so
# a missing process is visible in the data instead of being silently absent.
#
# Stereo stack (20C review finding 5, fixed in 25C): the right eye is a SECOND INSTANCE of the same two
# executables, differing only in its ROS node name (hil_simulation.launch.py: sim_camera_bridge/ovcam_bridge
# against sim_camera_bridge_right/ovcam_bridge_right). The left patterns below exclude the right node name
# and the right instances have their own rows, so each member of a bridge pair is measured on its own.
# 25C (DEVIATIONS 17.86): check_run.py's slam_coload_alive now reads this file (the ov2slam_node pid per row
# and, for a SLAM-off cell, the absence of every SLAM row's pid); the memory columns stay covariates.
import os
# 25A: Pss in smaps_rollup is readable only by the owner or root, and every stack process except the
# detector producer runs as root in a container, so the sampler is started with sudo -n by its caller.
import re
import signal
import sys
import time

signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))

PATTERNS = (
    ("detector_producer", r"src/yolo_producer/.*producer\.py"),
    ("yolo_bridge_node", r"/yolo_bridge_node( |$)"),
    ("controller", r"/(visp_servo_node|hil_servo_node|h_vs_servo_node)( |$)"),
    ("ov2slam_node", r"/ov2slam_node( |$)"),
    ("orbslam3_node", r"/(rgbd_node_cpp|stereo_node_cpp|mono_node_cpp)( |$)"),
    ("sim_camera_bridge", r"/sim_camera_bridge_node( |$)(?!.*__node:=sim_camera_bridge_right)"),
    ("ovcam_bridge", r"/ovcam_bridge_node( |$)(?!.*__node:=ovcam_bridge_right)"),
    ("sim_camera_bridge_right", r"__node:=sim_camera_bridge_right( |$)"),
    ("ovcam_bridge_right", r"__node:=ovcam_bridge_right( |$)"),
)
SLAM_NAMES = ("ov2slam_node", "orbslam3_node")


def cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode("utf-8", "replace")
    except OSError:
        return ""


def find_pids():
    """name -> first matching pid, or None. One scan of /proc per sample, no subprocess."""
    out = {name: None for name, _ in PATTERNS}
    rx = [(name, re.compile(pat)) for name, pat in PATTERNS]
    for pid in sorted((p for p in os.listdir("/proc") if p.isdigit()), key=int):
        line = cmdline(pid)
        if not line:
            continue
        for name, r in rx:
            if out[name] is None and r.search(line):
                out[name] = pid
    return out


def kb(path, keys):
    """Sum of the named kB fields of a /proc file, or None when unreadable."""
    try:
        got = {}
        with open(path) as f:
            for line in f:
                k = line.split(":", 1)[0]
                if k in keys:
                    got[k] = int(line.split()[1])
        return got or None
    except (OSError, ValueError, IndexError):
        return None


def mem_of(pid):
    st = kb(f"/proc/{pid}/status", {"VmRSS", "VmHWM"}) or {}
    ro = kb(f"/proc/{pid}/smaps_rollup", {"Pss"}) or {}
    f = lambda v: round(v / 1024.0, 2) if v is not None else ""
    return f(ro.get("Pss")), f(st.get("VmRSS")), f(st.get("VmHWM"))


def cgroup_of(pid):
    try:
        with open(f"/proc/{pid}/cgroup") as f:
            return f.read().strip()
    except OSError:
        return None


def group_mem(pid):
    """Sums of Pss, VmRSS and VmHWM (MB) over every process sharing pid's cgroup, or blanks."""
    cg = cgroup_of(pid)
    if not cg or "docker" not in cg:                     # a container's own cgroup only, never a host session
        return "", "", ""
    pss = rss = hwm = 0
    ok = False
    for p in (q for q in os.listdir("/proc") if q.isdigit()):
        if cgroup_of(p) != cg:
            continue
        st = kb(f"/proc/{p}/status", {"VmRSS", "VmHWM"}) or {}
        ro = kb(f"/proc/{p}/smaps_rollup", {"Pss"}) or {}
        if "Pss" in ro:
            ok = True
        pss += ro.get("Pss", 0); rss += st.get("VmRSS", 0); hwm += st.get("VmHWM", 0)
    f = lambda v: round(v / 1024.0, 2)
    return (f(pss), f(rss), f(hwm)) if ok else ("", f(rss), f(hwm))


def counters(iface):
    try:
        base = f"/sys/class/net/{iface}/statistics"
        return int(open(f"{base}/rx_bytes").read()), int(open(f"{base}/tx_bytes").read())
    except (OSError, ValueError):
        return None, None


def main():
    pm_path, link_path = sys.argv[1], sys.argv[2]
    interval = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    iface = sys.argv[4] if len(sys.argv) > 4 else "eth0"
    utc = lambda: time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    pm = open(pm_path, "w", buffering=1)
    lk = open(link_path, "w", buffering=1)
    if os.environ.get("SUDO_UID"):                       # 25A: started with sudo, files stay the caller's
        for path in (pm_path, link_path):
            os.chown(path, int(os.environ["SUDO_UID"]), int(os.environ.get("SUDO_GID", os.environ["SUDO_UID"])))
    pm.write("utc,name,pid,pss_mb,rss_mb,hwm_mb\n")
    lk.write("utc,rx_bytes,tx_bytes,rx_MBps,tx_MBps\n")
    prx, ptx = counters(iface)
    pt = time.monotonic()
    while True:
        time.sleep(interval)
        u = utc()
        pids = find_pids()
        for name, _ in PATTERNS:
            pid = pids[name]
            if pid is None:
                pm.write(f"{u},{name},,,,\n")
            else:
                pss, rss, hwm = mem_of(pid)
                pm.write(f"{u},{name},{pid},{pss},{rss},{hwm}\n")
        slam = next((pids[n] for n in SLAM_NAMES if pids[n] is not None), None)
        if slam is None:
            pm.write(f"{u},slam_container,,,,\n")
        else:
            pss, rss, hwm = group_mem(slam)
            pm.write(f"{u},slam_container,{slam},{pss},{rss},{hwm}\n")
        rx, tx = counters(iface)
        now = time.monotonic()
        dt = now - pt
        if None not in (rx, tx, prx, ptx) and dt > 0:
            lk.write(f"{u},{rx},{tx},{(rx - prx) / dt / 1e6:.3f},{(tx - ptx) / dt / 1e6:.3f}\n")
        else:
            lk.write(f"{u},{rx if rx is not None else ''},{tx if tx is not None else ''},,\n")
        prx, ptx, pt = rx, tx, now


if __name__ == "__main__":
    main()
