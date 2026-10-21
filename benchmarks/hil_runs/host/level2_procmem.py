#!/usr/bin/env python3
# level2_procmem.py (prompt 20C, DEVIATIONS 17.85): per-process memory and eth0 link counters.
# usage: level2_procmem.py <procmem.csv> <link.csv> [interval_s] [iface]
#
# Why: the Pi kernel boots with cgroup_disable=memory (VERIFIED in /proc/cmdline on Pi-3), so
# `docker stats` reports 0 for every container's memory and anything built on cgroup memory reads 0.
# Amar's benchmarks/slam_cpu_sampler.sh at 1beef72 solves this with sudo docker stats + sudo docker
# top + /proc, three sudo docker calls every 2 s. This reads /proc only, resolves the processes by
# the same command-line patterns smoke_run_hold.sh:38-40 and :49-52 already use with pgrep -f, and
# spawns nothing per sample, so it adds no measurable load to the run it is measuring.
#
# procmem.csv: utc, name, pid, pss_mb, rss_mb, hwm_mb      (one row per named process per sample)
#   pss  = /proc/<pid>/smaps_rollup Pss          shared pages split between processes
#   rss  = /proc/<pid>/status VmRSS
#   hwm  = /proc/<pid>/status VmHWM              peak RSS, so peaks between samples still count
# link.csv:    utc, rx_bytes, tx_bytes, rx_MBps, tx_MBps   (<iface> counters, default eth0)
#
# The processes are the four the run already pins and records (smoke_run_hold.sh's _procs.txt), plus
# the two camera bridges. A name that is not running is written with empty fields, never skipped, so
# a missing process is visible in the data instead of being silently absent.
#
# KNOWN LIMITATION on a stereo stack (20C review finding 5; harmless for a mono row, which is all of
# Level 2 session 1). The right eye is a SECOND INSTANCE of the same two executables, differing only
# in its ROS node name (hil_simulation.launch.py: sim_camera_bridge/ovcam_bridge against
# sim_camera_bridge_right/ovcam_bridge_right), so the sim_camera_bridge and ovcam_bridge patterns
# below match both instances while find_pids keeps only the lowest matching pid. On cells B0, C1 and
# X1 one member of each bridge pair is therefore not measured, and the rows named sim_camera_bridge
# and ovcam_bridge are whichever of the pair started first, not a sum over the pair. The smallest
# fix, deliberately left for the commit that precedes session 2 rather than slipped into this one:
# add ("sim_camera_bridge_right", r"__node:=sim_camera_bridge_right") and the matching ovcam entry,
# or record every matching pid per name. Nothing in a validity check or a stored run reads these
# files, so the fix cannot move any scored result.
import os
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
    ("sim_camera_bridge", r"/sim_camera_bridge_node( |$)"),
    ("ovcam_bridge", r"/ovcam_bridge_node( |$)"),
)


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
