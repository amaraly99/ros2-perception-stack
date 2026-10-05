#!/usr/bin/env python3
# usage: level2_coresampler.py <cores.csv> <threads.csv>
# per-core utilization from /proc/stat every 1 s; top 8 threads by CPU over each 15 s window, with the core each last ran on
import os, sys, time, signal
signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))
HZ = os.sysconf("SC_CLK_TCK")

def cpu_times():
    out = {}
    for line in open("/proc/stat"):
        if line.startswith("cpu") and line[3].isdigit():
            f = line.split(); v = list(map(int, f[1:9]))
            out[f[0]] = (sum(v), v[3] + v[4])          # total, idle+iowait
    return out

def threads():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit(): continue
        try:
            for tid in os.listdir(f"/proc/{pid}/task"):
                try: s = open(f"/proc/{pid}/task/{tid}/stat").read()
                except OSError: continue
                comm = s[s.index("(") + 1:s.rindex(")")]; f = s[s.rindex(")") + 2:].split()
                out[tid] = (pid, comm, int(f[11]) + int(f[12]), int(f[36]))   # utime+stime, processor
        except OSError: pass
    return out

utc = lambda: time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
cf = open(sys.argv[1], "w", buffering=1); tf = open(sys.argv[2], "w", buffering=1)
prev = cpu_times(); names = sorted(prev, key=lambda c: int(c[3:]))
cf.write("utc," + ",".join(f"{c}_pct" for c in names) + "\n")
tf.write("utc,rank,tid,pid,comm,core,cpu_pct_15s\n")
tprev, tt = threads(), time.monotonic()
while True:
    time.sleep(1)
    cur = cpu_times()
    cf.write(utc() + "," + ",".join(
        f"{100 * (1 - (cur[c][1] - prev[c][1]) / max(1, cur[c][0] - prev[c][0])):.1f}" for c in names) + "\n")
    prev = cur
    if time.monotonic() - tt >= 15:
        tnow, now = threads(), time.monotonic(); dt = now - tt
        d = sorted(((v[2] - tprev[k][2], k, v) for k, v in tnow.items() if k in tprev), reverse=True)[:8]
        u = utc()
        for i, (ticks, tid, (pid, comm, _, core)) in enumerate(d, 1):
            tf.write(f"{u},{i},{tid},{pid},{comm.replace(',', ';')},{core},{100 * ticks / HZ / dt:.1f}\n")
        tprev, tt = tnow, now
