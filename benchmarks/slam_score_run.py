#!/usr/bin/env python3
"""slam_score_run.py — score and check ONE SLAM run.

    python3 benchmarks/slam_score_run.py bags/run_<...> [--end-pad 3]

Scoring is scripts/hil_matrix/aggregate_matrix.py evaluate(run_dir, end_pad), unchanged. Writes into the run dir:
  slam_metrics.csv   every metric evaluate() returned, as "metric,value" rows
  run_check.json     outcome (valid | backend_failed | rig_failed), reason, and the message timing after t=0
                     of /sim/heartbeat, /sim/drone_pose, /slam/pose (same fields as ctrl_run_check.py),
                     plus the Wi-Fi receive rate from wifi.csv.

t=0 is the first /sim/heartbeat after the hold (plot_controller_hil.heartbeat_t0, needs resume_wall_ns in
meta.txt); without it, the first sample in the bag.
Exit code: 0 valid, 2 not valid.
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "scripts" / "hil_matrix"))
from ctrl_run_check import timing, wifi_load  # noqa: E402  (the controller check; one source for the timing fields)
from plot_controller_hil import find_bag, heartbeat_t0  # noqa: E402
import aggregate_matrix  # noqa: E402

TIMED_TOPICS = ("/sim/heartbeat", "/sim/drone_pose", "/slam/pose")


def read_times(bagdir):
    times = {t: [] for t in TIMED_TOPICS}
    with AnyReader([bagdir]) as reader:
        conns = [c for c in reader.connections if c.topic in TIMED_TOPICS]
        for conn, ts, _ in reader.messages(connections=conns):
            times[conn.topic].append(ts * 1e-9)
    return times


def message_timing(run_dir):
    """{"window_s", "t0_rule", "timing"} of the bag, or {"error": text}."""
    try:
        bagdir = find_bag(run_dir)
    except SystemExit as e:
        return {"error": str(e)}
    times = read_times(bagdir)
    all_t = [t for v in times.values() for t in v]
    if not all_t:
        return {"error": "empty bag"}
    resume_t, rule, _ = heartbeat_t0(run_dir, bagdir)
    t0 = resume_t if resume_t is not None else min(all_t)
    t_end = max(all_t)
    return {"window_s": float(t_end - t0), "t0_rule": rule if resume_t is not None else "first_sample",
            "timing": {topic: timing(ts, t0, t_end) for topic, ts in times.items()}}


def clean(v):
    """CSV text of a metric value."""
    if isinstance(v, float) and math.isnan(v):
        return "nan"
    return str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--end-pad", type=int, default=3)
    a = ap.parse_args()
    run_dir = Path(a.run_dir)

    try:
        metrics = aggregate_matrix.evaluate(run_dir, a.end_pad)
    except Exception as e:      # a scorer crash is the harness' problem, not the backend's
        metrics = {"error": f"{type(e).__name__}: {e}"}

    with open(run_dir / "slam_metrics.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value"])
        for k, v in metrics.items():
            w.writerow([k, clean(v)])

    err = metrics.get("error")
    if err is None:
        outcome, reason = "valid", "ok"
    elif str(err).startswith(("SLAM never initialised", "SLAM covers only")):
        outcome, reason = "backend_failed", str(err)         # fewer than 10 non-zero poses: no map
    else:
        outcome, reason = "rig_failed", str(err)             # no GT motion, evo failed, unreadable bag ...
    res = {"valid": outcome == "valid", "outcome": outcome, "reason": reason, "sequence": [],
           "end_pad": a.end_pad, "wifi": wifi_load(run_dir)}
    try:
        res.update(message_timing(run_dir))
    except Exception as e:
        res["timing_error"] = f"{type(e).__name__}: {e}"
    if err is None:
        res["slam"] = {k: metrics.get(k) for k in ("n_poses", "ape_rmse", "umeyama_scale", "gt_path_m", "window_s", "warn")
                       if k in metrics}
    (run_dir / "run_check.json").write_text(json.dumps(res, indent=2, default=lambda o: float(o) if isinstance(o, np.floating) else str(o)))

    print(f"{outcome}: {reason}")
    if err is None:
        print(f"  ATE RMSE {metrics.get('ape_rmse', float('nan')):.3f} m  poses {metrics.get('n_poses')}  "
              f"scale {metrics.get('umeyama_scale', float('nan')):.3f}  path {metrics.get('gt_path_m', float('nan')):.1f} m")
    hb = res.get("timing", {}).get("/sim/heartbeat")
    if hb:
        print(f"  /sim/heartbeat n={hb['count']} rate={hb['rate_hz']:.2f} Hz gap p95={hb['gap_p95_ms']:.1f} ms "
              f"max={hb['gap_max_ms']:.1f} ms >0.2s: {hb['gaps_over_0.2s']}")
    return 0 if outcome == "valid" else 2


if __name__ == "__main__":
    sys.exit(main())
