#!/usr/bin/env python3
"""scorer_consistency.py <bag_a.mcap> <bag_b.mcap> <run_prefix> <clock_offset.json> <out_dir> [--fx F]  (5R step 3c)

Scores Table 14 from two bags of the same run (the approach bag and the stack bag) over the window both cover, and compares
every value per segment and bin. Window = detection header stamps in [max of the two first stamps, min of the two last
stamps]; each bag keeps its own poses, so a difference points at the pose source or at a message one bag dropped.
run_prefix = <stack dir>/<label>_run1 (for _telemetry.csv and _cam_stamps.csv). Bins and target from config/.
Outputs (exclusive create): out_dir/consistency.json, out_dir/score_a.json, out_dir/score_b.json and their .txt logs.
Exit 0 = every value equal, 1 = a difference (listed), 2 = cannot compare. Run with ~/venvs/mcap/bin/python."""
import argparse, json, math, os, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import hilpaths


def stamps(bag):
    from mcap_ros2.reader import read_ros2_messages
    return sorted(m.ros_msg.header.stamp.sec * 10**9 + m.ros_msg.header.stamp.nanosec
                  for m in read_ros2_messages(bag, topics=["/yolo/detections"]))


def flat(d, pre=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flat(v, f"{pre}{k}."))
        else:
            out[pre + k] = v
    return out


def main():
    ap = argparse.ArgumentParser()
    for a in ("bag_a", "bag_b", "run_prefix", "clock_offset", "out_dir"):
        ap.add_argument(a)
    ap.add_argument("--fx", type=float, default=None)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    for f in ("consistency.json", "score_a.json", "score_b.json"):
        if os.path.exists(os.path.join(a.out_dir, f)):
            sys.exit(f"scorer_consistency: REFUSE: {f} exists in {a.out_dir}")
    geom = json.load(open(hilpaths.GEOM))
    proto = json.load(open(os.path.join(hilpaths.HIL_RUNS, "config", "protocol_fx554.json")))
    sa, sb = stamps(a.bag_a), stamps(a.bag_b)
    res = {"bag_a": a.bag_a, "bag_b": a.bag_b, "det_msgs_a": len(sa), "det_msgs_b": len(sb)}
    if not sa or not sb:
        res["error"] = "a bag has no /yolo/detections"
        json.dump(res, open(os.path.join(a.out_dir, "consistency.json"), "x"), indent=1)
        print(f"cannot compare: detection messages a={len(sa)} b={len(sb)}"); sys.exit(2)
    lo, hi = max(sa[0], sb[0]), min(sa[-1], sb[-1])
    res["window_stamp_ns"] = [lo, hi]
    res["window_s"] = round((hi - lo) / 1e9, 3)
    res["msgs_in_window_a"] = sum(lo <= s <= hi for s in sa)
    res["msgs_in_window_b"] = sum(lo <= s <= hi for s in sb)
    co = str(json.load(open(a.clock_offset))["realtime_minus_monotonic_ns"])
    bins = ",".join(str(v) for v in sorted({e for b in proto["t14_bins_m"] for e in b}))
    out = {}
    for tag, bag in (("a", a.bag_a), ("b", a.bag_b)):
        j = os.path.join(a.out_dir, f"score_{tag}.json")
        cmd = [sys.executable, os.path.join(hilpaths.BENCH, "inloop_accuracy.py"), "--bag", bag,
               "--target", ",".join(str(v) for v in geom["target_xyz"]), "--fx", str(a.fx or geom["fx"]),
               "--clock-offset-ns", co, "--telemetry", a.run_prefix + "_telemetry.csv",
               "--cam-stamps", a.run_prefix + "_cam_stamps.csv", "--bins", bins,
               "--det-stamp-window", f"{lo},{hi}", "--json-out", j, "--force",
               "--cache-dir", os.path.join(a.out_dir, f".cache_{tag}")]
        p = subprocess.run(cmd, capture_output=True, text=True)
        open(os.path.join(a.out_dir, f"score_{tag}.txt"), "x").write(p.stdout + p.stderr)
        if p.returncode != 0 or not os.path.exists(j):
            res["error"] = f"scorer failed on bag {tag} (rc {p.returncode})"
            json.dump(res, open(os.path.join(a.out_dir, "consistency.json"), "x"), indent=1)
            print(res["error"]); sys.exit(2)
        out[tag] = json.load(open(j))
    diffs, n = [], 0
    for seg in sorted(set(out["a"]["segments"]) | set(out["b"]["segments"])):
        A = out["a"]["segments"].get(seg, {}).get("bins", {})
        B = out["b"]["segments"].get(seg, {}).get("bins", {})
        for k in sorted(set(A) | set(B)):
            fa, fb = flat(A.get(k, {})), flat(B.get(k, {}))
            for key in sorted(set(fa) | set(fb)):
                n += 1
                va, vb = fa.get(key), fb.get(key)
                same = va == vb or (isinstance(va, float) and isinstance(vb, float) and math.isclose(va, vb, rel_tol=0, abs_tol=1e-9))
                if not same:
                    diffs.append({"segment": seg, "bin": k, "value": key, "approach_bag": va, "stack_bag": vb})
    res.update({"values_compared": n, "differences": diffs, "equal": not diffs,
                "geometry_rms_capture_px": [out[t]["geometry"].get("frozen_target_reprojection_rms_px") for t in ("a", "b")]})
    json.dump(res, open(os.path.join(a.out_dir, "consistency.json"), "x"), indent=1)
    print(f"window {res['window_s']} s, messages a {res['msgs_in_window_a']} b {res['msgs_in_window_b']}; "
          f"{n} bin values compared, {len(diffs)} differ")
    for d in diffs[:40]:
        print(f"  {d['segment']} {d['bin']} {d['value']}: approach {d['approach_bag']} stack {d['stack_bag']}")
    sys.exit(0 if not diffs else 1)


if __name__ == "__main__":
    main()
