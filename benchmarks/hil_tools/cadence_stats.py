#!/usr/bin/env python3
"""cadence_stats.py (17D B3, class A: outside the guarded run code). Per configuration of the pooled batches (and, with
--ablation, of the session 5a rows): the inter-detection gaps the controller saw (consecutive /yolo/detections messages that
carry a target box, class "stop sign" at conf >= 0.20, by recorder time after resume: p50, p95, max over the row's runs pooled;
the number of gaps above 0.4 s and above 0.6 s; the runs with a gap above 0.6 s while n_lost = 0), the detector latency
(telemetry e2e_capture_to_commit_ms: pooled p50 and p95 over the row's frames) and the exact Clopper-Pearson 95 pct interval of
the row's reach count. Inputs: the batch list, the read_bag cache of the approach bags (one or more --cache directories, the
scorer's own JSON, keyed by bag name, size and mtime) and the run folders (t_resume_ns.txt, stack mission.json and
telemetry.csv); nothing under /mnt/usb is written. Usage:
  python3 cadence_stats.py --batches <list.tsv> --cache <dir> [--cache <dir> ...] --out <dir> [--ablation] [--exclude <sess>:<run prefix> ...]
<list.tsv> lines "<session> <batch folder>" under /mnt/usb/hil_runs; the pooled tables take the five pooled batches, --ablation the
session 5a batch. --exclude sets a run aside (the rig fault 032: s5a:032). The Clopper-Pearson bounds use scipy.stats.beta."""
import argparse, csv, glob, json, os, re, statistics, sys
from scipy.stats import beta

CONF = 0.20; TARGET = "stop sign"
ORDER = ["yolo26n", "yolo26s", "yolo26m", "yolov8n", "yolov8s", "yolov8m", "yolov11n", "yolov11s", "yolov11m", "centernet_resnet_v1_18"]


def clopper_pearson(k, n, alpha=0.05):
    lo = 0.0 if k == 0 else beta.ppf(alpha / 2, k, n - k + 1)
    hi = 1.0 if k == n else beta.ppf(1 - alpha / 2, k + 1, n - k)
    return lo, hi


def pct(v, p):
    v = sorted(v)
    if not v:
        return None
    i = (len(v) - 1) * p; lo = int(i); hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (i - lo)


def cache_blob(bag, dirs):
    """The scorer's read_bag cache entry of a bag, searched under each --cache directory (os.walk: the re-score trees keep the
    cache in a hidden .inloop_cache folder per run, which glob would skip)."""
    st = os.stat(bag); key = f"{os.path.basename(bag)}.{st.st_size}.{int(st.st_mtime)}.json"
    for d in dirs:
        for root, _dirs, files in os.walk(d):
            if key in files:
                return json.load(open(os.path.join(root, key)))
    return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--batches", required=True); ap.add_argument("--cache", action="append", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--usb", default="/mnt/usb/hil_runs"); ap.add_argument("--ablation", action="store_true")
    ap.add_argument("--exclude", nargs="*", default=[], help="<sess>:<run dir prefix> runs set aside (for example s5a:032, the rig fault of DEVIATIONS 17.79)")
    a = ap.parse_args()
    excl = [tuple(e.split(":", 1)) for e in a.exclude]
    if os.path.exists(a.out):
        sys.exit(f"REFUSE: {a.out} exists")
    os.makedirs(a.out)
    rows = []
    for ln in open(a.batches):
        if not ln.strip() or ln.startswith("#"):
            continue
        sess, b = ln.split()[:2]
        for rd in sorted(glob.glob(os.path.join(a.usb, b, "0[0-9][0-9]_*"))):
            name = os.path.basename(rd); m = re.match(r"^\d{3}_(.+)_(npu|cpu)_([AB])_r(.+)$", name)
            if not m or not os.path.isdir(os.path.join(rd, "approach_bag")):
                continue
            model, bk, key = m.group(1), m.group(2), m.group(4); label = "level2_" + name[4:]
            role = "sentinel" if key.startswith("sen") else ("ablation" if key.startswith("lw") else "row")
            if (role == "ablation") != a.ablation:
                continue
            if any(sess == es and name.startswith(ep) for es, ep in excl):
                print("excluded", sess, name, file=sys.stderr); continue
            W = None
            if a.ablation:
                W = {"lw25": 2.5, "lw10": 1.0}.get(key.split("_")[0][:4], None)
                if key.startswith("lw25c"):
                    W = 2.5
            tr = int(open(os.path.join(rd, "t_resume_ns.txt")).read().split()[0])
            bags = glob.glob(os.path.join(rd, "approach_bag", "*.mcap")); tels = glob.glob(os.path.join(rd, "stack", f"{label}_run1_telemetry.csv"))
            if not bags or not tels:
                print("no bag or telemetry for", rd, file=sys.stderr); continue
            bag = bags[0]
            blob = cache_blob(bag, a.cache)
            if blob is None:
                print("no cache for", rd, file=sys.stderr); continue
            dets = blob["dets"]
            tb = sorted(d["log_ns"] for d in dets if d["log_ns"] > tr and any(x[0] == TARGET and x[1] >= CONF for x in d["dets"]))
            gaps = [(b2 - a2) / 1e9 for a2, b2 in zip(tb, tb[1:])]
            ms = json.load(open(os.path.join(rd, "stack", f"{label}_run1_mission.json")))
            lat = []
            for r in csv.DictReader(open(tels[0])):
                try:
                    lat.append(float(r["e2e_capture_to_commit_ms"]))
                except (KeyError, ValueError):
                    pass
            rows.append({"sess": sess, "run": name, "model": model, "backend": bk, "role": role, "W": W, "reached": bool(ms.get("reached")), "n_lost": ms.get("n_lost"),
                         "final_state": ms.get("final_state"), "n_target_msgs": len(tb), "gaps": gaps, "lat": lat,
                         "gap_p50": pct(gaps, 0.5), "gap_p95": pct(gaps, 0.95), "gap_max": max(gaps) if gaps else None,
                         "n_gap_gt_0p4": sum(1 for g in gaps if g > 0.4), "n_gap_gt_0p6": sum(1 for g in gaps if g > 0.6),
                         "lat_p50": pct(lat, 0.5), "lat_p95": pct(lat, 0.95)})
    # per-run table
    with open(os.path.join(a.out, "per_run_cadence.tsv"), "x") as f:
        w = csv.writer(f, delimiter="\t")
        w.writerow(["sess", "run", "model", "backend", "role", "W_s", "reached", "n_lost", "final_state", "n_target_msgs", "gap_p50_s", "gap_p95_s", "gap_max_s", "n_gap_gt_0.4", "n_gap_gt_0.6", "lat_p50_ms", "lat_p95_ms"])
        for r in rows:
            w.writerow([r["sess"], r["run"], r["model"], r["backend"], r["role"], r["W"], r["reached"], r["n_lost"], r["final_state"], r["n_target_msgs"]] +
                       [None if v is None else round(v, 3) for v in (r["gap_p50"], r["gap_p95"], r["gap_max"])] + [r["n_gap_gt_0p4"], r["n_gap_gt_0p6"]] +
                       [None if v is None else round(v, 2) for v in (r["lat_p50"], r["lat_p95"])])
    # per configuration
    groups = {}
    for r in rows:
        groups.setdefault((r["model"], r["backend"], r["role"], r["W"]), []).append(r)
    out = []
    for (model, bk, role, W), rs in sorted(groups.items(), key=lambda kv: (kv[0][2] != "row", kv[0][1] != "npu", ORDER.index(kv[0][0]) if kv[0][0] in ORDER else 99, kv[0][3] or 0)):
        gaps = [g for r in rs for g in r["gaps"]]; lat = [l for r in rs for l in r["lat"]]
        k = sum(1 for r in rs if r["reached"]); n = len(rs); lo, hi = clopper_pearson(k, n)
        viol = [r["run"] for r in rs if r["n_gap_gt_0p6"] > 0 and (r["n_lost"] or 0) == 0 and r["reached"]]
        out.append({"model": model, "backend": bk, "role": role, "W_s": W, "runs": n, "reached": k, "cp95_lo": round(lo, 3), "cp95_hi": round(hi, 3),
                    "msgs": sum(r["n_target_msgs"] for r in rs), "gap_p50_s": round(pct(gaps, 0.5), 3) if gaps else None, "gap_p95_s": round(pct(gaps, 0.95), 3) if gaps else None,
                    "gap_max_s": round(max(gaps), 3) if gaps else None, "n_gap_gt_0.4": sum(1 for g in gaps if g > 0.4), "n_gap_gt_0.6": sum(1 for g in gaps if g > 0.6),
                    "reached_runs_with_gap_gt_0.6_and_n_lost_0": viol, "lat_p50_ms": round(pct(lat, 0.5), 1) if lat else None, "lat_p95_ms": round(pct(lat, 0.95), 1) if lat else None,
                    "lat_p95_per_run_min_max_ms": [round(min(r["lat_p95"] for r in rs if r["lat_p95"] is not None), 1), round(max(r["lat_p95"] for r in rs if r["lat_p95"] is not None), 1)] if any(r["lat_p95"] is not None for r in rs) else None})
    with open(os.path.join(a.out, "per_config_cadence.tsv"), "x") as f:
        w = csv.writer(f, delimiter="\t"); w.writerow(list(out[0].keys()))
        for o in out:
            w.writerow([json.dumps(v) if isinstance(v, (list, dict)) else v for v in o.values()])
    json.dump(out, open(os.path.join(a.out, "per_config_cadence.json"), "x"), indent=1)
    # pooled Clopper-Pearson lines
    pooled = {}
    for o in out:
        if o["role"] != "row":
            continue
        fam = "NPU rows" if o["backend"] == "npu" else ("CPU n-models" if o["model"].endswith("n") and o["backend"] == "cpu" else "slow CPU rows")
        pooled.setdefault(fam, [0, 0]); pooled[fam][0] += o["reached"]; pooled[fam][1] += o["runs"]
    lines = ["# Clopper-Pearson exact 95 pct intervals (scipy.stats.beta)", "| set | reached | n | lower | upper |", "|---|---|---|---|---|"]
    for fam, (k, n) in pooled.items():
        lo, hi = clopper_pearson(k, n); lines.append(f"| {fam} | {k} | {n} | {lo:.3f} | {hi:.3f} |")
    for k, n in ((6, 6), (18, 18), (60, 60), (65, 65), (0, 42), (3, 3), (2, 2), (2, 3), (0, 3), (0, 9), (0, 18)):
        lo, hi = clopper_pearson(k, n); lines.append(f"| {k} of {n} | {k} | {n} | {lo:.3f} | {hi:.3f} |")
    open(os.path.join(a.out, "clopper_pearson.md"), "x").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\n{'config':34s} {'runs':>4s} {'reach':>5s} {'CP lo':>6s} {'msgs':>5s} {'gap p50':>7s} {'p95':>6s} {'max':>6s} {'>0.4':>5s} {'>0.6':>5s} {'lat p50':>7s} {'p95':>7s}  reached runs with a gap > 0.6 s and n_lost 0")
    for o in out:
        print(f"{o['model'] + ' ' + o['backend'] + ' ' + o['role'] + (' W' + str(o['W_s']) if o['W_s'] else ''):34s} {o['runs']:4d} {o['reached']:5d} {o['cp95_lo']:6.3f} {o['msgs']:5d} {str(o['gap_p50_s']):>7s} {str(o['gap_p95_s']):>6s} {str(o['gap_max_s']):>6s} {o['n_gap_gt_0.4']:5d} {o['n_gap_gt_0.6']:5d} {str(o['lat_p50_ms']):>7s} {str(o['lat_p95_ms']):>7s}  {o['reached_runs_with_gap_gt_0.6_and_n_lost_0']}")


if __name__ == "__main__":
    main()
