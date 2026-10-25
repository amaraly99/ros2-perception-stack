#!/usr/bin/env python3
"""Export the aggregated tables of the paper (Verso SLAM campaign + unified controller runs) as CSV.

    python3 scripts/export_verso_tables.py RUN_METRICS_CSV OUT_DIR
RUN_METRICS_CSV is the per-run output of scripts/verso_rescore.py (one row per healthy run, re-scored with and
without scale). Controller values come from reports/<report>/summary.md on this machine.
Writes OUT_DIR/{runs_rescored.csv, table_slam_accuracy.csv, table_slam_profiling.csv, table_controller_unified.csv,
run_dates.csv}.
"""
import csv
import re
import shutil
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

src, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
ARMS = [("stereo-ov2acc", "Stereo", "OV2SLAM-A", ""), ("stereo-ov2fast", "Stereo", "OV2SLAM-F", ""),
        ("stereo-orb2", "Stereo", "ORB-SLAM2", ""), ("stereo-orb3", "Stereo", "ORB-SLAM3", ""),
        ("stereo-rtab", "Stereo", "RTAB-Map", ""), ("mono-ov2acc", "Mono", "OV2SLAM-A", ""),
        ("mono-ov2fast", "Mono", "OV2SLAM-F", "never initialised"), ("mono-orb2fix", "Mono", "ORB-SLAM2", "mono fix (scripts/orb_mono_fix.patch)"),
        ("mono-orb3fix", "Mono", "ORB-SLAM3", "mono fix (scripts/orb_mono_fix.patch)"), ("rgbd-orb2", "RGB-D", "ORB-SLAM2", ""),
        ("rgbd-orb3", "RGB-D", "ORB-SLAM3", ""), ("rgbd-rtab", "RGB-D", "RTAB-Map", "")]
runs = defaultdict(list)
for r in csv.DictReader(open(src)):
    runs[r["arm_key"]].append(r)
shutil.copy(src, out / "runs_rescored.csv")


def v(k, f, pos=False):
    x = [float(r[f]) for r in runs[k] if r.get(f) not in ("", None)]
    return [y for y in x if y > 0] if pos else x


def m(x):
    return round(st.mean(x), 4) if x else ""


def s(x):
    return round(st.stdev(x), 4) if len(x) > 1 else ""


def med(x):
    return round(st.median(x), 4) if x else ""


acc, prof, dates = [], [], []
for k, mode, alg, note in ARMS:
    a = {"mode": mode, "algorithm": alg, "note": note, "healthy_of_10": len(runs[k])}
    for tag in ("sim3", "se3"):
        x = v(k, f"{tag}_ate_rmse")
        a[f"{tag}_ate_rmse_mean_m"], a[f"{tag}_ate_rmse_sd_m"], a[f"{tag}_ate_rmse_median_m"] = m(x), s(x), med(x)
        p = v(k, f"{tag}_path_ratio_pct")
        a[f"{tag}_path_ratio_mean_pct"], a[f"{tag}_path_ratio_sd_pct"] = m(p), s(p)
    d = v(k, "scale_drift_pct")
    a["scale_drift_mean_pct"], a["scale_drift_sd_pct"] = m(d), s(d)
    acc.append(a)
    mem = v(k, "mem_max_mb", pos=True)
    cpu = v(k, "cpu_mean_pct")
    prof.append({"mode": mode, "algorithm": alg, "note": note,
                 "pose_rate_hz": m(v(k, "pose_rate_hz")), "max_frontend_rate_hz": m(v(k, "capacity_hz")),
                 "latency_p50_ms": m(v(k, "fe_ms_p50")), "latency_p95_ms": m(v(k, "fe_ms_p95")),
                 "latency_p99_ms": m(v(k, "fe_ms_p99")), "cpu_mean_pct_of_one_core": m(cpu), "cpu_sd_pct": s(cpu),
                 "mem_max_mean_mib": m(mem), "mem_max_sd_mib": s(mem), "mem_runs_recorded": len(mem)})
    ds = sorted(re.search(r"_(\d{8})_\d{6}$", r["run_dir"]).group(1) for r in runs[k] if re.search(r"_(\d{8})_\d{6}$", r["run_dir"]))
    dates.append({"mode": mode, "algorithm": alg, "first_flight": ds[0] if ds else "", "last_flight": ds[-1] if ds else "", "runs": len(ds)})

for name, rows in (("table_slam_accuracy.csv", acc), ("table_slam_profiling.csv", prof), ("run_dates.csv", dates)):
    with open(out / name, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

# controller table (unified runs 2026-10-17/18): mean and sd from each report's summary.md
CTRL = [("ViSP IBVS", "reports/ctrl_unified_ibvs_20261018_165347"),
        ("Homography-based VS (h_vs)", "reports/ctrl_unified_h_vs_20261018_172621"),
        ("Custom proportional controller", "reports/ctrl_controller_unified_proportional_20261017_194012")]
KEYS = [("valid runs", "valid"), ("settling time (s)", "settle_s"), ("steady-state error (m)", "ss_err_m"),
        ("IAE (m·s)", "iae_m_s"), ("path efficiency", "path_eff"), ("RMS command speed (m/s)", "rms_cmd_m_s"),
        ("controller CPU (% of a core)", "cpu_pct_of_one_core"), ("controller memory (MB)", "mem_mb")]
crow = []
for name, rep in CTRL:
    row = {"controller": name, "report": rep.split("/")[-1]}
    text = (Path(rep) / "summary.md").read_text() if (Path(rep) / "summary.md").exists() else ""
    for label, key in KEYS:
        mm = re.search(r"\|\s*" + re.escape(label) + r"\s*\|\s*([^|]+)\|", text)
        cell = mm.group(1).strip() if mm else ""
        if key == "valid":
            row["valid_runs"] = cell
            continue
        parts = [p.strip() for p in cell.split("±")]
        row[key + "_mean"] = parts[0] if parts and parts[0] else ""
        row[key + "_sd"] = parts[1] if len(parts) > 1 else ""
    crow.append(row)
with open(out / "table_controller_unified.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(crow[0].keys())); w.writeheader(); w.writerows(crow)
for d in dates:
    print(f"{d['mode']:7s} {d['algorithm']:10s} {d['first_flight']} .. {d['last_flight']}  ({d['runs']} runs)")
print("wrote", sorted(p.name for p in out.iterdir()))
