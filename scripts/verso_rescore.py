#!/usr/bin/env python3
"""Offline re-score of the Verso healthy runs, with scale (Sim(3)) and without scale (SE(3)).

Run from ~/ROS2-sim-unified-start with /usr/bin/python3:
    python3 verso_rescore.py OUT_DIR < arms.txt        (arms.txt lines: "<arm_key> <report folder>")
Writes OUT_DIR/run_metrics.csv (one row per run) and OUT_DIR/run_series.csv (100 resampled points per run).
Same scoring window as aggregate_matrix.evaluate: mission start .. arrival + 3 s.
"""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "scripts/hil_matrix")
import aggregate_matrix as am  # noqa: E402

TOL = am.GT_SLAM_MATCH_TOL_S
N_PTS = 100


def umeyama(src, dst, with_scale):
    """Least-squares (s, R, t) with dst ~ s R src + t (Umeyama 1991)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = (np.trace(np.diag(D) @ S) / (xs ** 2).sum(1).mean()) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def associate(est, gt):
    """Nearest GT pose in time for each estimate pose, kept within TOL."""
    idx = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    left, right = gt[idx - 1, 0], gt[idx, 0]
    idx = np.where(np.abs(est[:, 0] - left) <= np.abs(right - est[:, 0]), idx - 1, idx)
    keep = np.abs(gt[idx, 0] - est[:, 0]) <= TOL
    return est[keep], gt[idx[keep]]


def path_len(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def cpu_stats(run):
    p = Path(run) / "slam_process_cpu.csv"
    if not p.exists():
        return ("", "", "")
    rows = list(csv.DictReader(open(p)))
    if not rows:
        return ("", "", "")
    col = "rss_mb" if "rss_mb" in rows[0] else ("mem_mb" if "mem_mb" in rows[0] else None)
    cpu = np.array([float(r["cpu_percent"]) for r in rows])
    if col:
        mem = np.array([float(r[col] or 0) for r in rows])
        loaded = cpu[mem >= 0.5 * mem.max()] if mem.max() > 0 else cpu[1:]   # skip the start-up samples
    else:
        loaded = cpu[1:]
    if len(loaded) == 0:
        loaded = cpu
    return (f"{loaded.min():.2f}", f"{loaded.mean():.2f}", f"{loaded.max():.2f}")


def metrics_of(run):
    m = {}
    p = Path(run) / "slam_metrics.csv"
    if p.exists():
        for r in csv.reader(open(p)):
            if len(r) == 2:
                m[r[0]] = r[1]
    return m


def score(run):
    gt, slam = am.read_trajectories_from_bag(Path(run) / "bag")
    first = slam[0, 0]
    arrival = am.arrival_time(gt)
    start = max(first, am.mission_start_time(gt, first))
    end = am.scoring_window_end(slam, arrival, 3)
    est = am.slice_by_time(slam, start, end)
    e, g = associate(est, gt)
    P, G = e[:, 1:4], g[:, 1:4]
    out, series = {"n_matched": len(P), "gt_path_m": path_len(G)}, {}
    for tag, ws in (("sim3", True), ("se3", False)):
        s, R, t = umeyama(P, G, ws)
        A = (s * (R @ P.T)).T + t
        err = np.linalg.norm(A - G, axis=1)
        out[f"{tag}_ate_rmse"] = float(np.sqrt((err ** 2).mean()))
        out[f"{tag}_ate_median"] = float(np.median(err))
        out[f"{tag}_ate_max"] = float(err.max())
        out[f"{tag}_path_ratio_pct"] = 100 * path_len(A) / out["gt_path_m"]
        out[f"{tag}_scale"] = float(s)
        series[tag] = A
        if ws:
            h = len(P) // 2
            s1 = umeyama(P[:h], G[:h], True)[0]
            s2 = umeyama(P[h:], G[h:], True)[0]
            out["scale_drift_pct"] = 100 * abs(s2 - s1) / s1
    # local scale over 1 s windows, on the Sim(3)-aligned path (1.0 = consistent scale)
    tt = e[:, 0]
    A = series["sim3"]
    loc = np.full(len(tt), np.nan)
    for i in range(len(tt)):
        j = np.searchsorted(tt, tt[i] - 1.0)
        if i - j >= 3:
            dg = path_len(G[j:i + 1])
            if dg > 0.2:
                loc[i] = path_len(A[j:i + 1]) / dg
    # resample everything on 100 points of normalised time over the scoring window
    u = (tt - tt[0]) / (tt[-1] - tt[0])
    grid = np.linspace(0, 1, N_PTS)
    res = {"t": grid}
    for name, arr in (("gt", G), ("sim", series["sim3"]), ("se", series["se3"])):
        for k, ax in enumerate("xy"):
            res[f"{name}_{ax}"] = np.interp(grid, u, arr[:, k])
    ok = ~np.isnan(loc)
    res["scale_local"] = np.interp(grid, u[ok], loc[ok]) if ok.sum() > 2 else np.full(N_PTS, np.nan)
    return out, res


def main():
    out_dir = Path(sys.argv[1]); out_dir.mkdir(parents=True, exist_ok=True)
    mcols = ["arm_key", "run", "run_dir", "n_matched", "gt_path_m",
             "sim3_ate_rmse", "sim3_ate_median", "sim3_ate_max", "sim3_path_ratio_pct", "sim3_scale",
             "se3_ate_rmse", "se3_ate_median", "se3_ate_max", "se3_path_ratio_pct", "se3_scale",
             "scale_drift_pct", "scorer_ape_rmse", "pose_rate_hz", "capacity_hz",
             "fe_ms_p50", "fe_ms_p95", "fe_ms_p99", "cpu_mean_pct", "cpu_max_pct",
             "cpu_loaded_min", "cpu_loaded_mean", "cpu_loaded_max", "mem_max_mb"]
    scols = ["arm_key", "run", "i", "t", "gt_x", "gt_y", "sim_x", "sim_y", "se_x", "se_y", "scale_local"]
    with open(out_dir / "run_metrics.csv", "w", newline="") as fm, open(out_dir / "run_series.csv", "w", newline="") as fs:
        wm = csv.DictWriter(fm, fieldnames=mcols); wm.writeheader()
        ws = csv.writer(fs); ws.writerow(scols)
        for line in sys.stdin:
            if not line.strip():
                continue
            key, rep = line.split()
            runs = [r["run_dir"] for r in csv.DictReader(open(Path("reports") / rep / "runs.csv")) if r.get("run_dir")]
            for k, run in enumerate(runs, 1):
                try:
                    o, res = score(run)
                except Exception as exc:  # report and go on
                    print(f"{key} run {k}: FAILED {exc}", file=sys.stderr)
                    continue
                m = metrics_of(run)
                cmin, cmean, cmax = cpu_stats(run)
                row = {"arm_key": key, "run": k, "run_dir": Path(run).name, **{c: f"{o[c]:.4f}" if isinstance(o.get(c), float) else o.get(c, "") for c in mcols if c in o},
                       "scorer_ape_rmse": m.get("ape_rmse", ""), "pose_rate_hz": m.get("pose_rate_hz", ""),
                       "capacity_hz": m.get("capacity_hz", ""), "fe_ms_p50": m.get("fe_ms_p50", ""),
                       "fe_ms_p95": m.get("fe_ms_p95", ""), "fe_ms_p99": m.get("fe_ms_p99", ""),
                       "cpu_mean_pct": m.get("cpu_mean_pct", ""), "cpu_max_pct": m.get("cpu_max_pct", ""),
                       "cpu_loaded_min": cmin, "cpu_loaded_mean": cmean, "cpu_loaded_max": cmax,
                       "mem_max_mb": m.get("mem_max_mb", "")}
                wm.writerow(row)
                for i in range(N_PTS):
                    ws.writerow([key, k, i] + [f"{res[c][i]:.4f}" for c in scols[3:]])
                print(f"{key} run {k}: sim3 {o['sim3_ate_rmse']:.3f} (scorer {m.get('ape_rmse','?')[:5]}) se3 {o['se3_ate_rmse']:.3f}", file=sys.stderr)


main()
