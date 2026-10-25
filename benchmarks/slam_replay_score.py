#!/usr/bin/env python3
"""slam_replay_score.py — score replayed SLAM poses against the recorded ground truth of the flight.

    python3 benchmarks/slam_replay_score.py <run_dir> [--metric sim3|se3] [replay_subdir ...]

Time base: SLAM poses carry the image stamps; the flight bag gives (image stamp -> bag time) for
/ovcam/image_raw, and /sim/drone_pose in bag time. Per backend:
  init_s        bag time of the first non-zero pose, from the start of the bag
  approach_*    the straight approach: from GT forward motion start (x +0.3 m) to the stop
  survival_pct  share of the approach tracked before divergence. Divergence needs no alignment: the step
                ratio |dSLAM|/|dGT| over 0.5 s leaves [r0/3, 3 r0] while the drone moves > 0.6 m/s (r0 = its median over the first
                3 s of tracked motion), or SLAM moves > 0.5 m (scaled) while the drone hovers, for 1.0 s
  ate_rmse_m    RMSE over everything tracked up to the divergence (init leg included), sim3 or se3
  scale         sim3 scale of that fit (1.0 for se3)
Writes <replay_subdir>/score.json and prints one line per backend.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def umeyama(src, dst, with_scale):
    """dst ~ s R src + t (Umeyama 1991). Returns s, R, t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = (np.trace(np.diag(D) @ S) / (xs ** 2).sum(1).mean()) if with_scale else 1.0
    return s, R, mu_d - s * R @ mu_s


def load_flight(run_dir):
    gt, stamp_map = [], {}
    with AnyReader([run_dir / "bag"]) as r:
        cons = [c for c in r.connections if c.topic in ("/sim/drone_pose", "/ovcam/image_raw")]
        t0 = None
        for con, ts, raw in r.messages(connections=cons):
            t0 = t0 or ts
            t = (ts - t0) / 1e9
            m = r.deserialize(raw, con.msgtype)
            if con.topic == "/sim/drone_pose":
                gt.append((t, *m.data[:3]))
            else:
                st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                stamp_map.setdefault(round(st, 6), t)
    k = np.array(sorted(stamp_map))
    return np.array(gt), k, np.array([stamp_map[x] for x in k])


def score(run_dir, sub, metric):
    gt, ks, kt = load_flight(run_dir)
    P = np.loadtxt(sub / "poses.csv", delimiter=",", comments="#", skiprows=1, ndmin=2)
    I = np.loadtxt(sub / "images.csv", delimiter=",", comments="#", skiprows=1, ndmin=2)
    if len(P) and P.shape[1] == 9 and len(I):
        # pair each pose with the last image that arrived before it; use that image's stamp
        j = np.searchsorted(I[:, 0], P[:, 0], side="right") - 1
        ok = j >= 0
        P = np.column_stack([np.where(ok, I[np.clip(j, 0, None), 1], np.nan), P[:, 2:]])
        P = P[np.isfinite(P[:, 0])]
    res = {"backend": sub.name, "metric": metric, "poses": int(len(P))}
    if len(P) == 0:
        res["status"] = "no poses"
        return res
    P = P[np.any(np.abs(P[:, 1:4]) > 1e-9, axis=1)]
    if len(P) < 10:
        res["status"] = "never initialised"
        return res
    t = np.interp(P[:, 0], ks, kt)                         # image stamp -> bag time
    g = np.column_stack([np.interp(t, gt[:, 0], gt[:, i]) for i in (1, 2, 3)])
    s = P[:, 1:4]
    res["init_s"] = round(float(t[0]), 2)
    # approach window from GT
    x0 = gt[0, 1]
    i_start = np.argmax(gt[:, 1] > x0 + 0.3)
    t_a0 = gt[i_start, 0]
    v = np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1) / np.maximum(np.diff(gt[:, 0]), 1e-3)
    moving = np.where((gt[1:, 0] > t_a0) & (v > 0.05))[0]
    t_a1 = gt[moving[-1] + 1, 0] if len(moving) else gt[-1, 0]
    res["approach_start_s"], res["approach_end_s"] = round(float(t_a0), 2), round(float(t_a1), 2)
    res["initialised_before_approach"] = bool(t[0] < t_a0)
    sel = (t >= max(t_a0, t[0])) & (t <= t_a1)
    if sel.sum() < 10:
        res["status"] = "no poses in the approach"
        return res
    # Divergence without alignment (a straight line cannot fix a Sim(3) rotation about itself):
    # step ratio r = |dSLAM| / |dGT| over 0.5 s, against its median r0 over the first 3 s of tracked motion.
    grid = np.arange(t[0] + 0.5, t[-1], 0.25)
    def at(arr, tt):
        return np.column_stack([np.interp(tt, t, arr[:, i]) for i in range(3)])
    dS = np.linalg.norm(at(s, grid) - at(s, grid - 0.5), axis=1)
    dG = np.linalg.norm(at(g, grid) - at(g, grid - 0.5), axis=1)
    moving = dG > 0.3          # > 0.6 m/s: the ratio is meaningful
    still = dG < 0.05          # hovering
    first = moving & (grid <= grid[moving][0] + 3.0) if moving.any() else moving
    r0 = float(np.median(dS[first] / dG[first])) if first.any() else float("nan")
    res["scale_ratio_r0"] = round(r0, 4)
    bad = np.zeros(len(grid), bool)
    if np.isfinite(r0) and r0 > 0:
        r = np.where(moving, dS / np.maximum(dG, 1e-6) / r0, np.nan)
        bad = np.where(moving, (r > 3.0) | (r < 1 / 3.0), still & (dS / r0 > 0.5))
    t_div = None
    for i in range(len(grid) - 3):
        if bad[i:i + 4].all():                 # 1.0 s sustained
            t_div = float(grid[i])
            break
    t_end_track = t_div if t_div is not None else float(t[-1])
    # survival = share of the approach that was actually tracked: from max(first pose, approach start)
    surv = 100.0 * (min(t_end_track, t_a1) - max(float(t[0]), t_a0)) / max(t_a1 - t_a0, 1e-6)
    res["survival_pct"] = round(float(np.clip(surv, 0, 100)), 1)
    res["diverged_at_s"] = None if t_div is None else round(t_div, 2)
    keep = (t <= t_end_track) & (t <= t_a1)
    if keep.sum() >= 10:
        sc2, R2, tr2 = umeyama(s[keep], g[keep], metric == "sim3")
        e2 = np.linalg.norm((sc2 * (R2 @ s[keep].T)).T + tr2 - g[keep], axis=1)
        res["ate_rmse_m"] = round(float(np.sqrt((e2 ** 2).mean())), 3)
        res["scale"] = round(float(sc2), 4)
        res["scored_path_m"] = round(float(np.linalg.norm(np.diff(g[keep], axis=0), axis=1).sum()), 2)
    res["status"] = "survived" if t_div is None else "diverged"
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("subs", nargs="*")
    ap.add_argument("--metric", choices=("sim3", "se3"), help="default: sim3 for *_mono*, se3 otherwise")
    a = ap.parse_args()
    run_dir = Path(a.run_dir).resolve()
    subs = [run_dir / "replay" / s for s in a.subs] if a.subs else sorted((run_dir / "replay").iterdir())
    for sub in subs:
        metric = a.metric or ("sim3" if "_mono" in sub.name else "se3")
        try:
            r = score(run_dir, sub, metric)
        except Exception as e:  # noqa: BLE001
            r = {"backend": sub.name, "status": f"error: {e}"}
        (sub / "score.json").write_text(json.dumps(r, indent=1))
        print(f"{r['backend']:<44} {r.get('status',''):<18} init {r.get('init_s','-')!s:>6}s  "
              f"survival {r.get('survival_pct','-')!s:>5}%  ATE {r.get('ate_rmse_m','-')!s:>6} m  scale {r.get('scale','-')!s:>8}  "
              f"path {r.get('scored_path_m','-')!s:>6} m")


if __name__ == "__main__":
    main()
