#!/usr/bin/env python3
"""slam_ate.py (20C, stage 2 of 2 of the SLAM accuracy scorer; proposed/07_ate_scorer.md): absolute trajectory
error between a SLAM trajectory and the simulator ground truth, with Umeyama alignment in Sim(3) (scale solved,
MANDATORY for monocular) and in SE(3) (scale fixed at 1). Thirty lines of linear algebra instead of the external
`evo` binary, which is not installed on Pi-3 and may not be installed by this prompt.

Run under the SYSTEM python (numpy). Stage 1 (slam_traj_export.py, under $HIL_MCAP_PY) writes the inputs.

Method, fixed before any number is quoted:
  association   nearest time, both in bag log ns, accepted only within --tol (default 0.05 s, which is Amar's
                GT_SLAM_MATCH_TOL_S in scripts/hil_matrix/aggregate_matrix.py:60). The largest accepted gap is
                reported with every result, so a loose association is visible.
  alignment     Umeyama (Least-Squares Estimation of Transformation Parameters Between Two Point Patterns,
                1991): R from the SVD of the cross-covariance with a reflection guard, s = tr(D S) / var(slam)
                for Sim(3) and s = 1 for SE(3), t = mean(gt) - s R mean(slam).
  error         per-pair Euclidean distance after alignment; RMSE, mean, median, max, std.
  windows       the full association, and the same with the last 0, 3 and 10 samples dropped (Amar's
                WINDOW_PADDINGS), so an end-of-run divergence is visible rather than averaged in.
  frames        the SLAM estimate is the LEFT CAMERA pose and the ground truth is the BODY pose of
                /sim/drone_pose. The camera sits 0.2 m forward and 0.2 m up of the body origin, pitched 5 deg,
                so the two differ by a constant rigid offset of norm about 0.283 m in the body frame. Umeyama
                solves for exactly one rotation and one translation, so a CONSTANT offset is absorbed and does
                not inflate the RMSE. Only the part of the offset that rotates during the run would, and on a
                fixed-heading, fixed-attitude approach there is none (DEVIATIONS/D1 of prompt 20C).

Self test (--selftest), which must pass before any real number is quoted:
  1. a synthetic trajectory transformed by a known Sim(3): RMSE below 1e-9 m and the scale factor within 1e-9
     relative of the known one;
  2. the same with Gaussian noise of known sigma: RMSE within 5 pct of sigma;
  3. true scale 1: the SE(3) and Sim(3) results equal, and the recovered scale within 1e-9 of 1.

Usage:
  python3 slam_ate.py --selftest
  python3 slam_ate.py --gt <gt.tsv> --slam <slam.tsv> [--tol 0.05] [--label <name>] [--tsv-line]"""
import argparse, math, sys

import numpy as np


def read_tsv(path, ncol):
    """t_ns and the next ncol-1 float columns of a TSV written by slam_traj_export.py."""
    t, v = [], []
    for line in open(path):
        if line.startswith("#") or line.startswith("t_ns"):
            continue
        p = line.split()
        if len(p) < ncol:
            continue
        t.append(int(p[0]))
        v.append([float(x) for x in p[1:ncol]])
    return np.asarray(t, dtype=np.int64), np.asarray(v, dtype=np.float64)


def associate(t_gt, p_gt, t_sl, p_sl, tol_ns):
    """Nearest-time pairing of each SLAM sample with one ground-truth sample, inside tol_ns."""
    if len(t_gt) == 0 or len(t_sl) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3)), 0, 0
    idx = np.searchsorted(t_gt, t_sl)
    idx = np.clip(idx, 1, len(t_gt) - 1)
    left, right = t_gt[idx - 1], t_gt[idx]
    pick = np.where(np.abs(t_sl - left) <= np.abs(right - t_sl), idx - 1, idx)
    gap = np.abs(t_gt[pick] - t_sl)
    keep = gap <= tol_ns
    return p_gt[pick][keep], p_sl[keep], int(keep.sum()), int(gap[keep].max()) if keep.any() else 0


def umeyama(src, dst, with_scale):
    """Sim(3) or SE(3) that maps src onto dst in the least-squares sense. Returns s, R, t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    u, d, vt = np.linalg.svd(cov)
    s_mat = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:          # reflection guard
        s_mat[2, 2] = -1.0
    rot = u @ s_mat @ vt
    var_s = (xs ** 2).sum() / len(src)
    scale = float((np.trace(np.diag(d) @ s_mat) / var_s)) if with_scale and var_s > 0 else 1.0
    trans = mu_d - scale * rot @ mu_s
    return scale, rot, trans


def errors(gt, sl, with_scale):
    scale, rot, trans = umeyama(sl, gt, with_scale)
    res = gt - (scale * (rot @ sl.T).T + trans)
    e = np.linalg.norm(res, axis=1)
    return scale, e


def dist(e):
    return dict(rmse=float(np.sqrt((e ** 2).mean())), mean=float(e.mean()), median=float(np.median(e)),
                max=float(e.max()), std=float(e.std()), n=int(len(e)))


def score(gt, sl, pads=(0, 3, 10)):
    out = {}
    for pad in pads:
        g, s = (gt, sl) if pad == 0 else (gt[:len(gt) - pad], sl[:len(sl) - pad])
        if len(g) < 4:
            out[pad] = None
            continue
        sc_sim, e_sim = errors(g, s, True)
        sc_se3, e_se3 = errors(g, s, False)
        out[pad] = dict(sim3=dist(e_sim), scale=sc_sim, se3=dist(e_se3), se3_scale=sc_se3)
    return out


def selftest():
    rng = np.random.default_rng(20261020)
    n = 1200
    # a straight 17 m approach with a little lateral and vertical structure, like the real leg
    tt = np.linspace(0, 1, n)
    traj = np.stack([17.0 * tt, 0.4 * np.sin(2 * np.pi * tt), 0.2 * tt], axis=1)
    ax = np.array([0.3, -0.5, 0.81]); ax /= np.linalg.norm(ax); th = 0.7
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    rot_true = np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * (k @ k)
    t_true = np.array([2.5, -1.25, 0.75])
    ok = True

    # 1. exact Sim(3)
    s_true = 2.5
    est = ((traj - t_true) @ rot_true) / s_true            # inverse of s R x + t
    s_rec, e = errors(traj, est, True)
    c1 = e.max() < 1e-9 and abs(s_rec - s_true) / s_true < 1e-9
    ok &= c1
    print(f"selftest 1 exact Sim(3) (scale {s_true}): RMSE {np.sqrt((e**2).mean()):.3e} m "
          f"(tol 1e-9), scale {s_rec:.12f} rel err {abs(s_rec - s_true) / s_true:.3e} (tol 1e-9) -> "
          f"{'PASS' if c1 else 'FAIL'}")

    # 2. known Gaussian noise
    sigma = 0.08
    noise = rng.normal(0.0, sigma / math.sqrt(3.0), size=traj.shape)   # per-axis, so |noise| has rms sigma
    _, e2 = errors(traj + noise, est, True)
    rmse2 = float(np.sqrt((e2 ** 2).mean()))
    c2 = abs(rmse2 - sigma) / sigma < 0.05
    ok &= c2
    print(f"selftest 2 noise sigma {sigma} m: RMSE {rmse2:.6f} m, rel err {abs(rmse2 - sigma) / sigma:.4f} "
          f"(tol 0.05) -> {'PASS' if c2 else 'FAIL'}")

    # 3. true scale 1: SE(3) and Sim(3) must agree
    est1 = (traj - t_true) @ rot_true
    s3, e3 = errors(traj, est1, True)
    s3b, e3b = errors(traj, est1, False)
    c3 = abs(s3 - 1.0) < 1e-9 and abs(float(np.sqrt((e3 ** 2).mean())) - float(np.sqrt((e3b ** 2).mean()))) < 1e-9
    ok &= c3
    print(f"selftest 3 true scale 1: Sim(3) scale {s3:.12f} (tol 1e-9 of 1), SE(3) scale {s3b:.1f}, "
          f"RMSE Sim(3) {np.sqrt((e3**2).mean()):.3e} vs SE(3) {np.sqrt((e3b**2).mean()):.3e} -> "
          f"{'PASS' if c3 else 'FAIL'}")
    print("SELFTEST " + ("PASS (3 of 3)" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--gt"); ap.add_argument("--slam")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--label", default="")
    ap.add_argument("--tsv-line", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if not (a.gt and a.slam):
        sys.exit("slam_ate.py: --selftest, or both --gt and --slam")
    t_gt, p_gt = read_tsv(a.gt, 4)
    t_sl, p_sl = read_tsv(a.slam, 4)
    gt, sl, n, gap = associate(t_gt, p_gt, t_sl, p_sl, int(a.tol * 1e9))
    if n < 4:
        print(f"{a.label}\tNOT SCORED\tmatched {n} pairs of {len(t_sl)} slam and {len(t_gt)} gt samples")
        sys.exit(2)
    r = score(gt, sl)
    f = r[0]
    if a.tsv_line:
        print("%s\t%d\t%d\t%.4f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%s" % (
            a.label, n, len(t_sl), f["scale"], f["sim3"]["rmse"], f["sim3"]["mean"], f["sim3"]["median"],
            f["sim3"]["max"], f["sim3"]["std"], f["se3"]["rmse"], gap))
        return
    print(f"# {a.label}: {n} pairs of {len(t_sl)} /slam/pose and {len(t_gt)} /sim/drone_pose samples, "
          f"largest accepted stamp gap {gap / 1e6:.3f} ms (tol {a.tol * 1e3:.0f} ms)")
    for pad in (0, 3, 10):
        v = r[pad]
        if v is None:
            print(f"pad {pad:2d}: too few pairs")
            continue
        s3 = v["sim3"]
        print(f"pad {pad:2d}: Sim(3) rmse {s3['rmse']:.4f} mean {s3['mean']:.4f} median {s3['median']:.4f} "
              f"max {s3['max']:.4f} std {s3['std']:.4f} m, scale {v['scale']:.4f}; "
              f"SE(3) rmse {v['se3']['rmse']:.4f} m  (n {s3['n']})")


if __name__ == "__main__":
    main()
