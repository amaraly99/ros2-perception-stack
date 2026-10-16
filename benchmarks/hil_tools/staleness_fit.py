#!/usr/bin/env python3
"""staleness_fit.py (16D, class A, read-only, exploratory): the stale-box model over every approaching run of
run_outcomes.py's per_run_outcomes.tsv. Per run the extraction holds x = v x (L + G) with v the ground speed at the capture
of the last target box before the closest point, L the mean capture-to-commit latency over the frames captured between resume
and the closest point, G the mean gap between target-box captures over the same span, and S = fy x 0.914 / (target_bbox_ratio
x 480) + 0.2 m, the vx-zero standoff at the drone origin. This script fits ONE straight line closest = a + b x by least squares
over the selected runs, prints a and b with the residual spread, and the residuals against the fit and against the identity
model (closest = S - x) by configuration family (n, s, m, CenterNet), backend and window. It writes the points for the figure.
Usage: python3 staleness_fit.py <per_run_outcomes.tsv> [--exclude <sess>:<run prefix> ...] [--fit-without-family m] [--out <dir>]"""
import argparse, csv, os, statistics, sys


def rows_of(path):
    return [r for r in csv.DictReader((l for l in open(path) if not l.startswith("#")), delimiter="\t")]


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def fam(m):
    return "centernet" if m.startswith("centernet") else m[-1]


def lsq(xs, ys):
    n = len(xs); mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs); sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    b = sxy / sxx if sxx else float("nan"); a = my - b * mx
    res = [y - (a + b * x) for x, y in zip(xs, ys)]
    return a, b, res


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("tsv"); ap.add_argument("--exclude", nargs="*", default=[]); ap.add_argument("--fit-without-family", default=None)
    ap.add_argument("--out", default=None); a = ap.parse_args()
    rows = rows_of(a.tsv)
    def excluded(r):
        return any(r["sess"] == e.split(":")[0] and r["run"].startswith(e.split(":")[1]) for e in a.exclude)
    pts = [r for r in rows if r["approached"] == "True" and f(r["x_vLG_m"]) is not None and not excluded(r)]
    skipped = [r["run"] for r in rows if r["approached"] == "True" and f(r["x_vLG_m"]) is None and not excluded(r)]
    S = f(pts[0]["S_origin_m"])
    print(f"# staleness model over {len(pts)} approaching runs (set aside {a.exclude}; approaching runs without a term {skipped}); S = {S:.3f} m at the drone origin")
    xs = [f(r["x_vLG_m"]) for r in pts]; ys = [f(r["closest_m"]) for r in pts]
    A, B, res = lsq(xs, ys)
    print(f"one straight line over all points: closest = {A:.3f} + ({B:.3f}) x; residual sd {statistics.pstdev(res):.3f} m, max |res| {max(abs(v) for v in res):.3f} m")
    if a.fit_without_family:
        sub = [(x, y) for x, y, r in zip(xs, ys, pts) if fam(r["model"]) != a.fit_without_family]
        A2, B2, res2 = lsq([p[0] for p in sub], [p[1] for p in sub])
        print(f"for comparison, without family '{a.fit_without_family}' ({len(sub)} points): closest = {A2:.3f} + ({B2:.3f}) x; residual sd {statistics.pstdev(res2):.3f} m")
    print("\n# residuals by configuration family, backend and window: against the fitted line and against the identity model closest = S - x")
    print("family\tbackend\tW_s\tn\tx min\tx max\tclosest min\tclosest max\tv min\tv max\tL mean\tG mean\tres_fit mean\tres_fit sd\tres_identity mean\tres_identity min\tres_identity max\tverdict")
    groups = {}
    for r, x, y, e in zip(pts, xs, ys, res):
        groups.setdefault((fam(r["model"]), r["backend"], f"{f(r['W_s']):.1f}"), []).append((r, x, y, e))
    for g in sorted(groups, key=lambda g: ("nsmc".index(g[0][0]), g[1], float(g[2]))):
        G = groups[g]; rf = [e for _, _, _, e in G]; ri = [y - (S - x) for _, x, y, _ in G]
        verdict = "holds (identity within 0.5 m)" if max(abs(v) for v in ri) <= 0.5 else ("fails (identity off by more than 1 m)" if min(abs(v) for v in ri) > 1.0 else "mixed")
        print("\t".join([g[0], g[1], g[2], str(len(G)), f"{min(x for _, x, _, _ in G):.2f}", f"{max(x for _, x, _, _ in G):.2f}", f"{min(y for _, _, y, _ in G):.2f}", f"{max(y for _, _, y, _ in G):.2f}",
                         f"{min(f(r['v_at_last_prebox_m_s']) for r, _, _, _ in G):.2f}", f"{max(f(r['v_at_last_prebox_m_s']) for r, _, _, _ in G):.2f}",
                         f"{statistics.fmean(f(r['lat_L_s']) for r, _, _, _ in G):.2f}", f"{statistics.fmean(f(r['gap_G_s']) for r, _, _, _ in G):.2f}",
                         f"{statistics.fmean(rf):.2f}", f"{statistics.pstdev(rf):.2f}", f"{statistics.fmean(ri):.2f}", f"{min(ri):.2f}", f"{max(ri):.2f}", verdict]))
    if a.out:
        if os.path.exists(a.out):
            sys.exit(f"staleness_fit: REFUSE: {a.out} exists")
        os.makedirs(a.out)
        with open(os.path.join(a.out, "staleness_points.tsv"), "w") as o:
            o.write(f"# 16D staleness model points (staleness_fit.py); fit closest = {A:.4f} + ({B:.4f}) x over {len(pts)} runs; S = {S:.3f} m; INFERRED exploratory model\n")
            o.write("sess\trun\tmodel\tbackend\tfamily\tW_s\trole\tv_m_s\tL_s\tG_s\tx_m\tclosest_m\tpred_identity_m\tres_identity_m\tres_fit_m\toutcome_class\n")
            for r, x, y, e in zip(pts, xs, ys, res):
                o.write("\t".join([r["sess"], r["run"], r["model"], r["backend"], fam(r["model"]), f"{f(r['W_s']):.1f}", r["role"], r["v_at_last_prebox_m_s"], r["lat_L_s"], r["gap_G_s"],
                                   f"{x:.4f}", f"{y:.4f}", f"{S - x:.4f}", f"{y - (S - x):.4f}", f"{e:.4f}", r["outcome_class"]]) + "\n")
        with open(os.path.join(a.out, "fit.txt"), "w") as o:
            o.write(f"closest = {A:.4f} + ({B:.4f}) x over {len(pts)} runs; S {S:.3f}; residual sd {statistics.pstdev(res):.4f}\n")
        print("wrote staleness_points.tsv and fit.txt in", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
