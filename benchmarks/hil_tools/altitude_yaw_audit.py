#!/usr/bin/env python3
"""altitude_yaw_audit.py (16F, class A, read-only): two audits over run_outcomes.py's per_run_outcomes.tsv (16F columns).
Altitude (prompt 16F step 1): per run the minimum z of /sim/drone_pose after resume, when it was attained, whether it reached
the ground (z <= 0.01 m) and when; grouped by set, family, backend and window; every pooled run below 1.0 m is listed, and so is
every run that reached the ground. Yaw (step 3): per run the largest and the mean |yaw - 360 deg| over the poses while the FSM
was APPROACHING and the lateral share sin(max dev) x 100 pct (the body-forward component of a world-y lateral command at that
heading: the simulator integrates cmd_vel along the world axes, SIM_PARAMS.md); runs above the bounds --max-dev-deg (3.0) or
--max-share-pct (5.0) are named per set. The REACHED intervals' maximum is reported as an extra.
Usage: python3 altitude_yaw_audit.py <per_run_outcomes.tsv> [--exclude <sess>:<run prefix> ...] [--max-dev-deg 3.0]
       [--max-share-pct 5.0] [--out <dir>]   (writes altitude_audit.tsv, yaw_audit.tsv and summary.txt; <dir> must not exist)"""
import argparse, csv, io, os, statistics, sys


def rows_of(path):
    return [r for r in csv.DictReader((l for l in open(path) if not l.startswith("#")), delimiter="\t")]


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def fam(m):
    return "centernet" if m.startswith("centernet") else m[-1]


def rng(vals, d=2):
    v = [x for x in vals if x is not None]
    return f"{min(v):.{d}f} to {max(v):.{d}f}" if v else "-"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("tsv"); ap.add_argument("--exclude", nargs="*", default=[])
    ap.add_argument("--max-dev-deg", type=float, default=3.0); ap.add_argument("--max-share-pct", type=float, default=5.0); ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.out and os.path.exists(a.out):
        sys.exit(f"altitude_yaw_audit: REFUSE: {a.out} exists")
    rows = rows_of(a.tsv)
    for c in ("min_z_m", "t_min_z_s", "t_ground_s", "z_reached_zero", "appr_yaw_dev_max_deg", "appr_lateral_share_max_pct"):
        if c not in rows[0]:
            sys.exit(f"altitude_yaw_audit: {a.tsv} has no 16F column {c}")
    def excluded(r):
        return any(r["sess"] == e.split(":")[0] and r["run"].startswith(e.split(":")[1]) for e in a.exclude)
    kept = [r for r in rows if not excluded(r)]; dropped = [r for r in rows if excluded(r)]
    buf = io.StringIO()
    def P(*x):
        print(*x); print(*x, file=buf)
    for r in rows:
        r["set"] = "5a" if r["sess"] == "s5a" else "pooled"; r["fam"] = fam(r["model"])
    P(f"# 16F altitude and yaw audit over {len(rows)} in-loop runs ({len(dropped)} set aside from the verdicts: {[(r['sess'], r['run']) for r in dropped]})")
    # ---- altitude ----
    P("\n== altitude after resume (z of /sim/drone_pose; the sign's face centre at 3.08 m; the start at 7.50 m)")
    P("set\tfamily\tbackend\tW_s\tlocked_on\tn\tmin_z m (min to max)\tt_min_z s (min to max)\treached ground (z <= 0.01 m)\tt_ground s\tz_last m (min to max)")
    groups = {}
    for r in rows:
        groups.setdefault((r["set"], r["fam"], r["backend"], f"{f(r['W_s']):.1f}", r["locked_on"]), []).append(r)
    for g in sorted(groups, key=lambda g: (g[0] != "pooled", float(g[3]), "nsmc".index(g[1][0]), g[2], g[4])):
        rs = groups[g]; ng = sum(1 for r in rs if r["z_reached_zero"] == "True")
        P("\t".join([g[0], g[1], g[2], g[3], g[4], str(len(rs)), rng(f(r["min_z_m"]) for r in rs), rng((f(r["t_min_z_s"]) for r in rs), 1), f"{ng}/{len(rs)}",
                     rng((f(r["t_ground_s"]) for r in rs), 1), rng(f(r["z_last_m"]) for r in rs)]))
    pooled = [r for r in rows if r["set"] == "pooled"]; fivea = [r for r in rows if r["set"] == "5a"]
    low = [r for r in pooled if f(r["min_z_m"]) is not None and f(r["min_z_m"]) < 1.0]
    P(f"\npooled runs below 1.0 m: {len(low)}" + ("".join(f"\n  {r['sess']} {r['run']} min z {f(r['min_z_m']):.3f} m at {f(r['t_min_z_s']):.1f} s" for r in low)))
    P(f"pooled runs that reached the ground: {sum(1 for r in pooled if r['z_reached_zero'] == 'True')} of {len(pooled)}; pooled minimum z overall {min(f(r['min_z_m']) for r in pooled):.3f} m "
      f"({next(r['sess'] + ' ' + r['run'] for r in pooled if f(r['min_z_m']) == min(f(x['min_z_m']) for x in pooled))}); lock-on runs {rng(f(r['min_z_m']) for r in pooled if r['locked_on'] == 'True')} m; "
      f"no-lock-on runs {rng(f(r['min_z_m']) for r in pooled if r['locked_on'] != 'True')} m")
    gc = [r for r in rows if r["z_reached_zero"] == "True"]
    P(f"runs that reached the ground: {len(gc)} (all sets)" + "".join(f"\n  {r['sess']} {r['run']} {r['model']} {r['backend']} W {f(r['W_s']):.1f}: z {f(r['min_z_m']):.3f} m, first at the ground {f(r['t_ground_s']):.1f} s, minimum at {f(r['t_min_z_s']):.1f} s, "
                                                                     f"z at the end {f(r['z_last_m']):.3f} m; closest {f(r['closest_m']):.2f} m, final {f(r['final_m']):.2f} m; class '{r['outcome_class']}'" for r in gc))
    notm = [r for r in gc if not (r["set"] == "5a" and r["fam"] == "m")]
    P(f"ground contact outside the 5a m-model rows: {len(notm)} {[(r['sess'], r['run']) for r in notm]}")
    P(f"5a m-model runs at W 2.5 s: {sum(1 for r in fivea if r['fam'] == 'm')} listed, {sum(1 for r in fivea if r['fam'] == 'm' and r['z_reached_zero'] == 'True')} reached the ground")
    # ---- yaw ----
    P(f"\n== yaw deviation from the world x axis while APPROACHING (|yaw - 360 deg| over the poses in APPROACHING intervals; share = sin(max dev) x 100 pct; bounds {a.max_dev_deg} deg, {a.max_share_pct} pct)")
    P("set\tfamily\tbackend\tW_s\tn with an approach\tmax dev deg (min to max over runs)\tmean dev deg (min to max)\tshare pct (min to max)\truns above the bounds\tREACHED max dev deg (min to max)")
    for g in sorted(groups, key=lambda g: (g[0] != "pooled", float(g[3]), "nsmc".index(g[1][0]), g[2], g[4])):
        rs = [r for r in groups[g] if f(r["appr_yaw_dev_max_deg"]) is not None]
        if not rs:
            continue
        bad = [r["run"][:3] for r in rs if f(r["appr_yaw_dev_max_deg"]) > a.max_dev_deg or f(r["appr_lateral_share_max_pct"]) > a.max_share_pct]
        P("\t".join([g[0], g[1], g[2], g[3], str(len(rs)), rng(f(r["appr_yaw_dev_max_deg"]) for r in rs), rng(f(r["appr_yaw_dev_mean_deg"]) for r in rs),
                     rng(f(r["appr_lateral_share_max_pct"]) for r in rs), f"{len(bad)}" + (f" {bad}" if bad else ""), rng(f(r["reached_yaw_dev_max_deg"]) for r in rs)]))
    for st, rs_ in (("pooled", pooled), ("5a", fivea)):
        rs = [r for r in rs_ if f(r["appr_yaw_dev_max_deg"]) is not None and not excluded(r)]
        na = [r for r in rs_ if f(r["appr_yaw_dev_max_deg"]) is None]
        bad = [r for r in rs if f(r["appr_yaw_dev_max_deg"]) > a.max_dev_deg or f(r["appr_lateral_share_max_pct"]) > a.max_share_pct]
        P(f"\n{st}: {len(rs)} runs with an APPROACHING interval ({len(na)} without: never locked on" + (f"; set aside {[(r['sess'], r['run'][:3]) for r in rs_ if excluded(r)]}" if any(excluded(r) for r in rs_) else "") + ")"
          f"; max dev {rng(f(r['appr_yaw_dev_max_deg']) for r in rs)} deg, mean dev {rng(f(r['appr_yaw_dev_mean_deg']) for r in rs)} deg, share {rng(f(r['appr_lateral_share_max_pct']) for r in rs)} pct; "
          f"above the bounds: {len(bad)} run(s)" + "".join(f"\n  {r['sess']} {r['run']} {r['model']} {r['backend']} W {f(r['W_s']):.1f} class '{r['outcome_class']}': max dev {f(r['appr_yaw_dev_max_deg']):.2f} deg (mean {f(r['appr_yaw_dev_mean_deg']):.2f}), share {f(r['appr_lateral_share_max_pct']):.2f} pct, {r['appr_n_poses']} poses" for r in sorted(bad, key=lambda r: -f(r["appr_yaw_dev_max_deg"]))))
    if a.out:
        os.makedirs(a.out)
        ac = ["sess", "run", "model", "backend", "W_s", "role", "locked_on", "reached", "outcome_class", "z_at_resume_m", "min_z_m", "t_min_z_s", "t_ground_s", "z_reached_zero", "z_last_m", "closest_m", "final_m"]
        yc = ["sess", "run", "model", "backend", "W_s", "role", "locked_on", "reached", "outcome_class", "appr_n_poses", "appr_yaw_dev_max_deg", "appr_yaw_dev_mean_deg", "appr_lateral_share_max_pct", "reached_n_poses", "reached_yaw_dev_max_deg"]
        with open(os.path.join(a.out, "altitude_audit.tsv"), "w") as o:
            o.write("# 16F altitude audit per run (altitude_yaw_audit.py over per_run_outcomes.tsv): z of /sim/drone_pose after resume\n"); o.write("\t".join(ac) + "\n")
            for r in rows:
                o.write("\t".join(r[c] for c in ac) + "\n")
        with open(os.path.join(a.out, "yaw_audit.tsv"), "w") as o:
            o.write(f"# 16F yaw deviation per run while APPROACHING (altitude_yaw_audit.py over per_run_outcomes.tsv); bounds {a.max_dev_deg} deg and {a.max_share_pct} pct\n"); o.write("\t".join(yc) + "\n")
            for r in rows:
                o.write("\t".join(r[c] for c in yc) + "\n")
        with open(os.path.join(a.out, "summary.txt"), "w") as o:
            o.write(buf.getvalue())
        print("wrote altitude_audit.tsv, yaw_audit.tsv and summary.txt in", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
