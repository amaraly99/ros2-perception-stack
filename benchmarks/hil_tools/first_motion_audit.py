#!/usr/bin/env python3
"""first_motion_audit.py (16D, class A, read-only): the first-motion audit over run_outcomes.py's per_run_outcomes.tsv. Per run
the extraction already holds: the first /cmd_vel after resume with |vx| > 0.05 m/s and the forward displacement over the 3 s
after it (pass >= 0.10 m in the sign of vx); the first with |wz| > 0.05 rad/s and the unwrapped yaw change over 3 s (pass
>= 0.05 rad in the sign of wz); the same for vy as an extra (relative test). This script prints the counts per session, the
ranges of the responses, the sign consistency, and lists every run whose verdict is not True with its evidence.
Usage: python3 first_motion_audit.py <per_run_outcomes.tsv> [--out <audit.tsv>]"""
import argparse, csv, os, sys


def rows_of(path):
    return [r for r in csv.DictReader((l for l in open(path) if not l.startswith("#")), delimiter="\t")]


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("tsv"); ap.add_argument("--out", default=None)
    a = ap.parse_args(); rows = rows_of(a.tsv)
    print(f"# first-motion audit over {len(rows)} runs ({a.tsv})")
    print("session\truns\tvx tested\tvx pass\twz tested\twz pass\tvy tested\tvy pass\tverdict True\tverdict False\tno vx/wz cmd")
    for sess in sorted({r["sess"] for r in rows}, key=lambda s: (len(s), s)):
        rs = [r for r in rows if r["sess"] == sess]
        def tp(ch):
            t = [r for r in rs if r[f"{ch}_ok"] not in ("n/a", "", "no pose window")]
            return len(t), sum(1 for r in t if r[f"{ch}_ok"] == "True")
        vx, wz, vy = tp("vx"), tp("wz"), tp("vy")
        print("\t".join(str(x) for x in [sess, len(rs), vx[0], vx[1], wz[0], wz[1], vy[0], vy[1],
                                         sum(1 for r in rs if r["moved_as_commanded"] == "True"), sum(1 for r in rs if r["moved_as_commanded"] == "False"),
                                         sum(1 for r in rs if r["moved_as_commanded"].startswith("no "))]))
    # ranges over the passing runs and sign consistency
    fwd = [f(r["fwd_disp_3s_m"]) for r in rows if r["vx_ok"] == "True"]
    yaw = [(f(r["first_wz_cmd"]), f(r["dyaw_3s_rad"])) for r in rows if r["wz_ok"] == "True"]
    print(f"\nforward displacement over 3 s after the first vx command, passing runs: n {len(fwd)}, min {min(fwd):.2f}, max {max(fwd):.2f} m")
    print(f"yaw change over 3 s after the first wz command, passing runs: n {len(yaw)}, |dyaw| min {min(abs(y) for _, y in yaw):.3f}, max {max(abs(y) for _, y in yaw):.3f} rad; "
          f"sign(dyaw) == sign(wz) in {sum(1 for w, y in yaw if (w > 0) == (y > 0))} of {len(yaw)}")
    first_vx = [f(r["t_first_vx_cmd_s"]) for r in rows if f(r["t_first_vx_cmd_s"]) is not None]
    print(f"first vx command after resume: n {len(first_vx)}, {min(first_vx):.2f} to {max(first_vx):.2f} s; runs with no vx command after resume: "
          f"{sum(1 for r in rows if r['vx_ok'] == 'n/a')} (expected: the runs that never locked on)")
    nolock = [r for r in rows if r["locked_on"] != "True"]
    print(f"runs that never locked on: {len(nolock)}; of these, vx never commanded: {sum(1 for r in nolock if r['vx_ok'] == 'n/a')}, "
          f"wz tested: {sum(1 for r in nolock if r['wz_ok'] not in ('n/a', ''))}, wz pass: {sum(1 for r in nolock if r['wz_ok'] == 'True')}, "
          f"vy tested: {sum(1 for r in nolock if r['vy_ok'] not in ('n/a', ''))}, vy pass: {sum(1 for r in nolock if r['vy_ok'] == 'True')}")
    bad = [r for r in rows if r["moved_as_commanded"] != "True"]
    print(f"\nruns whose verdict is not True: {len(bad)}")
    cols = ["sess", "run", "model", "backend", "W_s", "moved_as_commanded", "t_first_vx_cmd_s", "first_vx_cmd", "fwd_disp_3s_m", "vx_ok",
            "t_first_wz_cmd_s", "first_wz_cmd", "dyaw_3s_rad", "wz_ok", "poses_after_resume", "distinct_poses_after_resume", "max_disp_from_start_m", "closest_m", "final_state"]
    print("\t".join(cols))
    for r in bad:
        print("\t".join(r[c] for c in cols))
    vy_t = [r for r in rows if r["vy_ok"] in ("True", "False")]
    print(f"\nextra channel vy (first |vy| >= 0.2 m/s, a search strafe): tested {len(vy_t)}, pass {sum(1 for r in vy_t if r['vy_ok'] == 'True')}; lateral displacement "
          f"{min(abs(f(r['lat_disp_3s_m'])) for r in vy_t):.2f} to {max(abs(f(r['lat_disp_3s_m'])) for r in vy_t):.2f} m over 3 s; sign relation to the command: "
          f"{ {k: sum(1 for r in vy_t if r['lat_sign_rel'] == k) for k in ('same', 'opposite')} } (the simulator's lateral sign is opposite to the controller code's body convention; a convention, recorded, not a fault)")
    vy_bad = [r for r in vy_t if r["vy_ok"] == "False"]
    print(f"vy runs below the relative threshold: {len(vy_bad)}")
    for r in vy_bad:
        print(f"  {r['sess']} {r['run']}: vy {r['first_vy_cmd']} m/s at {r['t_first_vy_cmd_s']} s, lateral {r['lat_disp_3s_m']} m")
    if a.out:
        if os.path.exists(a.out):
            sys.exit(f"REFUSE: {a.out} exists (no overwrite)")
        with open(a.out, "w") as o:
            o.write("# 16D first-motion audit per run (first_motion_audit.py over per_run_outcomes.tsv)\n")
            o.write("\t".join(cols + ["t_first_vy_cmd_s", "first_vy_cmd", "lat_disp_3s_m", "vy_ok", "lat_sign_rel"]) + "\n")
            for r in rows:
                o.write("\t".join(r[c] for c in cols + ["t_first_vy_cmd_s", "first_vy_cmd", "lat_disp_3s_m", "vy_ok", "lat_sign_rel"]) + "\n")
        print("wrote", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
