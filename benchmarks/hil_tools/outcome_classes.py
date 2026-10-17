#!/usr/bin/env python3
"""outcome_classes.py (16D, class A, read-only): outcome classes per configuration and loss window from run_outcomes.py's
per_run_outcomes.tsv, the band count that shows the 1.0 m line's influence, and the consistency of the extraction with the
stored values (check_run's closest approach, mission.json's final distance, the /bench/state lock-on) and, when given, with
the 16A pool_tables per_run.tsv (reached, final_state, closest, final).
Classes (fixed in stage6/analysis_16D/EXPECTED.md before counting): never locked on; reached cleanly (closest >= 1.0 m);
reached after an overshoot (closest < 1.0 m); overshot and lost the sign (not REACHED, closest < 1.0 m); arrived in the window
without REACHED (not REACHED, locked on, closest in [1.6, 2.8] m); other (named in outcome_note). 16F (DEVIATIONS 17.80,
POST-HOC): ground contact (minimum z after resume <= 0.01 m in 16F; <= 0.5 m since 16G, DEVIATIONS 17.80 f), which takes precedence over every other class; the classes are
assigned by run_outcomes.py, this script counts them and lists the ground-contact runs with their minimum z and its time.
Usage: python3 outcome_classes.py <per_run_outcomes.tsv> [--exclude <sess>:<run dir prefix> ...] [--compare-16A <per_run.tsv>]
       [--out <dir>]   (writes classes_by_config.tsv and classes_per_run.tsv into <dir>; it must not exist)"""
import argparse, csv, os, sys
CLASSES = ["reached cleanly", "reached after an overshoot", "overshot and lost the sign", "ground contact", "arrived in the window without REACHED", "never locked on", "other"]
ORDER = ["yolo26n", "yolo26s", "yolo26m", "yolov8n", "yolov8s", "yolov8m", "yolov11n", "yolov11s", "yolov11m", "centernet_resnet_v1_18"]


def rows_of(path):
    return [r for r in csv.DictReader((l for l in open(path) if not l.startswith("#")), delimiter="\t")]


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("tsv"); ap.add_argument("--exclude", nargs="*", default=[])
    ap.add_argument("--compare-16A", default=None); ap.add_argument("--out", default=None); ap.add_argument("--band", default="0.5,1.4")
    a = ap.parse_args(); rows = rows_of(a.tsv)
    lo_b, hi_b = (float(x) for x in a.band.split(","))
    excl = set(a.exclude)
    def excluded(r):
        return any(r["sess"] == e.split(":")[0] and r["run"].startswith(e.split(":")[1]) for e in excl)
    kept = [r for r in rows if not excluded(r)]; dropped = [r for r in rows if excluded(r)]
    print(f"# outcome classes over {len(kept)} runs ({len(dropped)} set aside: {[(r['sess'], r['run']) for r in dropped]})")
    # consistency
    dc = max(abs(f(r["closest_m"]) - f(r["closest_stored_m"])) for r in rows if f(r["closest_stored_m"]) is not None)
    df = max(abs(f(r["final_m"]) - f(r["final_stored_m"])) for r in rows if f(r["final_stored_m"]) is not None)
    dl = [abs(f(r["lock_log_s"]) - f(r["lock_bench_s"])) for r in rows if f(r["lock_log_s"]) is not None and f(r["lock_bench_s"]) is not None]
    lock_mismatch = [r["run"] for r in rows if (f(r["lock_log_s"]) is None) != (f(r["lock_bench_s"]) is None)]
    print(f"consistency: |closest - stored closest| max {dc:.4f} m; |final - stored final| max {df:.4f} m; lock-on (stack log against /bench/state): "
          f"max |dt| {max(dl):.3f} s over {len(dl)} runs, presence mismatch {lock_mismatch}; replay_consistent False in "
          f"{sum(1 for r in rows if r['replay_consistent'] != 'True')} runs")
    if a.compare_16A:
        ref = {(r["sess"], r["run"]): r for r in rows_of(a.compare_16A)}
        n = 0; bad = []
        for r in rows:
            k = (r["sess"], r["run"])
            if k not in ref:
                continue
            n += 1; q = ref[k]
            if (q["reached"] != r["reached"] or q["final_state"] != r["final_state"] or abs(f(q["closest_m"]) - f(r["closest_m"])) > 1e-3
                    or (f(q["final_m"]) is not None and abs(f(q["final_m"]) - f(r["final_m"])) > 1e-3)):
                bad.append(k)
        print(f"against 16A per_run.tsv: {n} runs matched, {len(bad)} differ in reached/final_state/closest/final: {bad}")
    unknown = sorted({r["outcome_class"] for r in rows} - set(CLASSES))
    if unknown:
        sys.exit(f"outcome_classes: REFUSE: class names not in CLASSES: {unknown}")
    # 16F: the ground-contact runs (precedence over every other class)
    gc = [r for r in kept if r["outcome_class"] == "ground contact"]
    print(f"ground contact (16G rule, min z after resume <= 0.5 m, DEVIATIONS 17.80 f; z_reached_zero still means z <= 0.01 m): {len(gc)} runs" + (": " + "; ".join(
        f"{r['sess']} {r['run'][:3]} {r['model']} {r['backend']} W {f(r['W_s']):.1f} (z {f(r['min_z_m']):.3f} m at {(f(r['t_ground_s']) if f(r['t_ground_s']) is not None else f(r['t_min_z_s'])):.1f} s; closest {f(r['closest_m']):.2f}, final {f(r['final_m']):.2f} m)" for r in gc) if gc else ""))
    if "min_z_m" in rows[0]:
        low = [r for r in kept if f(r["min_z_m"]) is not None and f(r["min_z_m"]) < 1.0 and r["outcome_class"] != "ground contact"]
        print(f"runs below 1.0 m that are not ground contact: {len(low)} {[(r['sess'], r['run'][:3], f(r['min_z_m'])) for r in low]}")
    # the band around the line
    band = [r for r in kept if f(r["closest_m"]) is not None and lo_b <= f(r["closest_m"]) <= hi_b]
    locked = [r for r in kept if r["locked_on"] == "True"]
    below = sorted(f(r["closest_m"]) for r in locked if f(r["closest_m"]) < 1.0); above = sorted(f(r["closest_m"]) for r in locked if f(r["closest_m"]) >= 1.0)
    print(f"closest approach in [{lo_b}, {hi_b}] m: {len(band)} runs {[(r['sess'], r['run']) for r in band]}; among locked-on runs the largest closest below 1.0 m is "
          f"{max(below) if below else None} and the smallest at or above 1.0 m is {min(above) if above else None}")
    # per configuration and window
    def key(r):
        return (r["sess"] == "s5a", f(r["W_s"]), ORDER.index(r["model"]) if r["model"] in ORDER else 99, r["backend"], r["role"] == "sentinel")
    groups = {}
    for r in kept:
        groups.setdefault((("5a" if r["sess"] == "s5a" else "pooled"), r["model"], r["backend"], f"{f(r['W_s']):.1f}", r["role"]), []).append(r)
    hdr = ["set", "model", "backend", "W_s", "role", "n"] + CLASSES + ["closest_min_m", "closest_max_m", "notes"]
    out_rows = []
    for g in sorted(groups, key=lambda g: (g[0] != "pooled", float(g[3]), ORDER.index(g[1]) if g[1] in ORDER else 99, g[2], g[4])):
        rs = groups[g]; counts = {c: sum(1 for r in rs if r["outcome_class"] == c) for c in CLASSES}
        clo = [f(r["closest_m"]) for r in rs if f(r["closest_m"]) is not None]
        notes = "; ".join(f"{r['run'][:3]}: {r['outcome_note']}" for r in rs if r["outcome_class"] == "other")
        out_rows.append([g[0], g[1], g[2], g[3], g[4], len(rs)] + [counts[c] for c in CLASSES] + [f"{min(clo):.3f}", f"{max(clo):.3f}", notes])
    print("\n" + "\t".join(hdr))
    for o in out_rows:
        print("\t".join(str(x) for x in o))
    tot = {s: {c: sum(1 for r in kept if r["outcome_class"] == c and (r["sess"] == "s5a") == (s == "5a")) for c in CLASSES} for s in ("pooled", "5a")}
    print("\ntotals:", tot)
    if a.out:
        if os.path.exists(a.out):
            sys.exit(f"outcome_classes: REFUSE: {a.out} exists")
        os.makedirs(a.out)
        with open(os.path.join(a.out, "classes_by_config.tsv"), "w") as o:
            o.write(f"# 16D outcome classes per configuration and loss window (outcome_classes.py; 16F: ground contact class, DEVIATIONS 17.80, precedence over every other class; 16G: minimum z <= 0.5 m, 17.80 f); set aside: {[(r['sess'], r['run']) for r in dropped]}; the 1.0 m line fixed before counting; band [{lo_b}, {hi_b}] m holds {len(band)} runs\n")
            o.write("\t".join(hdr) + "\n")
            for r_ in out_rows:
                o.write("\t".join(str(x) for x in r_) + "\n")
        with open(os.path.join(a.out, "classes_per_run.tsv"), "w") as o:
            o.write("# 16D outcome class per run (outcome_classes.py; 16F columns min_z_m, t_ground_s)\n"); cols = ["sess", "run", "key", "model", "backend", "W_s", "role", "locked_on", "reached", "final_state", "closest_m", "final_m", "closest_in_window", "final_in_window"] + (["min_z_m", "t_ground_s"] if "min_z_m" in rows[0] else []) + ["outcome_class", "outcome_note"]
            o.write("\t".join(cols) + "\n")
            for r in rows:
                o.write("\t".join(r[c] for c in cols) + ("\tSET ASIDE" if excluded(r) else "") + "\n")
        print("wrote classes_by_config.tsv and classes_per_run.tsv in", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
