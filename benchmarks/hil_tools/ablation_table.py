#!/usr/bin/env python3
"""ablation_table.py (16D, class A, read-only): DRAFT loss-window ablation table per configuration and window from
run_outcomes.py's per_run_outcomes.tsv (the 5a session; never pooled into Tables 13 or 14). Columns: model, backend, window,
n valid runs, lock-on k/n, reach k/n, the step 3 outcome classes (counts; 16F: with the ground contact class of DEVIATIONS 17.80, which
takes precedence over every other class), closest approach (min, median, max), final distance
to the frozen target (min to max over the valid runs; and over the reached runs), n_lost runs, det_hz mean, notes. A run named
with --rig-fault is listed in a footer with its reason and is not counted (its cell's n is the valid count). Final distances
carry the frozen target's along-axis uncertainty (REPORT_16A); the header says so.
Usage: python3 ablation_table.py <per_run_outcomes.tsv> --sess s5a [--rig-fault <sess>:<run prefix>:<reason> ...]
       [--note <sess>:<run prefix>:<text>] [--note ...] [--axis-uncertainty "+0.1 to +0.3"] --out <dir>
       (16F: repeated --note and --rig-fault flags are all kept; before, a repeated flag kept only its last occurrence)
       (writes DRAFT_ablation_table.tsv and DRAFT_ablation_table.md)"""
import argparse, csv, os, statistics, sys
CLASSES = ["reached cleanly", "reached after an overshoot", "overshot and lost the sign", "ground contact", "arrived in the window without REACHED", "never locked on", "other"]
SHORT = {"reached cleanly": "reached cleanly", "reached after an overshoot": "reached after overshoot", "overshot and lost the sign": "overshot, lost",
         "ground contact": "ground contact", "arrived in the window without REACHED": "arrived, no REACHED", "never locked on": "no lock-on", "other": "other"}
ORDER = ["yolo26n", "yolo26s", "yolo26m", "yolov8n", "yolov8s", "yolov8m", "yolov11n", "yolov11s", "yolov11m", "centernet_resnet_v1_18"]
DISPLAY = {"yolo26n": "YOLO26n", "yolo26s": "YOLO26s", "yolo26m": "YOLO26m", "yolov8n": "YOLOv8n", "yolov8s": "YOLOv8s", "yolov8m": "YOLOv8m",
           "yolov11n": "YOLOv11n", "yolov11s": "YOLOv11s", "yolov11m": "YOLOv11m", "centernet_resnet_v1_18": "CenterNet-R18"}


def rows_of(path):
    return [r for r in csv.DictReader((l for l in open(path) if not l.startswith("#")), delimiter="\t")]


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("tsv"); ap.add_argument("--sess", default="s5a")
    ap.add_argument("--rig-fault", action="extend", nargs="+", default=[], help="<sess>:<run prefix>:<reason>; the flag may be repeated (16F: every occurrence kept)")
    ap.add_argument("--axis-uncertainty", default="+0.1 to +0.3"); ap.add_argument("--out", required=True)
    ap.add_argument("--note", action="extend", nargs="+", default=[], help="<sess>:<run prefix>:<text> appended to the notes of that run's row (a finding about one run); the flag may be repeated (16F: every occurrence kept)")
    a = ap.parse_args()
    if os.path.exists(a.out):
        sys.exit(f"ablation_table: REFUSE: {a.out} exists")
    os.makedirs(a.out)
    rows = [r for r in rows_of(a.tsv) if r["sess"] == a.sess]
    faults = {}
    for e in a.rig_fault:
        sess, pref, reason = e.split(":", 2)
        for r in rows:
            if r["sess"] == sess and r["run"].startswith(pref):
                faults[r["run"]] = reason
    valid = [r for r in rows if r["run"] not in faults]
    unknown = sorted({r["outcome_class"] for r in rows} - set(CLASSES))
    if unknown:
        sys.exit(f"ablation_table: REFUSE: class names not in CLASSES: {unknown}")
    groups = {}
    for r in valid:
        groups.setdefault((r["role"] == "sentinel", f(r["W_s"]), ORDER.index(r["model"]) if r["model"] in ORDER else 99, r["model"], r["backend"]), []).append(r)
    hdr = ["model", "backend", "window_s", "n_valid", "lock_on", "reach", "outcome_classes", "closest_min_m", "closest_med_m", "closest_max_m",
           "final_min_m", "final_max_m", "final_reached_min_m", "final_reached_max_m", "n_lost_runs", "det_hz_mean", "notes"]
    out = []
    for g in sorted(groups):
        rs = groups[g]; sen, W, _, model, bk = g
        cls = [r["outcome_class"] for r in rs]
        oc = "; ".join(f"{cls.count(c)} {SHORT[c]}" for c in CLASSES if cls.count(c))
        clo = [f(r["closest_m"]) for r in rs]; fin = [f(r["final_m"]) for r in rs]; finr = [f(r["final_m"]) for r in rs if r["reached"] == "True"]
        notes = []
        if sen:
            notes.append("sentinel (default window, not an ablation row)")
        others = [f"{r['run'][:3]}: {r['outcome_note']}" for r in rs if r["outcome_class"] == "other"]
        notes += others
        for e in a.note:
            sess, pref, text = e.split(":", 2)
            if any(r["sess"] == sess and r["run"].startswith(pref) for r in rs):
                notes.append(f"{pref}: {text}")
        fx = [r for r in rows if r["run"] in faults and (r["model"], r["backend"], f(r["W_s"])) == (model, bk, W)]
        if fx:
            notes.append("set aside as a rig fault, not counted: " + "; ".join(f"run {r['run'][:3]} ({faults[r['run']]})" for r in fx) + f"; planned n {len(rs) + len(fx)}")
        out.append([model, bk, f"{W:.1f}", len(rs), f"{sum(1 for r in rs if r['locked_on'] == 'True')}/{len(rs)}", f"{sum(1 for r in rs if r['reached'] == 'True')}/{len(rs)}", oc,
                    f"{min(clo):.3f}", f"{statistics.median(clo):.3f}", f"{max(clo):.3f}", f"{min(fin):.3f}", f"{max(fin):.3f}",
                    f"{min(finr):.3f}" if finr else "", f"{max(finr):.3f}" if finr else "", sum(1 for r in rs if f(r["n_lost"]) and f(r["n_lost"]) > 0),
                    f"{statistics.fmean(f(r['det_hz']) for r in rs):.3f}", "; ".join(notes)])
    head = (f"# DRAFT ablation table (16D ablation_table.py over per_run_outcomes.tsv, session {a.sess}): per model, backend and loss window; the sentinel listed; "
            f"never pooled into Tables 13 or 14; closest approach and final distance are horizontal distances from the drone origin to the frozen target and carry its "
            f"along-axis uncertainty of {a.axis_uncertainty} m alike (REPORT_16A); outcome classes of EXPECTED_16D (the 1.0 m line) with the 16F ground contact class (minimum z after resume <= 0.01 m, DEVIATIONS 17.80, precedence over every other class); rig faults listed in the footer, not counted")
    with open(os.path.join(a.out, "DRAFT_ablation_table.tsv"), "w") as o:
        o.write(head + "\n"); o.write("\t".join(hdr) + "\n")
        for r_ in out:
            o.write("\t".join(str(x) for x in r_) + "\n")
        for run, reason in faults.items():
            r = next(x for x in rows if x["run"] == run)
            o.write(f"# rig fault, not counted: {a.sess} run {run} ({r['model']} {r['backend']}, W {f(r['W_s']):.1f} s): {reason}\n")
    with open(os.path.join(a.out, "DRAFT_ablation_table.md"), "w") as o:
        o.write("DRAFT ablation table (16D, 16F classes). " + head[2:] + "\n\n")
        o.write("| model, backend | W s | n | lock-on | reach | outcome classes | closest m (min, median, max) | final m (min to max; reached runs) | n_lost runs | det_hz | notes |\n|---|---|---|---|---|---|---|---|---|---|---|\n")
        for r_ in out:
            o.write(f"| {DISPLAY.get(r_[0], r_[0])} {r_[1].upper()} | {r_[2]} | {r_[3]} | {r_[4]} | {r_[5]} | {r_[6]} | {r_[7]}, {r_[8]}, {r_[9]} | {r_[10]} to {r_[11]}" + (f"; {r_[12]} to {r_[13]}" if r_[12] else "") + f" | {r_[14]} | {r_[15]} | {r_[16]} |\n")
        for run, reason in faults.items():
            r = next(x for x in rows if x["run"] == run)
            o.write(f"\nRig fault, not counted: run {run[:3]} ({DISPLAY.get(r['model'], r['model'])} {r['backend'].upper()}, W {f(r['W_s']):.1f} s): {reason}.\n")
    print(f"wrote DRAFT_ablation_table.tsv and .md in {a.out}: {len(out)} rows, {len(valid)} valid runs, {len(faults)} rig fault(s)")
    for r_ in out:
        print("\t".join(str(x) for x in r_))
    return 0


if __name__ == "__main__":
    sys.exit(main())
