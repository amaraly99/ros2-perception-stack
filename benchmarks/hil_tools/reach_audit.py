#!/usr/bin/env python3
"""reach_audit.py (16D, class A, read-only): reach declared against arrived, from run_outcomes.py's per_run_outcomes.tsv and
transitions.jsonl. For the runs selected (default: every 5a lw row that did not reach, plus the reached CenterNet rows) it prints
per run: the final FSM state; every state change after lock-on with its time after resume and reason (a message without a
target box, a box rejected as a jump above 0.30 of the image, or the watchdog after a gap above W); the closest approach from
the drone origin and from the camera against the hold ratio's range (reach needs the smoothed box height ratio >= hold_bbox_ratio,
so the face within fy x 0.914 / (hold_bbox_ratio x 480) of the camera) and the vx-zero range; whether any message was close and
centred and the longest such streak against reach_consec_ticks; the last target box before the final loss (where it was captured
and delivered, the drone altitude, its smoothed ratio and ey) and what the camera saw at the capture of the first message after
it (the projected target position, the sign's apparent height, the altitude); the final distance against the provisional
window. The reach rule as coded is printed first with its constants from the run params.
Usage: python3 reach_audit.py <per_run_outcomes.tsv> <transitions.jsonl> [--select all5a|nonreach5a|<sess>:<run prefix> ...] [--out <tsv>]"""
import argparse, csv, json, os, sys


def rows_of(path):
    return [r for r in csv.DictReader((l for l in open(path) if not l.startswith("#")), delimiter="\t")]


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def fmt(v, d=2):
    x = f(v); return "" if x is None else f"{x:.{d}f}"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("tsv"); ap.add_argument("jsonl"); ap.add_argument("--select", nargs="*", default=["nonreach5a"])
    ap.add_argument("--out", default=None); a = ap.parse_args()
    rows = rows_of(a.tsv); tj = {(j["sess"], j["run"]): j for j in (json.loads(l) for l in open(a.jsonl))}
    def selected(r):
        for s in a.select:
            if s == "all5a" and r["sess"] == "s5a":
                return True
            if s == "nonreach5a" and r["sess"] == "s5a" and r["key"].startswith("lw") and (r["reached"] != "True" or r["model"].startswith("centernet")):
                return True
            if ":" in s and r["sess"] == s.split(":")[0] and r["run"].startswith(s.split(":")[1]):
                return True
        return False
    sel = [r for r in rows if selected(r)]
    r0 = sel[0]
    print("# reach rule as coded (servo_fsm_node.cpp, update_state_on_detection, case APPROACHING; identical in the pinned and the ablation worktree):")
    print(f"#   close = smoothed box height / image height >= hold_bbox_ratio {r0['hold_ratio']}; centred = |ex| < {r0['center_tol']} and |ey| < {r0['center_tol']}")
    print(f"#   (ex, ey from the {r0['smoothing']}-message mean of the box centre); REACHED after reach_consec_ticks = {r0['reach_ticks_needed']} consecutive detection callbacks")
    print(f"#   with a box that are close and centred; the counter resets on a box that is not, on REACQUIRE -> APPROACHING, and the state leaves APPROACHING")
    print(f"#   on any message without a target box or a box jump above 0.30 of the image (handle_no_detection) or after W s without a message (watchdog_tick)")
    print(f"#   close in range terms: face within fy x 0.914 / ({r0['hold_ratio']} x 480) = {fmt(r0['hold_face_range_m'], 3)} m of the camera (origin range + 0.2 m); vx = 0 at {fmt(r0['vx_zero_face_range_m'], 3)} m")
    cols = ["sess", "run", "model", "W_s", "final_state", "reached", "t_reach_sim_s", "closest_m", "closest_face_m", "met_distance_cond", "max_smoothed_ratio", "met_size_cond",
            "n_close_centered_msgs", "max_close_centered_streak", "n_losses_after_lock", "final_loss_s", "final_loss_reason", "final_loss_since_last_msg_s",
            "range_at_final_loss_m", "z_at_final_loss_m", "last_box_capture_range_m", "last_box_delivery_range_m", "last_box_z_m", "last_box_ratio", "last_box_ey",
            "after_last_box_range_m", "after_last_box_z_m", "after_last_box_v", "after_last_box_face_px", "after_last_box_what", "z_at_closest_m",
            "target_in_view_at_closest", "target_v_px_at_closest", "final_m", "final_in_window", "closest_in_window"]
    print("\n" + "\t".join(cols))
    for r in sel:
        print("\t".join(r[c] for c in cols))
    print("\n# transitions after lock-on, per run (t after resume in s)")
    for r in sel:
        j = tj[(r["sess"], r["run"])]; tl = f(r["lock_log_s"])
        parts = []
        for e in j["transitions"]:
            if tl is not None and e["t_s"] < tl:
                continue
            s = f"{e['t_s']:+.2f} {e['from']}->{e['to']}"
            if "reason" in e:
                s += f" [{e['reason']}; {e.get('since_last_det_msg_s')} s since the last message; drone at {fmt(e.get('range_at_loss_m'))} m, z {fmt(e.get('z_at_loss_m'))}]"
            if "timed_out_s" in e:
                s += f" [timed out {e['timed_out_s']} s]"
            if "consec" in e:
                s += f" [consec {e['consec']} ratio {e['ratio']} ex {e['ex']} ey {e['ey']}]"
            parts.append(s)
        print(f"{r['run']} (W {r['W_s']} s): " + " | ".join(parts))
    # group summary
    def fam(m):
        return "centernet" if m.startswith("centernet") else m[-1]
    print("\n# group summary (min to max over the runs of the group)")
    groups = {}
    for r in sel:
        groups.setdefault((fam(r["model"]), r["W_s"], r["reached"]), []).append(r)
    for g in sorted(groups):
        rs = groups[g]
        def rng(c, d=2):
            v = [f(r[c]) for r in rs if f(r[c]) is not None]; return f"{min(v):.{d}f} to {max(v):.{d}f}" if v else "-"
        print(f"family {g[0]} W {g[1]} reached {g[2]}: n {len(rs)}; closest {rng('closest_m')} m (face {rng('closest_face_m')}); met distance cond "
              f"{sum(1 for r in rs if r['met_distance_cond'] == 'True')}/{len(rs)}; max ratio {rng('max_smoothed_ratio')}; close-and-centred msgs {rng('n_close_centered_msgs', 0)}; "
              f"max streak {rng('max_close_centered_streak', 0)}; final loss reason: {sorted({r['final_loss_reason'] for r in rs})}; since last msg {rng('final_loss_since_last_msg_s')} s; "
              f"range at final loss {rng('range_at_final_loss_m')} m, z {rng('z_at_final_loss_m')} m; last box captured at {rng('last_box_capture_range_m')} m, delivered at "
              f"{rng('last_box_delivery_range_m')} m, z {rng('last_box_z_m')}, ratio {rng('last_box_ratio')}, ey {rng('last_box_ey')}; next frame: {sorted({r['after_last_box_what'] for r in rs})}, "
              f"range {rng('after_last_box_range_m')} m, z {rng('after_last_box_z_m')} m, v {rng('after_last_box_v', 0)} px; at the closest point z {rng('z_at_closest_m')} m, "
              f"target v {rng('target_v_px_at_closest', 0)} px, in view {sum(1 for r in rs if r['target_in_view_at_closest'] == 'True')}/{len(rs)}; "
              f"closest in window {sum(1 for r in rs if r['closest_in_window'] == 'True')}/{len(rs)}, final in window {sum(1 for r in rs if r['final_in_window'] == 'True')}/{len(rs)}")
    if a.out:
        if os.path.exists(a.out):
            sys.exit(f"REFUSE: {a.out} exists (no overwrite)")
        with open(a.out, "w") as o:
            o.write("# 16D reach declared against arrived (reach_audit.py over per_run_outcomes.tsv)\n"); o.write("\t".join(cols) + "\n")
            for r in sel:
                o.write("\t".join(r[c] for c in cols) + "\n")
        print("wrote", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
