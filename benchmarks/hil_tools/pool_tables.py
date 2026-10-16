#!/usr/bin/env python3
"""pool_tables.py (16A, class A: outside the guarded run code): DRAFT Tables 13 and 14 of the fx 554 consolidation pass, pooled
per configuration over the pooled batches, from the run folders on /mnt/usb and the one-scorer re-score of every run.

Table 14 (DEVIATIONS 17.66 a; the manuscript's tab:detector_inloop_acc columns): per configuration (model, backend; sentinel runs
excluded, "not counted in a row"), the approach-phase detections with the reference in view, pooled over the row's runs and over
the four fx 554 bins 18-20, 12-18, 6-12, 2-6 m (horizontal camera ground range at capture, the scorer's own binning):
n, centroid displacement median at capture and at delivery (px), Delta = their difference, Ref. motion (median px the
projected reference moves between the capture and the delivery pose, over detections with the reference in view at both
instants; no box involved), miss per processed frame and per camera frame, both over camera frames with the target IN VIEW at
capture (MISS_RATE = in_view: a skipped camera frame counts as a miss only if the target was in view at its capture time,
located through cam_stamps t_shm_write_mono_ns + the clock offset). Per bin: pooled messages and runs; the floor of 10 pooled
messages prints "below floor (n messages, k runs)" and never a number. Per-detection rows are re-derived with the scorer's own
functions (project, PoseTrack, drop_known_signs, the 17.69 yaw bound) in the same order as score(); the tool then calls score()
itself and REFUSES a run whose score() output differs from the one-scorer re-score given as --rescored (so the rows are the
scorer's rows), and refuses a run whose row-derived per-segment summaries differ from score()'s. Miss rates are over the whole
run (the caption), n over the approach phase in the bins. Processed in-view frames are counted over the whole bag while skipped
frames can only be counted inside the cam_stamps/telemetry overlap window (as the scorer's own frames block); the mismatch is
under 0.3 points on every run checked. Row-level medians print from any pooled count; the floor of 17.66 a is per bin.
Table 13 (tab:detector_e2e columns plus the fixed decisions): per configuration, runs, reached k/n, reach sim time mean and
std over reached runs, final distance to the frozen target (mean, std, min, max over reached runs), closest approach, n_lost,
det_hz, cmd_vel_hz and det latency (det_path mean) mean and std over all runs; SSD rows as the dropped decision of 17.74.
Every output is labelled DRAFT. Usage:
  ~/venvs/mcap/bin/python benchmarks/hil_tools/pool_tables.py --batches <list.tsv> --rescored <prescore dir> --out <dir>
      [--outcomes <per_run_outcomes.tsv>] [--sensitivity <sess>:<run prefix> ...] [--axis-uncertainty "+0.1 to +0.3"] [--cache <dir>]
<list.tsv>: lines "<session> <batch folder>"; <prescore dir>: the re-score tree (made with invariance/rescore.sh) with
<batch basename>/<run>/<label>_run1_inloop.json and _check.json. <out dir> must not exist (no overwrite).
16D additions (prompt 16D decisions, all presentation; no number of 16A changes): --outcomes adds the step 3 outcome class of
run_outcomes.py as a Table 13 column (counts per class) and to per_run.tsv, and supplies the lock-on count; a Table 14 row whose
runs never locked on and have no approach message prints "no approach frames (0 of n runs locked on, see Table 13)" in place
of n and the medians (with --outcomes the lock-on count comes from the stack log; messages taken during a no-lock-on run's
search motion are set aside and reported on stdout; without --outcomes the text needs zero such messages too); --sensitivity
writes, under every Table 13 and Table 14 row (and bin line) that contains a listed run,
the same row recomputed without those runs, labelled "(without ...)"; --axis-uncertainty writes that text next to every final
distance of Table 13 (the frozen target's along-axis uncertainty, REPORT_16A); --cache reuses an existing read_bag cache
directory instead of <out>/.cache (on a cache miss read_bag writes a new entry into it, so point it at a copy, never at an
archived output folder).
"""
import argparse, bisect, csv, glob, importlib.util, json, math, os, statistics, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
spec = importlib.util.spec_from_file_location("ia", os.path.join(ROOT, "benchmarks", "inloop_accuracy.py"))
ia = importlib.util.module_from_spec(spec); spec.loader.exec_module(ia)
GEOM = json.load(open(os.path.join(ROOT, "benchmarks", "hil_runs", "config", "sim_geometry.json")))
PROTO = json.load(open(os.path.join(ROOT, "benchmarks", "hil_runs", "config", "protocol_fx554.json")))
FX = float(GEOM["fx"]); T = GEOM["target_xyz"]; SIGNS = GEOM.get("nontarget_signs_xyz") or []
BINS = [tuple(b) for b in PROTO["t14_bins_m"]]
EDGES = sorted({e for b in BINS for e in b})
FLOOR = ia.MIN_N_MEDIAN
DISPLAY = {"yolo26n": "YOLO26n", "yolo26s": "YOLO26s", "yolo26m": "YOLO26m", "yolov8n": "YOLOv8n", "yolov8s": "YOLOv8s", "yolov8m": "YOLOv8m",
           "yolov11n": "YOLOv11n", "yolov11s": "YOLOv11s", "yolov11m": "YOLOv11m", "centernet_resnet_v1_18": "CenterNet-R18", "ssd_mobilenet_v2": "SSD-MobileNetV2"}
ORDER = ["yolo26n", "yolo26s", "yolo26m", "yolov8n", "yolov8s", "yolov8m", "yolov11n", "yolov11s", "yolov11m", "centernet_resnet_v1_18", "ssd_mobilenet_v2"]


def med(v):
    return statistics.median(v) if v else None


def fmt(v, d=2):
    return "" if v is None else f"{v:.{d}f}"


def setup_scorer():
    ia.FX = ia.FY = FX; ia.CAM_PITCH = math.radians(-5.0)
    ia.RANGE_BINS = [(int(a) if float(a).is_integer() else a, int(b) if float(b).is_integer() else b) for a, b in zip(EDGES, EDGES[1:])]


def run_rows(rd, label, cache_dir):
    """Per-detection rows of one run, the scorer's own loop (score() at HEAD) with Ref. motion and the camera-frame visibility added.
    Returns rows, the scorer's own score() dict (for the self-check), and the camera-frame miss counts (in view only)."""
    co = int(json.load(open(os.path.join(rd, "stack", f"{label}_clock_offset.json")))["realtime_minus_monotonic_ns"])
    bag = glob.glob(os.path.join(rd, "approach_bag", "*.mcap"))[0]
    poses, dets, _ = ia.read_bag(bag, cache_dir=cache_dir)
    caps = ia.load_capture_times(os.path.join(rd, "stack", f"{label}_run1_telemetry.csv"))
    gate = ia.default_rms_gate(FX)
    sc = ia.score(poses, dets, T, caps, co, False, SIGNS, gate)
    track = ia.PoseTrack(poses)
    rows = []
    for rec in dets:
        pose_del = track.at(rec["log_ns"])
        if pose_del is None:
            continue
        uv_del = ia.project(pose_del, T)
        cap_bag_ns = rec["stamp_ns"] + co
        pose_cap = track.at(cap_bag_ns)
        if pose_cap is not None:
            yr = track.yaw_rate_at(cap_bag_ns)
            if yr is not None and abs(yr) > ia.YAW_RATE_BOUND_RAD_S:
                continue
        uv_cap = ia.project(pose_cap, T) if pose_cap else None
        speed = track.speed_at(cap_bag_ns)
        visible = ia.in_view(uv_cap)
        hits = [d for d in rec["dets"] if d[0] == ia.TARGET_CLASS]
        hits, _k = ia.drop_known_signs(hits, pose_cap, T, SIGNS, gate)
        best = max(hits, key=lambda d: d[1]) if hits else None
        err_cap = err_del = None
        if best is not None:
            if visible:
                err_cap = math.hypot(uv_cap[0] - best[2], uv_cap[1] - best[3])
            if ia.in_view(uv_del):
                err_del = math.hypot(uv_del[0] - best[2], uv_del[1] - best[3])
        refmo = math.hypot(uv_del[0] - uv_cap[0], uv_del[1] - uv_cap[1]) if (visible and ia.in_view(uv_del)) else None
        rows.append({"range": uv_cap[2] if uv_cap else None, "bin": ia._bin_of(uv_cap[2]) if uv_cap else None, "speed": speed,
                     "visible": visible, "no_detection": visible and best is None, "err_cap": err_cap, "err_del": err_del,
                     "ref_motion": refmo, "stamp_ns": rec["stamp_ns"]})
    # camera frames: every frame the camera produced (cam_stamps), processed or skipped, in view at capture or not (MISS_RATE = in_view)
    cam = []
    with open(os.path.join(rd, "stack", f"{label}_run1_cam_stamps.csv")) as f:
        for r in csv.DictReader(f):
            try:
                cam.append(int(r["t_shm_write_mono_ns"]))
            except (KeyError, ValueError):
                pass
    processed = {c for _, c in caps}
    lo = max(min(cam), min(processed)) if cam and processed else None
    hi = min(max(cam), max(processed)) if cam and processed else None
    cam_w = [t for t in cam if lo is not None and lo <= t <= hi]
    skipped_inview = skipped_total = 0
    for t in cam_w:
        if t in processed:
            continue
        skipped_total += 1
        p = track.at(t + co)
        if p is not None:
            yr = track.yaw_rate_at(t + co)
            if yr is not None and abs(yr) > ia.YAW_RATE_BOUND_RAD_S:
                continue
            if ia.in_view(ia.project(p, T)):
                skipped_inview += 1
    vis_rows = [r for r in rows if r["visible"]]
    cam_counts = {"processed_in_view": len(vis_rows), "processed_in_view_missed": sum(1 for r in vis_rows if r["no_detection"]),
                  "skipped_in_view": skipped_inview, "skipped_total": skipped_total, "camera_frames_window": len(cam_w)}
    return rows, sc, cam_counts


def summarise_like_scorer(rows):
    """The scorer's segment/bin summaries from the tool's rows (self-check 2: must equal score()'s)."""
    out = {}
    for name, sel in (("approach", lambda r: r["speed"] is not None and r["speed"] >= ia.PARKED_SPEED),
                      ("parked", lambda r: r["speed"] is not None and r["speed"] < ia.PARKED_SPEED)):
        seg = [r for r in rows if sel(r)]
        if not seg:
            continue
        blk = {"overall": _summ(seg), "bins": {}}
        for lo, hi in ia.RANGE_BINS:
            key = f"{lo}-{hi}m"; b = [r for r in seg if r["bin"] == key]
            s = _summ(b)
            if s and not s.get("below_reporting_floor"):
                blk["bins"][key] = s
            elif b:
                blk["bins"][key] = {"n_messages": len(b), "below_reporting_floor": True}
        out[name] = blk
    return out


def _summ(rs):
    if not rs:
        return None
    cap = [r["err_cap"] for r in rs if r["err_cap"] is not None]; dlv = [r["err_del"] for r in rs if r["err_del"] is not None]
    vis = [r for r in rs if r["visible"]]
    out = {"n_messages": len(rs), "n_target_visible": len(vis), "centroid_err_at_capture_median": med(cap), "centroid_err_at_delivery_median": med(dlv)}
    if vis:
        out["miss_rate_per_processed_frame"] = sum(1 for r in vis if r["no_detection"]) / len(vis)
    if len(vis) < FLOOR:
        out["below_reporting_floor"] = True
    return out


def scorer_view(sc):
    """The same fields taken from score()'s output, for the comparison."""
    out = {}
    for name, blk in sc["segments"].items():
        o = blk["overall"]; b2 = {}
        for key, s in blk["bins"].items():
            if s.get("below_reporting_floor") and "n_target_visible" not in s:
                b2[key] = {"n_messages": s["n_messages"], "below_reporting_floor": True}
            else:
                b2[key] = _view(s)
        out[name] = {"overall": _view(o), "bins": b2}
    return out


def _view(s):
    o = {"n_messages": s["n_messages"], "n_target_visible": s["n_target_visible"],
         "centroid_err_at_capture_median": (s["centroid_err_at_capture"] or {}).get("median_px"),
         "centroid_err_at_delivery_median": (s["centroid_err_at_delivery"] or {}).get("median_px")}
    if "miss_rate_per_processed_frame" in s:
        o["miss_rate_per_processed_frame"] = s["miss_rate_per_processed_frame"]
    if s.get("below_reporting_floor"):
        o["below_reporting_floor"] = True
    return o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batches", required=True); ap.add_argument("--rescored", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--usb", default="/mnt/usb/hil_runs")
    ap.add_argument("--outcomes", default=None, help="16D: per_run_outcomes.tsv of run_outcomes.py (outcome class and lock-on per run)")
    ap.add_argument("--sensitivity", nargs="*", default=[], help="16D: <sess>:<run dir prefix> runs; rows containing them are repeated without them")
    ap.add_argument("--axis-uncertainty", default=None, help="16D: text written next to every final distance, e.g. '+0.1 to +0.3'")
    ap.add_argument("--cache", default=None, help="16D: an existing read_bag cache directory to reuse (default <out>/.cache)")
    a = ap.parse_args()
    setup_scorer()
    if os.path.exists(a.out):
        sys.exit(f"pool_tables: REFUSE: {a.out} exists (every run writes into a new folder)")
    os.makedirs(a.out); cache = a.cache or os.path.join(a.out, ".cache")
    outcomes = {}
    if a.outcomes:
        for r in csv.DictReader((l for l in open(a.outcomes) if not l.startswith("#")), delimiter="\t"):
            outcomes[(r["sess"], r["run"])] = {"class": r["outcome_class"], "locked_on": r["locked_on"] == "True"}
    def sens_hit(p):
        return any(p["sess"] == e.split(":")[0] and p["run"].startswith(e.split(":")[1]) for e in a.sensitivity)
    def sens_label(rs):
        return "without " + ", ".join(f"{p['sess']} {p['run'][:3]}" for p in rs if sens_hit(p))
    runs = []
    for line in open(a.batches):
        if not line.strip() or line.startswith("#"):
            continue
        sess, folder = line.split()[:2]
        for r in csv.DictReader(open(os.path.join(a.usb, folder, "batch_state.tsv")), delimiter="\t"):
            rd = os.path.join(a.usb, folder, r["dir"]); label = "level2_" + r["dir"][4:]
            runs.append((sess, folder, r, rd, label))
    print(f"{len(runs)} runs listed")
    per_run = []; refused = []
    selfcheck = open(os.path.join(a.out, "selfcheck.tsv"), "w")
    selfcheck.write("sess\tbatch\trun\tscore_equals_rescored\trows_equal_score\tn_rows\n")
    for sess, folder, r, rd, label in runs:
        rows, sc, camc = run_rows(rd, label, cache)
        stored_p = os.path.join(a.rescored, os.path.basename(folder), r["dir"], label + "_run1_inloop.json")
        stored = json.load(open(stored_p))
        keep = {k: v for k, v in stored.items() if k not in ("geometry", "label", "target", "capture_times_from_telemetry", "capture_timing", "frames", "miss_rate_per_camera_frame", "miss_rate_per_processed_frame")}
        mine = {k: v for k, v in sc.items() if k not in ("geometry",)}
        eq1 = keep == mine
        eq2 = summarise_like_scorer(rows) == scorer_view(sc)
        selfcheck.write(f"{sess}\t{folder}\t{r['dir']}\t{eq1}\t{eq2}\t{len(rows)}\n")
        if not (eq1 and eq2):
            refused.append((folder, r["dir"], eq1, eq2)); continue
        cj = json.load(open(os.path.join(a.rescored, os.path.basename(folder), r["dir"], label + "_run1_check.json")))["info"]
        st = json.load(open(glob.glob(os.path.join(rd, "stack", "*_run1_stats.json"))[0]))
        sm = json.load(open(glob.glob(os.path.join(rd, "stack", "*_summary.json"))[0]))
        ms = json.load(open(glob.glob(os.path.join(rd, "stack", "*_run1_mission.json"))[0]))
        per_run.append({"sess": sess, "batch": folder, "run": r["dir"], "n": r["n"], "key": r["key"], "model": r["model"], "backend": r["backend"], "role": r["role"],
                        "rows": rows, "cam": camc, "det_hz": st["det"]["hz_mean"], "cmd_vel_hz": st["cmd_vel"]["hz_mean"],
                        "det_latency_ms": sm["detector_latency_ms"]["det_path"]["mean"], "reached": bool(ms.get("reached")),
                        "t_reach": ms.get("time_to_reached_sim_s"), "final_m": ms.get("final_dist_frozen_sign_m"), "closest_m": cj.get("closest_dist_frozen_sign_m"),
                        "n_lost": ms.get("n_lost"), "n_reacquire": ms.get("n_reacquire"), "final_state": ms.get("final_state"),
                        "gate_px": (cj.get("geometry_gate") or {}).get("frozen_target_reprojection_rms_px"),
                        "outcome": (outcomes.get((sess, r["dir"])) or {}).get("class"), "locked_on": (outcomes.get((sess, r["dir"])) or {}).get("locked_on")})
        print(f"  {sess} {r['dir']}: rows {len(rows)}, self-check {eq1} {eq2}")
    selfcheck.close()
    print(f"self-check: {len(per_run)} runs accepted, {len(refused)} refused: {refused}")
    # ---- Table 14 ----
    cfgs = sorted({(p["model"], p["backend"]) for p in per_run if p["role"] != "sentinel"}, key=lambda c: (ORDER.index(c[0]) if c[0] in ORDER else 99, c[1]))
    t14 = open(os.path.join(a.out, "DRAFT_table14.tsv"), "w"); t14b = open(os.path.join(a.out, "DRAFT_table14_bins.tsv"), "w")
    t14.write("# DRAFT Table 14 (16A pool_tables.py, 16D rows): n and the medians over the approach phase in the bins " + ",".join(f"{lo}-{hi}" for lo, hi in BINS) + " m pooled; the miss columns over the WHOLE RUN, camera frames with the target in view at capture (denominators in per_run.tsv); row medians print from any pooled count (the 17.66 floor is per bin)\n")
    t14.write("model\tbackend\truns\tn\tcap_med_px\tdel_med_px\tdelta_px\tref_motion_px\tmiss_proc_pct\tmiss_cam_pct\tn_detections_with_box\n")
    t14b.write("# DRAFT Table 14 per bin (16A): pooled messages with the reference in view and runs per bin; below floor printed as text\n")
    t14b.write("model\tbackend\tbin_m\tmessages\truns\tcap_med_px\tdel_med_px\tdelta_px\tref_motion_px\tstatus\n")
    tex = open(os.path.join(a.out, "DRAFT_table14_rows.tex"), "w"); tex.write("% DRAFT rows (16A pool_tables.py); columns: Model & Backend & Runs & n & at capture & at delivery & Delta & Ref. motion & Miss/proc. & Miss/cam.\n")
    def t14_row(model, bk, rs, name, texname):
        appr = [(p["run"], r) for p in rs for r in p["rows"] if r["speed"] is not None and r["speed"] >= ia.PARKED_SPEED and r["bin"] is not None]
        vis = [(run, r) for run, r in appr if r["visible"]]
        cap = [r["err_cap"] for _, r in vis if r["err_cap"] is not None]; dlv = [r["err_del"] for _, r in vis if r["err_del"] is not None]
        refm = [r["ref_motion"] for _, r in vis if r["ref_motion"] is not None]
        mc = {k: sum(p["cam"][k] for p in rs) for k in ("processed_in_view", "processed_in_view_missed", "skipped_in_view")}
        miss_proc = 100 * mc["processed_in_view_missed"] / mc["processed_in_view"] if mc["processed_in_view"] else None
        miss_cam = 100 * (mc["processed_in_view_missed"] + mc["skipped_in_view"]) / (mc["processed_in_view"] + mc["skipped_in_view"]) if (mc["processed_in_view"] + mc["skipped_in_view"]) else None
        mcap, mdel = med(cap), med(dlv)
        delta = (mdel - mcap) if (mcap is not None and mdel is not None) else None
        # 16D: a row whose runs never locked on and have no approach message prints the reason in place of n and the medians
        if outcomes:
            n_lock = sum(1 for p in rs if p["locked_on"])
        else:
            n_lock = sum(1 for p in rs if not (p["final_state"] == "SEARCHING" and not p["reached"] and not p["n_lost"] and not p["n_reacquire"]))
        if n_lock == 0 and (outcomes or not vis):
            # a row that never locked on made no approach; messages taken during the search motion (speed above the parked
            # threshold while yawing or strafing at the start distance) are set aside and counted in the log line
            txt = f"no approach frames (0 of {len(rs)} runs locked on, see Table 13)"
            if vis:
                print(f"  note: {name} {bk}: {len(vis)} in-view messages from {len({run for run, _ in vis})} run(s) during the search motion set aside (no lock-on)")
            t14.write("\t".join([name, bk, str(len(rs)), txt, "", "", "", "", fmt(miss_proc, 1), fmt(miss_cam, 1), str(len(cap))]) + "\n")
            tex.write(f"{texname} & {bk.upper()} & {len(rs)} & \\multicolumn{{5}}{{l}}{{{txt}}} & {fmt(miss_proc, 1)} & {fmt(miss_cam, 1)} \\\\\n")
        else:
            t14.write("\t".join([name, bk, str(len(rs)), str(len(vis)), fmt(mcap), fmt(mdel), fmt(delta), fmt(med(refm)), fmt(miss_proc, 1), fmt(miss_cam, 1), str(len(cap))]) + "\n")
            tex.write(f"{texname} & {bk.upper()} & {len(rs)} & {len(vis)} & {fmt(mcap)} & {fmt(mdel)} & {'' if delta is None else ('$' + ('+' if delta >= 0 else '') + f'{delta:.2f}$')} & {fmt(med(refm))} & {fmt(miss_proc, 1)} & {fmt(miss_cam, 1)} \\\\\n")
        for lo, hi in BINS:
            key = f"{lo}-{hi}m"
            bv = [(run, r) for run, r in vis if r["bin"] == key]
            k = len({run for run, _ in bv})
            if len(bv) < FLOOR:
                t14b.write("\t".join([name, bk, key, str(len(bv)), str(k), "", "", "", "", f"below floor ({len(bv)} messages, {k} runs)"]) + "\n"); continue
            c = [r["err_cap"] for _, r in bv if r["err_cap"] is not None]; d = [r["err_del"] for _, r in bv if r["err_del"] is not None]; m = [r["ref_motion"] for _, r in bv if r["ref_motion"] is not None]
            dd = (med(d) - med(c)) if (c and d) else None
            t14b.write("\t".join([name, bk, key, str(len(bv)), str(k), fmt(med(c)), fmt(med(d)), fmt(dd), fmt(med(m)), "ok"]) + "\n")
    for model, bk in cfgs:
        rs = [p for p in per_run if (p["model"], p["backend"]) == (model, bk) and p["role"] != "sentinel"]
        t14_row(model, bk, rs, model, DISPLAY.get(model, model))
        if any(sens_hit(p) for p in rs):   # 16D sensitivity line: the same row without the listed runs
            lab = sens_label(rs); t14_row(model, bk, [p for p in rs if not sens_hit(p)], f"{model} ({lab})", f"{DISPLAY.get(model, model)} ({lab})")
    t14.close(); t14b.close(); tex.close()
    # ---- Table 13 ----
    t13 = open(os.path.join(a.out, "DRAFT_table13.tsv"), "w")
    t13.write("# DRAFT Table 13 (16A pool_tables.py, 16D columns): per configuration over the pooled rows (sentinels listed separately); reach = sim time to REACHED over reached runs; final = horizontal distance to the frozen target (drone origin)"
              + (f"; every final distance carries the frozen target's along-axis uncertainty of {a.axis_uncertainty} m (REPORT_16A, DEVIATIONS 17.76 b iv), the same for every row" if a.axis_uncertainty else "")
              + ("; outcome classes from run_outcomes.py (16D step 3)" if outcomes else "") + (f"; sensitivity lines without {a.sensitivity}" if a.sensitivity else "") + "\n")
    t13.write("model\tbackend\truns\treached\tno_lockon\treach_s_mean\treach_s_std\tfinal_m_mean\tfinal_m_std\tfinal_m_min\tfinal_m_max\tfinal_axis_uncert_m\tclosest_m_min\tn_lost_runs\tdet_hz_mean\tdet_hz_std\tcmd_vel_hz_mean\tdet_lat_ms_mean\tdet_lat_ms_std\tgate_px_max\toutcome_classes\n")
    def ms_(v):
        return (statistics.fmean(v), statistics.stdev(v) if len(v) > 1 else 0.0) if v else (None, None)
    def t13_row(name, bk, rs):
        re_ = [p for p in rs if p["reached"]]
        nolock_h = sum(1 for p in rs if p["final_state"] == "SEARCHING" and not p["reached"] and not p["n_lost"] and not p["n_reacquire"])
        nolock = sum(1 for p in rs if not p["locked_on"]) if outcomes else nolock_h
        if outcomes and nolock != nolock_h:
            print(f"  note: {name} {bk}: lock-on from the outcomes file ({nolock} never locked on) differs from the mission heuristic ({nolock_h})")
        tm, ts = ms_([p["t_reach"] for p in re_ if p["t_reach"] is not None]); fm, fs = ms_([p["final_m"] for p in re_ if p["final_m"] is not None])
        dm, ds = ms_([p["det_hz"] for p in rs]); cm, _ = ms_([p["cmd_vel_hz"] for p in rs]); lm, ls = ms_([p["det_latency_ms"] for p in rs])
        fin = [p["final_m"] for p in re_ if p["final_m"] is not None]; clo = [p["closest_m"] for p in rs if p["closest_m"] is not None]
        if outcomes:
            cls = [p["outcome"] for p in rs if p["outcome"]]
            oc = "; ".join(f"{cls.count(c)} {c}" for c in ("reached cleanly", "reached after an overshoot", "overshot and lost the sign", "arrived in the window without REACHED", "never locked on", "other") if cls.count(c))
        else:
            oc = ""
        t13.write("\t".join([name, bk, str(len(rs)), f"{len(re_)}/{len(rs)}", str(nolock), fmt(tm), fmt(ts), fmt(fm, 3), fmt(fs, 3), fmt(min(fin), 3) if fin else "", fmt(max(fin), 3) if fin else "",
                             (a.axis_uncertainty or "") if fin else "",
                             fmt(min(clo), 3) if clo else "", str(sum(1 for p in rs if p["n_lost"])), fmt(dm, 3), fmt(ds, 3), fmt(cm, 3), fmt(lm, 1), fmt(ls, 1),
                             fmt(max(p["gate_px"] for p in rs if p["gate_px"] is not None), 1) if any(p["gate_px"] is not None for p in rs) else "", oc]) + "\n")
    for role_sel, tag in ((lambda p: p["role"] != "sentinel", ""), (lambda p: p["role"] == "sentinel", " (sentinels)")):
        for model, bk in sorted({(p["model"], p["backend"]) for p in per_run if role_sel(p)}, key=lambda c: (ORDER.index(c[0]) if c[0] in ORDER else 99, c[1])):
            rs = [p for p in per_run if (p["model"], p["backend"]) == (model, bk) and role_sel(p)]
            t13_row(model + tag, bk, rs)
            if any(sens_hit(p) for p in rs):   # 16D sensitivity line
                t13_row(f"{model}{tag} ({sens_label(rs)})", bk, [p for p in rs if not sens_hit(p)])
    t13.write("ssd_mobilenet_v2\tcpu/npu\t0\tdropped\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tno box on the byte-identical held render (17.74; session gate evidence)\t\n")
    t13.close()
    with open(os.path.join(a.out, "per_run.tsv"), "w") as f:
        f.write("# DRAFT per-run inputs of the DRAFT tables (16A pool_tables.py)\n")
        f.write("sess\tbatch\trun\tkey\tmodel\tbackend\trole\treached\tfinal_state\tt_reach_s\tfinal_m\tclosest_m\tn_lost\tdet_hz\tcmd_vel_hz\tdet_lat_ms\tgate_px\trows\tproc_in_view\tproc_in_view_missed\tskipped_in_view\tskipped_total\tcam_frames_window\toutcome_class\n")
        for p in per_run:
            c = p["cam"]
            f.write("\t".join(str(x) for x in [p["sess"], p["batch"], p["run"], p["key"], p["model"], p["backend"], p["role"], p["reached"], p["final_state"], p["t_reach"], p["final_m"], p["closest_m"], p["n_lost"],
                                             round(p["det_hz"], 3), round(p["cmd_vel_hz"], 3), round(p["det_latency_ms"], 1), None if p["gate_px"] is None else round(p["gate_px"], 2), len(p["rows"]),
                                             c["processed_in_view"], c["processed_in_view_missed"], c["skipped_in_view"], c["skipped_total"], c["camera_frames_window"], p["outcome"] or ""]) + "\n")
    print("wrote DRAFT_table13.tsv, DRAFT_table14.tsv, DRAFT_table14_bins.tsv, DRAFT_table14_rows.tex, per_run.tsv, selfcheck.tsv in", a.out)
    return 0 if not refused else 2


if __name__ == "__main__":
    sys.exit(main())
