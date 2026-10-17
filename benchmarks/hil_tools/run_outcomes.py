#!/usr/bin/env python3
"""run_outcomes.py (16D, class A: outside the HIL_CODE_COMMIT guard; read-only on the run folders): one row per in-loop run with
the first-motion audit, the FSM transitions and their reasons, the reach rule's conditions as the data met them, the outcome
class, the closest approach and the staleness terms. Feeds first_motion_audit.py, reach_audit.py, outcome_classes.py,
staleness_fit.py, ablation_table.py and the figure scripts. Nothing here touches the scorer or any stored result.

Per run it reads: approach_bag/r20_bag_0.mcap (/sim/drone_pose, /cmd_vel, /bench/state, /yolo/detections), t_resume_ns.txt,
stack/<label>_run1_stack.log (state lines with wall time), stack/<label>_run1_params.yaml (the FSM constants), stack/<label>_
clock_offset.json, stack/<label>_run1_telemetry.csv (capture-to-commit latency), stack/<label>_run1_mission.json, stack/*_run1_
stats.json, stack/*_summary.json, check/<label>_run1_check.json (the stored closest approach, compared with the recomputed one),
and batch_state.tsv for the identity columns.

Definitions (stage6/analysis_16D/EXPECTED.md, fixed before any number was computed):
- resume = t_resume_ns.txt; "after resume" = bag log time minus it.
- first motion: the first /cmd_vel after resume with |vx| > 0.05 m/s; forward motion = the pose displacement over the next
  3 s projected on the heading at the command; pass if >= 0.10 m in the sign of vx. The same for wz (unwrapped yaw change over
  3 s, pass if >= 0.05 rad in the sign of wz) and, as an extra, for vy (the first |vy| >= 0.2 m/s, a strafe; the lateral
  displacement over 3 s, pass if at least 25 pct of |vy| x 3 s in magnitude; the sign relation to the command is recorded,
  not tested). The verdict moved_as_commanded uses vx and wz; a channel never commanded after resume is "n/a".
- reach rule as coded (servo_fsm_node.cpp, update_state_on_detection): in APPROACHING, 8 consecutive detection callbacks that
  carry a target box with the 5-sample smoothed height ratio >= hold_bbox_ratio (0.50) and |ex|, |ey| < hold_center_tol
  (0.12); the counter resets on a box that fails, on REACQUIRE -> APPROACHING, and through any message without a box
  (immediate APPROACHING -> REACQUIRE) or a gap above the loss window W (watchdog). Here the smoothed ratio, ex, ey are
  replayed per message exactly as the node computes them (best box = highest confidence of the target class at or above
  min_confidence; a jump above 0.30 of the image counts as no detection; histories are not cleared by empty messages).
- loss reasons: an APPROACHING -> REACQUIRE transition is attributed to the latest detection message bagged up to 50 ms after
  the FSM's log stamp (the recorder receives the causing message a few ms after the node logs), unless that message carries a
  usable box (it cannot have caused the transition; then the last message at or before the stamp is judged): a message
  without a target box, a box rejected as a jump above 0.30 of the image, or the watchdog when the time since that message
  is at least W.
- closest approach = the minimum horizontal distance from the drone origin to the frozen target over the approach bag
  (check_run's closest_dist_frozen_sign_m); the camera is 0.2 m ahead of the origin along the heading (the scorer's mount).
- outcome classes: 1 never locked on; 2 reached cleanly (REACHED, closest >= 1.0 m); 3 reached after an overshoot (REACHED,
  closest < 1.0 m); 4 overshot and lost the sign (not REACHED, closest < 1.0 m); 5 arrived in the window without REACHED (not
  REACHED, locked on, closest in [1.6, 2.8] m); 6 other, named.
- staleness terms: v = horizontal ground speed (central difference over +/-0.25 s) at the capture of the last target box before
  the closest point; L = mean telemetry
  e2e_capture_to_commit over frames captured between resume and the closest point; G = mean gap between consecutive target-box
  captures between resume and the closest point; S = fy x 0.914 / (target_bbox_ratio x 480) + 0.2 m (vx-zero standoff at the
  drone origin).
16F additions (stage6/analysis_16F/EXPECTED.md, DEVIATIONS 17.80 POST-HOC):
- altitude: over the poses after resume, the minimum z, the first time it is attained, the first time z <= GROUND_Z_M (0.01 m,
  the simulator's ground at 0.0 with a float guard), z at resume and at the last pose.
- ground contact: a run whose minimum z after resume is <= GROUND_CONTACT_Z_M (0.5 m since 16G, DEVIATIONS 17.80 f: the
  simulator has no ground model and class membership is the same for any threshold from 0.09 to 2.3 m, so 0.5 m sits inside
  the range where the result does not depend on it, while 0.0 m sits on a boundary; 16F used GROUND_Z_M = 0.01 m); the class
  is tested first and takes precedence over every other class. The other classes keep their 16D definitions and order. The
  audit columns z_reached_zero and t_ground_s keep the 0.01 m meaning (the ground itself).
- yaw deviation while APPROACHING (and, as an extra, while REACHED): for every pose whose log time falls in an interval of
  that state in the stack log, dev = |wrap(yaw)| in degrees (the pose yaw is near 2 pi, so dev = |yaw - 360 deg|); the
  maximum, the mean, the pose count, and the lateral share = sin(max dev) x 100 pct (the body-forward component of a
  world-y lateral command at that heading; the simulator integrates cmd_vel along the world axes, SIM_PARAMS.md).
Usage:
  ~/venvs/mcap/bin/python benchmarks/hil_tools/run_outcomes.py --batches <list.tsv> --out <dir> [--usb /mnt/usb/hil_runs]
<list.tsv>: lines "<session> <batch folder under --usb>" (a folder may contain a slash). <out dir> must not exist.
Outputs: per_run_outcomes.tsv (one row per run), transitions.jsonl (the FSM transitions and loss events per run), log.txt.
Self-check: replay_consistent is True when the replayed close-and-centred streak reaches 8 exactly for the runs that logged REACHED.
"""
import argparse, bisect, csv, glob, importlib.util, json, math, os, re, statistics, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
spec = importlib.util.spec_from_file_location("ia", os.path.join(ROOT, "benchmarks", "inloop_accuracy.py"))
ia = importlib.util.module_from_spec(spec); spec.loader.exec_module(ia)
GEOM = json.load(open(os.path.join(ROOT, "benchmarks", "hil_runs", "config", "sim_geometry.json")))
PROTO = json.load(open(os.path.join(ROOT, "benchmarks", "hil_runs", "config", "protocol_fx554.json")))
T = GEOM["target_xyz"]; S1 = (GEOM.get("nontarget_signs_xyz") or [[None, None, None]])[0]
WINDOW = GEOM["smoke_final_dist_window_m"]; H_SIGN = PROTO["sign_height_m"]
FWD_MIN_M, YAW_MIN_RAD, LAT_FRAC, CMD_EPS, VY_MIN, RESP_S = 0.10, 0.05, 0.25, 0.05, 0.2, 3.0
CLASS_LINE_M = 1.0
GROUND_Z_M = 0.01   # 16F: the simulator's ground (z = 0.000 m) with a float guard; z_reached_zero and t_ground_s (DEVIATIONS 17.80 a)
GROUND_CONTACT_Z_M = 0.5   # 16G: minimum z after resume at or below this is the ground contact class (DEVIATIONS 17.80 f, Ahmed)
CAUSE_SLACK_S = 0.05   # a detection message bagged within this time after an FSM transition stamp is taken as its cause
ia.FX = ia.FY = float(GEOM["fx"]); ia.CAM_PITCH = math.radians(-5.0)
RESCORED = None   # optional re-score tree for a run folder without check/<label>_run1_check.json

COLS = ["sess", "batch", "run", "n", "key", "model", "backend", "role", "W_s", "W_source", "fy", "hold_ratio", "target_ratio", "center_tol",
        "reach_ticks_needed", "smoothing", "min_conf",
        # first motion
        "t_resume_ns", "poses_after_resume", "distinct_poses_after_resume", "max_disp_from_start_m",
        "t_first_vx_cmd_s", "first_vx_cmd", "fwd_disp_3s_m", "vx_ok", "t_first_wz_cmd_s", "first_wz_cmd", "dyaw_3s_rad", "wz_ok",
        "t_first_vy_cmd_s", "first_vy_cmd", "lat_disp_3s_m", "vy_ok", "lat_sign_rel", "moved_as_commanded",
        # FSM
        "lock_log_s", "lock_bench_s", "locked_on", "final_state", "reached", "t_reach_sim_s", "n_lost", "n_reacquire",
        "n_transitions_after_lock", "n_losses_after_lock", "first_loss_s", "first_loss_reason", "first_loss_since_last_msg_s",
        "final_loss_s", "final_loss_reason", "final_loss_since_last_msg_s", "range_at_final_loss_m", "z_at_final_loss_m",
        # closest, final
        "closest_m", "closest_stored_m", "t_closest_s", "closest_face_m", "z_at_closest_m", "target_in_view_at_closest",
        "target_v_px_at_closest", "final_m", "final_stored_m", "final_recomputed_m", "final_S1_m", "closest_in_window", "final_in_window",
        # reach rule conditions
        "hold_face_range_m", "vx_zero_face_range_m", "met_distance_cond", "max_smoothed_ratio", "met_size_cond",
        "n_close_centered_msgs", "max_close_centered_streak", "streak_reaches_8", "replay_consistent",
        "last_box_capture_range_m", "last_box_delivery_range_m", "last_box_z_m", "last_box_ratio", "last_box_ex", "last_box_ey",
        "after_last_box_first_msg_s", "after_last_box_target_in_view", "after_last_box_u", "after_last_box_v", "after_last_box_face_px",
        "after_last_box_range_m", "after_last_box_z_m", "after_last_box_what",
        # staleness
        "approached", "v_at_last_prebox_m_s", "lat_L_s", "gap_G_s", "x_vLG_m", "S_origin_m", "closest_pred_m", "resid_m",
        "last_prebox_capture_range_m", "travel_prebox_to_closest_m",
        # rates
        "det_hz", "cmd_vel_hz", "det_lat_ms",
        # 16F: altitude and yaw deviation
        "z_at_resume_m", "min_z_m", "t_min_z_s", "t_ground_s", "z_reached_zero", "z_last_m",
        "appr_n_poses", "appr_yaw_dev_max_deg", "appr_yaw_dev_mean_deg", "appr_lateral_share_max_pct", "reached_n_poses", "reached_yaw_dev_max_deg",
        "outcome_class", "outcome_note"]


def fnum(v, d=3):
    return "" if v is None else (f"{v:.{d}f}" if isinstance(v, float) else str(v))


def read_params(path):
    out = {}
    for line in open(path):
        m = re.match(r"^\s+([A-Za-z_0-9]+):\s*(-?[0-9.]+)\s*$", line)
        if m:
            try:
                out[m.group(1)] = float(m.group(2))
            except ValueError:
                pass
    return out


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def read_approach_bag(path):
    from mcap_ros2.reader import read_ros2_messages
    poses, cmds, states, dets = [], [], [], []
    for m in read_ros2_messages(path):
        t = m.log_time_ns; top = m.channel.topic
        if top == "/sim/drone_pose":
            d = list(m.ros_msg.data)
            if len(d) >= 5:
                poses.append((t, d[0], d[1], d[2], d[4]))
        elif top == "/cmd_vel":
            v = m.ros_msg; cmds.append((t, v.linear.x, v.linear.y, v.linear.z, v.angular.z))
        elif top == "/bench/state":
            states.append((t, m.ros_msg.data.split(",")[0]))
        elif top == "/yolo/detections":
            h = m.ros_msg.header.stamp
            dets.append({"log_ns": t, "stamp_ns": h.sec * 10**9 + h.nanosec,
                         "dets": [(d.class_name, d.confidence, d.center_x, d.center_y, d.size_width, d.size_height) for d in m.ros_msg.detections]})
    poses.sort(); cmds.sort(); states.sort(); dets.sort(key=lambda r: r["log_ns"])
    return poses, cmds, states, dets


LOG_RE = re.compile(r"\[(\d+)\.(\d+)\] \[visp_servo_node\]: (.*)$")
STATE_RE = re.compile(r"State: (\w+) -> (\w+)\s+\(consec=(\d+) ratio=([\d.]+) ex=([+-][\d.]+) ey=([+-][\d.]+)\)")
REACQ_RE = re.compile(r"APPROACHING -> REACQUIRE \(dur=([\d.]+)s, timeout=([\d.]+)s\)")
SEARCH_RE = re.compile(r"REACQUIRE -> SEARCHING \(timed out ([\d.]+)s\)")
WIN_RE = re.compile(r"Detection-loss window: ([0-9.]+) s \(([^)]*)\)")


def read_stack_log(path):
    """FSM transitions (wall ns, from, to, detail) and the logged loss window."""
    trans, window = [], None
    for line in open(path, errors="replace"):
        m = LOG_RE.search(line)
        if not m:
            continue
        t = int(m.group(1)) * 10**9 + int(m.group(2).ljust(9, "0")[:9]); body = m.group(3)
        w = WIN_RE.search(body)
        if w:
            window = (float(w.group(1)), w.group(2)); continue
        s = STATE_RE.search(body)
        if s:
            trans.append((t, s.group(1), s.group(2), {"consec": int(s.group(3)), "ratio": float(s.group(4)), "ex": float(s.group(5)), "ey": float(s.group(6))})); continue
        s = REACQ_RE.search(body)
        if s:
            trans.append((t, "APPROACHING", "REACQUIRE", {"dur_s": float(s.group(1)), "timeout_s": float(s.group(2))})); continue
        s = SEARCH_RE.search(body)
        if s:
            trans.append((t, "REACQUIRE", "SEARCHING", {"timed_out_s": float(s.group(1))}))
    trans.sort(key=lambda x: x[0])
    return trans, window


def replay_boxes(dets, params):
    """Per detection message, the node's own smoothed ratio, ex, ey (or None for a message it treats as no detection)."""
    minc = params.get("min_confidence", 0.2); win = int(params.get("smoothing_window", 5))
    iw, ih = params.get("image_width", 640.0), params.get("image_height", 480.0)
    hist = {k: [] for k in ("cx", "cy", "bw", "bh")}; prev = None; out = []
    for rec in dets:
        best = None
        for d in rec["dets"]:
            if d[0] == ia.TARGET_CLASS and d[1] >= minc and (best is None or d[1] > best[1]):
                best = d
        if best is None:
            prev = None; out.append(None); continue
        if prev is not None and (abs(best[2] - prev[0]) / iw > 0.30 or abs(best[3] - prev[1]) / ih > 0.30):
            prev = (best[2], best[3]); out.append(None); continue
        prev = (best[2], best[3])
        for k, v in zip(("cx", "cy", "bw", "bh"), best[2:6]):
            hist[k].append(v); hist[k] = hist[k][-win:]
        cx = sum(hist["cx"]) / len(hist["cx"]); cy = sum(hist["cy"]) / len(hist["cy"]); bh = sum(hist["bh"]) / len(hist["bh"])
        out.append({"ratio": bh / ih, "ex": (cx - iw / 2) / (iw / 2), "ey": (cy - ih / 2) / (ih / 2), "raw_h": best[5], "conf": best[1]})
    return out


def state_at(trans, t, initial="SEARCHING"):
    s = initial
    for tt, a, b, _ in trans:
        if tt <= t:
            s = b
        else:
            break
    return s


def one_run(sess, folder, row, rd, label, log):
    R = {"sess": sess, "batch": folder, "run": row["dir"], "n": row["n"], "key": row["key"], "model": row["model"], "backend": row["backend"], "role": row["role"]}
    tr = int(open(os.path.join(rd, "t_resume_ns.txt")).read().split()[0]); R["t_resume_ns"] = tr
    co = int(json.load(open(os.path.join(rd, "stack", f"{label}_clock_offset.json")))["realtime_minus_monotonic_ns"])
    params = read_params(os.path.join(rd, "stack", f"{label}_run1_params.yaml"))
    fy = params.get("cam_fy", 554.0); hold = params.get("hold_bbox_ratio", 0.5); tgt = params.get("target_bbox_ratio", 0.55)
    ctol = params.get("hold_center_tol", 0.12); ih = params.get("image_height", 480.0)
    R.update({"fy": fy, "hold_ratio": hold, "target_ratio": tgt, "center_tol": ctol, "reach_ticks_needed": int(params.get("reach_consec_ticks", 8)),
              "smoothing": int(params.get("smoothing_window", 5)), "min_conf": params.get("min_confidence", 0.2)})
    trans, window = read_stack_log(os.path.join(rd, "stack", f"{label}_run1_stack.log"))
    if window:
        W, wsrc = window
    else:
        dl = params.get("detection_lost_sec", -1.0); wd = params.get("watchdog_period_sec", 0.2)
        W, wsrc = (dl, "params detection_lost_sec") if dl > 0 else (2.0 * wd, "2 x watchdog_period_sec (pinned rule)")
    R["W_s"], R["W_source"] = W, wsrc
    poses, cmds, states, dets = read_approach_bag(os.path.join(rd, "approach_bag", "r20_bag_0.mcap"))
    track = ia.PoseTrack(poses)
    # ---- first motion ----
    post = [p for p in poses if p[0] >= tr]
    R["poses_after_resume"] = len(post); R["distinct_poses_after_resume"] = len({(p[1], p[2], p[3]) for p in post})
    p0 = post[0] if post else None
    R["max_disp_from_start_m"] = max((math.dist(p[1:4], p0[1:4]) for p in post), default=None) if p0 else None
    def respond(chan, idx, kind):
        thr = VY_MIN if chan == "vy" else CMD_EPS
        c = next((c for c in cmds if c[0] > tr and abs(c[idx]) > thr), None)
        if c is None:
            R[f"t_first_{chan}_cmd_s"] = None; R[f"first_{chan}_cmd"] = None; R[f"{kind}"] = None; R[f"{chan}_ok"] = "n/a"; return None
        pa = track.at(c[0]); pb = track.at(c[0] + int(RESP_S * 1e9))
        R[f"t_first_{chan}_cmd_s"] = (c[0] - tr) / 1e9; R[f"first_{chan}_cmd"] = c[idx]
        if pa is None or pb is None or pb[0] - pa[0] < 0.5 * RESP_S * 1e9:
            R[kind] = None; R[f"{chan}_ok"] = "no pose window"; return False
        yaw = pa[4]; dx, dy = pb[1] - pa[1], pb[2] - pa[2]
        if chan == "vx":
            val = dx * math.cos(yaw) + dy * math.sin(yaw); ok = (val * math.copysign(1, c[idx])) >= FWD_MIN_M
        elif chan == "vy":
            # extra channel, magnitude only: the lateral displacement (left positive by the pose yaw) must be at least
            # LAT_FRAC of |vy| x 3 s; the sign relation is recorded in lat_sign_rel (the simulator moves the drone to the
            # side opposite to the body convention of the controller code, consistently; not a fault, a convention)
            val = -dx * math.sin(yaw) + dy * math.cos(yaw); ok = abs(val) >= LAT_FRAC * abs(c[idx]) * RESP_S
            R["lat_sign_rel"] = "same" if (val > 0) == (c[idx] > 0) else "opposite"
        else:
            val = wrap(pb[4] - pa[4]); ok = (val * math.copysign(1, c[idx])) >= YAW_MIN_RAD
        R[kind] = val; R[f"{chan}_ok"] = bool(ok); return ok
    ok_vx = respond("vx", 1, "fwd_disp_3s_m"); ok_wz = respond("wz", 4, "dyaw_3s_rad"); respond("vy", 2, "lat_disp_3s_m")
    tested = [o for o in (ok_vx, ok_wz) if o is not None]   # the verdict uses vx and wz (the prompt's two channels); vy is reported
    R["moved_as_commanded"] = ("no vx or wz command after resume" if not tested else (all(tested)))
    # ---- FSM ----
    rel = lambda t: (t - tr) / 1e9
    lock_log = next((rel(t) for t, a, b, _ in trans if a == "SEARCHING" and b == "APPROACHING"), None)
    prev = None; lock_bench = None
    for t, s in states:
        if prev == "SEARCHING" and s == "APPROACHING":
            lock_bench = rel(t); break
        prev = s
    if lock_bench is None and states and states[0][1] == "APPROACHING":
        lock_bench = rel(states[0][0])
    R["lock_log_s"], R["lock_bench_s"] = lock_log, lock_bench
    R["locked_on"] = lock_log is not None
    ms = json.load(open(os.path.join(rd, "stack", f"{label}_run1_mission.json")))
    R["final_state"] = ms.get("final_state"); R["reached"] = bool(ms.get("reached")); R["t_reach_sim_s"] = ms.get("time_to_reached_sim_s")
    R["n_lost"] = ms.get("n_lost"); R["n_reacquire"] = ms.get("n_reacquire")
    # loss events with reasons: the time since the last detection message at the transition against W
    det_log_t = [d["log_ns"] for d in dets]
    rep = replay_boxes(dets, params)   # None where the node treats the message as no detection (no box, or a jump)
    events = []
    t_lock_ns = next((t for t, a, b, _ in trans if a == "SEARCHING" and b == "APPROACHING"), None)
    for t, a, b, d in trans:
        e = {"t_s": round(rel(t), 3), "from": a, "to": b, **d}
        if a == "APPROACHING" and b == "REACQUIRE":
            # the message that caused the transition is bagged by the recorder up to a few ms after the FSM logs the
            # transition, so the latest message within CAUSE_SLACK_S after the stamp counts as the last one (16D review)
            i = bisect.bisect_right(det_log_t, t + int(CAUSE_SLACK_S * 1e9)) - 1
            if i >= 0 and det_log_t[i] > t and rep[i] is not None:
                # a message inside the slack that carries a usable box cannot have caused APPROACHING -> REACQUIRE (it is the
                # box that follows a watchdog trip), so the last message at or before the stamp is the one to judge
                i = bisect.bisect_right(det_log_t, t) - 1
            since = max(0.0, (t - det_log_t[i]) / 1e9) if i >= 0 else None
            e["since_last_det_msg_s"] = None if since is None else round(since, 3)
            if since is not None and since >= W - 0.05:
                e["reason"] = "watchdog: gap above W"
            else:
                last = dets[i] if i >= 0 else None
                has_box = bool(last and any(x[0] == ia.TARGET_CLASS and x[1] >= R["min_conf"] for x in last["dets"]))
                e["reason"] = "message with a box but a jump above 0.30 of the image" if has_box else "message without a target box"
        events.append(e)
    after_lock = [e for e in events if t_lock_ns is not None and e["t_s"] >= rel(t_lock_ns)]
    R["n_transitions_after_lock"] = len(after_lock) - 1 if t_lock_ns is not None else 0
    losses = [e for e in after_lock if e["from"] == "APPROACHING" and e["to"] == "REACQUIRE"]
    R["n_losses_after_lock"] = len(losses)
    # ---- closest and final ----
    dist = [(p[0], math.hypot(T[0] - p[1], T[1] - p[2])) for p in poses]
    tmin, dmin = min(dist, key=lambda v: v[1])
    R["closest_m"], R["t_closest_s"] = dmin, rel(tmin)
    pc = track.at(tmin); cam = ia.camera_pose(pc); R["closest_face_m"] = math.hypot(T[0] - cam[0], T[1] - cam[1])
    R["z_at_closest_m"] = pc[3]; uvc = ia.project(pc, T)
    R["target_in_view_at_closest"] = ia.in_view(uvc); R["target_v_px_at_closest"] = uvc[1] if uvc else None
    cjp = os.path.join(rd, "check", f"{label}_run1_check.json")
    if not os.path.exists(cjp) and RESCORED:
        cjp = os.path.join(RESCORED, os.path.basename(folder), row["dir"], f"{label}_run1_check.json")
    cj = json.load(open(cjp))["info"]
    R["closest_stored_m"] = cj.get("closest_dist_frozen_sign_m")
    fp = ms.get("final_pose_xyz_yaw") or [None] * 4
    # mission.json rounds final_pose_xyz_yaw to 2 decimals but computed final_dist_frozen_sign_m from the unrounded pose, so the
    # stored value is primary (final_m) and the recomputation from the rounded pose is the check (final_recomputed_m, within 1 mm)
    R["final_recomputed_m"] = math.hypot(fp[0] - T[0], fp[1] - T[1]) if fp[0] is not None else None
    R["final_stored_m"] = ms.get("final_dist_frozen_sign_m")
    R["final_m"] = R["final_stored_m"] if R["final_stored_m"] is not None else R["final_recomputed_m"]
    R["final_S1_m"] = math.hypot(fp[0] - S1[0], fp[1] - S1[1]) if (fp[0] is not None and S1[0] is not None) else None
    R["closest_in_window"] = WINDOW[0] <= dmin <= WINDOW[1]
    R["final_in_window"] = (R["final_m"] is not None) and WINDOW[0] <= R["final_m"] <= WINDOW[1]
    # ---- reach rule conditions ----
    hold_face = fy * H_SIGN / (hold * ih); vx0_face = fy * H_SIGN / (tgt * ih)
    R["hold_face_range_m"], R["vx_zero_face_range_m"] = hold_face, vx0_face
    R["met_distance_cond"] = R["closest_face_m"] <= hold_face
    boxes = [(d, r) for d, r in zip(dets, rep) if r is not None]
    R["max_smoothed_ratio"] = max((r["ratio"] for _, r in boxes), default=None)
    R["met_size_cond"] = (R["max_smoothed_ratio"] or 0) >= hold
    # streaks of close-and-centred box messages in APPROACHING, broken by an empty message or a gap above W
    n_cc = 0; streak = 0; best = 0; prev_t = None
    for d, r in zip(dets, rep):
        if r is None:
            streak = 0; prev_t = d["log_ns"]; continue
        cc = r["ratio"] >= hold and abs(r["ex"]) < ctol and abs(r["ey"]) < ctol
        in_appr = state_at(trans, d["log_ns"] - int(0.15e9)) in ("APPROACHING", "REACHED")   # the state just before the callback
        gap_break = prev_t is not None and (d["log_ns"] - prev_t) / 1e9 > W
        if cc and in_appr and not gap_break:
            streak += 1; n_cc += 1
        elif cc and in_appr:
            streak = 1; n_cc += 1
        else:
            streak = 0
        best = max(best, streak); prev_t = d["log_ns"]
    R["n_close_centered_msgs"], R["max_close_centered_streak"] = n_cc, best
    R["streak_reaches_8"] = best >= R["reach_ticks_needed"]
    R["replay_consistent"] = (R["streak_reaches_8"] == R["reached"])   # self-check of the replay against the logged outcome
    # per loss event: the drone at the loss, the last box message before it (capture and delivery), and the first message after
    # that box (what the camera saw at its capture); the first and the final loss after lock-on also go to the TSV columns
    def loss_detail(e):
        t_loss = int(e["t_s"] * 1e9) + tr
        pl = track.at(t_loss); out = {"range_at_loss_m": math.hypot(T[0] - pl[1], T[1] - pl[2]) if pl else None, "z_at_loss_m": pl[3] if pl else None}
        cand = [(d, r) for d, r in boxes if d["log_ns"] <= t_loss]
        if not cand:
            return out
        d, r = cand[-1]
        pcap = track.at(d["stamp_ns"] + co); pdel = track.at(d["log_ns"])
        out["last_box"] = {"t_capture_s": rel(d["stamp_ns"] + co), "t_delivery_s": rel(d["log_ns"]),
                           "capture_range_m": math.hypot(T[0] - pcap[1], T[1] - pcap[2]) if pcap else None,
                           "delivery_range_m": math.hypot(T[0] - pdel[1], T[1] - pdel[2]) if pdel else None,
                           "z_m": pcap[3] if pcap else None, "ratio": r["ratio"], "ex": r["ex"], "ey": r["ey"]}
        if e.get("reason", "").startswith("watchdog"):
            out["next_msg"] = {"what": "watchdog: no detection message for W after the last box"}
            return out
        nxt = next((x for x in dets if x["log_ns"] > d["log_ns"]), None)
        if nxt:
            pn = track.at(nxt["stamp_ns"] + co); uv = ia.project(pn, T) if pn else None
            face_px = (fy * H_SIGN / uv[3]) if uv else None
            has_box = any(x[0] == ia.TARGET_CLASS and x[1] >= R["min_conf"] for x in nxt["dets"])
            if uv is None:
                what = "target behind the camera"
            elif not ia.in_view(uv):
                what = "target outside the image (above)" if uv[1] < 0 else ("target outside the image (below)" if uv[1] >= ih else "target outside the image (side)")
            elif face_px >= ih:
                what = "target in view but taller than the frame"
            elif has_box:
                what = "a box was present but rejected as a jump above 0.30 of the image"
            else:
                what = "target in view, no box (detector miss)"
            out["next_msg"] = {"t_s": rel(nxt["log_ns"]), "t_capture_s": rel(nxt["stamp_ns"] + co), "target_in_view": ia.in_view(uv),
                               "u": uv[0] if uv else None, "v": uv[1] if uv else None, "face_px": face_px,
                               "range_m": math.hypot(T[0] - pn[1], T[1] - pn[2]) if pn else None, "z_m": pn[3] if pn else None,
                               "has_box": has_box, "what": what}
        return out
    for e in losses:
        e.update(loss_detail(e))
    if losses:
        fl, ll = losses[0], losses[-1]
        R["first_loss_s"], R["first_loss_reason"], R["first_loss_since_last_msg_s"] = fl["t_s"], fl["reason"], fl.get("since_last_det_msg_s")
        R["final_loss_s"], R["final_loss_reason"], R["final_loss_since_last_msg_s"] = ll["t_s"], ll["reason"], ll.get("since_last_det_msg_s")
        R["range_at_final_loss_m"], R["z_at_final_loss_m"] = ll.get("range_at_loss_m"), ll.get("z_at_loss_m")
        lb, nm = ll.get("last_box") or {}, ll.get("next_msg") or {}
        R["last_box_capture_range_m"], R["last_box_delivery_range_m"], R["last_box_z_m"] = lb.get("capture_range_m"), lb.get("delivery_range_m"), lb.get("z_m")
        R["last_box_ratio"], R["last_box_ex"], R["last_box_ey"] = lb.get("ratio"), lb.get("ex"), lb.get("ey")
        R["after_last_box_first_msg_s"], R["after_last_box_target_in_view"] = nm.get("t_s"), nm.get("target_in_view")
        R["after_last_box_u"], R["after_last_box_v"], R["after_last_box_face_px"] = nm.get("u"), nm.get("v"), nm.get("face_px")
        R["after_last_box_range_m"], R["after_last_box_z_m"], R["after_last_box_what"] = nm.get("range_m"), nm.get("z_m"), nm.get("what")
    # ---- staleness ----
    start_d = dist[0][1] if dist else None
    R["approached"] = bool(R["locked_on"] and start_d is not None and dmin <= start_d - 1.0)
    box_caps = sorted(d["stamp_ns"] + co for d, _ in boxes)
    pre = [c for c in box_caps if tr < c < tmin]
    S = vx0_face + ia.MOUNT_FWD; R["S_origin_m"] = S
    def ground_speed_at(ts):
        # central-difference horizontal speed over +/-0.25 s (the scorer's speed_at is 3D and would count the descent)
        a_ = [p for p in poses if abs(p[0] - ts) <= 0.25e9]
        if len(a_) < 2 or a_[-1][0] == a_[0][0]:
            return None
        return math.hypot(a_[-1][1] - a_[0][1], a_[-1][2] - a_[0][2]) / ((a_[-1][0] - a_[0][0]) / 1e9)
    if R["approached"] and pre:
        v = ground_speed_at(pre[-1]); R["v_at_last_prebox_m_s"] = v
        pp = track.at(pre[-1]); R["last_prebox_capture_range_m"] = math.hypot(T[0] - pp[1], T[1] - pp[2])
        R["travel_prebox_to_closest_m"] = R["last_prebox_capture_range_m"] - dmin
        gaps = [(pre[i] - pre[i - 1]) / 1e9 for i in range(1, len(pre))]
        R["gap_G_s"] = statistics.fmean(gaps) if gaps else None
        lat = []
        with open(os.path.join(rd, "stack", f"{label}_run1_telemetry.csv")) as f:
            for rw in csv.DictReader(f):
                try:
                    tc = int(rw["ts_capture"]) + co
                    if tr <= tc <= tmin:
                        lat.append(float(rw["e2e_capture_to_commit_ms"]) / 1000)
                except (KeyError, ValueError):
                    pass
        R["lat_L_s"] = statistics.fmean(lat) if lat else None
        if v is not None and R["gap_G_s"] is not None and R["lat_L_s"] is not None:
            R["x_vLG_m"] = v * (R["lat_L_s"] + R["gap_G_s"]); R["closest_pred_m"] = S - R["x_vLG_m"]; R["resid_m"] = dmin - R["closest_pred_m"]
    # ---- rates ----
    st = json.load(open(glob.glob(os.path.join(rd, "stack", "*_run1_stats.json"))[0]))
    sm = json.load(open(glob.glob(os.path.join(rd, "stack", "*_summary.json"))[0]))
    R["det_hz"] = st["det"]["hz_mean"]; R["cmd_vel_hz"] = st["cmd_vel"]["hz_mean"]; R["det_lat_ms"] = sm["detector_latency_ms"]["det_path"]["mean"]
    # ---- 16F: altitude after resume (16G: the class threshold GROUND_CONTACT_Z_M is separate from the ground itself) ----
    ground = False; contact = False
    if post:
        zmin_t, zmin = min(((p[0], p[3]) for p in post), key=lambda v: (v[1], v[0]))   # the lowest z; the earliest pose at that value
        R["z_at_resume_m"], R["min_z_m"], R["t_min_z_s"], R["z_last_m"] = post[0][3], zmin, rel(zmin_t), post[-1][3]
        tg = next((p[0] for p in post if p[3] <= GROUND_Z_M), None)
        R["t_ground_s"] = None if tg is None else rel(tg); ground = tg is not None; R["z_reached_zero"] = ground
        contact = zmin <= GROUND_CONTACT_Z_M
    # ---- 16F: yaw deviation from the world x axis while APPROACHING (and while REACHED) ----
    def yaw_dev_in(state):
        iv = []; cur = "SEARCHING"; t0 = None
        for tt, a_, b_, _ in trans:
            if cur == state and t0 is not None:
                iv.append((t0, tt))
            cur = b_; t0 = tt
        if cur == state and t0 is not None:
            iv.append((t0, poses[-1][0] + 1 if poses else t0))
        devs = [abs(math.degrees(wrap(p[4]))) for p in poses if any(lo <= p[0] < hi for lo, hi in iv)]
        return devs
    da = yaw_dev_in("APPROACHING"); dr = yaw_dev_in("REACHED")
    R["appr_n_poses"] = len(da)
    if da:
        R["appr_yaw_dev_max_deg"] = max(da); R["appr_yaw_dev_mean_deg"] = statistics.fmean(da)
        R["appr_lateral_share_max_pct"] = 100.0 * math.sin(math.radians(max(da)))
    R["reached_n_poses"] = len(dr)
    if dr:
        R["reached_yaw_dev_max_deg"] = max(dr)
    # ---- outcome class ----
    if contact:   # 16F, DEVIATIONS 17.80: ground contact takes precedence over every other class; 16G: minimum z <= 0.5 m (17.80 f)
        cls, note = "ground contact", f"z {R['min_z_m']:.3f} m at {(R['t_ground_s'] if ground else R['t_min_z_s']):.1f} s, closest {dmin:.2f} m, final {R['final_m']:.2f} m"
    elif not R["locked_on"]:
        cls, note = "never locked on", ""
    elif R["reached"] and dmin >= CLASS_LINE_M:
        cls, note = "reached cleanly", ""
    elif R["reached"]:
        cls, note = "reached after an overshoot", f"closest {dmin:.2f} m"
    elif dmin < CLASS_LINE_M:
        cls, note = "overshot and lost the sign", f"closest {dmin:.2f} m, final {R['final_m']:.2f} m"
    elif R["closest_in_window"]:
        cls, note = "arrived in the window without REACHED", f"closest {dmin:.2f} m, final {R['final_m']:.2f} m"
    elif dmin > WINDOW[1] and dmin <= 4.0:
        cls, note = "other", f"stopped short beyond the window (closest {dmin:.2f} m, final {R['final_m']:.2f} m)"
    elif dmin > 4.0 and R["final_state"] != "APPROACHING":
        cls, note = "other", f"lost mid-approach far from the sign (closest {dmin:.2f} m, final {R['final_m']:.2f} m)"
    else:
        cls, note = "other", f"state {R['final_state']}, closest {dmin:.2f} m, final {R['final_m']:.2f} m"
    R["outcome_class"], R["outcome_note"] = cls, note
    log.write(f"{sess} {row['dir']}: W {W} lock {fnum(lock_log, 2)} closest {dmin:.3f} class '{cls}' moved {R['moved_as_commanded']}\n"); log.flush()
    return R, {"sess": sess, "batch": folder, "run": row["dir"], "W_s": W, "transitions": events}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batches", required=True); ap.add_argument("--out", required=True); ap.add_argument("--usb", default="/mnt/usb/hil_runs")
    ap.add_argument("--rescored", default=None, help="re-score tree (invariance/rescore.sh layout) used only when a run folder has no check json")
    a = ap.parse_args()
    global RESCORED; RESCORED = a.rescored
    if os.path.exists(a.out):
        sys.exit(f"run_outcomes: REFUSE: {a.out} exists (every run writes into a new folder)")
    os.makedirs(a.out)
    log = open(os.path.join(a.out, "log.txt"), "w")
    runs = []
    for line in open(a.batches):
        if not line.strip() or line.startswith("#"):
            continue
        sess, folder = line.split()[:2]
        for r in csv.DictReader(open(os.path.join(a.usb, folder, "batch_state.tsv")), delimiter="\t"):
            runs.append((sess, folder, r, os.path.join(a.usb, folder, r["dir"]), "level2_" + r["dir"][4:]))
    print(f"{len(runs)} runs listed"); log.write(f"{len(runs)} runs listed\n")
    tsv = open(os.path.join(a.out, "per_run_outcomes.tsv"), "w"); tsv.write("# 16D run_outcomes.py: one row per in-loop run; definitions in the docstring and stage6/analysis_16D/EXPECTED.md; 16F columns (altitude, yaw deviation) and the ground contact class (DEVIATIONS 17.80; 16G: minimum z <= 0.5 m, 17.80 f) per stage6/analysis_16F/EXPECTED.md and analysis_16G/EXPECTED.md\n")
    tsv.write("\t".join(COLS) + "\n")
    jl = open(os.path.join(a.out, "transitions.jsonl"), "w")
    n_err = 0
    for sess, folder, r, rd, label in runs:
        try:
            R, J = one_run(sess, folder, r, rd, label, log)
        except Exception as e:  # noqa: BLE001
            n_err += 1; log.write(f"ERROR {sess} {r['dir']}: {e!r}\n"); print(f"ERROR {sess} {r['dir']}: {e!r}"); continue
        tsv.write("\t".join(fnum(R.get(c)) for c in COLS) + "\n"); tsv.flush()
        jl.write(json.dumps(J) + "\n"); jl.flush()
        print(f"  {sess} {r['dir']}: lock {fnum(R['lock_log_s'], 2)} closest {R['closest_m']:.3f} {R['outcome_class']} moved={R['moved_as_commanded']}")
    tsv.close(); jl.close(); log.close()
    print(f"wrote per_run_outcomes.tsv and transitions.jsonl in {a.out}; errors {n_err}")
    return 0 if n_err == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
