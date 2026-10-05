#!/usr/bin/env python3
"""check_run.py <stage3_dir> <label> [--npu-sentinel] [--pre-resume-dir DIR] [--out-dir DIR]: Prompt 3F Stage 3 pass checks.
5J: start_pose_expected uses the last /sim/drone_pose recorded BEFORE resume, from DIR/approach_bag/*.mcap (rec.sh bag,
up before resume) and DIR/t_resume_ns.txt (sidecar); it fails loudly if there is no such pose. Outputs (_check.json,
_inloop.json, _inloop.txt) go to --out-dir (default: the stage3 dir) and are never overwritten (refuse: exit 1, message on stderr).
Run with ~/venvs/mcap/bin/python (needs mcap_ros2). Plumbing checks only: no number goes into a table.
5Q (benchmarks/hil_runs): paths from hilpaths.py; the scorer gets the Table 14 bins of config/protocol_fx554.json and its cache
goes to --out-dir (never into the input folder); new checks geometry_gate_is_protocol (the scorer gate equals the protocol's
18.47 px) and held_render_is_reference (DIR/held_final_sha1.txt, written by hold_gate.sh, is the render of config/held_reference.json); new info
standoff_expected_m (from the run's own FSM ratios), closest_dist_frozen_sign_m and overshoot_m (always reported)."""
import glob, json, math, os, re, subprocess, sys
from collections import deque
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hilpaths

d, label = sys.argv[1], sys.argv[2]
npu_sentinel = "--npu-sentinel" in sys.argv
def opt(name):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else None
P = os.path.join(d, f"{label}_run1")                       # inputs (stage3)
Q = os.path.join(opt("--out-dir") or d, f"{label}_run1")  # outputs
for _f in (Q + "_check.json", Q + "_inloop.json", Q + "_inloop.txt"):
    if os.path.exists(_f):
        sys.exit(f"check_run: REFUSE: {_f} exists (5J rule: never overwrite; use a new --out-dir)")
OURS = hilpaths.OURS
SENTINEL_MS = 35.839
# Expected geometry from one file (2026-10-04), not constants here.
GEOM = json.load(open(hilpaths.GEOM))
PROTO = json.load(open(os.path.join(hilpaths.HIL_RUNS, "config", "protocol_fx554.json")))
HELD = json.load(open(hilpaths.HELD_REF))
FROZEN = tuple(GEOM["target_xyz"][:2])
r = {"label": label, "checks": {}, "info": {}}
C, I = r["checks"], r["info"]

def rd(p):
    try:
        return open(p).read()
    except OSError:
        return ""

# 1. rates
try:
    st = json.load(open(P + "_stats.json"))
    det, cmd = st["det"]["hz_mean"], st["cmd_vel"]["hz_mean"]
    I["det_hz"], I["cmd_vel_hz"] = det, cmd
    C["det_hz_gt_0"] = det > 0
    C["cmd_within_5pct_of_det"] = det > 0 and abs(cmd - det) / det <= 0.05
    I["cmd_vs_det_pct"] = round(100 * (cmd - det) / det, 2) if det else None
except Exception as e:                                   # noqa: BLE001
    C["det_hz_gt_0"] = C["cmd_within_5pct_of_det"] = False
    I["stats_error"] = str(e)

# 2. network / RMW / SLAM pose / fx
log = rd(P + "_stack.log")
m = re.search(r"PI_INTERFACE=(\S+)", log); I["pi_interface"] = m and m.group(1)
m = re.search(r"RMW=(\S+)", log); I["rmw"] = m and m.group(1)
C["pi_interface_eth0"] = I["pi_interface"] == "eth0"
C["rmw_fastrtps"] = I["rmw"] == "rmw_fastrtps_cpp"
par = rd(P + "_params.yaml")
def pval(k):
    m = re.search(rf"^\s*{k}:\s*(\S+)", par, re.M)
    return m and m.group(1)
I["param_cam_fx"], I["param_cam_fy"] = pval("cam_fx"), pval("cam_fy")
I["param_use_slam_pose"], I["param_use_slam_depth"] = pval("use_slam_pose"), pval("use_slam_depth")
I["expected_cam_fx"] = GEOM["fx"]
C["cam_fx_expected"] = I["param_cam_fx"] is not None and float(I["param_cam_fx"]) == float(GEOM["fx"])
if I["param_use_slam_pose"] is None and I["param_use_slam_depth"] is None:
    # hil_servo (proportional) does not declare these: take what the launcher passed
    rc = rd(P + "_run_config.yaml")
    sp = re.search(r"^\s*use_slam_pose:\s*(\S+)", rc, re.M)
    sd = re.search(r"^\s*use_slam_depth:\s*(\S+)", rc, re.M)
    I["slam_pose_source"] = "run_config.yaml (controller does not declare the parameters)"
    C["slam_pose_off"] = bool(sp and sd and sp.group(1) == "false" and sd.group(1) == "false")
else:
    I["slam_pose_source"] = "ros2 param dump"
    C["slam_pose_off"] = I["param_use_slam_pose"] == "false" and I["param_use_slam_depth"] == "false"
m = re.search(r"slam_pose=(ON|OFF)", log); I["ibvs_log_slam_pose"] = m and m.group(1)

# 3. bag: three scorer topics, plus everything we need below
mc = sorted(glob.glob(P + "_bag/*.mcap"))
I["mcap"] = mc[0] if mc else None
msgs = []
if mc:
    from mcap_ros2.reader import read_ros2_messages
    for x in read_ros2_messages(mc[0]):
        msgs.append((x.log_time_ns, x.channel.topic, x.ros_msg))
counts = {}
for _, t, _ in msgs:
    counts[t] = counts.get(t, 0) + 1
I["topic_counts"] = counts
C["bag_has_scorer_topics"] = all(counts.get(t, 0) > 0 for t in
                                 ("/sim/drone_pose", "/sim/target_pose", "/yolo/detections"))

# 3b. start pose (5J). The stack's run bag starts after resume (5I: first pose 1.614 s after resume, 1.25 m into the
#     approach), so its first pose is not the start. Use the LAST /sim/drone_pose before resume from the pre-resume recorder
#     (rec.sh approach bag in --pre-resume-dir, resume time from <dir>/t_resume_ns.txt). No such pose -> loud FAIL.
p0 = next((m for _, t, m in msgs if t == "/sim/drone_pose"), None)
if p0 is not None and len(p0.data) >= 5:
    I["run_bag_first_pose"] = [round(v, 3) for v in (p0.data[0], p0.data[1], p0.data[2])] + [round(math.degrees(p0.data[4]), 3)]
pre_dir, sp_err, sp = opt("--pre-resume-dir"), None, None
if not pre_dir:
    sp_err = "no --pre-resume-dir given (the run bag starts after resume and cannot show the start)"
else:
    tr_p = os.path.join(pre_dir, "t_resume_ns.txt"); ab = sorted(glob.glob(os.path.join(pre_dir, "approach_bag", "*.mcap")))
    if not os.path.exists(tr_p):
        sp_err = f"missing {tr_p}"
    elif not ab:
        sp_err = f"no approach bag in {pre_dir}/approach_bag"
    else:
        tr = int(open(tr_p).read().strip()); pre = []
        try:
            from mcap_ros2.reader import read_ros2_messages as _rr
            for x in _rr(ab[0], topics=["/sim/drone_pose"]):
                if x.log_time_ns < tr and len(x.ros_msg.data) >= 5:
                    pre.append((x.log_time_ns, list(x.ros_msg.data)))
        except Exception:                                    # noqa: BLE001  (truncated bag: no summary) -> stream read
            from mcap.reader import NonSeekingReader
            from mcap_ros2.decoder import DecoderFactory
            pre = []
            with open(ab[0], "rb") as fh:
                try:
                    for _s, ch, m, dm in NonSeekingReader(fh, decoder_factories=[DecoderFactory()]).iter_decoded_messages(
                            topics=["/sim/drone_pose"], log_time_order=False):
                        if m.log_time < tr and len(dm.data) >= 5:
                            pre.append((m.log_time, list(dm.data)))
                except Exception:                            # noqa: BLE001
                    pass
        if not pre:
            sp_err = f"no /sim/drone_pose before resume in {ab[0]}"
        else:
            pre.sort(); sp = pre[-1][1]
            I["start_pose_source"] = f"last pre-resume /sim/drone_pose of {len(pre)}, {ab[0]}"
            I["start_pose_t_before_resume_s"] = round((tr - pre[-1][0]) / 1e9, 3)
            I["pre_resume_pose_spread_m"] = round(max(math.dist(q[1][:3], sp[:3]) for q in pre), 4)
if sp is not None:
    x0, y0, z0, yaw0 = sp[0], sp[1], sp[2], math.degrees(sp[4])
    exp = GEOM["start_xyz_yaw"]; dpos = math.dist((x0, y0, z0), exp[:3])
    dyaw = abs((yaw0 - GEOM["start_yaw_deg"] + 180) % 360 - 180)
    I["start_pose"] = [round(x0, 3), round(y0, 3), round(z0, 3), round(yaw0, 3)]
    I["start_offset_m"], I["start_dyaw_deg"] = round(dpos, 3), round(dyaw, 3)
    C["start_pose_expected"] = dpos <= GEOM["start_tol_m"] and dyaw <= GEOM["start_yaw_tol_deg"]
else:
    I["start_pose_error"] = sp_err
    print(f"START POSE CHECK FAILED: {sp_err}", file=sys.stderr)
    C["start_pose_expected"] = False

# 4. our scorer (benchmarks/inloop_accuracy.py at this repo's HEAD). The capture pose needs the recording
#    host's realtime - monotonic offset, recorded before the run in <label>_clock_offset.json; the
#    scorer refuses to run without it, so a missing file fails here instead of falling back.
co = None
try:
    co = int(json.load(open(os.path.join(d, f"{label}_clock_offset.json")))["realtime_minus_monotonic_ns"])
except (OSError, KeyError, ValueError) as e:
    I["clock_offset_error"] = str(e)
I["clock_offset_ns"] = co
C["clock_offset_recorded"] = co is not None
# 5Q: Table 14 is scored from the pre-resume approach bag (rec.sh, up before resume, carries /yolo/detections), because the
#     stack's bag starts about 1.6 s after resume and misses most of 18-20 m (5I: 9 messages there). The stack bag is
#     scored too (_inloop_stackbag.json, cross-check only), and every detection message in the stack bag must appear,
#     identical, in the approach bag.
ab_mc = sorted(glob.glob(os.path.join(pre_dir, "approach_bag", "*.mcap"))) if pre_dir else []
I["t14_source_bag"] = ab_mc[0] if ab_mc else None


def _score(bag, out_json, out_txt):
    cmdl = [sys.executable, f"{OURS}/benchmarks/inloop_accuracy.py", "--bag", bag,
            "--target", ",".join(str(v) for v in GEOM["target_xyz"]),
            "--fx", str(float(I["param_cam_fx"]) if I["param_cam_fx"] else GEOM["fx"]),
            "--clock-offset-ns", str(co),
            "--telemetry", P + "_telemetry.csv",
            "--cam-stamps", P + "_cam_stamps.csv", "--label", label,
            "--bins", ",".join(str(v) for v in sorted({e for b in PROTO["t14_bins_m"] for e in b})),
            "--json-out", out_json, "--cache-dir", os.path.join(os.path.dirname(Q), ".inloop_cache")]
    p = subprocess.run(cmdl, capture_output=True, text=True)
    open(out_txt, "x").write(p.stdout + p.stderr)
    return p.returncode, (p.stdout + p.stderr).strip().splitlines()[-3:]


def _dets_by_stamp(bag):
    from mcap_ros2.reader import read_ros2_messages as _rr
    out = {}
    for x in _rr(bag, topics=["/yolo/detections"]):
        h = x.ros_msg.header.stamp
        out[(h.sec, h.nanosec)] = [(d.class_name, round(d.confidence, 6), round(d.center_x, 3), round(d.center_y, 3),
                                    round(d.size_width, 3), round(d.size_height, 3)) for d in x.ros_msg.detections]
    return out


if ab_mc and co is not None:
    rc, I["scorer_tail"] = _score(ab_mc[0], Q + "_inloop.json", Q + "_inloop.txt")
    I["scorer_rc"] = rc
    C["scorer_runs"] = rc == 0 and os.path.exists(Q + "_inloop.json")
    try:
        g = json.load(open(Q + "_inloop.json"))["geometry"] if C["scorer_runs"] else {}
    except (OSError, KeyError, ValueError):
        g = {}
    I["geometry_gate"] = {k: g.get(k) for k in ("gate_timing", "rms_gate_px", "frozen_target_reprojection_rms_px",
                                                "frozen_target_reprojection_rms_delivery_px")}
    C["geometry_gate_is_protocol"] = (g.get("rms_gate_px") is not None
                                      and abs(g["rms_gate_px"] - PROTO["geometry_gate_px"]) <= 0.01)
    C["geometry_gate_capture_pass"] = (g.get("gate_timing") == "capture"
                                       and g.get("frozen_target_reprojection_rms_px") is not None
                                       and g["frozen_target_reprojection_rms_px"] <= g["rms_gate_px"])
    if mc:
        rc2, _t = _score(mc[0], Q + "_inloop_stackbag.json", Q + "_inloop_stackbag.txt")
        I["scorer_stackbag_rc"] = rc2
        da, ds = _dets_by_stamp(ab_mc[0]), _dets_by_stamp(mc[0])
        same = sum(1 for k, v in ds.items() if da.get(k) == v)
        I["dets_msgs_approach_bag"], I["dets_msgs_stack_bag"], I["dets_stack_msgs_identical_in_approach"] = len(da), len(ds), same
        C["approach_bag_has_all_stack_dets"] = len(ds) > 0 and same == len(ds)
    else:
        C["approach_bag_has_all_stack_dets"] = False
else:
    if not ab_mc:
        I["t14_source_error"] = "no approach bag (pre-resume recorder) for this run"
    C["scorer_runs"] = False
    C["geometry_gate_is_protocol"] = False
    C["geometry_gate_capture_pass"] = False
    C["approach_bag_has_all_stack_dets"] = False

# 5. final distance, mission
try:
    ms = json.load(open(P + "_mission.json"))
    for k in ("reached", "final_state", "time_to_reached_sim_s", "final_dist_frozen_sign_m",
              "reached_dist_frozen_sign_m", "n_reacquire", "n_lost"):
        I[k] = ms.get(k)
except Exception as e:                                   # noqa: BLE001
    I["mission_error"] = str(e)
# PROVISIONAL (pending Amar on the 0.55 / 0.50 ratios): final horizontal distance to the frozen
# sign must fall in the window for the current fx (sim_geometry.json).
lo, hi = GEOM["smoke_final_dist_window_m"]
I["final_dist_window_m"] = [lo, hi]
I["final_dist_window_status"] = GEOM["smoke_final_dist_window_status"]
fd = I.get("final_dist_frozen_sign_m")
C["final_dist_in_window_provisional"] = fd is not None and lo <= fd <= hi

# 5b. (5Q) fixed hold: the run's final held render (hold_gate.sh, DIR/held_final_sha1.txt) must be the reference render
hr = os.path.join(pre_dir, "held_final_sha1.txt") if pre_dir else None
try:
    last = open(hr).read().strip() or None
except (OSError, TypeError) as e:
    last = None; I["held_render_error"] = str(e)
I["held_render_sha1"], I["held_reference_sha1"] = last, HELD["render_sha1"]
C["held_render_is_reference"] = last == HELD["render_sha1"]

# 5c. (5Q) expected standoff from the run's own FSM ratios, closest approach and overshoot (reported whether or not it happens)
def _pf(k):
    v = pval(k)
    try:
        return float(v)
    except (TypeError, ValueError):
        return None
fy, ih, rt, rh = _pf("cam_fy"), _pf("image_height"), _pf("target_bbox_ratio"), _pf("hold_bbox_ratio")
H_SIGN = PROTO["sign_height_m"]
if fy and ih and rt and rh:
    I["standoff_expected_m"] = {"vx_zero_at_target_bbox_ratio": round(fy * H_SIGN / (rt * ih), 3),
                                "reached_needs_within_hold_bbox_ratio": round(fy * H_SIGN / (rh * ih), 3),
                                "ratios": [rt, rh], "cam_fy": fy, "sign_height_m": H_SIGN}
tx, ty = GEOM["target_xyz"][:2]
dd = [(t, math.hypot(tx - m.data[0], ty - m.data[1])) for t, top, m in msgs if top == "/sim/drone_pose" and len(m.data) >= 2]
if dd:
    tmin, dmin = min(dd, key=lambda x: x[1])
    I["closest_dist_frozen_sign_m"] = round(dmin, 3)
    I["closest_at_s_after_bag_start"] = round((tmin - dd[0][0]) / 1e9, 2)
    I["last_dist_frozen_sign_m"] = round(dd[-1][1], 3)
    I["overshoot_m"] = round(dd[-1][1] - dmin, 3)          # how far the drone moved back out after its closest point
    se = I.get("standoff_expected_m")
    I["inside_vx_zero_standoff_m"] = round(max(0.0, se["vx_zero_at_target_bbox_ratio"] - dmin), 3) if se else None

# 6. OV2SLAM: tracking state topic sample + /slam/pose flow over the run
I["slam_state_sample"] = rd(P + "_slam_state.txt").strip()[:200]
sp = [t for t, top, _ in msgs if top == "/slam/pose"]
if msgs:
    t_end = msgs[-1][0]
    I["slam_pose_msgs"] = len(sp)
    I["slam_pose_last_age_s"] = round((t_end - sp[-1]) / 1e9, 2) if sp else None
    I["slam_tracking"] = "tracking" if sp and (t_end - sp[-1]) / 1e9 < 2.0 else "lost"
m = re.findall(r"(reset|lost|LOST|tracking lost|Reset)", log)
I["slam_log_reset_or_lost_mentions"] = len(m)

# 7. forward-speed law per APPROACHING tick against /cmd_vel (ramp limiter applied).
#    Mimics servo_fsm_node.cpp: best "stop sign" box with conf >= 0.20 (max conf), 0.30 jump
#    filter, 5-sample mean of size_height, ratio = bh/480; vx_law = 3.0*max(0, 0.55 - ratio),
#    clamp 3.0, then ramp 1.5 per published tick from the previous published vx.
K, RT, VMAX, RAMP, H, WIN = 3.0, 0.55, 3.0, 1.5, 480.0, 5
hist = deque(maxlen=WIN); prev = None; pending = None; prev_vx = 0.0
rows = []
for i, (t, top, msg) in enumerate(msgs):
    if top == "/yolo/detections":
        best = None
        for dd in msg.detections:
            if dd.class_name == "stop sign" and dd.confidence >= 0.20 and \
               (best is None or dd.confidence > best.confidence):
                best = dd
        pending = None
        if best is None:
            prev = None
            continue
        if prev is not None and (abs(best.center_x - prev[0]) / 640 > 0.30 or
                                 abs(best.center_y - prev[1]) / H > 0.30):
            prev = (best.center_x, best.center_y)
            continue
        prev = (best.center_x, best.center_y)
        hist.append(best.size_height)
        pending = sum(hist) / len(hist) / H
    elif top == "/cmd_vel":
        vx = msg.linear.x
        if pending is not None:
            state = None                    # /bench/state follows each /cmd_vel publish
            for t2, top2, m2 in msgs[i + 1:i + 8]:
                if top2 == "/bench/state":
                    state = m2.data.split(",")[0]
                    break
            if state == "APPROACHING":
                law = min(VMAX, K * max(0.0, RT - pending))
                exp = prev_vx + max(-RAMP, min(RAMP, law - prev_vx))
                rows.append((pending, law, exp, vx))
            pending = None
        prev_vx = vx
if rows:
    res = sorted(abs(e - v) for _, _, e, v in rows)
    I["fwd_law_ticks"] = len(rows)
    I["fwd_law_abs_resid_median"] = round(res[len(res) // 2], 4)
    I["fwd_law_abs_resid_p95"] = round(res[int(0.95 * (len(res) - 1))], 4)
    I["fwd_law_abs_resid_max"] = round(res[-1], 4)
    I["fwd_law_frac_within_0.02"] = round(sum(1 for x in res if x <= 0.02) / len(res), 3)
    I["fwd_law_ramp_limited_ticks"] = sum(1 for _, law, e, _ in rows if abs(law - e) > 1e-9)
else:
    I["fwd_law_ticks"] = 0

# 8. NPU YOLO26n only: Det latency vs sentinel (context only)
if npu_sentinel:
    try:
        sm = json.load(open(os.path.join(d, f"{label}_summary.json")))
        dl = sm["detector_latency_ms"]["det_path"]["mean"]
        I["det_latency_ms"] = dl
        I["det_latency_vs_sentinel_pct"] = round(100 * (dl - SENTINEL_MS) / SENTINEL_MS, 2) if dl else None
    except Exception as e:                               # noqa: BLE001
        I["det_latency_error"] = str(e)

r["pass"] = all(C.values())
json.dump(r, open(Q + "_check.json", "x"), indent=2)
print(json.dumps(r, indent=2))
sys.exit(0 if r["pass"] else 1)
