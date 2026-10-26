#!/usr/bin/env python3
"""check_run.py <stage3_dir> <label> [--npu-sentinel] [--pre-resume-dir DIR] [--out-dir DIR]: Prompt 3F Stage 3 pass checks.
5J: start_pose_expected uses the last /sim/drone_pose recorded BEFORE resume, from DIR/approach_bag/*.mcap (rec.sh bag,
up before resume) and DIR/t_resume_ns.txt (sidecar); it fails loudly if there is no such pose. Outputs (_check.json,
_inloop.json, _inloop.txt) go to --out-dir (default: the stage3 dir) and are never overwritten (refuse: exit 1, message on stderr).
Run with ~/venvs/mcap/bin/python (needs mcap_ros2). Plumbing checks only: no number goes into a table.
5Q (benchmarks/hil_runs): paths from hilpaths.py; the scorer gets the Table 14 bins of config/protocol_fx554.json and its cache
goes to --out-dir (never into the input folder); new checks geometry_gate_is_protocol (the scorer gate equals the protocol's
18.47 px) and held_render_is_reference (DIR/held_final_sha1.txt, written by hold_gate.sh, is the render of config/held_reference.json); new info
standoff_expected_m (from the run's own FSM ratios), closest_dist_frozen_sign_m and overshoot_m (always reported).
8I: new check first_det_before_resume (the detector's first target box must land before the resume; source r20_dets.log and
t_resume_ns.txt in --pre-resume-dir). 8J: new check latched_at_resume (the controller's sim-health latch must be engaged at
resume, from the stack log's "Sim health LOST"/"RECOVERED" stamps), and t_resume_ns.txt now carries the resume gate's exact
wall ns (the sidecar's polled value moved to t_resume_ns_sidecar.txt). See DEVIATIONS 17.67.
9C: geometry_gate_is_protocol asks a CONFIGURATION question (does this run's scoring use the protocol's gate?) and is computed
from the run's fx through the scorer's own default_rms_gate, independent of whether the scorer produced output (17.69).
17D/18A/18B (DEVIATIONS 17.81, 17.82, 17.83): two liveness checks, both validity checks in batch.sh. first_motion_ok,
proportional and anchored: per channel, the first 3 s interval after resume starting at a /cmd_vel message that ITSELF commands
the tested channel (|vx| or |vy| > 0 for translation, wz != 0 for yaw; 18B, the 028 tail-catch mechanism of RESULTS_18A) and
whose zero-order-hold command integral reaches 0.20 m (horizontal velocity vector, as a magnitude) or 0.10 rad (wz) must show
at least half the commanded amount in the measured pose (horizontal displacement magnitude; unwrapped yaw change in the sign
of the integral); a channel with no qualifying interval inside the mission window is n/a; False if a tested channel fails or
if the run resumed with both channels n/a; a run that never resumed is n/a. The same ratio over the interval shifted 1.0 s
later is a covariate, never a check (18B: the transport delay is 0.3 to 0.8 s, so the shifted window shows the applied motion). slam_coload_alive: the OV2SLAM co-load ran (stack log: "SLAM sidecar started", no "error
while loading shared libraries"; ov2slam_node in the last topthreads sample at or before resume + mission_window_s OR in the
first sample after it: level2_coresampler.py line 43 lists only threads that existed in the previous 15 s sample, so a co-load
restarted late in the window is invisible for one sample). The /slam/pose count, first-pose delay and largest pose gap are
covariates (info), never checks.
25C (R3, DEVIATIONS 17.86): slam_coload_alive asks for the cell's EXPECTED SLAM process from the run's run_config.yaml
(slam.enabled, slam.type): none for a SLAM-off cell, whose absence is required (stack log, topthreads, procmem); ov2slam_node
otherwise, as above, plus, where the Level 2 sampler ran, one pid without a gap from its first row in the window to the end,
that first row at most 10 s after resume, and the first row after the window with the same pid; a non-frozen setup run
without procmem.csv fails."""
import datetime, glob, importlib.util, json, math, os, re, subprocess, sys
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
        tr = int(open(tr_p).read().split()[0]); pre = []   # 8J: field 1 is the gate's exact resume wall ns
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

# 3c. 8I (DEVIATIONS 17.67): the held start's precondition. The detector's FIRST target box must land BEFORE resume.
#     Until 8I the hold was released as soon as the stack was up, so a detector that needs seconds per frame left the FSM
#     in SEARCHING, and its 0.5 rad/s search spin was the /cmd_vel the supervisor applied at resume (run 006 of
#     2026-10-08: 53.00 deg of heading lost before the first detection arrived at resume + 2.87 s). Plumbing, not a
#     result: the session gate has already shown that every configuration detects the held frame, so a run that starts
#     without one is INVALID and the batch stops. Source: the sidecar's detection logger r20_dets.log (field 1 host wall
#     ns, field 3 the count of stop-sign boxes at conf >= 0.20, the controller's own target class and min_confidence),
#     on the same host clock as t_resume_ns.txt.
tr_ns, fd_ns = None, None
if pre_dir:
    try:
        tr_ns = int(open(os.path.join(pre_dir, "t_resume_ns.txt")).read().split()[0])
    except (OSError, ValueError) as e:
        I["t_resume_ns_error"] = str(e)
    dl = os.path.join(pre_dir, "r20_dets.log")
    if os.path.exists(dl):
        for _ln in open(dl, errors="replace"):
            _f = _ln.split()
            if len(_f) >= 3 and _f[2].isdigit() and int(_f[2]) > 0:
                fd_ns = int(_f[0]); break
        if fd_ns is None:
            I["first_det_error"] = f"no detection message with a target box in {dl}"
    else:
        I["first_det_error"] = f"missing {dl}"
else:
    I["first_det_error"] = "no --pre-resume-dir given"
I["t_resume_ns"], I["first_det_ns"] = tr_ns, fd_ns
I["first_det_before_resume_s"] = round((tr_ns - fd_ns) / 1e9, 3) if (tr_ns and fd_ns) else None
C["first_det_before_resume"] = bool(tr_ns and fd_ns and fd_ns < tr_ns)

# 8J (B3): the other half of one start state. The controller's sim-health latch must be ENGAGED at resume, so the run
#     begins motionless with cmd_vel held at 0 and takes its first command about 0.5 s later when the recovery debounce
#     clears. Without this, the latch's state at resume depended on how long the controller had been up before the first
#     box: session 1 resumed 001, 003, 004 latched and 002, 005, 006 with a live command already in force, a split that
#     lines up with CPU against NPU. Read from the stack log's own ROS stamps (wall clock, the same clock as
#     t_resume_ns): the last "Sim health LOST" before resume, with no "Sim health RECOVERED" between it and resume.
lost = [int(a) * 10**9 + int(b) for a, b in re.findall(r"\[(\d{10})\.(\d{9})\][^\n]*Sim health LOST", log)]
recov = [int(a) * 10**9 + int(b) for a, b in re.findall(r"\[(\d{10})\.(\d{9})\][^\n]*Sim health RECOVERED", log)]
lat_t = max((t for t in lost if tr_ns and t < tr_ns), default=None)
I["latch_lost_before_resume_s"] = round((tr_ns - lat_t) / 1e9, 3) if lat_t else None
I["latch_recovered_after_resume_s"] = round((min((t for t in recov if tr_ns and t > tr_ns), default=0) - tr_ns) / 1e9, 3) \
    if (tr_ns and any(t > tr_ns for t in recov)) else None
I["latch_n_lost"], I["latch_n_recovered"] = len(lost), len(recov)
C["latched_at_resume"] = bool(lat_t) and not any(lat_t < t < tr_ns for t in recov)

# 3d. 17D/18A/18B (DEVIATIONS 17.81, 17.82, 17.83): first_motion_ok. Two transport-class faults in 181 in-loop runs (REPORT_S4
#     attempt 1 run 006, 17.79 run 032) showed that nothing checked that the simulator APPLIED the commands the controller
#     published.
#     PROPORTIONAL (18A, planning chat): a fixed displacement floor after the first command above a fixed threshold could stop
#     a batch on a small real command (17.81 f, the yaw margin on 5a run 007), so the commanded amount itself sets the floor.
#     ANCHORED (18B, planning chat on RESULTS_18A): the interval's starting /cmd_vel message must itself command the tested
#     channel (|vx| or |vy| > 0 for translation, wz != 0 for yaw). Without it, an interval can start in a zero-command span and
#     catch the first fraction of a late burst at its tail, where the 0.3 to 0.8 s transport delay pushes the measured response
#     outside the interval: first_motion_ok as settled in 18A was False on 30 of 161 stored valid runs through that artifact
#     (the 028_yolov8s_cpu_B_rc1 mechanism, RESULTS_18A); with the anchor it is False on the one true rig fault (032) alone.
#     Per channel, from the approach bag (rec.sh, up before resume; /cmd_vel and /sim/drone_pose on the same recorder clock):
#     the first FM_WIN_S interval after resume, starting at a /cmd_vel message that commands the tested channel, over which the
#     commanded horizontal velocity vector (zero-order hold) integrates to a magnitude of at least FM_CMD_DISP_M (translation),
#     or the integrated wz reaches
#     FM_CMD_YAW_RAD in magnitude (yaw); the measured horizontal displacement magnitude (translation) or unwrapped yaw change
#     in the sign of the integral (yaw) over that interval must be at least FM_RATIO of the commanded amount. Magnitudes keep
#     the check independent of the simulator's command frame (world axes today, SIM_PARAMS.md line 17, and that convention may
#     change; the 16D audit's heading projection is not used); the signed world-x and world-y displacements stay covariates.
#     A channel with no qualifying interval inside the mission window is n/a. False if a tested channel fails, or if the run
#     resumed and both channels are n/a (a live controller always searches or approaches). A run that never resumed is n/a
#     (True here; first_det_before_resume and the resume gate catch it). Interval start, commanded amount, measured amount and
#     their ratio are covariates per channel, plus (18B) the measured amount and ratio over the interval shifted 1.0 s later
#     (same commanded amount; the transport-delay view): covariates only, never a check.
FM_WIN_S, FM_CMD_DISP_M, FM_CMD_YAW_RAD, FM_RATIO = 3.0, 0.20, 0.10, 0.5
_mb = re.match(r"^level2_(.+)_(npu|cpu)_[AB]_r", label)
MODEL_BACKEND = f"{_mb.group(1)}_{_mb.group(2)}" if _mb else None
MISSION_WINDOW_S = PROTO["mission_window_s"].get(MODEL_BACKEND, PROTO["mission_window_s"]["default"])


def _zoh_integral(cmds, t0, t1):
    """Integrals of (vx, vy, wz) over [t0, t1] ns, zero-order hold: each command holds until the next message (the last one
    to t1). Commands are sorted; a command issued before t0 and still held at t0 contributes from t0."""
    ivx = ivy = iwz = 0.0
    for i, c in enumerate(cmds):
        a = max(c[0], t0); b = min(cmds[i + 1][0], t1) if i + 1 < len(cmds) else t1
        if b <= a:
            continue
        dt = (b - a) / 1e9
        ivx += c[1] * dt; ivy += c[2] * dt; iwz += c[3] * dt
    return ivx, ivy, iwz


def _read_cmd_pose(bag):
    """(/cmd_vel as (log ns, vx, vy, wz), /sim/drone_pose as (log ns, x, y, z, yaw)) over the whole approach bag; the same
    truncated-bag fallback as the start-pose read."""
    cmds, poses = [], []
    try:
        from mcap_ros2.reader import read_ros2_messages as _rr
        for x in _rr(bag, topics=["/cmd_vel", "/sim/drone_pose"]):
            if x.channel.topic == "/cmd_vel":
                cmds.append((x.log_time_ns, x.ros_msg.linear.x, x.ros_msg.linear.y, x.ros_msg.angular.z))
            elif len(x.ros_msg.data) >= 5:
                poses.append((x.log_time_ns, x.ros_msg.data[0], x.ros_msg.data[1], x.ros_msg.data[2], x.ros_msg.data[4]))
    except Exception:                                        # noqa: BLE001  (truncated bag: no summary) -> stream read
        from mcap.reader import NonSeekingReader
        from mcap_ros2.decoder import DecoderFactory
        cmds, poses = [], []
        with open(bag, "rb") as fh:
            try:
                for _s, ch, m, dm in NonSeekingReader(fh, decoder_factories=[DecoderFactory()]).iter_decoded_messages(
                        topics=["/cmd_vel", "/sim/drone_pose"], log_time_order=False):
                    if ch.topic == "/cmd_vel":
                        cmds.append((m.log_time, dm.linear.x, dm.linear.y, dm.angular.z))
                    elif len(dm.data) >= 5:
                        poses.append((m.log_time, dm.data[0], dm.data[1], dm.data[2], dm.data[4]))
            except Exception:                                # noqa: BLE001
                pass
    cmds.sort(); poses.sort()
    return cmds, poses


def _pose_at_or_before(poses, t):
    import bisect
    i = bisect.bisect_right([p[0] for p in poses], t) - 1
    return poses[i] if i >= 0 else (poses[0] if poses else None)


FM = {"rule": f"per channel, the first {FM_WIN_S} s interval after resume starting at a /cmd_vel message that itself commands "
              f"the tested channel and whose zero-order-hold command integral reaches {FM_CMD_DISP_M} m (horizontal vector "
              f"magnitude) or {FM_CMD_YAW_RAD} rad (wz) must show at least {FM_RATIO} of the commanded amount in the measured "
              f"pose, in the sign of the integral for yaw",
      "resumed": bool(tr_ns), "vx": "n/a", "wz": "n/a"}
if not tr_ns:
    FM["status"] = "n/a: the run never resumed"; C["first_motion_ok"] = True
elif not ab:
    FM["status"] = "no approach bag"; C["first_motion_ok"] = False
else:
    _cmds, _poses = _read_cmd_pose(ab[0])
    FM["cmd_vel_msgs_after_resume"] = sum(1 for c in _cmds if c[0] > tr_ns)
    FM["poses_after_resume"] = sum(1 for p in _poses if p[0] > tr_ns)
    _win_ns, _w_end_ns = int(FM_WIN_S * 1e9), tr_ns + int(MISSION_WINDOW_S * 1e9)
    tested = []
    for chan in ("vx", "wz"):
        t0 = cmd_amt = None
        for c in _cmds:
            if c[0] <= tr_ns:
                continue
            if c[0] + _win_ns > _w_end_ns:
                break
            if (chan == "vx" and c[1] == 0 and c[2] == 0) or (chan == "wz" and c[3] == 0):
                continue                 # 18B anchor: the starting message must itself command the tested channel
            ivx, ivy, iwz = _zoh_integral(_cmds, c[0], c[0] + _win_ns)
            amt = math.hypot(ivx, ivy) if chan == "vx" else iwz
            if abs(amt) >= (FM_CMD_DISP_M if chan == "vx" else FM_CMD_YAW_RAD):
                t0, cmd_amt = c[0], amt; break
        if t0 is None:
            continue                     # channel n/a: no qualifying interval inside the mission window
        FM[f"{chan}_interval_start_s"] = round((t0 - tr_ns) / 1e9, 3); FM[f"{chan}_cmd_amount"] = round(cmd_amt, 4)
        pa = _pose_at_or_before(_poses, t0); pb = _pose_at_or_before(_poses, t0 + _win_ns)
        if pa is None or pb is None or pb[0] - pa[0] < 0.5 * _win_ns:
            FM[f"{chan}_pose_window_s"] = round((pb[0] - pa[0]) / 1e9, 3) if (pa and pb) else None
            FM[chan] = False; FM[f"{chan}_note"] = "no pose window of at least 1.5 s over the interval"; tested.append(False); continue
        if chan == "vx":
            dx, dy = pb[1] - pa[1], pb[2] - pa[2]
            meas = math.hypot(dx, dy); FM["disp_x_world_m"], FM["disp_y_world_m"] = round(dx, 4), round(dy, 4)
            FM[chan] = meas >= FM_RATIO * cmd_amt
        else:
            seg = [p for p in _poses if pa[0] <= p[0] <= pb[0]]
            meas = sum((b[4] - a[4] + math.pi) % (2 * math.pi) - math.pi for a, b in zip(seg, seg[1:]))
            FM[chan] = meas * math.copysign(1.0, cmd_amt) >= FM_RATIO * abs(cmd_amt)
        FM[f"{chan}_meas_amount"] = round(meas, 4); FM[f"{chan}_ratio"] = round(meas / cmd_amt, 4)
        # 18B covariate, never a check: the same measured amount and ratio over the interval shifted 1.0 s later (same
        # commanded amount). The transport delay is 0.3 to 0.8 s, so the shifted window shows the motion the simulator applied.
        _ts = t0 + int(1.0 * 1e9)
        pa_s, pb_s = _pose_at_or_before(_poses, _ts), _pose_at_or_before(_poses, _ts + _win_ns)
        if pa_s is not None and pb_s is not None and pb_s[0] - pa_s[0] >= 0.5 * _win_ns:
            if chan == "vx":
                meas_s = math.hypot(pb_s[1] - pa_s[1], pb_s[2] - pa_s[2])
            else:
                seg_s = [p for p in _poses if pa_s[0] <= p[0] <= pb_s[0]]
                meas_s = sum((b[4] - a[4] + math.pi) % (2 * math.pi) - math.pi for a, b in zip(seg_s, seg_s[1:]))
            FM[f"{chan}_meas_amount_shift1s"] = round(meas_s, 4); FM[f"{chan}_ratio_shift1s"] = round(meas_s / cmd_amt, 4)
        else:
            FM[f"{chan}_meas_amount_shift1s"] = FM[f"{chan}_ratio_shift1s"] = None
        tested.append(FM[chan])
    if not tested:
        FM["status"] = "FAIL: resumed but no command interval reached the threshold on either channel"; C["first_motion_ok"] = False
    else:
        C["first_motion_ok"] = all(tested)
        FM["status"] = "ok" if C["first_motion_ok"] else "FAIL: a commanded channel produced too little motion"
I["first_motion"] = FM

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

# 9C (17.69): geometry_gate_is_protocol is a configuration check. It computes the gate the scorer WILL apply
# for this run, from the same fx that _score passes it, through the scorer's own default_rms_gate (and _score
# passes no --rms-gate override), and compares it to the protocol's value. It no longer reads scorer output,
# so a scorer refusal fails scorer_runs and geometry_gate_capture_pass (measurements) but not this check.
_spec = importlib.util.spec_from_file_location("_inloop_cfg", os.path.join(OURS, "benchmarks", "inloop_accuracy.py"))
_ia = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_ia)
_fx_used = float(I["param_cam_fx"]) if I["param_cam_fx"] else float(GEOM["fx"])
_gate_is_protocol = abs(_ia.default_rms_gate(_fx_used) - PROTO["geometry_gate_px"]) <= 0.01


def _score(bag, out_json, out_txt):
    cmdl = [sys.executable, f"{OURS}/benchmarks/inloop_accuracy.py", "--bag", bag,
            "--target", ",".join(str(v) for v in GEOM["target_xyz"]),
            "--fx", str(float(I["param_cam_fx"]) if I["param_cam_fx"] else GEOM["fx"]),
            "--clock-offset-ns", str(co),
            "--telemetry", P + "_telemetry.csv",
            "--cam-stamps", P + "_cam_stamps.csv", "--label", label,
            "--bins", ",".join(str(v) for v in sorted({e for b in PROTO["t14_bins_m"] for e in b})),
            "--json-out", out_json, "--cache-dir", os.path.join(os.path.dirname(Q), ".inloop_cache")]
    if GEOM.get("nontarget_signs_xyz"):    # 17.73: known non-target stop signs, never measured as the target
        cmdl += ["--nontarget-signs", ";".join(",".join(str(v) for v in s) for s in GEOM["nontarget_signs_xyz"])]
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
    C["geometry_gate_is_protocol"] = _gate_is_protocol
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
    C["geometry_gate_is_protocol"] = _gate_is_protocol
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

# 6b. 17D/18A (DEVIATIONS 17.81, 17.82): slam_coload_alive, Ahmed's definition (IMPROVEMENTS_16A item 1 note): the co-load
#     PROCESS ran. Session 5a attempt 1 ran four rows with the OV2SLAM sidecar dead (liblcdetector.so missing in the ablation
#     worktree) and nothing fired (REPORT_S5). Alive = the stack log says "SLAM sidecar started", holds no "error while loading
#     shared libraries" (the persisted sidecar tail is part of the stack log), and ov2slam_node appears in the LAST
#     topthreads.csv sample (smoke_run_hold.sh, 15 s cadence) at or before resume + mission_window_s (protocol_fx554.json, by
#     model_backend) OR in the FIRST sample after it (18A: level2_coresampler.py line 43 lists only threads that existed in the
#     previous 15 s sample, so a co-load restarted late in the window is absent from the last in-window sample while alive;
#     5a attempt 2 run 019, DEVIATIONS 17.81 e f). The deciding sample and its time are covariates. /slam/pose count,
#     first-pose delay and the largest pose gap are covariates, never checks: a pose gap is SLAM tracking behaviour, not a
#     missing co-load (s2 005 and s4 024 stay pooled). A run that never resumed is n/a (True here).
SC = {"sidecar_started": "SLAM sidecar started" in log, "library_error": "error while loading shared libraries" in log,
      "model_backend": MODEL_BACKEND, "mission_window_s": MISSION_WINDOW_S}
_tt = sorted(glob.glob(os.path.join(d, "*_topthreads.csv")))
SC["topthreads_file"] = os.path.basename(_tt[0]) if _tt else None
SC["ov2slam_in_last_window_sample"] = SC["ov2slam_in_first_sample_after_window"] = None
if tr_ns and _tt:
    import csv as _csv
    _samples = {}
    for row in _csv.DictReader(open(_tt[0])):
        try:
            tu = int(datetime.datetime.strptime(row["utc"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=datetime.timezone.utc).timestamp() * 1e9)
        except (KeyError, ValueError):
            continue
        _samples.setdefault(tu, []).append(row.get("comm", ""))
    _inwin = sorted(t for t in _samples if t <= tr_ns + int(SC["mission_window_s"] * 1e9))
    SC["samples_in_window"] = len(_inwin)
    SC["samples_in_window_with_ov2slam"] = sum(1 for t in _inwin if any(c.startswith("ov2slam") for c in _samples[t]))
    if _inwin:
        SC["last_window_sample_s_after_resume"] = round((_inwin[-1] - tr_ns) / 1e9, 1)
        SC["ov2slam_in_last_window_sample"] = any(c.startswith("ov2slam") for c in _samples[_inwin[-1]])
    _after = sorted(t for t in _samples if t > tr_ns + int(SC["mission_window_s"] * 1e9))
    SC["ov2slam_in_first_sample_after_window"] = any(c.startswith("ov2slam") for c in _samples[_after[0]]) if _after else None
    SC["first_sample_after_window_s_after_resume"] = round((_after[0] - tr_ns) / 1e9, 1) if _after else None
# 25C (R3, DEVIATIONS 17.86): the check now asks for the cell's EXPECTED SLAM process, read from the run's own launcher
#     record (<label>_run1_run_config.yaml, block slam:, keys enabled and type). enabled false (a SLAM-off cell): the expected
#     process is none and its ABSENCE is required over the whole run: the stack log says "SLAM will be skipped" and never
#     "SLAM sidecar started", no topthreads sample up to and including the first one after the mission window lists an ov2slam
#     thread, and the Level 2 sampler (<tag>_procmem.csv, required here, with rows past the window's end) never found
#     ov2slam_node, an ORB-SLAM3 node or a SLAM container. Any type
#     other than ov2slam with SLAM enabled has no expected process defined: False. Otherwise (every stored run: enabled true,
#     type ov2slam) the ov2slam branch below decides exactly as before, and where the Level 2 sampler ran it must also show
#     ov2slam_node without interruption, one pid, from its first row in the mission window to the window's end, that
#     first row at most SLAM_START_MAX_S after resume (the sidecar starts 4.2 to 7.2 s after resume by design in the 22 stored
#     Level 2 runs, so presence from resume itself is not asked; the 10 s bound is Ahmed's ruling of 2026-10-25, prompt 25C),
#     and the first row AFTER the window carrying the same pid, so one process ran to the window's end even if the sampler's
#     rows stop early. A run of a non-frozen simulator setup (sim_setup.json in --pre-resume-dir, written by run_one.sh for
#     such runs only) without procmem.csv fails: the Level 2 sampler is required there (batch.sh asks for L2_PROCMEM=1).
SLAM_START_MAX_S = 10.0
def _yaml_block(txt, name):
    out, on = {}, False
    for ln in txt.splitlines():
        if re.match(rf"^{name}:\s*$", ln):
            on = True; continue
        if on:
            if ln and not ln[0].isspace():
                break
            m = re.match(r"^\s+(\w+):\s*(.*?)\s*$", ln)
            if m:
                out[m.group(1)] = m.group(2).strip('"')
    return out
_slam_cfg = _yaml_block(rd(P + "_run_config.yaml"), "slam")
_slam_off = _slam_cfg.get("enabled") == "false"
_slam_other = not _slam_off and _slam_cfg.get("type") not in (None, "ov2slam")
_pm = sorted(glob.glob(os.path.join(d, "*_procmem.csv")))


def _pm_rows(names):
    import csv as _csv
    out = []
    for row in _csv.DictReader(open(_pm[0])):
        if row.get("name") not in names:
            continue
        try:
            tu = int(datetime.datetime.strptime(row["utc"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=datetime.timezone.utc).timestamp() * 1e9)
        except (KeyError, ValueError):
            continue
        out.append((tu, row.get("name"), row.get("pid") or ""))
    return out


if not tr_ns:
    SC["status"] = "n/a: the run never resumed"; C["slam_coload_alive"] = True
elif _slam_off:
    SC["expected_slam_process"] = None
    SC["log_says_slam_skipped"] = "SLAM will be skipped" in log
    _upto = []
    if _tt:
        _w = tr_ns + int(SC["mission_window_s"] * 1e9)
        _upto = sorted(t for t in _samples if t <= _w) + sorted(t for t in _samples if t > _w)[:1]
    SC["samples_checked"] = len(_upto)
    SC["samples_with_ov2slam"] = sum(1 for t in _upto if any(c.startswith("ov2slam") for c in _samples[t]))
    SC["procmem_file"] = os.path.basename(_pm[0]) if _pm else None
    _pr = _pm_rows(("ov2slam_node", "orbslam3_node", "slam_container")) if _pm else []
    SC["procmem_slam_rows"], SC["procmem_slam_rows_with_pid"] = len(_pr), sum(1 for r in _pr if r[2])
    SC["procmem_rows_after_window"] = sum(1 for r in _pr if r[0] > tr_ns + int(SC["mission_window_s"] * 1e9))
    C["slam_coload_alive"] = bool(SC["log_says_slam_skipped"] and not SC["sidecar_started"] and _tt and _upto
                                  and SC["samples_with_ov2slam"] == 0 and _pm and _pr and SC["procmem_slam_rows_with_pid"] == 0
                                  and SC["procmem_rows_after_window"] > 0)
    SC["status"] = "ok: no SLAM process, as the cell requires" if C["slam_coload_alive"] else "FAIL: " + "; ".join(w for w, bad in (
        ("the stack log does not say SLAM will be skipped", not SC["log_says_slam_skipped"]),
        ("SLAM sidecar started", SC["sidecar_started"]), ("no topthreads sample", not _upto),
        ("an ov2slam thread in a topthreads sample", SC["samples_with_ov2slam"] > 0),
        ("no procmem record to show the absence", not (_pm and _pr)),
        ("no procmem row after the mission window (the sampler stopped early)", bool(_pm and _pr) and SC["procmem_rows_after_window"] == 0),
        ("the procmem sampler found a SLAM process", SC["procmem_slam_rows_with_pid"] > 0)) if bad)
elif _slam_other:
    SC["expected_slam_process"] = None
    SC["status"] = f"FAIL: no expected SLAM process is defined for slam.type {_slam_cfg.get('type')}"; C["slam_coload_alive"] = False
else:
    SC["decided_by_sample"], SC["decided_sample_s_after_resume"] = (
        ("last_in_window", SC.get("last_window_sample_s_after_resume")) if SC["ov2slam_in_last_window_sample"]
        else ("first_after_window", SC.get("first_sample_after_window_s_after_resume"))
        if SC["ov2slam_in_first_sample_after_window"] else (None, None))
    _present = bool(SC["ov2slam_in_last_window_sample"]) or bool(SC["ov2slam_in_first_sample_after_window"])
    C["slam_coload_alive"] = bool(SC["sidecar_started"] and not SC["library_error"] and _present)
    SC["status"] = "ok" if C["slam_coload_alive"] else "FAIL: " + "; ".join(w for w, bad in (
        ("sidecar not started", not SC["sidecar_started"]), ("library error in the sidecar or stack log", SC["library_error"]),
        ("no topthreads sample", not _tt),
        ("ov2slam_node absent from the last sample in the mission window and from the first sample after it",
         _tt and not _present)) if bad)
    if _pm:   # 25C: the Level 2 sampler ran, so presence is also asked of every 2 s row from the first one that found it
        _w = tr_ns + int(SC["mission_window_s"] * 1e9)
        _pr = [r for r in _pm_rows(("ov2slam_node",)) if tr_ns <= r[0] <= _w]
        _i0 = next((k for k, r in enumerate(_pr) if r[2]), None)
        _run = _pr[_i0:] if _i0 is not None else []
        SC["procmem_rows_in_window"] = len(_pr)
        SC["procmem_first_present_s_after_resume"] = round((_pr[_i0][0] - tr_ns) / 1e9, 1) if _i0 is not None else None
        SC["procmem_rows_missing_after_start"] = sum(1 for r in _run if not r[2])
        SC["procmem_pids_after_start"] = len({r[2] for r in _run if r[2]})
        SC["procmem_first_present_max_s"] = SLAM_START_MAX_S
        _late = SC["procmem_first_present_s_after_resume"] is not None and SC["procmem_first_present_s_after_resume"] > SLAM_START_MAX_S
        _nx = next((r for r in _pm_rows(("ov2slam_node",)) if r[0] > _w), None)
        SC["procmem_first_row_after_window_s_after_resume"] = round((_nx[0] - tr_ns) / 1e9, 1) if _nx else None
        SC["procmem_same_pid_after_window"] = bool(_nx and _nx[2] and {r[2] for r in _run if r[2]} == {_nx[2]})
        _cont = (bool(_run) and not _late and SC["procmem_rows_missing_after_start"] == 0 and SC["procmem_pids_after_start"] == 1
                 and SC["procmem_same_pid_after_window"])
        if not _cont:
            C["slam_coload_alive"] = False
            SC["status"] = ("FAIL: " if SC["status"] == "ok" else SC["status"] + "; ") + (
                "ov2slam_node in no procmem row of the mission window" if not _run else
                f"ov2slam_node first present {SC['procmem_first_present_s_after_resume']} s after resume, later than {SLAM_START_MAX_S} s" if _late else
                "ov2slam_node not continuous in the procmem rows after its start (a gap or a second pid)"
                if SC["procmem_rows_missing_after_start"] or SC["procmem_pids_after_start"] != 1 else
                "no procmem row after the mission window with the same ov2slam_node pid")
    elif pre_dir and os.path.exists(os.path.join(pre_dir, "sim_setup.json")):   # 25C: a non-frozen setup run needs the sampler
        C["slam_coload_alive"] = False
        SC["status"] = ("FAIL: " if SC["status"] == "ok" else SC["status"] + "; ") + "no procmem.csv in a run of a non-frozen simulator setup"
I["slam_coload"] = SC
# covariates of the co-load's tracking (never checks): over the window after resume
if msgs and tr_ns:
    _w_end = tr_ns + int(SC["mission_window_s"] * 1e9)
    _sp = [t for t in sp if tr_ns <= t <= _w_end]
    I["slam_pose_msgs_in_window"] = len(_sp)
    I["slam_first_pose_delay_s"] = round((_sp[0] - tr_ns) / 1e9, 2) if _sp else None
    I["slam_max_pose_gap_s"] = round(max((b - a for a, b in zip(_sp, _sp[1:])), default=0) / 1e9, 2) if len(_sp) > 1 else None

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
