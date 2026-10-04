#!/usr/bin/env python3
"""
inloop_accuracy.py — detection quality against a geometric reference, inside the loop.

The COCO numbers say how good a detector is on a photograph given unlimited time.
They say nothing about the quality of the detections the *controller* receives. In
the closed loop the detector can skip frames because it is too slow, or deliver a
box computed from a frame that is already stale. The model's arithmetic does not
change under load; the usefulness of its output does.

This scores a recorded HIL run against the target's known world position:

    centroid error       distance between the detected box centre and the
                         projected true target centre, in px and normalised by
                         the projected sign width (see "Why normalised" below)
    hit / miss /         hit = a target-class box centred within HIT_FRAC of a
    mislocalisation      sign width; miss = target visible, nothing detected;
                         mislocalisation = detected, but not on the target
    false positives      split into class FP (wrong class, harmless: the
                         controller filters by class) and localisation FP
                         (right class, wrong object -- the one that would
                         actually steer the controller at the wrong thing)

Because the model is deterministic, any in-loop change must come from timing.
So the error is reported twice, and the pair is the experiment's own bug check:

    at-capture   against the drone pose when the frame was captured. This is
                 detection quality proper and should be FLAT across detector
                 configurations. If it moves, suspect a bug before a finding.
    at-delivery  against the drone pose when the detection reached the consumer.
                 This is what the controller acts on, and it should grow with
                 detector latency.

GEOMETRY (2026-09-22: corrected -- see "The camera is not at the drone origin")
camera is pinhole, 640x480, cx=320, cy=240, fx=fy set by --fx (1200 for the
current Simulink model). /sim/drone_pose carries [x, y, z, pitch, yaw]; only
x, y, z and yaw are used, because the body pitch channel is identically 0 in
every run checked (three B3 bags, 0.000 deg throughout).

    # score a run against a known target position
    python3 benchmarks/inloop_accuracy.py --bag BAG.mcap --target 35.1,2.92,3.08 \
        --telemetry run1_telemetry.csv --cam-stamps run1_cam_stamps.csv \
        --label yolo26n_cpu --json-out out.json

    # diagnostic only: try to estimate the target from the run itself
    python3 benchmarks/inloop_accuracy.py --bag BAG.mcap --triangulate

--triangulate is a DIAGNOSTIC, not a source of ground truth, and on the mission
trajectory it does not work at all: yaw is constant (integrator IC 2*pi), the
bearing spread is 1.9 deg, and the bearing-only fit walks out to x=70 m pinned
to the edge of its search box while reporting a comfortable 14 px RMS. Depth is
unobservable from bearings when the drone does not turn. The target must come
from the scene, or from the size-vs-motion fit documented in
paper_data/inloop_accuracy/README.md. It is frozen once and never refit per row.

Requires: mcap, mcap-ros2-support (pure python, no ROS install needed).
"""

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys

# ---------------------------------------------------------------------------
# Intrinsics. The default is the model every Table 13/14 run used (sha256
# 946390cf..., fx=fy=1200, HFOV 29.9 deg), checked against the block itself in
# commit 4839666. From 2026-10-04 the shared model renders at fx=fy=554 (HFOV
# 60.0 deg); runs on it are scored with --fx 554 and form a separate series.
# ---------------------------------------------------------------------------
FX = FY = 1200.0
CX, CY = 320.0, 240.0
WIDTH, HEIGHT = 640, 480

# Geometry gate (17.20). The 40 px threshold was set on the fx 1200 model, where
# it is an angle of atan(40/1200) = 1.91 deg. A pixel error is that angle times
# fx, so the default gate scales with fx: 40 px at 1200, 18.5 px at 554. At fx
# 1200 the gate is exactly 40 px, as before.
RMS_GATE_PX_AT_1200 = 40.0


def default_rms_gate(fx):
    return RMS_GATE_PX_AT_1200 * fx / 1200.0
TARGET_CLASS = "stop sign"

# ---------------------------------------------------------------------------
# The camera is NOT at the drone origin. Both of these come from the
# Simulation 3D Camera block in matlab/hil_closed_loop.slx:
#
#     tmountOffset = [0.2, 0, 0.2]   -> 0.2 m forward, 0.2 m up, body frame
#     rmountOffset = [0, 5, 0]       -> 5 degrees of camera pitch
#
# Neither was modelled before 2026-09-22, and the pitch is not a detail: it
# puts a CONSTANT fy*tan(5 deg) = 105 px vertical bias on every projected point,
# on a 480 px image. Left in, it fails essentially every long-range detection
# (the sign is 27 px wide at 40 m) and passes most close-range ones, which reads
# as a clean and completely false "detectors degrade at long range".
#
# The value is not taken on trust from the model file. Fitting a free pitch term
# to the three B3 bags recovers -4.981, -4.997 and -5.002 deg, and the vertical
# residual collapses from sd 0.73 m to 0.0015. The model says 5; the detections
# say 5.00, three times, without either knowing about the other. We use the
# model's declared magnitude and carry the sign that fits the data.
# ---------------------------------------------------------------------------
MOUNT_FWD = 0.2     # m, along the body heading
MOUNT_LAT = 0.0     # m, positive left
MOUNT_UP = 0.2      # m
CAM_PITCH = math.radians(-5.0)

# Physical target size, needed to turn a pixel error into a scale-free one and
# to project a hit radius. Fitted at 0.913-0.928 m from box width and 0.938-0.945
# from box height across three runs; a US 36-inch stop sign is 0.914 m, which is
# not a number the regression was pushed toward.
SIGN_WIDTH_M = 0.914

# A hit is a target-class box centred within HIT_FRAC sign-widths of the truth.
# Fixed pixel radii are meaningless here: the sign spans 27 px at 40 m and 250 px
# at 4.4 m, so any constant would be unreachable at one end and trivial at the other.
HIT_FRAC = 0.5
HIT_FRAC_SENS = 0.25    # sensitivity check, pre-registered alongside the primary

# Approach vs parked. The drone reaches its standoff at t ~= 37 s and then sits
# motionless; while it is stationary the at-capture and at-delivery poses are the
# same pose, so the headline metric is identically zero by construction. The two
# phases are therefore scored and reported separately, never pooled.
PARKED_SPEED = 0.05     # m/s
RANGE_BINS = [(4, 8), (8, 16), (16, 24), (24, 32), (32, 40)]
MIN_N_MEDIAN = 10       # below this a bin is not reported at all
MIN_N_P90 = 20          # below this the p90 is suppressed but the median stands

POSE_TOPIC = "/sim/drone_pose"
TARGET_TOPIC = "/sim/target_pose"
DET_TOPIC = "/yolo/detections"


def read_bag(path, cache_dir=None):
    """Extract the three streams we need. Cached: these bags are gigabytes."""
    cache = None
    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)
        st = os.stat(path)
        key = f"{os.path.basename(path)}.{st.st_size}.{int(st.st_mtime)}.json"
        cache = os.path.join(cache_dir, key)
        if os.path.exists(cache):
            with open(cache) as f:
                blob = json.load(f)
            return ([tuple(p) for p in blob["poses"]], blob["dets"], blob["target"])
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError:
        sys.exit("needs mcap-ros2-support: pip install mcap mcap-ros2-support")
    poses, dets = [], []
    target_pose = None
    for m in read_ros2_messages(path, topics=[POSE_TOPIC, TARGET_TOPIC, DET_TOPIC]):
        topic = m.channel.topic
        if topic == POSE_TOPIC:
            d = list(m.ros_msg.data)
            if len(d) >= 5:
                # d[3] is body pitch, identically 0 in every run checked, so the
                # projection ignores it. If a future model pitches the airframe,
                # this is the line to revisit.
                poses.append((m.log_time_ns, d[0], d[1], d[2], d[4]))
        elif topic == TARGET_TOPIC:
            target_pose = list(m.ros_msg.data)
        elif topic == DET_TOPIC:
            h = m.ros_msg.header.stamp
            dets.append({
                "log_ns": m.log_time_ns,
                "stamp_ns": h.sec * 10**9 + h.nanosec,
                "dets": [(d.class_name, d.confidence, d.center_x, d.center_y,
                          d.size_width, d.size_height) for d in m.ros_msg.detections],
            })
    poses.sort()
    dets.sort(key=lambda r: r["log_ns"])
    if cache:
        with open(cache, "w") as f:
            json.dump({"poses": poses, "dets": dets, "target": target_pose}, f)
    return poses, dets, target_pose


class PoseTrack:
    """Drone pose, looked up by bag (epoch) timestamp, with ground speed."""

    def __init__(self, poses):
        self.t = [p[0] for p in poses]
        self.p = poses
        self._speed = self._compute_speed(poses)

    @staticmethod
    def _compute_speed(poses):
        """Central-difference ground speed over +/-0.25 s.

        A single-sample difference is far too noisy at this pose rate (~16 Hz
        delivered) to threshold at 0.05 m/s; the window makes the approach/parked
        split stable to within a couple of samples at the transition.
        """
        sp = [0.0] * len(poses)
        for i in range(len(poses)):
            lo, hi = i, i
            while lo > 0 and poses[i][0] - poses[lo][0] < 0.25e9:
                lo -= 1
            while hi < len(poses) - 1 and poses[hi][0] - poses[i][0] < 0.25e9:
                hi += 1
            dt = (poses[hi][0] - poses[lo][0]) / 1e9
            if dt > 0:
                dist = math.dist(poses[lo][1:4], poses[hi][1:4])
                sp[i] = dist / dt
        return sp

    def _idx(self, ts):
        i = bisect.bisect_left(self.t, ts)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.p)]
        if not cands:
            return None
        return min(cands, key=lambda j: abs(self.t[j] - ts))

    def at(self, ts):
        i = self._idx(ts)
        return self.p[i] if i is not None else None

    def speed_at(self, ts):
        i = self._idx(ts)
        return self._speed[i] if i is not None else None


def camera_pose(pose):
    """Body pose -> camera position in the world, applying the mount translation."""
    _, x, y, z, yaw = pose
    cx = x + MOUNT_FWD * math.cos(yaw) - MOUNT_LAT * math.sin(yaw)
    cy = y + MOUNT_FWD * math.sin(yaw) + MOUNT_LAT * math.cos(yaw)
    return cx, cy, z + MOUNT_UP, yaw


def project(pose, target):
    """World target -> (u, v, ground_range, fwd). None if behind the camera.

    Image x is mirrored relative to the textbook right-handed convention on this
    simulator's camera: the minus sign on u is what makes project() agree with the
    bearing convention the data fits. Measured, not assumed -- against 671
    stop-sign observations the minus gives a 9.4 px median column error and the
    plus gives 202 px.

    The pitch rotation is applied about the camera's right axis, after the yaw
    frame and before the perspective divide. `fwd` after rotation is the depth
    along the optical axis, which is what the projected sign width divides by.
    """
    ccx, ccy, ccz, yaw = camera_pose(pose)
    dx, dy, dz = target[0] - ccx, target[1] - ccy, target[2] - ccz
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    right = -dx * math.sin(yaw) + dy * math.cos(yaw)
    cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    f = fwd * cp + dz * sp
    h = -fwd * sp + dz * cp
    if f <= 0.1:
        return None
    u = CX - FX * (right / f)
    v = CY - FY * (h / f)
    return u, v, math.hypot(dx, dy), f


def sign_width_px(fwd):
    """Apparent width of the sign at optical-axis depth `fwd`."""
    return FX * SIGN_WIDTH_M / fwd if fwd > 0.1 else float("inf")


def in_view(uv, margin=0):
    return uv is not None and margin <= uv[0] < WIDTH - margin \
        and margin <= uv[1] < HEIGHT - margin


def load_capture_times(path):
    """frame telemetry -> sorted (publish_mono_ns, capture_mono_ns) pairs."""
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                out.append((int(row["ts_yolo_shm_write"]), int(row["ts_capture"])))
            except (KeyError, ValueError):
                continue
    out.sort()
    return out


def count_skipped_frames(cam_stamps_path, capture_pairs):
    """Exact count of camera frames the detector never processed.

    The detector reads the newest slot of a 4-slot SHM ring and never queues, so
    a frame it was too busy to collect is overwritten and leaves no telemetry row
    and no detection message -- invisible to any per-message metric. But
    <run>_cam_stamps.csv logs EVERY camera frame, and its t_shm_write_mono_ns is
    the identical CLOCK_MONOTONIC value that telemetry records as ts_capture
    (the documented join key, latency_stage_report.py:11-13). So the skipped set
    is an exact set difference, not an estimate from a rate ratio.

    The two files are written by independently started and stopped processes, so their
    windows do not coincide. Measured on b3_bag r1: cam_stamps spans 99.3 s and telemetry
    104.2 s, and 22 telemetry rows (5.2 %) fall AFTER the stamp log closes -- the camera
    bridge flushes and closes stamp_log_ while it is still filling SHM slots, so the
    detector keeps processing for ~5 s with nothing logging those frames. Checked
    2026-09-23: 0 of the 22 fall inside the stamp-log window, so it is a shutdown-ordering
    artifact and not a lost-frame bug.

    The count is therefore taken over the OVERLAP of the two windows. Outside it one side
    is blind, and a frame that no one logged is not evidence that the detector skipped it.
    """
    cam = []
    with open(cam_stamps_path) as f:
        for row in csv.DictReader(f):
            try:
                cam.append(int(row["t_shm_write_mono_ns"]))
            except (KeyError, ValueError):
                continue
    processed = {c for _, c in (capture_pairs or [])}
    if not cam or not processed:
        return None
    lo = max(min(cam), min(processed))
    hi = min(max(cam), max(processed))
    if hi <= lo:
        return None
    cam_w = [t for t in cam if lo <= t <= hi]
    proc_w = {t for t in processed if lo <= t <= hi}
    if not cam_w:
        return None
    matched = sum(1 for t in cam_w if t in proc_w)
    return {
        "window_s": (hi - lo) / 1e9,
        "camera_frames": len(cam_w),
        "processed_frames": matched,
        "skipped_frames": len(cam_w) - matched,
        "skip_rate": (len(cam_w) - matched) / len(cam_w),
        "telemetry_rows_unmatched_in_window": len(proc_w) - matched,
        "cam_rows_outside_overlap": len(cam) - len(cam_w),
        "telemetry_rows_outside_overlap": len(processed) - len(proc_w),
    }


def _stats(vals, label_norm=None):
    if not vals:
        return None
    v = sorted(vals)
    out = {"n": len(v), "median_px": statistics.median(v),
           "mean_px": statistics.fmean(v)}
    if len(v) >= MIN_N_P90:
        out["p90_px"] = v[min(len(v) - 1, int(0.9 * len(v)))]
    if label_norm:
        w = sorted(label_norm)
        out["median_norm"] = statistics.median(w)
        if len(w) >= MIN_N_P90:
            out["p90_norm"] = w[min(len(w) - 1, int(0.9 * len(w)))]
    return out


def _bin_of(rng):
    for lo, hi in RANGE_BINS:
        if lo <= rng < hi:
            return f"{lo}-{hi}m"
    return None


def _summarise(rows):
    """rows: list of per-detection dicts -> one metric block."""
    if not rows:
        return None
    cap = [r["err_cap"] for r in rows if r["err_cap"] is not None]
    cap_n = [r["norm_cap"] for r in rows if r["norm_cap"] is not None]
    dlv = [r["err_del"] for r in rows if r["err_del"] is not None]
    dlv_n = [r["norm_del"] for r in rows if r["norm_del"] is not None]
    visible = [r for r in rows if r["visible"]]
    hits = [r for r in visible if r["hit"]]
    hits_s = [r for r in visible if r["hit_sens"]]
    miss = [r for r in visible if r["no_detection"]]
    mis = [r for r in visible if r["mislocalised"]]
    out = {
        "n_messages": len(rows),
        "n_target_visible": len(visible),
        "centroid_err_at_capture": _stats(cap, cap_n),
        "centroid_err_at_delivery": _stats(dlv, dlv_n),
    }
    if visible:
        out["hit_rate"] = len(hits) / len(visible)
        out["hit_rate_sensitivity_0p25"] = len(hits_s) / len(visible)
        out["miss_rate_per_processed_frame"] = len(miss) / len(visible)
        out["mislocalisation_rate"] = len(mis) / len(visible)
    if len(visible) < MIN_N_MEDIAN:
        out["below_reporting_floor"] = True
    return out


def score(poses, dets, target, capture_pairs=None, clock_offset_ns=None,
          legacy_publish_match=False):
    if clock_offset_ns is None and not legacy_publish_match:
        raise ValueError("score() needs clock_offset_ns (realtime - monotonic); the legacy "
                         "lookup runs only with legacy_publish_match=True")
    track = PoseTrack(poses)

    # Detection headers carry the frame's CAPTURE time (telemetry ts_capture: the
    # camera bridge's SHM write, CLOCK_MONOTONIC); the bag logs epoch. With the
    # recording host's realtime - monotonic offset (clock_offset_ns) the capture
    # pose is taken at header stamp + offset (DEVIATIONS 17.24). The legacy lookup
    # below runs only on explicit request (legacy_publish_match): it treats the
    # stamp as a publish time and adds the median (log - stamp) offset, which
    # includes the pipeline latency (17.23).
    offs = [r["log_ns"] - r["stamp_ns"] for r in dets]
    offset = statistics.median(offs) if offs else 0
    offset_spread_ms = (max(offs) - min(offs)) / 1e6 if offs else 0.0

    cap_t = [p[0] for p in capture_pairs] if capture_pairs else []

    rows = []
    fp_class = fp_loc = total_dets = 0

    for rec in dets:
        pose_del = track.at(rec["log_ns"])
        if pose_del is None:
            continue
        uv_del = project(pose_del, target)

        # Capture-time pose.
        if clock_offset_ns is not None:
            cap_bag_ns = rec["stamp_ns"] + clock_offset_ns
        else:
            cap_ns = rec["stamp_ns"]
            if cap_t:
                i = bisect.bisect_left(cap_t, rec["stamp_ns"])
                cands = [j for j in (i - 1, i) if 0 <= j < len(capture_pairs)]
                if cands:
                    j = min(cands, key=lambda j: abs(cap_t[j] - rec["stamp_ns"]))
                    cap_ns = capture_pairs[j][1]
            cap_bag_ns = cap_ns + offset
        pose_cap = track.at(cap_bag_ns)
        uv_cap = project(pose_cap, target) if pose_cap else None
        speed = track.speed_at(cap_bag_ns)

        total_dets += len(rec["dets"])
        fp_class += sum(1 for d in rec["dets"] if d[0] != TARGET_CLASS)

        visible = in_view(uv_cap)
        tgt_hits = [d for d in rec["dets"] if d[0] == TARGET_CLASS]
        best = max(tgt_hits, key=lambda d: d[1]) if tgt_hits else None

        err_cap = err_del = norm_cap = norm_del = None
        hit = hit_s = mislocalised = False

        if best is not None:
            w_px = sign_width_px(uv_cap[3]) if uv_cap else float("inf")
            if visible:
                err_cap = math.hypot(uv_cap[0] - best[2], uv_cap[1] - best[3])
                norm_cap = err_cap / w_px if w_px > 0 else None
                hit = norm_cap is not None and norm_cap <= HIT_FRAC
                hit_s = norm_cap is not None and norm_cap <= HIT_FRAC_SENS
                mislocalised = not hit
            if in_view(uv_del):
                err_del = math.hypot(uv_del[0] - best[2], uv_del[1] - best[3])
                wd = sign_width_px(uv_del[3])
                norm_del = err_del / wd if wd > 0 else None
            # A target-class box that is not on the target is a localisation FP:
            # the scene holds a second stop sign, and this is the failure that
            # would actually steer the controller at the wrong object.
            if visible and not hit:
                fp_loc += 1
            elif not visible:
                fp_loc += 1

        rows.append({
            "range": uv_cap[2] if uv_cap else None,
            "bin": _bin_of(uv_cap[2]) if uv_cap else None,
            "speed": speed,
            "visible": visible,
            "no_detection": visible and best is None,
            "mislocalised": mislocalised,
            "hit": hit, "hit_sens": hit_s,
            "err_cap": err_cap, "err_del": err_del,
            "norm_cap": norm_cap, "norm_del": norm_del,
        })

    # --- split by phase, then by range bin ---------------------------------
    out = {
        "geometry": {
            "fx": FX, "cx": CX, "cy": CY,
            "mount_fwd_lat_up_m": [MOUNT_FWD, MOUNT_LAT, MOUNT_UP],
            "cam_pitch_deg": math.degrees(CAM_PITCH),
            "sign_width_m": SIGN_WIDTH_M,
            "hit_frac": HIT_FRAC, "hit_frac_sensitivity": HIT_FRAC_SENS,
        },
        "clock_offset_spread_ms": offset_spread_ms,
        "frames_with_detections": len(dets),
        "false_positive_rate_class": (fp_class / total_dets) if total_dets else None,
        "false_positive_localisation_count": fp_loc,
        "segments": {},
    }

    for name, sel in (("approach", lambda r: r["speed"] is not None and r["speed"] >= PARKED_SPEED),
                      ("parked", lambda r: r["speed"] is not None and r["speed"] < PARKED_SPEED)):
        seg = [r for r in rows if sel(r)]
        if not seg:
            continue
        rngs = [r["range"] for r in seg if r["range"] is not None]
        block = {
            "overall": _summarise(seg),
            "range_m": [min(rngs), max(rngs)] if rngs else None,
            "bins": {},
        }
        for lo, hi in RANGE_BINS:
            key = f"{lo}-{hi}m"
            b = [r for r in seg if r["bin"] == key]
            s = _summarise(b)
            # A bin below the floor is reported as absent, not as zero: an
            # unstable median from 4 samples is worse than no number.
            if s and not s.get("below_reporting_floor"):
                block["bins"][key] = s
            elif b:
                block["bins"][key] = {"n_messages": len(b), "below_reporting_floor": True}
        out["segments"][name] = block

    return out


# ---------------------------------------------------------------------------
# Diagnostic only. Kept because it is the thing that proves the trajectory
# cannot yield a target position, which is worth being able to re-demonstrate.
# ---------------------------------------------------------------------------
def triangulate(poses, dets, min_conf=0.5):
    """Estimate the target position from the run, and say whether to believe it.

    On the mission trajectory this DOES NOT WORK and is expected to fail loudly:
    yaw is constant, the bearing spread is ~1.9 deg, and depth is unobservable.
    It is retained as a diagnostic and as the check that a future trajectory with
    real yaw variation would pass. Never use its output as ground truth.
    """
    track = PoseTrack(poses)
    rows = []
    for rec in dets:
        # Only frames holding exactly ONE instance of the target class. With two
        # signs in view, "highest confidence" jumps between objects and the fit
        # degrades badly.
        hits = [d for d in rec["dets"] if d[0] == TARGET_CLASS]
        if len(hits) != 1 or hits[0][1] < min_conf:
            continue
        pose = track.at(rec["log_ns"])
        if pose:
            rows.append((pose, hits[0]))
    if len(rows) < 10:
        return None

    a11 = a12 = a22 = b1 = b2 = 0.0
    thetas = []
    for pose, d in rows:
        _, x, y, _, yaw = pose
        th = yaw - math.atan2(d[2] - CX, FX)
        thetas.append(th)
        s_, c_ = math.sin(th), math.cos(th)
        rhs = x * s_ - y * c_
        a11 += s_ * s_; a12 += -s_ * c_; a22 += c_ * c_
        b1 += s_ * rhs; b2 += -c_ * rhs
    det = a11 * a22 - a12 * a12
    if abs(det) < 1e-9:
        return None
    tx = (b1 * a22 - a12 * b2) / det
    ty = (a11 * b2 - b1 * a12) / det
    tmin, tmax = min(thetas), max(thetas)
    spread = math.degrees(abs(math.atan2(math.sin(tmax - tmin), math.cos(tmax - tmin))))

    zs = []
    for pose, d in rows:
        _, x, y, z, _ = pose
        rng = math.hypot(tx - x, ty - y)
        if rng > 3:
            zs.append(z + rng * ((CY - d[3]) / FY))
    tz = statistics.median(zs) if zs else 0.0

    def cost(t):
        tot = n = 0
        for pose, d in rows:
            uv = project(pose, t)
            if uv is None:
                continue
            tot += (uv[0] - d[2]) ** 2 + (uv[1] - d[3]) ** 2
            n += 1
        return (tot / n) if n else float("inf")

    xs = [p[1] for p in poses]; ys = [p[2] for p in poses]; zs_p = [p[3] for p in poses]
    lo = [min(xs) - 40, min(ys) - 40, min(zs_p) - 10]
    hi = [max(xs) + 40, max(ys) + 40, max(zs_p) + 10]
    best_t, best_c = [tx, ty, tz], cost([tx, ty, tz])
    steps = 12
    for i in range(steps + 1):
        gx = lo[0] + (hi[0] - lo[0]) * i / steps
        for j in range(steps + 1):
            gy = lo[1] + (hi[1] - lo[1]) * j / steps
            for k in range(7):
                gz = lo[2] + (hi[2] - lo[2]) * k / 6
                c = cost([gx, gy, gz])
                if c < best_c:
                    best_t, best_c = [gx, gy, gz], c
    step = max((hi[0] - lo[0]) / steps, 1.0)
    for _ in range(300):
        start_c = best_c
        for axis in range(3):
            for delta in (step, -step):
                cand = list(best_t)
                cand[axis] = min(hi[axis], max(lo[axis], cand[axis] + delta))
                c = cost(cand)
                if c < best_c:
                    best_t, best_c = cand, c
        if best_c > start_c * (1.0 - 1e-6):
            step /= 2.0
            if step < 0.01:
                break
    at_bound = any(abs(best_t[a] - lo[a]) < 1e-6 or abs(best_t[a] - hi[a]) < 1e-6
                   for a in range(3))
    resid = []
    for (pose, d), th in zip(rows, thetas):
        _, x, y, _, _ = pose
        resid.append(abs((tx - x) * math.sin(th) - (ty - y) * math.cos(th)))
    return {
        "x": best_t[0], "y": best_t[1], "z": best_t[2],
        "pixel_rms": math.sqrt(best_c),
        "bearing_residual_m": statistics.median(resid),
        "bearing_spread_deg": spread,
        "at_search_bound": at_bound,
        "n": len(rows),
    }


def reprojection_rms(poses, dets, target, clock_offset_ns=None):
    """RMS reprojection error of the FROZEN target against single-sign frames.

    This is the geometry check that replaces the old refit-and-compare: with a
    frozen target there is nothing to fit, so the question is only whether that
    one point reprojects onto the detections. A large value means the geometry
    or the target is wrong, and nothing downstream is meaningful.

    With clock_offset_ns the pose is the CAPTURE pose (header stamp + offset,
    17.24), the pose the detector actually saw. Without it the pose is the
    delivery pose (bag log time), which also measures the detector's latency and
    so fails slow detectors for a reason that is not geometry (17.20).
    """
    track = PoseTrack(poses)
    tot = n = 0
    for rec in dets:
        hits = [d for d in rec["dets"] if d[0] == TARGET_CLASS]
        if len(hits) != 1:
            continue
        if track.at(rec["log_ns"]) is None:
            continue
        t_ns = rec["stamp_ns"] + clock_offset_ns if clock_offset_ns is not None else rec["log_ns"]
        pose = track.at(t_ns)
        if not pose:
            continue
        uv = project(pose, target)
        if uv is None or not in_view(uv):
            continue
        tot += (uv[0] - hits[0][2]) ** 2 + (uv[1] - hits[0][3]) ** 2
        n += 1
    return (math.sqrt(tot / n), n) if n else (None, 0)


def main():
    global FX, FY, CAM_PITCH, SIGN_WIDTH_M, MOUNT_FWD, MOUNT_LAT, MOUNT_UP
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="path to the .mcap file")
    ap.add_argument("--target", help="true target as x,y,z in world coordinates")
    ap.add_argument("--telemetry", help="<run>_telemetry.csv, for capture timestamps")
    ap.add_argument("--cam-stamps", help="<run>_cam_stamps.csv, for the exact skipped-frame count")
    ap.add_argument("--clock-offset-ns", type=int, default=None,
                    help="recording host's realtime - CLOCK_MONOTONIC (ns); capture pose at "
                         "header stamp + this (DEVIATIONS 17.24). Required for scoring")
    ap.add_argument("--legacy-publish-match", action="store_true",
                    help="use the pre-17.24 at-capture lookup (stamp treated as publish time) "
                         "instead of --clock-offset-ns; output labelled legacy_publish_match")
    ap.add_argument("--triangulate", action="store_true",
                    help="DIAGNOSTIC: estimate the target from the run and exit. "
                         "Degenerate on the mission trajectory; not ground truth")
    ap.add_argument("--force", action="store_true",
                    help="score even if the geometry check fails")
    ap.add_argument("--label", default=None)
    ap.add_argument("--json-out", default=None)
    ap.add_argument("--cache-dir", default=None)
    ap.add_argument("--fx", type=float, default=FX)
    ap.add_argument("--cam-pitch-deg", type=float, default=math.degrees(CAM_PITCH),
                    help="camera pitch; model declares 5 deg, data fits -5.00")
    ap.add_argument("--mount", default=f"{MOUNT_FWD},{MOUNT_LAT},{MOUNT_UP}",
                    help="camera mount offset fwd,lat,up in metres")
    ap.add_argument("--sign-width", type=float, default=SIGN_WIDTH_M)
    ap.add_argument("--rms-gate", type=float, default=None,
                    help="geometry gate in px; default 40 px scaled by fx/1200 "
                         "(40 at fx 1200, 18.5 at fx 554)")
    args = ap.parse_args()
    if not args.triangulate:
        if args.clock_offset_ns is None and not args.legacy_publish_match:
            sys.exit("error: --clock-offset-ns is required (the recording host's realtime - "
                     "CLOCK_MONOTONIC in ns, DEVIATIONS 17.24). The pre-17.24 lookup runs only "
                     "with --legacy-publish-match.")
        if args.clock_offset_ns is not None and args.legacy_publish_match:
            sys.exit("error: give either --clock-offset-ns or --legacy-publish-match, not both")

    FX = FY = args.fx
    rms_gate = args.rms_gate if args.rms_gate is not None else default_rms_gate(FX)
    CAM_PITCH = math.radians(args.cam_pitch_deg)
    SIGN_WIDTH_M = args.sign_width
    MOUNT_FWD, MOUNT_LAT, MOUNT_UP = [float(v) for v in args.mount.split(",")]

    print(f"camera: fx=fy={FX:g}  mount=[{MOUNT_FWD},{MOUNT_LAT},{MOUNT_UP}] m  "
          f"pitch={args.cam_pitch_deg:+.3f} deg  sign={SIGN_WIDTH_M:.3f} m")

    poses, dets, bag_target = read_bag(args.bag, args.cache_dir)
    print(f"poses={len(poses)}  detection messages={len(dets)}")
    if bag_target:
        print(f"/sim/target_pose in bag: {bag_target[:3]}  "
              "(hardcoded constant, NOT a live feed -- see README)")

    if args.triangulate:
        tri = triangulate(poses, dets)
        if not tri:
            print("estimation failed: too few target-class detections")
            return 0
        print(f"\nestimated from {tri['n']} detections: "
              f"x={tri['x']:.2f} y={tri['y']:.2f} z={tri['z']:.2f}")
        print(f"  reprojection RMS : {tri['pixel_rms']:.1f} px")
        print(f"  bearing spread   : {tri['bearing_spread_deg']:.1f} deg")
        if tri["bearing_spread_deg"] < 20.0:
            print("  WARNING: little angular baseline. Depth is unobservable and this")
            print("  position is NOT trustworthy. Expected on the mission trajectory.")
        if tri["at_search_bound"]:
            print("  WARNING: best fit sits on the edge of the search box.")
        return 0

    if not args.target:
        sys.exit("--target x,y,z is required. It is frozen in DEVIATIONS and must "
                 "not be refit per row; --triangulate is a diagnostic, not a source.")
    target = [float(v) for v in args.target.split(",")]
    if len(target) != 3:
        sys.exit("--target needs three comma-separated numbers")

    # Gate at the CAPTURE pose (header stamp + clock offset), the pose the detector
    # saw. At the delivery pose the RMS also contains the drone's motion during the
    # detector's latency (17.20), which is the effect this scorer measures, not a
    # geometry error. The delivery RMS is printed as a diagnostic only. The legacy
    # lookup has no clock offset, so it keeps the delivery gate.
    rms_del, n_del = reprojection_rms(poses, dets, target)
    if args.clock_offset_ns is not None:
        rms, n_rms = reprojection_rms(poses, dets, target, args.clock_offset_ns)
        gate_timing = "capture"
    else:
        rms, n_rms = rms_del, n_del
        gate_timing = "delivery"
    if rms is None:
        sys.exit("no single-sign in-view detections: cannot check the geometry")
    print(f"geometry check: frozen target reprojects at {rms:.1f} px RMS over {n_rms} frames "
          f"at {gate_timing} (gate {rms_gate:.1f} px at fx {FX:g})")
    if rms_del is not None and gate_timing == "capture":
        print(f"  diagnostic: {rms_del:.1f} px RMS over {n_del} frames at delivery (not gated)")
    if rms > rms_gate and not args.force:
        sys.exit(f"\nrefusing to score: {rms:.1f} px RMS exceeds the {rms_gate:.1f} px "
                 "gate, so the projection model does not describe this run.\nCheck the "
                 "target, --fx, the mount offsets and the camera pitch. --force to override.")

    caps = load_capture_times(args.telemetry) if args.telemetry else None
    if args.telemetry and not caps:
        print(f"warning: no usable rows in {args.telemetry}; at-capture will collapse "
              "onto the publish stamp and the bug check cannot run")

    if args.legacy_publish_match:
        print("warning: --legacy-publish-match: at-capture uses the pre-17.24 lookup "
              "(stamp treated as publish time, DEVIATIONS 17.23)", file=sys.stderr)
    res = score(poses, dets, target, caps, args.clock_offset_ns, args.legacy_publish_match)
    res["label"] = args.label or os.path.basename(args.bag)
    res["target"] = target
    res["capture_times_from_telemetry"] = bool(caps)
    res["capture_timing"] = ("header_stamp_plus_clock_offset" if args.clock_offset_ns is not None
                             else "legacy_publish_match")
    res["geometry"]["frozen_target_reprojection_rms_px"] = rms
    res["geometry"]["rms_gate_px"] = rms_gate
    res["geometry"]["gate_timing"] = gate_timing
    res["geometry"]["frozen_target_reprojection_rms_delivery_px"] = rms_del

    if args.cam_stamps:
        fr = count_skipped_frames(args.cam_stamps, caps)
        if fr:
            res["frames"] = fr
            vis = sum(s["overall"]["n_target_visible"]
                      for s in res["segments"].values() if s.get("overall"))
            miss_proc = sum(s["overall"].get("miss_rate_per_processed_frame", 0)
                            * s["overall"]["n_target_visible"]
                            for s in res["segments"].values() if s.get("overall"))
            # Per camera frame, a skipped frame and a missed frame are the same
            # thing to the controller: no detection arrived.
            if fr["camera_frames"]:
                res["miss_rate_per_camera_frame"] = \
                    (miss_proc + fr["skipped_frames"]) / fr["camera_frames"]
                res["miss_rate_per_processed_frame"] = \
                    (miss_proc / vis) if vis else None

    print()
    print(json.dumps(res, indent=2))
    if args.json_out:
        os.makedirs(os.path.dirname(args.json_out) or ".", exist_ok=True)
        with open(args.json_out, "w") as f:
            json.dump(res, f, indent=2)
        print(f"\nwrote {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
