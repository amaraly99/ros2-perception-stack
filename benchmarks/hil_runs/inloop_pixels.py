"""inloop_pixels.py (5M): shared by gate_render.py, tools/infer_frames.py and the 5M start decision.

1. to_inloop_rgb(bgr): the pixels the in-loop producer scores. Same steps as parity_5J/nv12_roundtrip.py:
   MATLAB bgr8 -> sim_camera_bridge_node.cpp:152 toCvCopy(msg, "rgb8") -> :161 cvtColor(RGB2YUV_I420) -> :165-197 U,V interleaved
   into NV12 -> yolo_producer.py:190-197 reshape (h*3/2, w), cvtColor(COLOR_YUV2RGB_NV12).
2. frame_hash(rgb): the producer's frame_hash (blake2b, 4 bytes, little endian, over the RGB bytes).
   selftest(): the 5I held render ef4a539b must give 3321961853, the hash the producer logged for it (5J A2). Tools refuse to
   score in-loop pixels if this fails. 5Q: the render is read from fixtures/selftest_ef4a539b.png (lossless; its pixels are the
   raw bgr8 bytes of ef4a539b, checked byte for byte when the fixture was written).
3. Box counting, after the detector (no detector or NMS change): stop-sign boxes (COCO 11) at conf >= 0.20 are grouped into
   distinct objects (single linkage, IoU >= 0.5); a box is on the sign if it intersects the square of the expected sign width
   centred on the projected sign (u, v); a stop-sign box not on the sign is a failure.
"""
import hashlib, math, os, sys
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hilpaths

STOP = 11
CONF = 0.20
CLUSTER_IOU = 0.5
SELFTEST_PNG = os.path.join(hilpaths.FIXTURES, "selftest_ef4a539b.png")
SELFTEST_SHA1 = "ef4a539bcb77377611f546ca47ba0de748a7ed23"
SELFTEST_HASH = 3321961853
GEOM = hilpaths.GEOM


def to_inloop_rgb(bgr):
    h, w = bgr.shape[:2]
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    i420 = cv2.cvtColor(rgb, cv2.COLOR_RGB2YUV_I420)
    y = i420[:h].reshape(-1); u = i420[h:h + h // 4].reshape(-1); v = i420[h + h // 4:].reshape(-1)
    uv = np.empty(h * w // 2, np.uint8); uv[0::2] = u; uv[1::2] = v
    return cv2.cvtColor(np.concatenate([y, uv]).reshape(h * 3 // 2, w), cv2.COLOR_YUV2RGB_NV12)


def to_raw_rgb(bgr):
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def frame_hash(rgb):
    return int.from_bytes(hashlib.blake2b(np.ascontiguousarray(rgb).tobytes(), digest_size=4).digest(), "little")


def selftest():
    bgr = cv2.imread(SELFTEST_PNG, cv2.IMREAD_UNCHANGED)
    if bgr is None or hashlib.sha1(bgr.tobytes()).hexdigest() != SELFTEST_SHA1:
        raise SystemExit(f"inloop_pixels SELFTEST FAILED: {SELFTEST_PNG} missing or not the ef4a539b render")
    got = frame_hash(to_inloop_rgb(bgr))
    if got != SELFTEST_HASH:
        raise SystemExit(f"inloop_pixels SELFTEST FAILED: ef4a539b in-loop frame_hash {got}, producer logged {SELFTEST_HASH}")
    return got


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy; ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def clusters(boxes, thr=CLUSTER_IOU):
    """Single-linkage groups of boxes (x1, y1, x2, y2, ...) with IoU >= thr. Returns lists of indices."""
    parent = list(range(len(boxes)))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            if iou(boxes[i], boxes[j]) >= thr:
                parent[find(i)] = find(j)
    g = {}
    for i in range(len(boxes)):
        g.setdefault(find(i), []).append(i)
    return list(g.values())


def on_sign(box, sign):
    u, v, w = sign
    sq = (u - w / 2, v - w / 2, u + w / 2, v + w / 2)
    return min(box[2], sq[2]) > max(box[0], sq[0]) and min(box[3], sq[3]) > max(box[1], sq[1])


def stop_summary(dets, sign):
    """dets: (x1, y1, x2, y2, cls, conf) rows. sign: (u, v, expected_w_px) or None.
    raw = stop-sign boxes at conf >= 0.20; distinct = IoU >= 0.5 groups; off_sign = boxes not on the projected sign;
    top_on = best conf on the sign (None if none); one_on_sign = raw >= 1, one group, no box off the sign."""
    st = [tuple(float(x) for x in d[:6]) for d in dets if int(d[4]) == STOP and float(d[5]) >= CONF]
    g = clusters(st)
    on = [b for b in st if sign is not None and on_sign(b, sign)]
    off = len(st) - len(on) if sign is not None else None
    top_on = round(max(b[5] for b in on), 4) if on else None
    return {"raw": len(st), "distinct": len(g), "off_sign": off, "top_on": top_on,
            "confs": [round(b[5], 4) for b in sorted(st, key=lambda b: -b[5])],
            "one_on_sign": bool(st) and len(g) == 1 and off == 0}


def xyxy_from_cwh(cx, cy, w, h):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def project_sign(pose_xyz_yaw, geom_path=GEOM):
    """(u, v, expected_w_px) of the frozen sign for a body pose (x, y, z, yaw rad), with the scorer's project() at the live fx."""
    import json
    sys.path.insert(0, hilpaths.BENCH)
    import inloop_accuracy as IA
    g = json.load(open(geom_path))
    IA.FX = IA.FY = float(g["fx"])
    p = IA.project((0, *pose_xyz_yaw), g["target_xyz"])
    return (p[0], p[1], IA.sign_width_px(p[3])) if p else None


def start_pose(geom_path=GEOM):
    import json
    x, y, z, yaw = json.load(open(geom_path))["start_xyz_yaw"]
    if isinstance(yaw, str):
        yaw = eval(yaw, {"__builtins__": {}}, {"pi": math.pi})
    return (float(x), float(y), float(z), float(yaw))
