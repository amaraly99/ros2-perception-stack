#!/usr/bin/env python3
"""
export_ssd_mobilenet_v2 — CPU ONNX for the Hailo Model Zoo's ssd_mobilenet_v2.

Same weights as the zoo's Hailo-10H HEF: the zoo compiles its HEF from this
pretrained file, cut at the same nodes (hailo_model_zoo cfg/networks/
ssd_mobilenet_v2.yaml), so the CPU and NPU rows run one model.

    source  https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ObjectDetection/
            Detection-COCO/ssd/ssd_mobilenet_v2/pretrained/2025-01-15/ssd_mobilenet_v2.zip
            -> ssd_mobilenet_v2.tflite
               sha256 2255f6ddbb529ff5b87a10a5f2e379430fb2968df70e36bfe1997fe98886abd1
    anchors hailo_model_zoo cfg/postprocess_config/mobilenet_v2_ssd_nms_postprocess_config.json
            (the per-layer anchor sizes and box-coder scales the HEF's on-chip NMS uses)

No TensorFlow or TFLite runtime is needed. The TFLite flatbuffer is read with
struct and its weights are copied bit for bit into an ONNX graph.

The exported graph speaks the engine's end-to-end layout ('yolo26_e2e' in
models/model_registry.json), so the existing ONNX path runs it unchanged:

    input  images [1,3,S,S] float32 RGB in 0..1  (the engine's letterboxed canvas,
                                                 S = max(frame_w, frame_h))
    output dets   [1,100,6]  x1,y1,x2,y2 (canvas px), conf, COCO-80 class id,
                             sorted by conf, zero rows are padding

In the graph: crop the frame back out of the letterbox (scale is 1 when
S = max(w, h)), plain-resize to 300x300 as the model was trained (bilinear,
align_corners 0, half_pixel_centers 0, as the TFLite file's own preprocessor),
normalise x*2/255-1, backbone and box predictors, SSD box decoding with the
zoo's anchors (scales 10,10,5,5), sigmoid, per-class NMS (IoU 0.6, 20 boxes per
class, as the HEF), top 100 by score. Labels are mapped from the 91-slot TF/COCO
space to COCO-80 ids (stop sign 13 -> 11), which yolo_bridge already names.

Usage:
    python3 models/export_ssd_mobilenet_v2.py --tflite <path/ssd_mobilenet_v2.tflite> \\
        --nms-config <mobilenet_v2_ssd_nms_postprocess_config.json> \\
        --out models/onnx/ssd_mobilenet_v2/ssd_mobilenet_v2_e2e.onnx
"""

import argparse
import hashlib
import json
import math
import struct
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

SOURCE_SHA256 = "2255f6ddbb529ff5b87a10a5f2e379430fb2968df70e36bfe1997fe98886abd1"

# TF/COCO 91-slot label ids of the 80 COCO classes, in COCO-80 order
# (index 11 -> label 13, stop sign).
COCO80_TO_91 = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                22, 23, 24, 25, 27, 28, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
                44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
                64, 65, 67, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87,
                88, 89, 90]


# ═════════════════════════════════════════════════════════════════════════════
# Minimal TFLite flatbuffer reader (tensorflow/lite/schema/schema.fbs subset)
# ═════════════════════════════════════════════════════════════════════════════
_OPS = {0: "ADD", 3: "CONV_2D", 4: "DEPTHWISE_CONV_2D", 18: "MUL", 41: "SUB",
        14: "LOGISTIC", 17: "MAX_POOL_2D", 34: "PAD", 67: "TRANSPOSE_CONV"}   # the last four: CenterNet export
_DTYPE = {0: np.float32, 2: np.int32, 3: np.uint8}


class _FB:
    def __init__(self, b):
        self.b = b

    def _i32(self, p): return struct.unpack_from("<i", self.b, p)[0]
    def _u32(self, p): return struct.unpack_from("<I", self.b, p)[0]
    def _u16(self, p): return struct.unpack_from("<H", self.b, p)[0]

    def field(self, t, i):
        vt = t - self._i32(t)
        o = 4 + 2 * i
        if o >= self._u16(vt):
            return None
        off = self._u16(vt + o)
        return t + off if off else None

    def deref(self, p): return p + self._u32(p)

    def scalar(self, t, i, fmt, default):
        p = self.field(t, i)
        return struct.unpack_from(fmt, self.b, p)[0] if p is not None else default

    def table(self, t, i):
        p = self.field(t, i)
        return self.deref(p) if p is not None else None

    def vec(self, t, i):
        p = self.field(t, i)
        if p is None:
            return None, 0
        v = self.deref(p)
        return v + 4, self._u32(v)

    def tables(self, t, i):
        s, n = self.vec(t, i)
        return [self.deref(s + 4 * k) for k in range(n)] if s is not None else []

    def ints(self, t, i):
        s, n = self.vec(t, i)
        return list(struct.unpack_from(f"<{n}i", self.b, s)) if s is not None else []

    def string(self, t, i):
        p = self.field(t, i)
        if p is None:
            return None
        s = self.deref(p)
        return bytes(self.b[s + 4:s + 4 + self._u32(s)]).decode("utf-8", "replace")


def read_tflite(path):
    """Subgraph 0: tensors (shape/type/name/data) and operators (name/io/options)."""
    buf = open(path, "rb").read()
    f = _FB(buf)
    root = f.deref(0)
    codes = [max(f.scalar(oc, 3, "<i", 0), f.scalar(oc, 0, "<b", 0)) for oc in f.tables(root, 1)]
    buffers = f.tables(root, 4)
    sg = f.tables(root, 2)[0]
    tensors = []
    for t in f.tables(sg, 0):
        shape, dt, bi = f.ints(t, 0), f.scalar(t, 1, "<b", 0), f.scalar(t, 2, "<I", 0)
        s, n = f.vec(buffers[bi], 0)
        data = None
        if s is not None and n and dt in _DTYPE:
            data = np.frombuffer(buf, dtype=_DTYPE[dt], count=n // np.dtype(_DTYPE[dt]).itemsize,
                                 offset=s).reshape(shape).copy()
        tensors.append({"shape": shape, "name": f.string(t, 3), "data": data})
    ops = []
    for o in f.tables(sg, 3):
        name = _OPS.get(codes[f.scalar(o, 0, "<I", 0)], "OTHER")
        opt = f.table(o, 4)
        if name == "CONV_2D":        # Conv2DOptions
            opts = dict(pad=f.scalar(opt, 0, "<b", 0), sw=f.scalar(opt, 1, "<i", 1),
                        sh=f.scalar(opt, 2, "<i", 1), act=f.scalar(opt, 3, "<b", 0),
                        dw=f.scalar(opt, 4, "<i", 1), dh=f.scalar(opt, 5, "<i", 1), mult=1)
        elif name == "DEPTHWISE_CONV_2D":   # DepthwiseConv2DOptions
            opts = dict(pad=f.scalar(opt, 0, "<b", 0), sw=f.scalar(opt, 1, "<i", 1),
                        sh=f.scalar(opt, 2, "<i", 1), mult=f.scalar(opt, 3, "<i", 1),
                        act=f.scalar(opt, 4, "<b", 0), dw=f.scalar(opt, 5, "<i", 1),
                        dh=f.scalar(opt, 6, "<i", 1))
        elif name in ("ADD", "MUL", "SUB"):   # AddOptions / MulOptions / SubOptions
            opts = dict(act=f.scalar(opt, 0, "<b", 0) if opt is not None else 0)
        elif name == "MAX_POOL_2D":          # Pool2DOptions
            opts = dict(pad=f.scalar(opt, 0, "<b", 0), sw=f.scalar(opt, 1, "<i", 1), sh=f.scalar(opt, 2, "<i", 1),
                        fw=f.scalar(opt, 3, "<i", 1), fh=f.scalar(opt, 4, "<i", 1), act=f.scalar(opt, 5, "<b", 0))
        elif name == "TRANSPOSE_CONV":       # TransposeConvOptions
            opts = dict(pad=f.scalar(opt, 0, "<b", 0), sw=f.scalar(opt, 1, "<i", 1), sh=f.scalar(opt, 2, "<i", 1),
                        act=f.scalar(opt, 3, "<b", 0) if opt is not None else 0)
        else:
            opts = {}
        ops.append({"op": name, "in": f.ints(o, 1), "out": f.ints(o, 2), "opts": opts})
    return tensors, ops, [tensors[i]["name"] for i in f.ints(sg, 2)], f.ints(sg, 2)


# ═════════════════════════════════════════════════════════════════════════════
# Graph construction
# ═════════════════════════════════════════════════════════════════════════════
class _G:
    def __init__(self):
        self.nodes, self.inits, self._n = [], [], 0

    def const(self, arr, name=None):
        name = name or f"c{len(self.inits)}"
        self.inits.append(numpy_helper.from_array(np.asarray(arr), name))
        return name

    def op(self, op_type, inputs, n_out=1, **attrs):
        self._n += 1
        outs = [f"{op_type.lower()}_{self._n}_{k}" for k in range(n_out)]
        self.nodes.append(helper.make_node(op_type, inputs, outs, **attrs))
        return outs[0] if n_out == 1 else outs


def _same_pads(n_in, n_out, k, s, d):
    total = max((n_out - 1) * s + (k - 1) * d + 1 - n_in, 0)
    return total // 2, total - total // 2      # TF SAME: the extra pixel goes at the end


def _act(g, x, act):
    if act == 0:
        return x
    if act == 1:
        return g.op("Relu", [x])
    if act == 3:                                  # RELU6
        return g.op("Clip", [x, g.const(np.float32(0.0)), g.const(np.float32(6.0))])
    raise ValueError(f"fused activation {act} not supported")


def build(tflite_path, nms_cfg, frame_w=640, frame_h=480, nms_score_floor=0.05,
          nms_iou=0.6, per_class=20, top_k=100, debug_outputs=False):
    tensors, ops, out_names, out_idx = read_tflite(tflite_path)
    g = _G()
    S = max(frame_w, frame_h)                      # engine canvas; letterbox scale = 1
    pad_left, pad_top = (S - frame_w) // 2, (S - frame_h) // 2

    # ── preprocessing: canvas 0..1 -> frame -> 300x300 plain resize -> x*2/255-1 ──
    mul = sub = None
    first = None
    for k, o in enumerate(ops):
        if o["op"] == "MUL" and mul is None:
            mul = [tensors[i]["data"] for i in o["in"] if tensors[i]["data"] is not None][0]
        if o["op"] == "SUB" and sub is None:
            sub = [tensors[i]["data"] for i in o["in"] if tensors[i]["data"] is not None][0]
            first = k + 1
            sub_out = o["out"][0]
            break
    assert mul is not None and sub is not None, "preprocessing MUL/SUB not found"
    net_h, net_w = tensors[sub_out]["shape"][1:3]
    x = g.op("Slice", ["images", g.const(np.array([pad_top, pad_left], np.int64)),
                       g.const(np.array([pad_top + frame_h, pad_left + frame_w], np.int64)),
                       g.const(np.array([2, 3], np.int64))])
    x = g.op("Mul", [x, g.const(np.float32(255.0))])
    x = g.op("Resize", [x, "", "", g.const(np.array([1, 3, net_h, net_w], np.int64))],
             mode="linear", coordinate_transformation_mode="asymmetric")
    x = g.op("Mul", [x, g.const(mul.astype(np.float32).reshape(()))])
    x = g.op("Sub", [x, g.const(sub.astype(np.float32).reshape(()))])
    names = {sub_out: x}
    norm_input = x

    # ── backbone + box predictors: every remaining op of subgraph 0 ──────────────
    for o in ops[first:]:
        if o["op"] in ("CONV_2D", "DEPTHWISE_CONV_2D"):
            xi, wi, bi = o["in"]
            w, b = tensors[wi]["data"], tensors[bi]["data"]
            _, ih, iw, cin = tensors[xi]["shape"]
            _, oh, ow, cout = tensors[o["out"][0]]["shape"]
            p = o["opts"]
            if o["op"] == "CONV_2D":                       # OHWI -> OIHW
                W, group = w.transpose(0, 3, 1, 2), 1
            else:                                          # 1,H,W,C*M -> C*M,1,H,W
                W, group = w.transpose(3, 0, 1, 2), cin
            kh, kw = W.shape[2:]
            if p["pad"] == 0:
                t, bt = _same_pads(ih, oh, kh, p["sh"], p["dh"])
                l, r = _same_pads(iw, ow, kw, p["sw"], p["dw"])
            else:
                t = bt = l = r = 0
            y = g.op("Conv", [names[xi], g.const(W.astype(np.float32)), g.const(b.astype(np.float32))],
                     strides=[p["sh"], p["sw"]], dilations=[p["dh"], p["dw"]],
                     pads=[t, l, bt, r], group=group, kernel_shape=[int(kh), int(kw)])
            names[o["out"][0]] = _act(g, y, p["act"])
        elif o["op"] == "ADD":
            a, b = o["in"]
            names[o["out"][0]] = _act(g, g.op("Add", [names[a], names[b]]), o["opts"]["act"])
        else:
            raise ValueError(f"unsupported op after preprocessing: {o['op']}")

    # ── heads: NHWC flatten per level, in the anchors' (y, x, anchor) order ──────
    levels = []
    for k in range(len(out_idx) // 2):
        bx = next(i for i in out_idx if tensors[i]["name"] == f"BoxPredictor_{k}/BoxEncodingPredictor/BiasAdd")
        cl = next(i for i in out_idx if tensors[i]["name"] == f"BoxPredictor_{k}/ClassPredictor/BiasAdd")
        levels.append((bx, cl))
    n_cls = tensors[levels[0][1]]["shape"][3] // (tensors[levels[0][0]]["shape"][3] // 4)
    enc, logit, anchors = [], [], []
    decs = nms_cfg["bbox_decoders"]
    assert len(decs) == len(levels)
    for (bx, cl), dec in zip(levels, decs):
        _, H, W_, c4 = tensors[bx]["shape"]
        A = c4 // 4
        assert A == len(dec["h"]) == len(dec["w"]), (A, dec["name"])
        e = g.op("Transpose", [names[bx]], perm=[0, 2, 3, 1])
        enc.append(g.op("Reshape", [e, g.const(np.array([1, H * W_ * A, 4], np.int64))]))
        c = g.op("Transpose", [names[cl]], perm=[0, 2, 3, 1])
        logit.append(g.op("Reshape", [c, g.const(np.array([1, H * W_ * A, n_cls], np.int64))]))
        for i in range(H):
            for j in range(W_):
                for a in range(A):
                    anchors.append(((i + 0.5) / H, (j + 0.5) / W_, dec["h"][a], dec["w"][a]))
    enc = g.op("Concat", enc, axis=1)                       # [1,N,4] ty,tx,th,tw
    logit = g.op("Concat", logit, axis=1)                   # [1,N,91]
    anc = np.array(anchors, np.float32)                     # yc, xc, h, w (normalised)
    n_anc = anc.shape[0]

    # ── SSD box decoding (TF OD API FasterRcnnBoxCoder, scales 10,10,5,5) ────────
    cs, ds = float(nms_cfg["centers_scale_factor"]), float(nms_cfg["bbox_dimensions_scale_factor"])
    ty, tx, th, tw = g.op("Split", [enc, g.const(np.array([1, 1, 1, 1], np.int64))], n_out=4, axis=2)
    yc = g.op("Add", [g.op("Mul", [g.op("Div", [ty, g.const(np.float32(cs))]), g.const(anc[:, 2:3])]), g.const(anc[:, 0:1])])
    xc = g.op("Add", [g.op("Mul", [g.op("Div", [tx, g.const(np.float32(cs))]), g.const(anc[:, 3:4])]), g.const(anc[:, 1:2])])
    hh = g.op("Mul", [g.op("Exp", [g.op("Div", [th, g.const(np.float32(ds))])]), g.const(anc[:, 2:3] * 0.5)])
    hw = g.op("Mul", [g.op("Exp", [g.op("Div", [tw, g.const(np.float32(ds))])]), g.const(anc[:, 3:4] * 0.5)])
    box = g.op("Concat", [g.op("Sub", [yc, hh]), g.op("Sub", [xc, hw]),
                          g.op("Add", [yc, hh]), g.op("Add", [xc, hw])], axis=2)
    box = g.op("Clip", [box, g.const(np.float32(0.0)), g.const(np.float32(1.0))])   # [1,N,4] y1,x1,y2,x2

    # ── scores: sigmoid, the 80 COCO labels in COCO-80 order ────────────────────
    sc = g.op("Sigmoid", [logit])
    sc = g.op("Gather", [sc, g.const(np.array(COCO80_TO_91, np.int64))], axis=2)   # [1,N,80]
    sc = g.op("Transpose", [sc], perm=[0, 2, 1])                                       # [1,80,N]

    # ── per-class NMS, then top-k by score, padded to a fixed [1,top_k,6] ───────
    sel = g.op("NonMaxSuppression", [box, sc, g.const(np.array([per_class], np.int64)),
                                     g.const(np.array([nms_iou], np.float32)),
                                     g.const(np.array([nms_score_floor], np.float32))],
               center_point_box=0)                                                     # [K,3]
    cls_box = g.op("Slice", [sel, g.const(np.array([1], np.int64)), g.const(np.array([3], np.int64)),
                             g.const(np.array([1], np.int64))])                         # [K,2]
    sc2 = g.op("Squeeze", [sc, g.const(np.array([0], np.int64))])                      # [80,N]
    conf = g.op("GatherND", [sc2, cls_box])                                            # [K]
    bidx = g.op("Squeeze", [g.op("Slice", [sel, g.const(np.array([2], np.int64)), g.const(np.array([3], np.int64)),
                                           g.const(np.array([1], np.int64))]), g.const(np.array([1], np.int64))])
    cidx = g.op("Squeeze", [g.op("Slice", [sel, g.const(np.array([1], np.int64)), g.const(np.array([2], np.int64)),
                                           g.const(np.array([1], np.int64))]), g.const(np.array([1], np.int64))])
    box2 = g.op("Squeeze", [box, g.const(np.array([0], np.int64))])                    # [N,4]
    kcnt = g.op("Min", [g.op("Shape", [conf]), g.const(np.array([top_k], np.int64))])  # [1]
    topv, topi = g.op("TopK", [conf, kcnt], n_out=2, axis=0, largest=1, sorted=1)
    bsel = g.op("Gather", [box2, g.op("Gather", [bidx, topi], axis=0)], axis=0)        # [k,4]
    csel = g.op("Cast", [g.op("Gather", [cidx, topi], axis=0)], to=TensorProto.FLOAT)  # [k]
    y1, x1, y2, x2 = g.op("Split", [bsel, g.const(np.array([1, 1, 1, 1], np.int64))], n_out=4, axis=1)
    fw, fh = g.const(np.float32(frame_w)), g.const(np.float32(frame_h))
    pl, pt = g.const(np.float32(pad_left)), g.const(np.float32(pad_top))
    X1 = g.op("Add", [g.op("Mul", [x1, fw]), pl]); X2 = g.op("Add", [g.op("Mul", [x2, fw]), pl])
    Y1 = g.op("Add", [g.op("Mul", [y1, fh]), pt]); Y2 = g.op("Add", [g.op("Mul", [y2, fh]), pt])
    col = g.const(np.array([1], np.int64))
    rows = g.op("Concat", [X1, Y1, X2, Y2, g.op("Unsqueeze", [topv, col]), g.op("Unsqueeze", [csel, col])],
                axis=1)                                                                  # [k,6]
    padn = g.op("Sub", [g.const(np.array([top_k], np.int64)), kcnt])
    pads = g.op("Concat", [g.const(np.array([0, 0], np.int64)), padn, g.const(np.array([0], np.int64))], axis=0)
    rows = g.op("Pad", [rows, pads, g.const(np.float32(0.0))], mode="constant")          # [top_k,6]
    g.nodes.append(helper.make_node("Unsqueeze", [rows, g.const(np.array([0], np.int64))], ["dets"]))

    outputs = [helper.make_tensor_value_info("dets", TensorProto.FLOAT, [1, top_k, 6])]
    if debug_outputs:   # raw TFLite outputs (NCHW) + the normalised network input, for verification
        dims = lambda r: [f"d{k}" for k in range(r)]
        for n, i in zip(out_names, out_idx):
            dn = n.replace("/", "_")
            g.nodes.append(helper.make_node("Identity", [names[i]], [dn]))
            outputs.append(helper.make_tensor_value_info(dn, TensorProto.FLOAT, dims(4)))
        g.nodes.append(helper.make_node("Identity", [norm_input], ["net_input"]))
        outputs.append(helper.make_tensor_value_info("net_input", TensorProto.FLOAT, dims(4)))
        g.nodes.append(helper.make_node("Identity", [box], ["boxes_norm"]))
        outputs.append(helper.make_tensor_value_info("boxes_norm", TensorProto.FLOAT, dims(3)))
        g.nodes.append(helper.make_node("Identity", [sc], ["scores80"]))
        outputs.append(helper.make_tensor_value_info("scores80", TensorProto.FLOAT, dims(3)))

    graph = helper.make_graph(
        g.nodes, "ssd_mobilenet_v2_e2e",
        [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, S, S])],
        outputs, initializer=g.inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)],
                              producer_name="export_ssd_mobilenet_v2")
    model.ir_version = 8
    meta = {"source_tflite_sha256": _sha256(tflite_path), "frame_wh": f"{frame_w}x{frame_h}",
            "anchors": str(n_anc), "nms_iou": str(nms_iou), "nms_per_class": str(per_class),
            "nms_score_floor": str(nms_score_floor), "labels": "COCO-80 ids (stop sign = 11)"}
    for k, v in meta.items():
        model.metadata_props.append(onnx.StringStringEntryProto(key=k, value=v))
    onnx.checker.check_model(model)
    return model


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--tflite", required=True)
    ap.add_argument("--nms-config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", default="640x480", help="camera frame WxH the engine letterboxes")
    ap.add_argument("--debug-outputs", action="store_true", help="also expose raw head outputs")
    a = ap.parse_args()
    sha = _sha256(a.tflite)
    if sha != SOURCE_SHA256:
        sys.exit(f"unexpected source file (sha256 {sha}); expected the zoo's 2025-01-15 ssd_mobilenet_v2.tflite")
    fw, fh = (int(v) for v in a.frame.lower().split("x"))
    model = build(a.tflite, json.load(open(a.nms_config)), frame_w=fw, frame_h=fh,
                  debug_outputs=a.debug_outputs)
    onnx.save(model, a.out)
    print(f"wrote {a.out}  (input [1,3,{max(fw, fh)},{max(fw, fh)}], output [1,100,6], sha256 {_sha256(a.out)})")


if __name__ == "__main__":
    main()
