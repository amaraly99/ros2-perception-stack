#!/usr/bin/env python3
"""
export_centernet_resnet_v1_18 — CPU ONNX for the Hailo Model Zoo's
centernet_resnet_v1_18_postprocess (CenterNet, "Objects as Points", ResNet-18).

Same weights as the zoo's Hailo-10H HEF: the zoo compiles its HEF from this
pretrained file, cut at the same nodes (hailo_model_zoo cfg/networks/
centernet_resnet_v1_18_postprocess.yaml).

    source  https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ObjectDetection/
            Detection-COCO/centernet/centernet_resnet_v1_18/pretrained/2023-07-18/centernet_resnet_v1_18.zip
            -> centernet_res18_with_postprocess.tflite
               sha256 539383ec42e2075e1678f599ba834e63e70b46188fe8925128157b5c6dd1e690

Decoding follows the zoo's CenternetPostProc (core/postprocessing/detection/
centernet.py): 3x3 max-pool peak mask on the sigmoid heatmap, top 100 peaks,
output stride 4, size channels (w, h), offset channels (dx, dy); no NMS.

The exported graph speaks the engine's end-to-end layout ('yolo26_e2e'), so the
existing ONNX path runs it unchanged:

    input  images [1,3,512,512] float32 RGB in 0..1  (the engine's letterbox,
                                                     as the zoo's resize_with_pad)
    output dets   [1,100,6]  x1,y1,x2,y2 (canvas px), conf, COCO-80 class id,
                             sorted by conf; rows below the engine's conf drop out

In the graph: x*255; the engine pads its letterbox with grey (114) where the
zoo pads with zeros, so the padding rows are zeroed for the configured camera
frame; (x - mean) / std (ImageNet RGB, the zoo's normalisation); the ResNet-18
and deconvolution head of the TFLite file; the zoo decode above. The TFLite
file's own on-chip peak finder (a TF Flex op) is replaced by the same decode.

Usage:
    python3 models/export_centernet_resnet_v1_18.py --tflite <centernet_res18_with_postprocess.tflite> \\
        --out models/onnx/centernet_resnet_v1_18/centernet_resnet_v1_18_e2e.onnx
"""

import argparse
import os
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_ssd_mobilenet_v2 import read_tflite, _G, _same_pads, _act, _sha256  # noqa: E402

SOURCE_SHA256 = "539383ec42e2075e1678f599ba834e63e70b46188fe8925128157b5c6dd1e690"
HEATMAP, SIZE, OFFSET = "activation_26/Sigmoid", "centernet0_conv3/BiasAdd", "centernet0_conv5/BiasAdd"
MEAN, STD = [123.675, 116.28, 103.53], [58.395, 57.12, 57.375]


def _const_nchw(g, arr):
    """A TFLite constant operand in NHWC terms -> an NCHW-broadcastable initializer."""
    a = np.asarray(arr, np.float32)
    if a.ndim == 1:
        a = a.reshape(1, -1, 1, 1)
    elif a.ndim == 4:
        a = a.transpose(0, 3, 1, 2)
    return g.const(a)


def build(tflite_path, frame_w=640, frame_h=480, top_k=100, debug_outputs=False):
    tensors, ops, _, _ = read_tflite(tflite_path)
    by_name = {t["name"]: i for i, t in enumerate(tensors)}
    producer = {o["out"][0]: k for k, o in enumerate(ops)}
    targets = [by_name[HEATMAP], by_name[SIZE], by_name[OFFSET]]

    # ops needed for the three heads (backward walk); the Flex peak-finder chain is left out
    need, stack = set(), list(targets)
    while stack:
        t = stack.pop()
        k = producer.get(t)
        if k is None or k in need:
            continue
        need.add(k)
        o = ops[k]
        if o["op"] == "TRANSPOSE_CONV":
            stack.extend(o["in"][1:])            # the output-shape tensor is static here
        else:
            stack.extend(i for i in o["in"] if i >= 0)

    g = _G()
    S = 512
    net_in = tensors[by_name["input_1"]]["shape"]
    assert net_in[1] == net_in[2] == S, net_in
    scale = min(S / frame_w, S / frame_h)
    nh, nw = int(round(frame_h * scale)), int(round(frame_w * scale))
    top, left = (S - nh) // 2, (S - nw) // 2
    mask = np.zeros((1, 1, S, S), np.float32)
    mask[:, :, top:top + nh, left:left + nw] = 1.0

    x = g.op("Mul", ["images", g.const(np.float32(255.0))])
    x = g.op("Mul", [x, g.const(mask)])                               # zoo pads with zeros
    x = g.op("Div", [g.op("Sub", [x, g.const(np.array(MEAN, np.float32).reshape(1, 3, 1, 1))]),
                     g.const(np.array(STD, np.float32).reshape(1, 3, 1, 1))])
    names = {by_name["input_1"]: x}
    net_input = x

    def val(i):
        if i in names:
            return names[i]
        return _const_nchw(g, tensors[i]["data"])

    for k in sorted(need):
        o = ops[k]
        t_out = o["out"][0]
        p = o["opts"]
        if o["op"] == "PAD":
            pads = tensors[o["in"][1]]["data"].reshape(4, 2)          # NHWC [before, after]
            nchw = [0, 0, int(pads[1, 0]), int(pads[2, 0]), 0, 0, int(pads[1, 1]), int(pads[2, 1])]
            y = g.op("Pad", [val(o["in"][0]), g.const(np.array(nchw, np.int64)), g.const(np.float32(0.0))],
                     mode="constant")
        elif o["op"] == "CONV_2D":
            xi, wi, bi = o["in"]
            w, b = tensors[wi]["data"], tensors[bi]["data"]
            _, ih, iw, _ = tensors[xi]["shape"]
            _, oh, ow, _ = tensors[t_out]["shape"]
            W = w.transpose(0, 3, 1, 2)
            kh, kw = W.shape[2:]
            if p["pad"] == 0:
                t_, b_ = _same_pads(ih, oh, kh, p["sh"], p["dh"])
                l_, r_ = _same_pads(iw, ow, kw, p["sw"], p["dw"])
            else:
                t_ = b_ = l_ = r_ = 0
            y = g.op("Conv", [val(xi), g.const(W.astype(np.float32)), g.const(b.astype(np.float32))],
                     strides=[p["sh"], p["sw"]], dilations=[p["dh"], p["dw"]], pads=[t_, l_, b_, r_],
                     kernel_shape=[int(kh), int(kw)])
            y = _act(g, y, p["act"])
        elif o["op"] == "TRANSPOSE_CONV":
            _, wi, xi = o["in"][:3]
            w = tensors[wi]["data"]                                   # [O, kh, kw, I]
            _, ih, iw, _ = tensors[xi]["shape"]
            _, oh, ow, _ = tensors[t_out]["shape"]
            W = w.transpose(3, 0, 1, 2)                               # -> [I, O, kh, kw]
            kh, kw = W.shape[2:]
            if p["pad"] == 0:
                th = max((ih - 1) * p["sh"] + kh - oh, 0)
                tw = max((iw - 1) * p["sw"] + kw - ow, 0)
                pads = [th // 2, tw // 2, th - th // 2, tw - tw // 2]
            else:
                pads = [0, 0, 0, 0]
            ins = [val(xi), g.const(W.astype(np.float32))]
            if len(o["in"]) > 3 and o["in"][3] >= 0:
                ins.append(g.const(tensors[o["in"][3]]["data"].astype(np.float32)))
            y = g.op("ConvTranspose", ins, strides=[p["sh"], p["sw"]], pads=pads, kernel_shape=[int(kh), int(kw)])
            y = _act(g, y, p.get("act", 0))
        elif o["op"] == "MAX_POOL_2D":
            _, ih, iw, _ = tensors[o["in"][0]]["shape"]
            _, oh, ow, _ = tensors[t_out]["shape"]
            if p["pad"] == 0:
                t_, b_ = _same_pads(ih, oh, p["fh"], p["sh"], 1)
                l_, r_ = _same_pads(iw, ow, p["fw"], p["sw"], 1)
            else:
                t_ = b_ = l_ = r_ = 0
            y = g.op("MaxPool", [val(o["in"][0])], kernel_shape=[p["fh"], p["fw"]], strides=[p["sh"], p["sw"]],
                     pads=[t_, l_, b_, r_])
            y = _act(g, y, p["act"])
        elif o["op"] in ("ADD", "MUL", "SUB"):
            a, b = o["in"]
            y = g.op({"ADD": "Add", "MUL": "Mul", "SUB": "Sub"}[o["op"]], [val(a), val(b)])
            y = _act(g, y, p["act"])
        elif o["op"] == "LOGISTIC":
            y = g.op("Sigmoid", [val(o["in"][0])])
        else:
            raise ValueError(f"op needed for the heads is not supported: {o['op']} (op {k})")
        names[t_out] = y

    heat, size, off = names[targets[0]], names[targets[1]], names[targets[2]]   # NCHW
    _, H, W_, C = tensors[targets[0]]["shape"]
    stride = S // H

    # ── zoo decode: 3x3 peak mask, top-k, stride, offsets and sizes ──────────────
    hmax = g.op("MaxPool", [heat], kernel_shape=[3, 3], strides=[1, 1], pads=[1, 1, 1, 1])
    peaks = g.op("Mul", [heat, g.op("Cast", [g.op("Equal", [heat, hmax])], to=TensorProto.FLOAT)])
    flat = g.op("Reshape", [peaks, g.const(np.array([1, C * H * W_], np.int64))])
    score, idx = g.op("TopK", [flat, g.const(np.array([top_k], np.int64))], n_out=2, axis=1, largest=1, sorted=1)
    hw = g.const(np.array(H * W_, np.int64))
    cls = g.op("Div", [idx, hw])                                      # [1,k] int64
    pos = g.op("Mod", [idx, hw])
    ys = g.op("Cast", [g.op("Div", [pos, g.const(np.array(W_, np.int64))])], to=TensorProto.FLOAT)
    xs = g.op("Cast", [g.op("Mod", [pos, g.const(np.array(W_, np.int64))])], to=TensorProto.FLOAT)
    pos1 = g.op("Squeeze", [pos, g.const(np.array([0], np.int64))])   # [k]
    sz = g.op("Gather", [g.op("Reshape", [size, g.const(np.array([2, H * W_], np.int64))]), pos1], axis=1)   # [2,k]
    of = g.op("Gather", [g.op("Reshape", [off, g.const(np.array([2, H * W_], np.int64))]), pos1], axis=1)
    sw_, sh_ = g.op("Split", [sz, g.const(np.array([1, 1], np.int64))], n_out=2, axis=0)   # [1,k] each
    dx, dy = g.op("Split", [of, g.const(np.array([1, 1], np.int64))], n_out=2, axis=0)
    st = g.const(np.float32(stride))
    cx = g.op("Mul", [g.op("Add", [xs, dx]), st])
    cy = g.op("Mul", [g.op("Add", [ys, dy]), st])
    hw_ = g.op("Mul", [sw_, g.const(np.float32(stride / 2.0))])
    hh_ = g.op("Mul", [sh_, g.const(np.float32(stride / 2.0))])
    cols = [g.op("Sub", [cx, hw_]), g.op("Sub", [cy, hh_]), g.op("Add", [cx, hw_]), g.op("Add", [cy, hh_]),
            score, g.op("Cast", [cls], to=TensorProto.FLOAT)]                              # each [1,k]
    rows = g.op("Concat", [g.op("Unsqueeze", [c_, g.const(np.array([2], np.int64))]) for c_ in cols], axis=2)
    g.nodes.append(helper.make_node("Identity", [rows], ["dets"]))                          # [1,k,6]

    outputs = [helper.make_tensor_value_info("dets", TensorProto.FLOAT, [1, top_k, 6])]
    if debug_outputs:
        for nm, v in (("heatmap", heat), ("size", size), ("offset", off), ("net_input", net_input)):
            g.nodes.append(helper.make_node("Identity", [v], [nm]))
            outputs.append(helper.make_tensor_value_info(nm, TensorProto.FLOAT, ["n", "c", "h", "w"]))
    graph = helper.make_graph(g.nodes, "centernet_resnet_v1_18_e2e",
                              [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, S, S])],
                              outputs, initializer=g.inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)],
                              producer_name="export_centernet_resnet_v1_18")
    model.ir_version = 8
    for k_, v_ in {"source_tflite_sha256": _sha256(tflite_path), "frame_wh": f"{frame_w}x{frame_h}",
                   "decode": "zoo CenternetPostProc: 3x3 peak mask, top-100, stride 4", "top_k": str(top_k),
                   "labels": "COCO-80 ids (stop sign = 11)"}.items():
        model.metadata_props.append(onnx.StringStringEntryProto(key=k_, value=v_))
    onnx.checker.check_model(model)
    return model


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--tflite", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", default="640x480", help="camera frame WxH the engine letterboxes")
    ap.add_argument("--debug-outputs", action="store_true", help="also expose the raw heads")
    a = ap.parse_args()
    sha = _sha256(a.tflite)
    if sha != SOURCE_SHA256:
        sys.exit(f"unexpected source file (sha256 {sha}); expected the zoo's 2023-07-18 centernet_res18 tflite")
    fw, fh = (int(v) for v in a.frame.lower().split("x"))
    model = build(a.tflite, frame_w=fw, frame_h=fh, debug_outputs=a.debug_outputs)
    onnx.save(model, a.out)
    print(f"wrote {a.out}  (input [1,3,512,512], output [1,100,6], sha256 {_sha256(a.out)})")


if __name__ == "__main__":
    main()
