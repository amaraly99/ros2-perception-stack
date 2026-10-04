#!/usr/bin/env python3
"""
centernet_hailo_producer — CenterNet (Hailo Model Zoo centernet_resnet_v1_18_postprocess)
on the Hailo-10H NPU.

The NPU half of the CenterNet row. The CPU half needs no node code: its
end-to-end ONNX (models/export_centernet_resnet_v1_18.py) runs through the
existing yolo_shm_producer ONNX path. The NPU half needs this adapter because
the zoo's CenterNet HEF has three outputs the YOLO parser does not read: box
size and centre offset maps (128x128x2 each) and a heatmap already reduced to
its 3x3 peaks on the device (128x128x80). Decoding follows the zoo's
CenternetPostProc: top 100 peaks, output stride 4, no NMS. Input is the zoo's
resize-with-pad to 512x512 (zero padding); normalisation is inside the HEF.

Selected by run_stack_hil.sh when the registry entry sets "npu_producer"; the
command is the YOLO NPU rows' own form: --model centernet_resnet_v1_18 --backend npu.

CLI:
    python3 centernet_hailo_producer.py --model_id centernet_resnet_v1_18 [--hef PATH]
            [--conf 0.20] [--telemetry_csv PATH]
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from detector_producer_base import run_producer  # noqa: E402

TOP_K = 100
STRIDE = 4


def _now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def decode(heat, size, offset, conf, scale, pad_left, pad_top, src_w, src_h):
    """Zoo CenternetPostProc on NHWC maps: heat (H,W,80, peaks only), size/offset (H,W,2).
    Returns [(x1,y1,x2,y2,class_id,conf)] in source pixels, class ids COCO-80."""
    flat = heat.reshape(-1)
    k = min(TOP_K, flat.size)
    idx = np.argpartition(flat, -k)[-k:]
    idx = idx[flat[idx] >= conf]
    idx = idx[np.argsort(-flat[idx])]
    H, W, C = heat.shape
    ys, rem = np.divmod(idx, W * C)
    xs, cls = np.divmod(rem, C)
    w, h = size[ys, xs, 0] * STRIDE, size[ys, xs, 1] * STRIDE
    cx, cy = (xs + offset[ys, xs, 0]) * STRIDE, (ys + offset[ys, xs, 1]) * STRIDE
    x1 = np.clip((cx - w / 2 - pad_left) / scale, 0, src_w - 1)
    y1 = np.clip((cy - h / 2 - pad_top) / scale, 0, src_h - 1)
    x2 = np.clip((cx + w / 2 - pad_left) / scale, 0, src_w - 1)
    y2 = np.clip((cy + h / 2 - pad_top) / scale, 0, src_h - 1)
    return [(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]), int(cls[i]), float(flat[idx[i]]))
            for i in range(len(idx))]


class CenterNetHailoPredictor:
    """Predictor contract of detector_producer_base, on pyHailoRT. The HailoRT calls mirror
    yolo_runtime_engine.HailoNPUYoloPredictor, which the YOLO NPU rows run."""

    def __init__(self, hef_path: str, conf: float):
        from hailo_platform import VDevice, FormatType      # lazy: CPU-only hosts lack it
        self.conf = float(conf)
        self._vdevice = VDevice()
        infer_model = self._vdevice.create_infer_model(hef_path)
        in_info = infer_model.input()
        self.in_h, self.in_w = in_info.shape[0], in_info.shape[1]
        outs = infer_model.outputs
        for o in outs:
            o.set_format_type(FormatType.FLOAT32)
        # Output order of the zoo's HEF = its host decode's endnode order: size, offset, heatmap.
        heat = [o for o in outs if o.shape[-1] == 80]
        two = [o for o in outs if o.shape[-1] == 2]
        if len(heat) != 1 or len(two) != 2:
            raise RuntimeError(f"unexpected outputs {[(o.name, o.shape) for o in outs]}")
        self._names = {"heat": heat[0].name, "size": two[0].name, "offset": two[1].name}
        self._configured = infer_model.configure()
        self._bindings = self._configured.create_bindings()
        self._input = np.empty((self.in_h, self.in_w, 3), dtype=np.uint8)
        self._bindings.input().set_buffer(self._input)
        self._buf = {}
        for o in outs:
            self._buf[o.name] = np.empty(o.shape, dtype=np.float32)
            self._bindings.output(o.name).set_buffer(self._buf[o.name])
        self._first = True
        print(f"[centernet_hailo] {os.path.basename(hef_path)} in={self.in_w}x{self.in_h} "
              f"outputs={[(o.name, tuple(o.shape)) for o in outs]} roles={self._names}", flush=True)

    def infer(self, rgb, timing=None):
        t = timing if timing is not None else {}
        src_h, src_w = rgb.shape[:2]
        scale = min(self.in_w / src_w, self.in_h / src_h)
        nw, nh = int(round(src_w * scale)), int(round(src_h * scale))
        pad_left, pad_top = (self.in_w - nw) // 2, (self.in_h - nh) // 2
        self._input[:] = 0                                        # zoo: resize_with_pad, zero padding
        self._input[pad_top:pad_top + nh, pad_left:pad_left + nw] = cv2.resize(rgb, (nw, nh))
        t["pre_done"] = _now_ns()
        t["inf_start"] = _now_ns()
        self._configured.run([self._bindings], timeout=2000)
        t["inf_done"] = _now_ns()
        heat = self._buf[self._names["heat"]].reshape(-1, self.in_w // STRIDE, 80)
        size = self._buf[self._names["size"]].reshape(-1, self.in_w // STRIDE, 2)
        offset = self._buf[self._names["offset"]].reshape(-1, self.in_w // STRIDE, 2)
        if self._first:      # role check for the first NPU run: offsets lie in [0,1), sizes are tens of px
            self._first = False
            print(f"[centernet_hailo] first frame: |size| max {np.abs(size).max():.2f}, "
                  f"|offset| max {np.abs(offset).max():.2f}, heat max {heat.max():.3f}", flush=True)
        dets = decode(heat, size, offset, self.conf, scale, pad_left, pad_top, src_w, src_h)
        t["nms_done"] = _now_ns()
        return dets

    def close(self):
        """Clean release (a leaked VDevice wedges the Hailo SoC off PCIe)."""
        try:
            self._configured.shutdown()
        except Exception:                                       # noqa: BLE001
            pass
        try:
            self._vdevice.release()
        except Exception:                                       # noqa: BLE001
            pass


def main():
    ap = argparse.ArgumentParser(description="CenterNet (Hailo zoo) NPU producer -> /yolo_shm")
    ap.add_argument("--model_id", default="centernet_resnet_v1_18", help="registry id (for the hef path)")
    ap.add_argument("--hef", default=None, help="explicit .hef (bypasses the registry)")
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--telemetry_csv", default=None)
    ap.add_argument("--frames", dest="max_frames", type=int, default=0,
                    help="stop after N frames (0 = unlimited)")
    cfg = ap.parse_args()
    cfg.no_image = True

    hef = cfg.hef
    if hef is None:
        from yolo_runtime_engine import ModelRegistry
        hef = ModelRegistry().resolve(cfg.model_id)["hef"]
    if not hef or not os.path.exists(hef):
        sys.exit(f"[centernet_hailo] HEF not found: {hef}")
    pred = CenterNetHailoPredictor(hef, cfg.conf)
    run_producer(cfg, pred, cfg.model_id,
                 f"[centernet_hailo] {cfg.model_id} NPU conf={cfg.conf} frame={{w}}x{{h}} -> /yolo_shm")


if __name__ == "__main__":
    main()
