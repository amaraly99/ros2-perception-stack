#!/usr/bin/env python3
"""
ssd_hailo_producer — SSD (Hailo Model Zoo ssd_mobilenet_v2) on the Hailo-10H NPU.

The NPU half of the non-YOLO row. The CPU half needs no node code: the model's
end-to-end ONNX (models/export_ssd_mobilenet_v2.py) runs through the existing
yolo_shm_producer ONNX path. The NPU half needs this adapter because the zoo's
SSD HEF differs from the YOLO HEFs in two ways the YOLO parser does not handle:

  * input: a plain resize of the whole frame to 300x300 (as the model was
    trained), not a letterbox; normalisation is inside the HEF;
  * output: NMS by class over the 90 TF/COCO labels (background removed), not
    the 80 COCO-80 ids. Labels are mapped to COCO-80 (stop sign 13 -> 11), so
    yolo_bridge names them exactly as it names YOLO detections.

Selected by run_stack_hil.sh when the registry entry sets "npu_producer"; the
command is the YOLO NPU rows' own form: --model ssd_mobilenet_v2 --backend npu.
SHM plumbing, telemetry and the /yolo_shm schema come from detector_producer_base.

CLI:
    python3 ssd_hailo_producer.py --model_id ssd_mobilenet_v2 [--hef PATH]
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

# TF/COCO 91-slot label ids of the 80 COCO classes, in COCO-80 order (as the ONNX export).
COCO80_TO_91 = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                22, 23, 24, 25, 27, 28, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
                44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
                64, 65, 67, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87,
                88, 89, 90]
TF91_TO_COCO80 = {tf: c for c, tf in enumerate(COCO80_TO_91)}


def _now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def parse_nms_by_class(flat, n_classes, label_offset, src_w, src_h, conf, max_per_class):
    """HAILO NMS BY CLASS, FLOAT32: per class [count][count x (y1, x1, y2, x2, score)],
    boxes normalised to the (plain-resized) network input, i.e. to the whole frame."""
    dets, off = [], 0
    for c in range(n_classes):
        if off >= flat.shape[0]:
            break
        count = min(int(flat[off]), max_per_class)
        off += 1
        coco = TF91_TO_COCO80.get(c + label_offset)
        for j in range(count):
            y1, x1, y2, x2, score = flat[off + 5 * j:off + 5 * j + 5]
            if coco is None or score < conf:
                continue
            dets.append((float(np.clip(x1 * src_w, 0, src_w - 1)), float(np.clip(y1 * src_h, 0, src_h - 1)),
                         float(np.clip(x2 * src_w, 0, src_w - 1)), float(np.clip(y2 * src_h, 0, src_h - 1)),
                         coco, float(score)))
        off += count * 5
    dets.sort(key=lambda d: -d[5])
    return dets


class SSDHailoPredictor:
    """Predictor contract of detector_producer_base, on pyHailoRT. The HailoRT calls mirror
    yolo_runtime_engine.HailoNPUYoloPredictor, which the YOLO NPU rows run."""

    def __init__(self, hef_path: str, conf: float, max_per_class: int = 20):
        from hailo_platform import VDevice, FormatType      # lazy: CPU-only hosts lack it
        self.conf = float(conf)
        self.max_per_class = int(max_per_class)
        self._vdevice = VDevice()
        infer_model = self._vdevice.create_infer_model(hef_path)
        in_info = infer_model.input()
        self.in_h, self.in_w = in_info.shape[0], in_info.shape[1]
        outs = infer_model.outputs
        if len(outs) != 1:
            raise RuntimeError(f"expected one NMS output, got {[o.name for o in outs]}")
        out = outs[0]
        out.set_format_type(FormatType.FLOAT32)
        # The zoo compiled this HEF's NMS with a 0.30 score threshold. Lower it to the
        # pipeline's confidence gate where HailoRT allows it (host-side NMS); otherwise
        # the row keeps the HEF's 0.30 and says so at startup.
        self.nms_threshold_note = "HEF default (0.30)"
        try:
            out.set_nms_score_threshold(self.conf)
            self.nms_threshold_note = f"set to {self.conf:.2f}"
        except Exception as e:                                  # noqa: BLE001
            self.nms_threshold_note = f"HEF default (0.30); lowering failed: {e}"
        self._configured = infer_model.configure()
        self._bindings = self._configured.create_bindings()
        self._input = np.empty((self.in_h, self.in_w, 3), dtype=np.uint8)
        self._bindings.input().set_buffer(self._input)
        self._out = np.empty(out.shape, dtype=np.float32)
        self._bindings.output(out.name).set_buffer(self._out)
        # 90 classes = background removed (index 0 = TF label 1); 91 = background kept.
        self.n_classes = self._n_classes(out)
        self.label_offset = 1 if self.n_classes == 90 else 0
        print(f"[ssd_hailo] {os.path.basename(hef_path)} in={self.in_w}x{self.in_h} out={out.name} "
              f"shape={tuple(out.shape)} classes={self.n_classes} label_offset={self.label_offset} "
              f"nms_score_threshold={self.nms_threshold_note}", flush=True)

    @staticmethod
    def _n_classes(out):
        for attr in ("nms_shape",):
            shp = getattr(out, attr, None)
            n = getattr(shp, "number_of_classes", None) if shp is not None else None
            if n:
                return int(n)
        return 90          # zoo config: 91 classes with background removal

    def infer(self, rgb, timing=None):
        t = timing if timing is not None else {}
        src_h, src_w = rgb.shape[:2]
        self._input[:] = cv2.resize(rgb, (self.in_w, self.in_h), interpolation=cv2.INTER_LINEAR)
        t["pre_done"] = _now_ns()
        t["inf_start"] = _now_ns()
        self._configured.run([self._bindings], timeout=2000)
        t["inf_done"] = _now_ns()
        dets = parse_nms_by_class(self._out.reshape(-1), self.n_classes, self.label_offset,
                                  src_w, src_h, self.conf, self.max_per_class)
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
    ap = argparse.ArgumentParser(description="SSD (Hailo zoo) NPU producer -> /yolo_shm")
    ap.add_argument("--model_id", default="ssd_mobilenet_v2", help="registry id (for the hef path)")
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
        sys.exit(f"[ssd_hailo] HEF not found: {hef}")
    pred = SSDHailoPredictor(hef, cfg.conf)
    run_producer(cfg, pred, cfg.model_id,
                 f"[ssd_hailo] {cfg.model_id} NPU conf={cfg.conf} frame={{w}}x{{h}} -> /yolo_shm")


if __name__ == "__main__":
    main()
