"""one_image_npu.py <png> <model_id>  — one NPU config on one image, one process (HailoRT VDevice cannot be reopened in-process)."""
import os, sys, json, io, contextlib, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hilpaths
sys.path.insert(0, hilpaths.PRODUCER)
from yolo_runtime_engine import build_predictor, ModelRegistry
from yolo_producer import COCO_NAMES
png, mid = sys.argv[1], sys.argv[2]
rgb = cv2.cvtColor(cv2.imread(png), cv2.COLOR_BGR2RGB)
with contextlib.redirect_stdout(io.StringIO()):
    if mid.startswith("ssd"):
        from ssd_hailo_producer import SSDHailoPredictor; p = SSDHailoPredictor(ModelRegistry().resolve(mid)["hef"], 0.20)
    elif mid.startswith("centernet"):
        from centernet_hailo_producer import CenterNetHailoPredictor; p = CenterNetHailoPredictor(ModelRegistry().resolve(mid)["hef"], 0.20)
    else:
        p = build_predictor("npu", model_id=mid, conf=0.20)
    dets = [d for d in p.infer(rgb, timing={}) if d[5] >= 0.20]
print(json.dumps({"id": f"{mid}_npu", "dets": [[COCO_NAMES[int(d[4])], round(float(d[5]),3), round(float(d[2]-d[0]),1), round(float(d[3]-d[1]),1),
      round(float(d[0]+d[2])/2,1), round(float(d[1]+d[3])/2,1)] for d in dets]}))
