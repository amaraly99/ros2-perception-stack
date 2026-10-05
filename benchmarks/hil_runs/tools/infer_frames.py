"""infer_frames.py <out_dir> <model_id> <cpu|npu> [--raw] [--threads N] [--bin lo hi]  (host python3)
Block 5, step 2. Every detection (class id, conf, w, h, centre) at conf >= 0.20 on every unique frame, one JSON per config.
CPU: build_predictor('cpu', model_id), 4 threads by default (no run in progress). NPU: YOLO via build_predictor('npu'),
SSD and CenterNet via their producers' predictor classes, i.e. the code paths the in-loop producers use.
5M: scores IN-LOOP pixels by default (stage6/tools_5M/inloop_pixels.py, NV12 round trip, self-tested against the producer's
frame_hash; each frame's in-loop frame_hash is stored). --raw keeps the old path (BGR2RGB) and labels the output "raw".
Output dets_<model>_<backend>_<inloop|raw>[_bin<lo>-<hi>].json, never overwritten (exclusive create); the old
dets_<model>_<backend>.json files are left as they are. Per frame also: stop-sign box count (conf >= 0.20), distinct objects
(IoU >= 0.5 groups) and boxes off the projected sign (manifest u, v, expected_w_px). No detector or NMS change.
--bin lo hi: only frames with lo <= range_h_m < hi (as summarise_5I.py bins). Old version: stage6/prep_5M/a/infer_frames_stage5.py."""
import argparse, json, os, sys, time
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # benchmarks/hil_runs (5Q: was two absolute paths)
import hilpaths
sys.path.insert(0, hilpaths.PRODUCER)
import inloop_pixels as P
from yolo_runtime_engine import build_predictor, ModelRegistry
ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("mid"); ap.add_argument("be", choices=["cpu", "npu"])
ap.add_argument("--raw", action="store_true"); ap.add_argument("--threads", type=int, default=4); ap.add_argument("--bin", nargs=2, type=float)
a = ap.parse_args(); out, mid, be = a.out, a.mid, a.be
pixels = "raw" if a.raw else "inloop"
dst = f"{out}/dets_{mid}_{be}_{pixels}" + (f"_bin{a.bin[0]:g}-{a.bin[1]:g}" if a.bin else "") + ".json"
if os.path.exists(dst):
    sys.exit(f"infer_frames: REFUSE: {dst} exists")
if not a.raw:
    P.selftest()
man = json.load(open(f"{out}/manifest.json"))
idx = [i for i, r in enumerate(man["frames"]) if not a.bin or (r.get("range_h_m") is not None and a.bin[0] <= r["range_h_m"] < a.bin[1])]
if be == "npu" and mid.startswith("ssd"):
    from ssd_hailo_producer import SSDHailoPredictor; pred = SSDHailoPredictor(ModelRegistry().resolve(mid)["hef"], 0.20)
elif be == "npu" and mid.startswith("centernet"):
    from centernet_hailo_producer import CenterNetHailoPredictor; pred = CenterNetHailoPredictor(ModelRegistry().resolve(mid)["hef"], 0.20)
elif be == "npu":
    pred = build_predictor("npu", model_id=mid, conf=0.20)
else:
    pred = build_predictor("cpu", model_id=mid, conf=0.20, threads=a.threads, inter_threads=1)
res, hashes, stop, t0 = [], [], [], time.time()
for i in idx:
    r = man["frames"][i]
    f = f"{out}/frames/{r['file']}"
    a3 = np.load(f) if f.endswith(".npy") else np.fromfile(f, np.uint8).reshape(r["h"], r["w"], -1)
    bgr = a3 if r["encoding"] == "bgr8" else cv2.cvtColor(a3, cv2.COLOR_RGB2BGR)
    rgb = P.to_raw_rgb(bgr) if a.raw else P.to_inloop_rgb(bgr)
    dets = [d for d in pred.infer(rgb, timing={}) if d[5] >= 0.20]
    res.append([[int(d[4]), round(float(d[5]), 4), round(float(d[2] - d[0]), 1), round(float(d[3] - d[1]), 1),
                 round(float(d[0] + d[2]) / 2, 1), round(float(d[1] + d[3]) / 2, 1)] for d in dets])
    hashes.append(P.frame_hash(rgb))
    sign = (r["u"], r["v"], r["expected_w_px"]) if r.get("u") is not None else None
    stop.append(P.stop_summary([tuple(float(v) for v in d[:6]) for d in dets], sign))
with open(dst, "x") as fo:
    json.dump({"model": mid, "backend": be, "pixels": pixels, "bin": a.bin, "threads": a.threads if be == "cpu" else None,
               "frame_index": idx, "n": len(res), "seconds": round(time.time() - t0, 1), "dets": res, "frame_hash": hashes,
               "stop": stop}, fo)
print(f"{mid} {be} ({pixels}): {len(res)} frames in {time.time() - t0:.0f} s -> {dst}")
