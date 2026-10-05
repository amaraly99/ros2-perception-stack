"""gate_render.py <png> <out_json> [--raw] [--sign u v w] [--threads N] [--configs ID_BACKEND ...]: pre-resume gate on the final
held render. 5Q: --configs limits the gate to the listed configurations (e.g. yolo26n_npu); default = all 22. The batch gates
only the run's own configuration per run (one NPU open at most) and all 22 once per session (hold_once.sh --gate).
All 22 configs: CPU in this process, NPU one process per config (stage4/one_image_npu.py, 5 s apart; HailoRT does not reopen
a VDevice in-process).
5M: scores IN-LOOP pixels by default (tools_5M/inloop_pixels.py: the NV12 round trip of the camera bridge and producer, checked
against the producer's frame_hash for ef4a539b; the in-loop image is written next to out_json as <stem>_inloop.png and that PNG
is what the NPU processes read). --raw keeps the 5I path (imread -> BGR2RGB) and labels the output "raw".
Box counting (no detector or NMS change): stop-sign boxes at conf >= 0.20, distinct objects = IoU >= 0.5 groups, a box not on
the projected sign (scorer project() at the live fx, live sim_geometry.json start pose, or --sign) is a failure.
Pass (5M rule a): every YOLO and CenterNet config finds the sign at conf >= 0.20, all its stop-sign boxes form ONE group on the
projected sign and none is elsewhere. Other classes are reported, not counted. SSD is exempt and reported.
Exit 0 = pass, 1 = fail. Old 5I version: prep_5M/a/gate_render.py."""
import argparse, json, os, subprocess, sys, time, io, contextlib, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hilpaths
sys.path.insert(0, hilpaths.PRODUCER)
import inloop_pixels as P
IDS = ["yolo26n","yolo26s","yolo26m","yolov8n","yolov8s","yolov8m","yolov11n","yolov11s","yolov11m","centernet_resnet_v1_18","ssd_mobilenet_v2"]
NPU = os.path.join(hilpaths.HIL_RUNS, "one_image_npu.py")


def cpu_dets(mid, rgb, threads=4, pred=None):
    """[(x1, y1, x2, y2, cls, conf)] at conf >= 0.20, CPU, as in 5I (threads 4 unless told otherwise)."""
    from yolo_runtime_engine import build_predictor
    with contextlib.redirect_stdout(io.StringIO()):
        pred = pred or build_predictor("cpu", model_id=mid, conf=0.20, threads=threads, inter_threads=1)
        return [tuple(float(v) for v in x[:6]) for x in pred.infer(rgb, timing={}) if x[5] >= 0.20]


def npu_dets(png, mid, gap_s=5, tries=3):
    """One one_image_npu.py process per call, 5 s gap, 3 tries, as in 5I. None if every try failed."""
    from yolo_producer import COCO_NAMES
    for _ in range(tries):
        time.sleep(gap_s)
        r = subprocess.run([sys.executable, NPU, png, mid], capture_output=True, text=True)
        line = [l for l in r.stdout.splitlines() if l.startswith("{")]
        if line:
            return [(*P.xyxy_from_cwh(x[4], x[5], x[2], x[3]), COCO_NAMES.index(x[0]), x[1]) for x in json.loads(line[-1])["dets"]]
    return None


def judge(k, d, sign):
    exempt = k.startswith("ssd")
    if d is None:
        return {"stop": None, "pass": False, "exempt": exempt}
    s = P.stop_summary(d, sign)
    return {"stop": s, "pass": s["one_on_sign"] and s["top_on"] is not None, "exempt": exempt}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("png"); ap.add_argument("out")
    ap.add_argument("--raw", action="store_true"); ap.add_argument("--sign", nargs=3, type=float); ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--configs", nargs="+", default=None, help="subset, e.g. yolo26n_npu centernet_resnet_v1_18_cpu (default: all 22)")
    a = ap.parse_args()
    want = a.configs or [f"{m}_{b}" for b in ("cpu", "npu") for m in IDS]
    bad = [k for k in want if k.rsplit("_", 1)[0] not in IDS or k.rsplit("_", 1)[1] not in ("cpu", "npu")]
    if bad:
        sys.exit(f"gate_render: unknown configuration(s) {bad}")
    from yolo_producer import COCO_NAMES
    if os.path.exists(a.out):
        sys.exit(f"gate_render: REFUSE: {a.out} exists")
    bgr = cv2.imread(a.png); t0 = time.time()
    if a.raw:
        pixels, rgb, npu_png = "raw", P.to_raw_rgb(bgr), a.png
    else:
        P.selftest()
        pixels, rgb = "inloop", P.to_inloop_rgb(bgr)
        npu_png = os.path.join(os.path.dirname(os.path.abspath(a.out)), os.path.splitext(os.path.basename(a.out))[0] + "_inloop.png")
        if os.path.exists(npu_png):
            sys.exit(f"gate_render: REFUSE: {npu_png} exists")
        cv2.imwrite(npu_png, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if P.frame_hash(cv2.cvtColor(cv2.imread(npu_png), cv2.COLOR_BGR2RGB)) != P.frame_hash(rgb):
            sys.exit("gate_render: in-loop PNG does not read back to the same pixels")
    if a.sign:
        sign, sign_src = tuple(a.sign), "--sign"
    else:
        sign, sign_src = P.project_sign(P.start_pose()), "sim_geometry.json start pose, scorer project()"
    res = {}
    for mid in IDS:
        if f"{mid}_cpu" in want:
            res[f"{mid}_cpu"] = cpu_dets(mid, rgb, a.threads)
    for mid in IDS:
        if f"{mid}_npu" in want:
            res[f"{mid}_npu"] = npu_dets(npu_png, mid)
    verdict = {k: judge(k, d, sign) for k, d in res.items()}
    ok = all(v["pass"] for v in verdict.values() if not v["exempt"])
    named = {k: None if d is None else [[COCO_NAMES[int(x[4])], round(x[5], 3), round(x[2] - x[0], 1), round(x[3] - x[1], 1),
                                         round((x[0] + x[2]) / 2, 1), round((x[1] + x[3]) / 2, 1)] for x in d] for k, d in res.items()}
    with open(a.out, "x") as f:
        json.dump({"png": a.png, "pixels": pixels, "npu_input_png": npu_png, "frame_hash": P.frame_hash(rgb),
                   "configs_gated": want, "sign_uvw": sign, "sign_source": sign_src, "rule": "5M a: conf >= 0.20, one IoU>=0.5 group on the projected sign, no stop box elsewhere",
                   "seconds": round(time.time() - t0, 1), "pass": ok, "configs": verdict, "all_dets": named}, f, indent=1)
    print(f"pixels: {pixels}  frame_hash {P.frame_hash(rgb)}  sign u {sign[0]:.1f} v {sign[1]:.1f} w {sign[2]:.1f}")
    for k, v in verdict.items():
        s = v["stop"]
        txt = "n/a (NPU error)" if s is None else f"raw {s['raw']} distinct {s['distinct']} off {s['off_sign']} top {s['top_on'] if s['top_on'] is not None else '-'}"
        print(f"{k:28s} {txt:42s} {'PASS' if v['pass'] else 'FAIL'}{' (exempt)' if v['exempt'] else ''}")
    print(f"GATE ({pixels}) {'PASS' if ok else 'FAIL'} in {time.time() - t0:.0f} s")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
