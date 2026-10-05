"""nv12_roundtrip.py <raw_bgr8> <out_prefix>: reproduce the in-loop pixels offline.
In-loop path: MATLAB Image (bgr8) -> sim_camera_bridge_node.cpp:152 cv_bridge toCvCopy(msg, "rgb8") (channel swap)
-> :161 cv::cvtColor(RGB2YUV_I420) -> :165-197 repack U,V into NV12 interleaved -> shm
-> yolo_producer.py:190-197 reshape (h*3/2, w), cv2.cvtColor(COLOR_YUV2RGB_NV12) -> predictor.infer(rgb).
Gate path: cv2.imread(png) BGR -> COLOR_BGR2RGB -> predictor.infer(rgb).
Prints sha1 of the raw, and the producer-style frame_hash (blake2b-4, little endian) of both RGB arrays."""
import hashlib, sys, numpy as np, cv2
raw, pre = sys.argv[1], sys.argv[2]
b = open(raw, "rb").read(); print("raw sha1", hashlib.sha1(b).hexdigest(), "opencv", cv2.__version__)
bgr = np.frombuffer(b, np.uint8).reshape(480, 640, 3)
rgb = np.ascontiguousarray(bgr[:, :, ::-1])                      # cv_bridge bgr8 -> rgb8
i420 = cv2.cvtColor(rgb, cv2.COLOR_RGB2YUV_I420)                 # (720, 640): Y 480 rows, U 120 rows, V 120 rows
H, W = 480, 640; y = i420[:H].reshape(-1); u = i420[H:H + H // 4].reshape(-1); v = i420[H + H // 4:].reshape(-1)
uv = np.empty(H * W // 2, np.uint8); uv[0::2] = u; uv[1::2] = v   # NV12 interleave, as the bridge loop does
nv12 = np.concatenate([y, uv]).reshape(H * 3 // 2, W)
rgb_loop = cv2.cvtColor(nv12, cv2.COLOR_YUV2RGB_NV12)
fh = lambda a: int.from_bytes(hashlib.blake2b(a.tobytes(), digest_size=4).digest(), "little")
gate_rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(cv2.imencode(".png", bgr)[1], np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
print("frame_hash gate-path RGB   ", fh(gate_rgb), " (identical to direct swap:", np.array_equal(gate_rgb, rgb), ")")
print("frame_hash NV12-round-trip ", fh(rgb_loop))
d = np.abs(rgb_loop.astype(int) - rgb.astype(int))
print(f"round trip vs original: mean abs {d.mean():.2f}, max {d.max()}, frac px any channel >10: {(d.max(axis=2) > 10).mean():.3f}")
s = d[372 - 60:400 - 60, 305:335] if False else None
np.save(pre + "_rgb_loop.npy", rgb_loop); cv2.imwrite(pre + "_loop.png", cv2.cvtColor(rgb_loop, cv2.COLOR_RGB2BGR))
