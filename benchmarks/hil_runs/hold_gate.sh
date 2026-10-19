#!/usr/bin/env bash
# hold_gate.sh <outdir>  — HOLD_GATE_CMD for one_run_hold_gated.sh (5D steps 5-6). Runs while held, before the stack boots.
# 1) wait until the newest render has >= 20 identical consecutive copies (render_hash_logger, /tmp/r20_hash.log), max 90 s
# 2) report every distinct render since the logger started, with first-seen time and copies
# 3) 5Q: the final render must be in the profile's accepted held-render set (config/held_reference*.json render_sha1
#    plus any accepted_renders members; held_members.py, DEVIATIONS 17.84); any other render -> rc 1 (no resume)
# 4) 5Q: gate_render.py on GATE_CONFIGS only (the run's own configuration, e.g. yolo26n_npu; "all" = the 22). Each NPU
#    configuration costs one VDevice open, and the Hailo-10H on Pi-3 has failed after about 70 opens per boot (5Q, INFERRED).
#    All 22 are gated once per session on the same render (hold_once.sh --gate all). fail -> rc 1 (no resume)
# 5) on pass: start the approach recorder (rec.sh start bag) so its bag holds the run, not the whole hold
set -u; O="$1"; mkdir -p "$O"; H=/tmp/r20_hash.log
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
: "${GATE_CONFIGS:?set GATE_CONFIGS to the configuration of the run (e.g. yolo26n_npu) or all}"
for _ in $(seq 1 180); do
  n=$(awk '{h[NR]=$2} END{c=0; for(i=NR;i>=1 && h[i]==h[NR];i--) c++; print c+0}' "$H" 2>/dev/null)
  [ "${n:-0}" -ge 20 ] && break; sleep 0.5
done
FINAL=$(tail -1 "$H" | awk '{print $2}')
python3 - "$H" "$O/hold_renders.json" <<'PY'
import json, sys
rows = [l.split() for l in open(sys.argv[1])]; t0 = int(rows[0][0]); seen = {}
for t, h, *_ in rows:
    s = seen.setdefault(h, {"sha1": h, "first_s": round((int(t) - t0) / 1e9, 3), "copies": 0}); s["copies"] += 1
r = list(seen.values()); json.dump(r, open(sys.argv[2], "w"), indent=1)
print(f"hold renders: {len(r)} distinct; " + "; ".join(f"{x['sha1'][:8]} at {x['first_s']} s x{x['copies']}" for x in r))
print(f"final render {r[-1]['sha1'][:8]}: first seen {r[-1]['first_s']} s after the first logged frame, {r[-1]['copies']} copies so far")
PY
echo "$FINAL" > "$O/held_final_sha1.txt"
[ "${n:-0}" -ge 20 ] || { echo "hold_gate: final render reached only ${n:-0} identical copies in 90 s"; exit 1; }
python3 "$HIL_RUNS/held_members.py" accept "$HIL_HELD_REF" "$FINAL" \
  || { echo "hold_gate: FAIL final held render $FINAL is not in the accepted set of $HIL_HELD_REF (17.84)"; exit 1; }
python3 -c "import numpy as np,cv2,sys; a=np.fromfile(sys.argv[1],np.uint8).reshape(480,640,3); cv2.imwrite(sys.argv[2],a)" "/tmp/r20_renders/$FINAL.raw" "$O/held_final_render.png"
GC=(); [ "$GATE_CONFIGS" = all ] || GC=(--configs $GATE_CONFIGS)
python3 "$HIL_RUNS/gate_render.py" "$O/held_final_render.png" "$O/gate.json" "${GC[@]}" 2>"$O/gate.err" || exit 1
bash "$HIL_RUNS/rec.sh" start bag || exit 1
