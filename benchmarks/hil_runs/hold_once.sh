#!/usr/bin/env bash
# hold_once.sh <outdir> [--gate all|<config> ...]: 5J Part B, one held start at the current start pose, no stack, no resume.
# render logger up -> pub_start 20 -> start_hold -> poll for state=held -> wait for >= 20 identical copies of the newest
# render (max 90 s) plus 5 s more -> save it -> stop, pub_stop -> logger down -> move the logger files into <outdir>.
# Refuses an existing <outdir> (5J rule). Times are wall clock (UTC ns) from this host; "sim start" = the start_hold command.
# 5Q: also the once-per-session gate of batch.sh. Checks, after the hold is released: the supervisor's t= at state=held is the
# fixed hold time and the final render is in the accepted set (held_members.py, 17.84); with --gate, gate_render.py on the final
# render (in-loop pixels; "all" = the 22 configurations, 11 NPU opens). hold_summary.json "final" is now the last logged render
# (it was the last render seen for the first time; the same in every 5J and 5Q hold).
# exit: 0 ok, 20 outdir exists, 21 logger refused, 22 never held, 23 wrong hold time, 24 not the reference render, 25 gate failed
set -u; set -o noclobber
O="$1"; shift; GATE=""; [ "${1:-}" = --gate ] && { shift; GATE="$*"; }
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
[ -e "$O" ] && { echo "hold: REFUSE: $O exists"; exit 20; }
mkdir -p "$(dirname "$O")"; mkdir "$O" || exit 20
cd "$O" || exit 20                                                       # HailoRT writes hailort.log into the cwd
ts() { date +%s%N; }
log() { echo "$(ts) $*" | tee -a "$O/hold.log"; }
log "status_pre: $(hil status)"
log "throttled: $(vcgencmd get_throttled) $(vcgencmd measure_temp)"
bash "$HIL_RUNS/rec.sh" start hash | tee -a "$O/hold.log"; [ "${PIPESTATUS[0]}" -eq 0 ] || { log "logger refused"; exit 21; }
sleep 4
log "pub_start: $(hil pub_start 20)"
T_SH=$(ts); r=$(hil start_hold); echo "$T_SH start_hold: $r" | tee -a "$O/hold.log"; echo "$T_SH" > "$O/t_start_hold_ns.txt"
HELD=""
for i in $(seq 1 90); do st=$(hil status); case "$st" in *state=held*) HELD="$st"; break ;; esac; sleep 1; done
log "held_status (${i} s): ${HELD:-NEVER HELD, last: $(hil status)}"
H=/tmp/r20_hash.log; n=0
for _ in $(seq 1 180); do
  n=$(awk '{h[NR]=$2} END{c=0; for(i=NR;i>=1 && h[i]==h[NR];i--) c++; print c+0}' "$H" 2>/dev/null)
  [ "${n:-0}" -ge 20 ] && break; sleep 0.5
done
sleep 5
FINAL=$(tail -1 "$H" | awk '{print $2}')
log "final render $FINAL, identical copies at check: $n"
cp "/tmp/r20_renders/$FINAL.raw" "$O/final_$FINAL.raw"
python3 -c "import numpy as np,cv2,sys; a=np.fromfile(sys.argv[1],np.uint8).reshape(480,640,3); cv2.imwrite(sys.argv[2],a)" "$O/final_$FINAL.raw" "$O/final.png"
log "stop: $(hil stop)"; log "pub_stop: $(hil pub_stop)"; sleep 2; log "status_after: $(hil status)"
bash "$HIL_RUNS/rec.sh" stop hash | tee -a "$O/hold.log"
sudo mv /tmp/r20_hash.log /tmp/r20_hash.err /tmp/r20_renders "$O/"
sudo chown -R "$(id -u):$(id -g)" "$O"
python3 - "$O" "$FINAL" <<'PY' | tee -a "$O/hold.log"
import json, sys
O, final = sys.argv[1], sys.argv[2]; t_sh = int(open(f"{O}/t_start_hold_ns.txt").read())
rows = [l.split() for l in open(f"{O}/r20_hash.log")]
seen = {}
for t, h, *_ in rows:
    s = seen.setdefault(h, {"sha1": h, "first_s_after_start_hold": round((int(t) - t_sh) / 1e9, 3), "copies": 0}); s["copies"] += 1
r = list(seen.values()); t0 = int(rows[0][0])
out = {"t_start_hold_ns": t_sh, "first_frame_s_after_start_hold": round((t0 - t_sh) / 1e9, 3), "n_messages": len(rows),
       "distinct": r, "final": final}
json.dump(out, open(f"{O}/hold_summary.json", "x"), indent=1)
print(f"hold summary: first frame {out['first_frame_s_after_start_hold']} s after start_hold; {len(r)} distinct: " +
      "; ".join(f"{x['sha1'][:8]} at {x['first_s_after_start_hold']} s x{x['copies']}" for x in r))
PY
[ -n "$HELD" ] || { log "FAIL never reached state=held"; exit 22; }
WANT_T=$(python3 -c "import json,sys; print(f\"{float(json.load(open(sys.argv[1]))['hold_sim_t']):.2f}\")" "$HIL_HELD_REF")
GOT_T=$(echo "$HELD" | grep -oP '\bt=\K[0-9.]+')
[ "${GOT_T:-x}" = "$WANT_T" ] || { log "FAIL hold time t=${GOT_T:-none}, expected $WANT_T"; exit 23; }
python3 "$HIL_RUNS/held_members.py" accept "$HIL_HELD_REF" "$FINAL" | tee -a "$O/hold.log"
[ "${PIPESTATUS[0]}" -eq 0 ] || { log "FAIL final render $FINAL is not in the accepted set (17.84)"; exit 24; }
log "hold ok: t=$GOT_T, final render ${FINAL:0:8} in the accepted set"
if [ -n "$GATE" ]; then
  GC=(); [ "$GATE" = all ] || GC=(--configs $GATE)
  python3 "$HIL_RUNS/gate_render.py" "$O/final.png" "$O/gate.json" "${GC[@]}" > "$O/gate.out" 2>"$O/gate.err"; GRC=$?
  tail -1 "$O/gate.out" | tee -a "$O/hold.log"
  [ "$GRC" -eq 0 ] || { log "FAIL gate rc=$GRC"; exit 25; }
fi
exit 0
