#!/usr/bin/env python3
"""sim_setup.py: the simulator setup check (prompt 25C, ruling R4, DEVIATIONS 17.86). Exactly two setups are accepted:

  frozen  the setup of every reported run before 25C (model a38a664c). Its check is the SIM_MODEL_SHA256 comparison in
          batch.sh and smoke_run_hold.sh, unchanged; this file plays no part in it.
  sim25w  25W's wrapper on the sim host (branch sim-25w, MODEL_PIN.md), mode mono or stereo. Its check is the supervisor's
          model_info reply: the same keys as the reply pinned for that mode in config/sim_setups.json, every value equal as
          text, no key missing and no other key. No tolerance, and no setup or mode outside the file.

The model is not identified by a hash here because every .slx save writes a new one (Amar's benchmarks/hilbench/model_check.py:1-5 at 28114bd); the reply
carries the parameters MATLAB has loaded (hil_run_supervisor.m model_info_line). check_files of model_check.py, which
compares the Pi's own repository copies of the .slx files, is not part of this run path and is not touched.

usage:
  sim_setup.py check   <setup> <mode>             ask model_info and compare; exit 0 match, 1 mismatch or no reply, 2 usage
  sim_setup.py record  <setup> <mode> <out.json>  check, then write setup, mode, the reply, the verdict and MODEL_PIN.md checksums
  sim_setup.py compare <setup> <mode> <reply>     compare a given reply line (dry runs; no network)
"""
import json, os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.environ.get("HIL_SIM_SETUPS") or os.path.join(HERE, "config", "sim_setups.json")


def parse(line):
    """'model_info k=v k=v ...' -> {k: v} with the values kept as text; None if it is not a model_info line."""
    f = (line or "").strip().split()
    if not f or f[0] != "model_info":
        return None
    out = {}
    for tok in f[1:]:
        k, sep, v = tok.partition("=")
        if not sep or not k or k in out:
            return None
        out[k] = v
    return out


def pinned(setup, mode):
    j = json.load(open(CFG))
    s = (j.get("setups") or {}).get(setup)
    if s is None:
        raise KeyError(f"setup {setup!r} is not in {CFG} (known: {', '.join(j.get('setups') or {}) or 'none'})")
    if not s.get("model_pin_md"):
        raise KeyError(f"setup {setup!r} has no MODEL_PIN.md record in {CFG}")
    m = (s.get("modes") or {}).get(mode)
    if m is None:
        raise KeyError(f"mode {mode!r} is not pinned for setup {setup!r} (pinned: {', '.join(s.get('modes') or {})})")
    want = parse(m["model_info"])
    if not want:
        raise KeyError(f"the pinned reply of {setup}/{mode} is not a model_info line")
    return s, want


def compare(setup, mode, reply):
    """[] when the reply is the pinned one of setup/mode, else one line per difference."""
    _s, want = pinned(setup, mode)
    got = parse(reply)
    if got is None:
        return [f"not a model_info reply: {reply!r}"]
    bad = [f"{k}: the Pi expects {v}, MATLAB has {got[k] if k in got else 'nothing'}" for k, v in want.items() if got.get(k) != v]
    bad += [f"{k}: MATLAB reports {got[k]}, a key the pinned reply does not have" for k in got if k not in want]
    return bad


def ask():
    p = subprocess.run([os.path.join(HERE, "host", "hil_ctl.sh"), "model_info"], capture_output=True, text=True)
    return p.stdout.strip()


def main(a):
    if len(a) < 3 or a[0] not in ("check", "record", "compare") or (a[0] == "record" and len(a) != 4) \
            or (a[0] == "compare" and len(a) != 4):
        print(__doc__.split("usage:")[1].strip()); return 2
    act, setup, mode = a[:3]
    if setup == "frozen":
        print("sim setup frozen: checked by SIM_MODEL_SHA256 in batch.sh and smoke_run_hold.sh, not here"); return 2
    try:
        s, _want = pinned(setup, mode)
    except (KeyError, OSError, ValueError) as e:
        print(f"sim setup REFUSED: {e}"); return 1
    reply = a[3] if act == "compare" else ask()
    bad = compare(setup, mode, reply)
    verdict = "match" if not bad else "MISMATCH"
    print(f"sim setup {setup}/{mode}: model_info {verdict}" + ("" if not bad else ": " + "; ".join(bad)))
    if act == "record":
        rec = {"setup": setup, "mode": mode, "asked_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "model_info_reply": reply, "verdict": verdict, "differences": bad,
               "pinned_model_info": s["modes"][mode]["model_info"], "model_pin_md": s.get("model_pin_md"),
               "sim_setups_file": CFG}
        with open(a[3], "x") as f:
            json.dump(rec, f, indent=1)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
