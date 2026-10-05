# benchmarks/hil_runs

Run code for the fx 554 HIL runs (Tables 13 and 14): fixed hold, pre-resume gate, stack, resume, teardown, checks.
Every table run is made from this folder at a committed, clean HEAD; the run's pin record names that commit.

The stack itself is Amar's commit 4490660 with our patch (`integration/ours_on_4490660.patch`, sha256 d27ca98d...:
the 2026-10-04b patch, e337034b..., plus `config/hil/stack/ours_fx554_B_ibvs.yaml` for CPU runs) in the worktree `HIL_WT`
(default `~/amar_4490660`). NPU runs use `ours_fx554_A_ibvs` (placement A), CPU runs `ours_fx554_B_ibvs` (placement B); the
stack config alone sets the backend, and `run_one.sh` refuses a run whose config does not match its backend. Nothing here writes to that worktree: the recorder mounts it read-only,
and the per-run tracked check reads it through a temporary index.

## Use

    export SIM_MODEL_SHA256=<sha256 printed by apply_sim_params.m for the running model>
    bash benchmarks/hil_runs/batch.sh benchmarks/hil_runs/plans/pilot_5Q.tsv ~/reproduction/hil_runs/pilot_<date>

One run by hand: `RUN_DIR=... RUN_KEY=... MODEL=yolo26n BACKEND=npu bash benchmarks/hil_runs/run_one.sh`.
Outputs never go into the repo. Every batch and every run writes into a new folder and refuses an existing one.

## Files

| file | what | from (before 5Q) |
|---|---|---|
| `hil_env.sh`, `hilpaths.py` | paths and pins, overridable by environment | new |
| `batch.sh` | runs a plan unattended; checks between runs; stops at the first failed check, never retries | new |
| `run_one.sh` | one run: recorders, sidecar, held start, gate, stack, check_run, pin record | `stage6/r20.sh` |
| `smoke_run_hold.sh` | throttle and tracked-state guards, stack run, pin of the stack | `integration_3F/smoke_run_hold.sh` |
| `one_run_hold_gated.sh` | supervisor hold, pre-run IC guard, hold-time check, gate hook, resume watcher, runner | `stage6/one_run_hold_gated.sh` |
| `runner_adapter_hold.sh` | maps the old runner flags to the worktree runner | `integration_3F/diag/runner_adapter_hold.sh` |
| `hold_gate.sh` | while held: render settles, is the reference render, own-config gate, approach recorder up | `stage6/hold_gate.sh` |
| `hold_once.sh` | one held start without the stack; with `--gate all` the once-per-session gate | `stage6/holds_5J/hold_once.sh` |
| `rec.sh` | render-hash logger and approach recorder, each in its own container | `stage6/rec.sh` |
| `sidecar_stoprule.sh` | detection logger in the stack container, 10 s stop rule | `stage6/sidecar_stoprule.sh` |
| `gate_render.py`, `one_image_npu.py`, `inloop_pixels.py` | in-loop pixel gate on the held render | `stage6/`, `stage4/`, `stage6/tools_5M/` |
| `check_run.py` | per-run plumbing checks and the in-loop scorer on the approach bag | `integration_3F/check_run.py` |
| `ic_guard.py`, `bearing_from_status.py` | start bearing checks | `~/reproduction/` |
| `host/` | supervisor client, telemetry, core sampler, frozen-camera watcher | `~/bin/` |
| `runtime/` | recorder inputs, checked against `SHA256SUMS` before use | `stage6/runtime/` |
| `config/` | sim geometry, held reference render, run protocol | `sim_unify_2026-10-04/sim_geometry.json`, new |
| `fixtures/` | lossless PNGs of the ef4a539b self-test render and the 58713f67 held reference | `run_5I`, `holds_5Q/hold1` |
| `tools/` | offline range tool, NV12 round-trip reference, scorer consistency (approach vs stack bag) | `stage5/`, `stage6/parity_5J/`, new |
| `plans/` | batch plans (tsv) | new |

`MANIFEST_sources.txt` (outside the repo, in `stage6/repo_5Q/`) lists the sha256 of every source file copied here.
