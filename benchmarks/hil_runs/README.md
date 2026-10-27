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
| `hold_once.sh` | one held start without the stack; with `--gate <configs>` the once-per-session gate (batch.sh: the 11 CPU configurations, no NPU, 5R) | `stage6/holds_5J/hold_once.sh` |
| `power_marker.sh` | run by Ahmed after a full power removal: the marker batch.sh needs and consumes (NPU open budget per power-on, 8A) | new |
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

## 25C: simulator setups, the expected SLAM process, and one flight frame per run (DEVIATIONS 17.86)

- **Simulator setup (ruling R4).** `HIL_SIM_SETUP` (hil_env.sh, default `frozen`) names the setup. `frozen` is the setup of
  every run before 25C and keeps its check exactly: the exported `SIM_MODEL_SHA256` must equal the model hash in
  `config/sim_geometry.json` and `config/held_reference.json`. `sim25w` (25W's wrapper on the sim host, `HIL_SIM_MODE` mono
  or stereo) is checked by `sim_setup.py`: the supervisor's `model_info` reply must have exactly the keys and values of the
  reply pinned for that mode in `config/sim_setups.json`, before the session gate and before every run (batch.sh), and at
  the stack run (smoke_run_hold.sh, exit 14). Each run records its setup, mode and reply in `sim_setup.json` and in
  `pin_record.json` (`sim_setup`). No other setup or mode is accepted. A sim25w batch also refuses an exported
  `SIM_MODEL_SHA256`, any `HIL_SIM_SETUPS` but the committed file, and a plan line whose stack config's camera.stereo does
  not match `HIL_SIM_MODE`.
- **SLAM co-load check (ruling R3).** `check_run.py` `slam_coload_alive` reads the cell's expected SLAM process from the
  run's `run_config.yaml`: none for `slam.enabled false` (absence required: the stack log, every topthreads sample up to
  and including the first one after the mission window, every procmem row), ov2slam_node otherwise (the 17.83 rule, plus,
  where `L2_PROCMEM=1` wrote procmem.csv, one pid without a gap from its first row in the mission window to the window's
  end, that first row at most 10 s after resume, and the first row after the window with the same pid). A SLAM-off run
  also needs procmem rows past the window's end; a sim25w run without procmem.csv fails. The stored runs keep their
  verdicts. A sim25w batch requires L2_PROCMEM=1.
- **No flight images but one (ruling R5; Ahmed, 2026-10-25).** With `REC_HELD_ONLY=1` the render logger saves no new
  render once hold_gate.sh has read the final held render (marker `/tmp/r20_hold_done`, moved into the run folder), and
  saves the last frame at exit (`terminal_<sha1>.raw`, `terminal.txt`), the one flight frame kept, for the terminal-state
  figure. batch.sh requires it for a `sim25w` batch. Neither bag records an image topic in any mode.
- **Configs.** `ours_fx554_{A,B}_ibvs_slamoff` (SLAM off) and `ours_fx554_B_ibvs_mono_accurate` (mono CPU) in the
  integration patch; the PSS fix of 25A in `host/level2_procmem.py` (sudo -n from one_run_hold_gated.sh).

## 27A: the RGB-D cell's source stamp and expected SLAM process (DEVIATIONS 17.87)

- **Source stamp (integration patch).** `rgbd_node_cpp` pairs colour and depth only on an identical stamp, and the depth
  image keeps MATLAB's stamp (INFERRED from Amar's RGB-D publisher, one clock read for both messages; 27B's stream check
  tests it), so the left chain must keep it too. A stack config with `camera.rgbd: true` makes `parse_stack.py` emit
  `RGBD_ON=true`, `run_stack_hil.sh` append `rgbd:=true` to the launch arguments, and `hil_simulation.launch.py` set
  `use_source_stamp` true on both left nodes (what Amar's `hil_simulation_rgbd.launch.py` does). Without the key nothing
  changes: the launch command and every node parameter are those of 25C.
- **SLAM co-load check (R3 extended).** An ORB-SLAM3 RGB-D run (`slam.type orbslam3`, `rgbd_node_cpp` in `slam.command`)
  expects the ORB-SLAM3 RGB-D node under the OV2SLAM rule: its threads by name in topthreads (`orb_*`, `rgbd_node_cpp`),
  its procmem row `orbslam3_node`, and the stack log must carry run_stack_hil.sh's line "RGB-D: left image keeps the
  MATLAB source stamp" (without camera.rgbd no frame pairs). The runner's `save_logs` (integration patch) appends the
  ORB-SLAM3 container's console (`/tmp/slam_sidecar_slam_orbslam3.log`, so the container must be named `slam_orbslam3`)
  to the stack log when it exists, as it does the OV2SLAM one, so the library-error test covers it. smoke_run_hold.sh
  waits for and records that node for `slam.type orbslam3`. Other ORB-SLAM3 commands still have no expected process.
  The stored runs keep their verdicts and outputs.
- **Not yet in the run code:** the RGB-D configs, an `rgbd` mode for `sim25w` in `config/sim_setups.json`, and the
  batch.sh plan-line check, which today accepts only mono and stereo sessions (stage6/rgbd_27A/PLAN_27B.md).
