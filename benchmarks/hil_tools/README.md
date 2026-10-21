# benchmarks/hil_tools (16A to 17D): the tools behind the fx 554 HIL tables and figures

Tools around the fx 554 HIL run code of `benchmarks/hil_runs`. This folder is OUTSIDE the HIL_CODE_COMMIT guard (batch.sh
diffs only `benchmarks/hil_runs`, `benchmarks/inloop_accuracy.py` and `integration/`), so nothing here can change a run or a
stored result; every tool reads run folders and re-score trees and writes only into the output folder it is given.

| tool | what | status |
|---|---|---|
| `pool_tables.py` | DRAFT Tables 13 and 14 pooled per configuration from the run folders and a one-scorer re-score tree; self-checks every run against that tree's `_inloop.json` (the `--rescored` argument, not the run folder's `check/`) (DEVIATIONS 17.66 a; MISS_RATE = in_view) | draft tables only; the scoring prompt makes them final |
| `invariance/archive_code.sh`, `invariance/rescore.sh`, `invariance/compare.py` | re-run `check_run.py` of any commit over a batch list into a tree, and compare two trees or a tree with the stored outputs (byte comparison with the out-dir path normalised; verdict and score() key summaries) | the same procedure as the ad hoc invariance runs of DEVIATIONS 17.69 f, 17.71 f and 17.73 g; generalised from the 17.76 e scripts |
| `worktree_inputs.sh` | inventory and comparison of every git-ignored path of two worktrees (count and content hash per path, differing entries) | written after PLAN_S5 A3 missed the `src/ov2slam_ros/Thirdparty` build products (found in 15K with git status --ignored); run on the before-copy manifests it lists those six paths as only in the reference |
| `run_outcomes.py` (16D) | one row per in-loop run from the run folders: the first-motion audit (did the pose follow the first vx and wz commands after resume), the FSM transitions with reasons from the stack log, the reach rule's conditions as the data met them (replayed smoothed box ratio, ex, ey; close-and-centred streak against reach_consec_ticks; the last box before the final loss and what the camera saw next), closest approach, outcome class, staleness terms; self-check `replay_consistent` (the replayed streak reaches 8 exactly for the runs that logged REACHED); 16F columns: minimum altitude after resume and when, ground reached (z <= 0.01 m), yaw deviation from the world x axis while APPROACHING with the lateral share sin(dev), and the ground contact class (DEVIATIONS 17.80, precedence over every other class; 16G: minimum z after resume <= 0.5 m, 17.80 f, Ahmed's decision; the audit columns keep the 0.01 m ground) | 16D: 157 of 157 runs consistent; definitions in stage6/analysis_16D/EXPECTED.md, analysis_16F/EXPECTED.md and analysis_16G/EXPECTED.md; 16F: every 16D column unchanged, 8 runs reclassified; 16G: 9 (run 025 joins), nothing else changes |
| `yaw_hold_check.py` (16F) | per run folder: /cmd_vel messages with a non-zero angular.z and linear.y while /bench/state is APPROACHING and while SEARCHING, the yaw range and distinct values of the poses in APPROACHING, the largest lateral share over those poses | 16F: no yaw command in APPROACHING in any sampled run; the heading stays at the start value |
| `altitude_yaw_audit.py` (16F) | the altitude audit (minimum z per run and group, pooled runs below 1.0 m, runs that reached the ground) and the yaw audit (largest and mean |yaw - 360 deg| while APPROACHING, the lateral share, runs above 3 deg or 5 pct) over `per_run_outcomes.tsv` | 16F: no pooled run below 2.94 m; 8 m-model runs at the ground; every pooled approach at 2.64 deg, share 4.6 pct |
| `first_motion_audit.py`, `reach_audit.py`, `outcome_classes.py`, `staleness_fit.py` (16D) | summaries over `per_run_outcomes.tsv`: the audit counts and the failing runs; reach declared against arrived per run and group; outcome classes per configuration and window with the 1.0 m line's band count and the consistency against the stored values and the 16A per_run.tsv; the one-line stale-box fit with residuals by family (exploratory) | 16D results in stage6/analysis_16D/ and REPORT_16D.md |
| `ablation_table.py` (16D) | DRAFT loss-window ablation table per configuration and window from `per_run_outcomes.tsv` (outcome classes, closest approach, final distance, n; rig faults in a footer, not counted; 16G: `--header-note` for a fact that holds for every row) | never pooled into Tables 13 or 14 |
| `fig_style.py`, `fig_t14_displacement.py`, `fig_closest_vs_staleness.py`, `fig_outcome_classes.py` (16D; b and c revised in 16F) | the three draft figures (PNG and PDF, half page width, system python3 with matplotlib): Table 14 displacement by bin NPU against CPU; closest approach against v x staleness (16F: the identity law only, no fitted parameter; ground-contact runs as a separate failure; residuals by family into `<stem>_caption.txt`); outcome classes per configuration at 0.4 and 2.5 s (16F: the ground contact class, fixed palette slot per class) | DRAFT; stage6/draft_tables_16D/figs/ (the 16F versions carry the suffix `_16F` beside the 16D files) |
| `fig_t14_displacement.py --final`, `fig_outcome_classes.py --final` (17D) | the same two figures without the DRAFT footer (and, for (c), without the DEVIATIONS reference), for the manuscript; both modes print `plotted_sha256`, the sha256 of the arrays drawn, so a final figure is checked against its draft run by hash (17D: identical for both) | final PDFs in the manuscript draft (draft_17D/new/Figures/); the array hashes in stage6/analysis_17D/figs_a/ |
| `cadence_stats.py` (17D) | per configuration: the inter-detection gaps the controller received (consecutive target-box messages after resume: p50, p95, max, counts above 0.4 and 0.6 s, reached runs with a gap above 0.6 s and n_lost 0), the detector latency p50 and p95 from the telemetry, and the exact Clopper-Pearson 95 pct interval of the reach count (scipy); `--ablation` selects the session 5a rows | stage6/analysis_17D/b3/ (pooled_v2/ and ablation_v3/, the rig fault 032 excluded; the earlier pooled/ and ablation_v2/ kept); RESULTS_B3.md |

`pool_tables.py` gained, in 16D, presentation options only (`--outcomes`, `--sensitivity`, `--axis-uncertainty`, `--cache`); with them
off its output is byte-identical to 16A, and with them on every 16A value is unchanged (checked in 16D: 0 differing rows in
Tables 13, 14, the bins and per_run.tsv; only the added lines and columns).

## One command per table and figure (17D; every path relative to the repo root unless absolute)

Inputs: `B` = the pooled batch list (one line per pooled batch, `<session> <folder under /mnt/usb/hil_runs>`: s1 consolidation_fx554_s1_20261009,
s2 consolidation_fx554_s2_20261015_b, s2c consolidation_fx554_s2c_20261015_b, s3 consolidation_fx554_s3_20261015, s4 consolidation_fx554_s4_20261015_b;
`B5` = the same list plus `s5a session5_fx554_20261016/ablation_lw_s5a`, as `stage6/analysis_16G/batches_16G.tsv`); `R` = the one-scorer
re-score tree of every pooled run at the scoring commit (`stage6/prescore_16A/new`, made with `invariance/rescore.sh`); `C` = a COPY of the
read_bag cache (`stage6/draft_tables_16D/cache_from_16A`; a cache miss writes into it, so never point it at an archived folder);
`S6` = `~/reproduction/sim_unify_2026-10-04/stage6`. Every tool refuses to overwrite: give a new `--out` each time.

    # per-run outcomes (first-motion audit, FSM transitions, reach-rule replay, classes, altitude and yaw columns): 157 runs, about 45 s
    ~/venvs/mcap/bin/python benchmarks/hil_tools/run_outcomes.py --batches B5 --out OUT/run_outcomes --rescored R
    # Table 13 (DRAFT_table13.tsv) and Table 14 (DRAFT_table14.tsv, _bins.tsv, _rows.tex), per_run.tsv: 125 runs, about 10 s with the cache
    ~/venvs/mcap/bin/python benchmarks/hil_tools/pool_tables.py --batches B --rescored R --out OUT/tables \
        --outcomes OUT/run_outcomes/per_run_outcomes.tsv --sensitivity s2:005 s4:024 --axis-uncertainty "+0.1 to +0.3" --cache C
    # outcome classes per configuration and window (classes_by_config.tsv), run 032 set aside
    python3 benchmarks/hil_tools/outcome_classes.py OUT/run_outcomes/per_run_outcomes.tsv --exclude s5a:032 --out OUT/outcome_classes
    # the loss-window ablation table (DRAFT_ablation_table.tsv and .md; 16G flags as in S6/analysis_16G)
    python3 benchmarks/hil_tools/ablation_table.py OUT/run_outcomes/per_run_outcomes.tsv --sess s5a \
        --rig-fault "s5a:032:commands published, pose static (DEVIATIONS 17.79)" --header-note "..." --note "s5a:004:..." --out OUT/ablation
    # altitude and yaw audit
    python3 benchmarks/hil_tools/altitude_yaw_audit.py OUT/run_outcomes/per_run_outcomes.tsv --exclude s5a:032 --out OUT/altitude_yaw_audit
    # figure (a), Table 14 displacement by bin; figure (c), outcome classes; --final for the manuscript (prints plotted_sha256)
    python3 benchmarks/hil_tools/fig_t14_displacement.py OUT/tables/DRAFT_table14_bins.tsv OUT/figs/fig_a_t14_displacement_by_bin --final
    python3 benchmarks/hil_tools/fig_outcome_classes.py OUT/outcome_classes/classes_by_config.tsv OUT/figs/fig_c_outcome_classes_16G --final
    # figure (b), kept out of the paper (decision 51): fig_closest_vs_staleness.py S6/analysis_16G/staleness/staleness_points.tsv OUT/figs/fig_b
    # cadence, latency tails and Clopper-Pearson bounds per configuration: the pooled rows from the five-batch list B, the session 5a
    # rows with --ablation (B5 and a re-score tree that holds the 5a caches), the rig fault set aside with --exclude
    python3 benchmarks/hil_tools/cadence_stats.py --batches B --cache C --out OUT/cadence_pooled
    python3 benchmarks/hil_tools/cadence_stats.py --ablation --exclude s5a:032 --batches B5 --cache <re-score tree> --out OUT/cadence_ablation
    # invariance of a check or scorer change: re-score two commits over the same runs and compare
    benchmarks/hil_tools/invariance/archive_code.sh <commit> <dir>; benchmarks/hil_tools/invariance/rescore.sh <root> <tag> <dir>/benchmarks/hil_runs/check_run.py B5
    python3 benchmarks/hil_tools/invariance/compare.py <root> <tagA> <tagB or stored> B5

The 16G outputs that the manuscript tables were pasted from: `S6/draft_tables_16D/tables_16G/` (Table 13, Table 14, bins, per_run),
`S6/draft_tables_16D/ablation_16G/` (the ablation table), `S6/analysis_16G/outcome_classes/`, `S6/draft_tables_16D/figs/` and the final
figures in `~/reproduction/manuscript/draft_17D/new/Figures/`. Run names in a session 5a attempt repeat across attempts under the same batch
basename, so a re-score of attempt 1 needs its own `<root>` (17D).

## SLAM accuracy, Level 2 only (20C): two stages, because no one interpreter has both readers

```
~/venvs/mcap/bin/python benchmarks/hil_tools/slam_traj_export.py <run dir> --out <dir>   # stage 1: bag -> gt.tsv, slam.tsv, tracking_state.tsv
python3 benchmarks/hil_tools/slam_ate.py --selftest                                      # stage 2: the three synthetic checks, all must pass first
python3 benchmarks/hil_tools/slam_ate.py --gt <dir>/gt.tsv --slam <dir>/slam.tsv --label <run>
```

Umeyama alignment in Sim(3) (scale solved, mandatory for monocular) and SE(3), nearest-time association within 0.05 s on the bag log
clock, end pads 0, 3 and 10. `evo` is not installed on Pi-3, so every ATE in a Level 2 table is labelled "our Sim(3) alignment, not
evo" until one of Amar's own runs has been scored both ways side by side. Read-only on stored data; neither tool is run code (both sit
outside the `HIL_CODE_COMMIT` guard) and neither can change any stored run's scoring.

## Which commit produced which data (VERIFIED from each batch.log, 17D)

| batch | runs | HEAD at the run (ours) | HIL_CODE_COMMIT | stored check outputs scored at | re-scored for the tables at |
|---|---|---|---|---|---|
| s1 `consolidation_fx554_s1_20261009` | 32 | 2cb5f60 | df19d71 | the run's HEAD | 325717e (`prescore_16A/new`, one scorer for all 125) |
| s2 `consolidation_fx554_s2_20261015_b` (runs 001 to 028; 028 re-scored under 17.73 in `check_15H/`) | 28 | 86b3436 | 0f433c8 | the run's HEAD (028: 15H) | 325717e |
| s2c `consolidation_fx554_s2c_20261015_b` | 6 | 9ef3353 | a7182ab | the run's HEAD | 325717e |
| s3 `consolidation_fx554_s3_20261015` | 30 | 67175ba | 68f86cb | the run's HEAD | 325717e |
| s4 `consolidation_fx554_s4_20261015_b` | 29 | 67175ba | 68f86cb | the run's HEAD | 325717e |
| 5a attempt 2 `session5_fx554_20261016/ablation_lw_s5a` (ablation, never pooled) | 32 (31 valid) | 13fed9c | 325717e | the run's HEAD | 325717e (`analysis_17D/b2/rescore/head`) |
| 5a attempt 1 `session5_fx554_20261015/ablation_lw_s5a` (superseded: no co-load) | 4 | 35928ee | 42f5e39 | the run's HEAD | 325717e (`analysis_17D/b2/rescore_att1/head`) |

Every run's `pin_record.json` carries `ours_repo_commit`, the pinned stack worktree commit (4490660) and the patch sha256 (d27ca98d for the
125 pooled runs; the session 5a runs ran under the ablation patch 24b84f65, `integration/ablation_lw/ours_on_4490660_lw.patch`), the
Simulink model sha256 (a38a664c), the detector artifact sha256, the stack image id and the sha256 of the four config inputs it ran under
(sim_geometry, held_reference, protocol, runtime SHA256SUMS).

## Environment pins (VERIFIED on Pi-3, 2026-10-17)

Raspberry Pi 5 (8 GB, Pi-3) with the AI HAT+ 2 (Hailo-10H); kernel 6.18.50+rpt-rpi-2712; HailoRT 5.1.1 (h10-hailort, h10-hailort-pcie-driver,
python3-h10-hailort 5.1.1); stack image `ros2_perception_stack:latest` 195667c737d4 (ROS 2 Jazzy, rmw_fastrtps_cpp); run-code Python
`~/venvs/mcap/bin/python` 3.13.5 with mcap 1.4.0, mcap-ros2-support 0.5.7, pillow 12.3.0 (no numpy: the bag readers and check_run.py need none);
tools Python: system python3 3.13.5 with numpy 2.2.4, scipy 1.17.1, matplotlib 3.10.8, OpenCV 4.13.0, onnxruntime 1.24.4 (the in-loop CPU
producer runs natively on the host with this onnxruntime), hailo_platform 5.1.1; TeX Live 2025/dev/Debian (pdfTeX 1.40.26) for the drafts.

## Data layout

`/mnt/usb/hil_runs/<batch>/`: `plan.tsv`, `batch.log`, `batch_state.tsv`, `000_session_gate/`, `ref_start/`, `ref_end/`, and one folder per run
`NNN_<model>_<backend>_<placement>_r<key>/` holding `run.log`, `pin_record.json`, `run_result.json`, the hold gate outputs (`gate.json`,
`held_final_render.png`, `held_final_sha1.txt`, `hold_renders.json`), `t_resume_ns.txt` and the other resume stamps, the pre-resume recorder's
`approach_bag/` (/sim/drone_pose, /yolo/detections, /bench/state, /cmd_vel), `r20_hash.log` and `r20_renders/<sha1>.raw` (every distinct camera
render of the run, bgr8 640x480), `check/` (check_run.py outputs: `_check.json`, `_inloop.json` scorer output and text), `consistency/`, and
`stack/` (the stack run: `<label>_run1_bag/`, `_stack.log`, `_telemetry.csv` per processed frame with the in-loop frame_hash, `_cam_stamps.csv`,
`_mission.json`, `_stats.json`, `_summary.json`, `_params.yaml`, `_clock_offset.json`, `_thermal.csv`, `_topthreads.csv`, `_cores.csv`,
`_pin.json`, `hailort.log`). The drive is mounted read-only for analysis; every tool writes only into its `--out`.

## Code and data availability sentence (manuscript, Conclusion; placeholder URL)

"The run code, the pooling and audit tools, and the per-run records (pin record, validity checks, bags and scorer outputs) of every
closed-loop run reported here are available at <URL> together with the batch-to-commit table that names the commit each batch ran from."
The URL is a placeholder marked `% TODO URL` in the draft until the repository is published (nothing was pushed in 17D).
