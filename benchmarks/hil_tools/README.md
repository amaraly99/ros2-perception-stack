# benchmarks/hil_tools (outline, 16A; the full write-up follows the scoring)

Tools around the fx 554 HIL run code of `benchmarks/hil_runs`. This folder is OUTSIDE the HIL_CODE_COMMIT guard (batch.sh
diffs only `benchmarks/hil_runs`, `benchmarks/inloop_accuracy.py` and `integration/`), so nothing here can change a run or a
stored result; every tool reads run folders and re-score trees and writes only into the output folder it is given.

| tool | what | status |
|---|---|---|
| `pool_tables.py` | DRAFT Tables 13 and 14 pooled per configuration from the run folders and a one-scorer re-score tree; self-checks every run against that tree's `_inloop.json` (the `--rescored` argument, not the run folder's `check/`) (DEVIATIONS 17.66 a; MISS_RATE = in_view) | draft tables only; the scoring prompt makes them final |
| `invariance/archive_code.sh`, `invariance/rescore.sh`, `invariance/compare.py` | re-run `check_run.py` of any commit over a batch list into a tree, and compare two trees or a tree with the stored outputs (byte comparison with the out-dir path normalised; verdict and score() key summaries) | the same procedure as the ad hoc invariance runs of DEVIATIONS 17.69 f, 17.71 f and 17.73 g; generalised from the 17.76 e scripts |
| `worktree_inputs.sh` | inventory and comparison of every git-ignored path of two worktrees (count and content hash per path, differing entries) | written after PLAN_S5 A3 missed the `src/ov2slam_ros/Thirdparty` build products (found in 15K with git status --ignored); run on the before-copy manifests it lists those six paths as only in the reference |
| `run_outcomes.py` (16D) | one row per in-loop run from the run folders: the first-motion audit (did the pose follow the first vx and wz commands after resume), the FSM transitions with reasons from the stack log, the reach rule's conditions as the data met them (replayed smoothed box ratio, ex, ey; close-and-centred streak against reach_consec_ticks; the last box before the final loss and what the camera saw next), closest approach, outcome class, staleness terms; self-check `replay_consistent` (the replayed streak reaches 8 exactly for the runs that logged REACHED); 16F columns: minimum altitude after resume and when, ground reached (z <= 0.01 m), yaw deviation from the world x axis while APPROACHING with the lateral share sin(dev), and the ground contact class (DEVIATIONS 17.80, precedence over every other class) | 16D: 157 of 157 runs consistent; definitions in stage6/analysis_16D/EXPECTED.md and analysis_16F/EXPECTED.md; 16F: every 16D column unchanged, 8 runs reclassified |
| `yaw_hold_check.py` (16F) | per run folder: /cmd_vel messages with a non-zero angular.z and linear.y while /bench/state is APPROACHING and while SEARCHING, the yaw range and distinct values of the poses in APPROACHING, the largest lateral share over those poses | 16F: no yaw command in APPROACHING in any sampled run; the heading stays at the start value |
| `altitude_yaw_audit.py` (16F) | the altitude audit (minimum z per run and group, pooled runs below 1.0 m, runs that reached the ground) and the yaw audit (largest and mean |yaw - 360 deg| while APPROACHING, the lateral share, runs above 3 deg or 5 pct) over `per_run_outcomes.tsv` | 16F: no pooled run below 2.94 m; 8 m-model runs at the ground; every pooled approach at 2.64 deg, share 4.6 pct |
| `first_motion_audit.py`, `reach_audit.py`, `outcome_classes.py`, `staleness_fit.py` (16D) | summaries over `per_run_outcomes.tsv`: the audit counts and the failing runs; reach declared against arrived per run and group; outcome classes per configuration and window with the 1.0 m line's band count and the consistency against the stored values and the 16A per_run.tsv; the one-line stale-box fit with residuals by family (exploratory) | 16D results in stage6/analysis_16D/ and REPORT_16D.md |
| `ablation_table.py` (16D) | DRAFT loss-window ablation table per configuration and window from `per_run_outcomes.tsv` (outcome classes, closest approach, final distance, n; rig faults in a footer, not counted) | never pooled into Tables 13 or 14 |
| `fig_style.py`, `fig_t14_displacement.py`, `fig_closest_vs_staleness.py`, `fig_outcome_classes.py` (16D; b and c revised in 16F) | the three draft figures (PNG and PDF, half page width, system python3 with matplotlib): Table 14 displacement by bin NPU against CPU; closest approach against v x staleness (16F: the identity law only, no fitted parameter; ground-contact runs as a separate failure; residuals by family into `<stem>_caption.txt`); outcome classes per configuration at 0.4 and 2.5 s (16F: the ground contact class, fixed palette slot per class) | DRAFT; stage6/draft_tables_16D/figs/ (the 16F versions carry the suffix `_16F` beside the 16D files) |

`pool_tables.py` gained, in 16D, presentation options only (`--outcomes`, `--sensitivity`, `--axis-uncertainty`, `--cache`); with them
off its output is byte-identical to 16A, and with them on every 16A value is unchanged (checked in 16D: 0 differing rows in
Tables 13, 14, the bins and per_run.tsv; only the added lines and columns).

## One command per table (to be completed after the scoring prompt)

    ~/venvs/mcap/bin/python benchmarks/hil_tools/pool_tables.py --batches <batches.tsv> --rescored <re-score tree> --out <dir> \
        [--outcomes <per_run_outcomes.tsv>] [--sensitivity s2:005 s4:024] [--axis-uncertainty "+0.1 to +0.3"] [--cache <dir>]
    ~/venvs/mcap/bin/python benchmarks/hil_tools/run_outcomes.py --batches <batches.tsv> --out <dir> [--rescored <re-score tree>]
    python3 benchmarks/hil_tools/ablation_table.py <per_run_outcomes.tsv> --sess s5a --rig-fault <sess>:<run>:<reason> --out <dir>
    python3 benchmarks/hil_tools/altitude_yaw_audit.py <per_run_outcomes.tsv> --exclude s5a:032 --out <dir>

`<batches.tsv>`: one line per pooled batch, `<session> <folder under /mnt/usb/hil_runs>` (the five of REPORT_SESSIONS_SUMMARY).
`<re-score tree>`: the one-scorer outputs of every pooled run at the scoring commit (made with `invariance/rescore.sh`).

## Which commit produced which data (to be completed after the scoring)

| batch | HEAD at the run | HIL_CODE_COMMIT | stored check outputs scored at | re-scored for the tables at |
|---|---|---|---|---|
| (filled in by the scoring prompt from each batch.log and pin_record.json) | | | | |

## Environment pins (to be completed)

Python for the tools: `~/venvs/mcap/bin/python` (mcap, mcap-ros2-support); the stack image, kernel and HailoRT per the pin
records. Exact versions are written here by the scoring prompt.
