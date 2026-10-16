# benchmarks/hil_tools (outline, 16A; the full write-up follows the scoring)

Tools around the fx 554 HIL run code of `benchmarks/hil_runs`. This folder is OUTSIDE the HIL_CODE_COMMIT guard (batch.sh
diffs only `benchmarks/hil_runs`, `benchmarks/inloop_accuracy.py` and `integration/`), so nothing here can change a run or a
stored result; every tool reads run folders and re-score trees and writes only into the output folder it is given.

| tool | what | status |
|---|---|---|
| `pool_tables.py` | DRAFT Tables 13 and 14 pooled per configuration from the run folders and a one-scorer re-score tree; self-checks every run against that tree's `_inloop.json` (the `--rescored` argument, not the run folder's `check/`) (DEVIATIONS 17.66 a; MISS_RATE = in_view) | draft tables only; the scoring prompt makes them final |
| `invariance/archive_code.sh`, `invariance/rescore.sh`, `invariance/compare.py` | re-run `check_run.py` of any commit over a batch list into a tree, and compare two trees or a tree with the stored outputs (byte comparison with the out-dir path normalised; verdict and score() key summaries) | the same procedure as the ad hoc invariance runs of DEVIATIONS 17.69 f, 17.71 f and 17.73 g; generalised from the 17.76 e scripts |
| `worktree_inputs.sh` | inventory and comparison of every git-ignored path of two worktrees (count and content hash per path, differing entries) | written after PLAN_S5 A3 missed the `src/ov2slam_ros/Thirdparty` build products (found in 15K with git status --ignored); run on the before-copy manifests it lists those six paths as only in the reference |

## One command per table (to be completed after the scoring prompt)

    ~/venvs/mcap/bin/python benchmarks/hil_tools/pool_tables.py --batches <batches.tsv> --rescored <re-score tree> --out <dir>

`<batches.tsv>`: one line per pooled batch, `<session> <folder under /mnt/usb/hil_runs>` (the five of REPORT_SESSIONS_SUMMARY).
`<re-score tree>`: the one-scorer outputs of every pooled run at the scoring commit (made with `invariance/rescore.sh`).

## Which commit produced which data (to be completed after the scoring)

| batch | HEAD at the run | HIL_CODE_COMMIT | stored check outputs scored at | re-scored for the tables at |
|---|---|---|---|---|
| (filled in by the scoring prompt from each batch.log and pin_record.json) | | | | |

## Environment pins (to be completed)

Python for the tools: `~/venvs/mcap/bin/python` (mcap, mcap-ros2-support); the stack image, kernel and HailoRT per the pin
records. Exact versions are written here by the scoring prompt.
