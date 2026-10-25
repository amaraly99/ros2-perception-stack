# paper_data/verso — aggregated results behind the paper tables

**"Verso"** is the codename of one SLAM benchmark campaign: the hardware-in-the-loop (HIL) runs flown
between **2026-10-18 and 2026-10-25**. Every arm flies the same scripted trajectory
(`matlab/slam_traj_probe_unified.m`, RGB-D: `slam_traj_probe_rgbd_unified.m`): start at (15.12, 3.84, 7.5) m,
a sideways initialisation leg, a helical cruise with ±8° yaw and ±3° pitch towards the sign, then the
approach. The name comes from the VERSO benchmark, whose reported values were the accuracy targets.
Simulator: MATLAB/Simulink + Unreal on the Windows PC; SLAM on a Raspberry Pi 5 over Wi-Fi.

## Files

| File | Content |
|---|---|
| `table_slam_accuracy.csv` | Per arm: healthy runs (of 10), ATE RMSE mean / SD / median and path ratio, with scale (Sim(3)) and without scale (SE(3)), scale drift |
| `table_slam_profiling.csv` | Per arm: pose rate, max front-end frame rate, front-end latency p50/p95/p99, CPU (% of one core, 4 cores pooled), peak memory (PSS) and how many runs recorded it |
| `table_controller_unified.csv` | IBVS, homography-based VS and proportional controller, unified setup, runs of 2026-10-17/18 (not part of Verso) |
| `runs_rescored.csv` | One row per healthy run (110 rows): every value the two SLAM tables are computed from |
| `run_dates.csv` | First and last flight date per arm |

## How the numbers were made

1. Each run is scored on the window from the first forward motion after the initialisation leg to 3 s after
   the drone stops at the sign. A run is **healthy** when SLAM initialises, its poses cover ≥ 90 % of that
   window, and its ATE is ≤ 1.0 m.
2. `scripts/verso_rescore.py` re-scores the 10 healthy runs of each arm from their bags, once with and once
   without scale; its Sim(3) ATE matches the original scorer within 0.001 m on all 110 runs.
3. `scripts/export_verso_tables.py` aggregates the per-run values into the table CSVs.

## Notes

- ORB-SLAM2 and ORB-SLAM3 **mono** rows use the mono fix (`scripts/orb_mono_fix.patch`, built with
  `scripts/build_orb_mono_fix.sh`) and ran on 2026-10-25 with a 16 Hz camera stream; the other mono rows
  ran at 19.8 Hz.
- OV²SLAM-F mono never initialised on this flight (0 of 40 attempts).
- Memory was not recorded for stereo runs flown before 2026-10-19 23:00; `mem_runs_recorded` gives the count.
- Mono without scale has no meaning (a monocular trajectory has no metric scale); it is listed for completeness.
