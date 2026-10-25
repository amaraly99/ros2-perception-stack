#!/usr/bin/env python3
"""slam_runner.py — the SLAM benchmark, live in a rich terminal.

    ./run_slam_benchmark.sh <slam> <tag> [options]
    ./run_slam_benchmark.sh orbslam2 unified_orb2_mono_N10            # one SLAM
    ./run_slam_benchmark.sh orbslam2,ov2_fast unified_mix_N10         # several, round robin
    ./run_slam_benchmark.sh all unified_all_mono_N10                  # every SLAM of the camera mode
    ./run_slam_benchmark.sh orbslam2 t --dry-run                      # preflight + status only
    ./run_slam_benchmark.sh --list                                    # the SLAM names you can give

<slam> and <tag> may also be given as --arms and --tag (the old form); the command line wins over
config/hil/slam_bench.yaml. Guide: benchmarks/SLAM_BENCHMARK.md
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hilbench.factory import main  # noqa: E402
from hilbench.kinds.slam import arm_names, load_arm  # noqa: E402


def slam_list():
    """'name  modes' lines for every arm file."""
    out = []
    for name in arm_names():
        modes = ", ".join(sorted((load_arm(name).get("stack") or {}).keys()))
        out.append(f"  {name:<14} {modes}")
    return out


def positional_to_options(argv):
    """[<slam>] [<tag>] before the options -> --arms / --tag. 'all' -> every arm of the camera mode.

    Raises SystemExit(2) with the SLAM list when <slam> is not a known name.
    """
    argv = list(argv)
    pos = []
    while argv and not argv[0].startswith("-") and len(pos) < 2:
        pos.append(argv.pop(0))
    if not pos:
        return argv
    known = arm_names()
    slams = pos[0]
    if slams != "all":
        bad = [s for s in slams.split(",") if s not in known]
        if bad:
            sys.stderr.write(f"unknown SLAM: {', '.join(bad)}\nknown SLAMs (name  camera modes):\n" + "\n".join(slam_list()) + "\n")
            raise SystemExit(2)
        argv = ["--arms", slams] + argv           # 'all' = no --arms: every arm that supports camera_mode
    if len(pos) == 2:
        argv = ["--tag", pos[1]] + argv
    return argv


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--list" in args:
        print("SLAMs (name  camera modes):\n" + "\n".join(slam_list()))
        sys.exit(0)
    sys.exit(main("slam", positional_to_options(args)))
