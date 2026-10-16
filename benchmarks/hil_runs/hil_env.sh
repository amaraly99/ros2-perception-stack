# hil_env.sh: sourced by every shell script in benchmarks/hil_runs. One place for paths and pins.
# Every value can be overridden by exporting the variable before the call. Python reads the same names (hilpaths.py).
# Paths inside the repo are derived from this file's location, so the folder works from any checkout.
HIL_RUNS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"                    # benchmarks/hil_runs
: "${HIL_OURS:=$(cd "$HIL_RUNS/../.." && pwd)}"                              # our repo root (exp/detector-accuracy)
: "${HIL_WT:=$HOME/amar_4490660}"                                            # pinned stack worktree, read only
: "${HIL_AMAR_PIN:=449066089465aa713382e13c1d4f043f29be96ab}"                # its HEAD
: "${HIL_PATCH:=$HIL_OURS/integration/ours_on_4490660.patch}"                # our stack on that HEAD, committed (5R: + ours_fx554_B_ibvs.yaml)
: "${HIL_PATCH_SHA:=d27ca98d75ff564b589653ef5b7db2a37e3789071e5e8681cb67fdb6d9262ea3}"   # 2026-10-04b (e337034b) + that one file
: "${HIL_CODE_COMMIT:=325717e36389868985ce6ad1cca64fdb3b8d397a}"            # commit of the run code; batch.sh refuses a HEAD whose
                                                                             # hil_runs, scorer or integration/ differ from it
: "${HIL_GEOM:=$HIL_RUNS/config/sim_geometry.json}"                         # sim geometry, read by every check
: "${HIL_HELD_REF:=$HIL_RUNS/config/held_reference.json}"                   # the fixed held render (5Q)
: "${HIL_RUNTIME:=$HIL_RUNS/runtime}"                                        # recorder inputs, checked against SHA256SUMS
: "${HIL_HOST:=$HIL_RUNS/host}"                                              # supervisor client, telemetry, watchers
: "${HIL_POWER_DIR:=$HOME/reproduction/sim_unify_2026-10-04/stage6/power}"    # 8A: power_cycled_<YYYYMMDD_HHMM> markers (power_marker.sh)
: "${HIL_MCAP_PY:=$HOME/venvs/mcap/bin/python}"                              # python with mcap_ros2 (check_run, ic_guard)
: "${HIL_CTL_HOST:=192.168.56.1}"; : "${HIL_CTL_PORT:=55556}"                # MATLAB supervisor
export HIL_RUNS HIL_OURS HIL_WT HIL_AMAR_PIN HIL_PATCH HIL_PATCH_SHA HIL_CODE_COMMIT HIL_GEOM HIL_HELD_REF HIL_RUNTIME HIL_HOST HIL_POWER_DIR HIL_MCAP_PY \
       HIL_CTL_HOST HIL_CTL_PORT          # the fixed hold time lives in config/held_reference.json only
export PYTHONDONTWRITEBYTECODE=1                                              # no __pycache__ written by a run
hil() { "$HIL_HOST/hil_ctl.sh" "$@"; }
