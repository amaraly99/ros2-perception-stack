# hil_env.sh: sourced by every shell script in benchmarks/hil_runs. One place for paths and pins.
# Every value can be overridden by exporting the variable before the call. Python reads the same names (hilpaths.py).
# Paths inside the repo are derived from this file's location, so the folder works from any checkout.
HIL_RUNS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"                    # benchmarks/hil_runs
: "${HIL_OURS:=$(cd "$HIL_RUNS/../.." && pwd)}"                              # our repo root (exp/detector-accuracy)
: "${HIL_WT:=$HOME/amar_4490660}"                                            # pinned stack worktree, read only
: "${HIL_AMAR_PIN:=449066089465aa713382e13c1d4f043f29be96ab}"                # its HEAD
: "${HIL_PATCH:=$HIL_OURS/integration/ours_on_4490660.patch}"                # our stack on that HEAD, committed (5R: + ours_fx554_B_ibvs.yaml)
: "${HIL_PATCH_SHA:=245f7a2e44bbf1856355eec213a0b49cd0382976a47a76057b2ebcf396945289}"   # 2026-10-04b (e337034b) + ours_fx554_B_ibvs.yaml (5R) + 20C: the Level 2 sessions 1-3 configs
                                                                             # and the tracking-state bag line; 21E: + the cell C1 config ours_fx554_A_proportional_mono_accurate.yaml
                                                                             # 25C: + ours_fx554_{A,B}_ibvs_slamoff.yaml and ours_fx554_B_ibvs_mono_accurate.yaml
                                                                             # 27A: + the RGB-D source-stamp switch (camera.rgbd: launch file, parse_stack.py,
                                                                             # run_stack_hil.sh) and the ORB-SLAM3 sidecar log in the runner's save_logs;
                                                                             # 74f65d74 before; deployed to HIL_WT in 27D (step 3)
                                                                             # 27D: + ours_fx554_{A,B}_ibvs_rgbd_orb3.yaml and Amar's flown HIL_SIM_RGBD.yaml (1beef72: Stereo.b
                                                                             # 0.61, ThDepth 72.1); df967e68 before. HIL_WT has the settings file (27D step 4); the two configs come
                                                                             # from stage6/rgbd_27D/step5/deploy_27D_two_configs.patch
: "${HIL_CODE_COMMIT:=20813a021858474b81ae89aad3f487fec45c394c}"            # commit of the run code; batch.sh refuses a HEAD whose
                                                                             # hil_runs, scorer or integration/ differ from it
: "${HIL_GEOM:=$HIL_RUNS/config/sim_geometry.json}"                         # sim geometry, read by every check
: "${HIL_HELD_REF:=$HIL_RUNS/config/held_reference.json}"                   # the fixed held render (5Q)
: "${HIL_RUNTIME:=$HIL_RUNS/runtime}"                                        # recorder inputs, checked against SHA256SUMS
: "${HIL_HOST:=$HIL_RUNS/host}"                                              # supervisor client, telemetry, watchers
: "${HIL_POWER_DIR:=$HOME/reproduction/sim_unify_2026-10-04/stage6/power}"    # 8A: power_cycled_<YYYYMMDD_HHMM> markers (power_marker.sh)
: "${HIL_MCAP_PY:=$HOME/venvs/mcap/bin/python}"                              # python with mcap_ros2 (check_run, ic_guard)
: "${HIL_CTL_HOST:=192.168.56.1}"; : "${HIL_CTL_PORT:=55556}"                # MATLAB supervisor
: "${HIL_SIM_SETUP:=frozen}"                                                 # 25C R4: frozen (model hash, every run before 25C) or sim25w
: "${HIL_SIM_MODE:=}"                                                       # 25C R4: mono or stereo for sim25w (sim_setup.py); 27D: or rgbd
: "${HIL_SIM_SETUPS:=$HIL_RUNS/config/sim_setups.json}"                     # 25C R4: the pinned model_info replies
export HIL_RUNS HIL_OURS HIL_WT HIL_AMAR_PIN HIL_PATCH HIL_PATCH_SHA HIL_CODE_COMMIT HIL_GEOM HIL_HELD_REF HIL_RUNTIME HIL_HOST HIL_POWER_DIR HIL_MCAP_PY \
       HIL_CTL_HOST HIL_CTL_PORT HIL_SIM_SETUP HIL_SIM_MODE HIL_SIM_SETUPS   # the fixed hold time lives in config/held_reference.json only
export PYTHONDONTWRITEBYTECODE=1                                              # no __pycache__ written by a run
hil() { "$HIL_HOST/hil_ctl.sh" "$@"; }
