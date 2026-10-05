#!/usr/bin/env bash
# runner_adapter_hold.sh — L2_RUNNER for one_run_hold.sh (used unmodified). Translates the old runner's flags
# into the 4490660 worktree runner. MATLAB control stays with one_run_hold.sh (--no-matlab-ctl is passed by it).
# Ignored on purpose: --detector --backend --placement --detector-core --slam-mode (the stack config sets them).
M=""; LBL=""; OUT=""; W=35; DUR=60; NOCTL=""
while [ $# -gt 0 ]; do
  case "$1" in
    --model) M="$2"; shift 2 ;; --label) LBL="$2"; shift 2 ;; --output-dir) OUT="$2"; shift 2 ;;
    --warmup) W="$2"; shift 2 ;; --duration) DUR="$2"; shift 2 ;; --no-matlab-ctl) NOCTL=--no-matlab-ctl; shift ;;
    --runs|--detector|--backend|--placement|--detector-core|--slam-mode) shift 2 ;;
    *) shift ;;
  esac
done
exec "${HIL_WT:?source hil_env.sh}/benchmarks/run_hil_experiments.sh" --stack-config "${ADAPTER_STACK_CONFIG:?}" \
  --model "$M" --duration "$DUR" --warmup "$W" --runs 1 --label "$LBL" --output-dir "${OUT%/}" $NOCTL
