#!/usr/bin/env python3
"""gen_probe.py - write the unified copies of the SLAM trajectory probe.

    python3 scripts/gen_probe.py            # write both copies into matlab/
    python3 scripts/gen_probe.py --check    # verify only, write nothing
    python3 scripts/gen_probe.py --out DIR  # write into DIR instead of matlab/

matlab/slam_traj_probe.m      -> matlab/slam_traj_probe_unified.m
matlab/slam_traj_probe_rgbd.m -> matlab/slam_traj_probe_rgbd_unified.m

Exactly two lines change in each copy:
  MODEL_NAME = '...'                 ->  MODEL_NAME = getenv('HIL_MODEL');
  target = [35.5; 23.7; 3.2];        ->  target = [<SIGN from benchmarks/bench_geometry.py>];
The script checks the diff. If it is not exactly two changed lines, it exits 1 and writes nothing.
The originals are never written.
"""
import argparse
import difflib
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "benchmarks"))
import bench_geometry as geo  # noqa: E402

PAIRS = [("slam_traj_probe.m", "slam_traj_probe_unified.m"),
         ("slam_traj_probe_rgbd.m", "slam_traj_probe_rgbd_unified.m")]
MODEL_RE = re.compile(r"^MODEL_NAME\s*=\s*'[^']*';.*$")
TARGET_RE = re.compile(r"^target\s*=\s*\[35\.5;\s*23\.7;\s*3\.2\];.*$")


def make_copy(text):
    """Return the new text, or raise ValueError when the source does not have the two lines once."""
    sx, sy, sz = geo.SIGN
    new_model = "MODEL_NAME = getenv('HIL_MODEL');"
    new_target = f"target = [{sx:g}; {sy:g}; {sz:g}];"
    lines = text.split("\n")
    out, n_model, n_target = [], 0, 0
    for line in lines:
        if MODEL_RE.match(line):
            out.append(new_model)
            n_model += 1
        elif TARGET_RE.match(line):
            out.append(new_target)
            n_target += 1
        else:
            out.append(line)
    if n_model != 1 or n_target != 1:
        raise ValueError(f"expected one MODEL_NAME line and one target line, found {n_model} and {n_target}")
    return "\n".join(out)


def changed_lines(old, new):
    d = [x for x in difflib.unified_diff(old.split("\n"), new.split("\n"), lineterm="", n=0)
         if x[:1] in "+-" and not x.startswith(("+++", "---"))]
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="verify only, write nothing")
    ap.add_argument("--out", help="output directory (default: matlab/)")
    a = ap.parse_args()
    src_dir = ROOT / "matlab"
    out_dir = Path(a.out) if a.out else src_dir
    results = []
    for src_name, dst_name in PAIRS:
        old = (src_dir / src_name).read_text(encoding="utf-8", errors="surrogateescape")
        try:
            new = make_copy(old)
        except ValueError as e:
            print(f"gen_probe: {src_name}: {e}", file=sys.stderr)
            return 1
        diff = changed_lines(old, new)
        if len(diff) != 4:            # 2 removed + 2 added = exactly 2 changed lines
            print(f"gen_probe: {src_name}: {len(diff) // 2} changed lines, expected exactly 2", file=sys.stderr)
            return 1
        results.append((src_name, dst_name, new, diff))
    for src_name, dst_name, new, diff in results:
        print(f"{src_name} -> {dst_name}: 2 changed lines")
        for d in diff:
            print("   ", d)
    if a.check:
        print("gen_probe: check ok, nothing written")
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)
    for src_name, dst_name, new, _ in results:
        (out_dir / dst_name).write_text(new, encoding="utf-8", errors="surrogateescape")
        print(f"wrote {out_dir / dst_name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
