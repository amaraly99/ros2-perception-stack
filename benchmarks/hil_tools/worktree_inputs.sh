#!/usr/bin/env bash
# worktree_inputs.sh <reference worktree> <worktree> <out dir>: inventory of every git-ignored top-level path of both worktrees
# (git status --ignored; one manifest per path: "relpath<TAB>sha256|link->target", sorted; file count and a content hash per
# path) and the comparison: SAME, DIFF (with the differing entries), or present on one side only. Read-only on both worktrees;
# root-owned trees are read with sudo. Written after PLAN_S5 A3 copied four ignored paths by hand and missed
# src/ov2slam_ros/Thirdparty (16A step 3). The caller judges the differences (an allow-list such as a rebuilt package or run
# outputs); nothing is copied.
set -u
REF="$1"; WT="$2"; OUT="$3"; mkdir -p "$OUT"
manifest() {  # manifest <worktree> <outdir>
  local w="$1" o="$2"; mkdir -p "$o"; : > "$o/summary.tsv"
  (cd "$w" && git status --ignored --porcelain | grep '^!!' | sed 's/^!! //') > "$o/ignored_list.txt"
  while read -r p; do
    p="${p%/}"; local name; name="$(echo "$p" | tr '/' '_')"
    if [ -d "$w/$p" ]; then
      (cd "$w" && sudo find "$p" \( -type f -o -type l \) -printf '%p\t%y\n' | sort | while IFS=$'\t' read -r f ty; do
        if [ "$ty" = l ]; then echo -e "$f\tlink->$(sudo readlink "$f")"; else echo -e "$f\t$(sudo sha256sum "$f" | cut -d' ' -f1)"; fi
      done) > "$o/$name.manifest"
    else
      echo -e "$p\t$(cd "$w" && sudo sha256sum "$p" | cut -d' ' -f1)" > "$o/$name.manifest"
    fi
    echo -e "$p\t$(wc -l < "$o/$name.manifest")\t$(sha256sum "$o/$name.manifest" | cut -d' ' -f1)" >> "$o/summary.tsv"
  done < "$o/ignored_list.txt"
}
manifest "$REF" "$OUT/ref"; manifest "$WT" "$OUT/wt"
echo "path	ref_entries	wt_entries	result"
join -t$'\t' -j1 <(sort "$OUT/ref/summary.tsv") <(sort "$OUT/wt/summary.tsv") | awk -F'\t' '{print $1"\t"$2"\t"$4"\t"($3==$5?"SAME":"DIFF")}'
comm -23 <(cut -f1 "$OUT/ref/summary.tsv" | sort) <(cut -f1 "$OUT/wt/summary.tsv" | sort) | sed 's/$/\t-\t-\tONLY IN REFERENCE/' | awk -F'\t' '{print $1"\t"$2"\t"$3"\t"$4}'
comm -13 <(cut -f1 "$OUT/ref/summary.tsv" | sort) <(cut -f1 "$OUT/wt/summary.tsv" | sort) | awk '{print $1"\t-\t-\tONLY IN WORKTREE"}'
echo; echo "differing entries per DIFF path (< reference, > worktree), first 40 each:"
join -t$'\t' -j1 <(sort "$OUT/ref/summary.tsv") <(sort "$OUT/wt/summary.tsv") | awk -F'\t' '$3!=$5 {print $1}' | while read -r p; do
  n="$(echo "$p" | tr '/' '_')"; echo "== $p"; diff <(sort "$OUT/ref/$n.manifest") <(sort "$OUT/wt/$n.manifest") | grep '^[<>]' | head -40
done
