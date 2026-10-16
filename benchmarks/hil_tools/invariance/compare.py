#!/usr/bin/env python3
"""compare.py <out root> <tag A> <'stored'|tag B> <batches.tsv> [--usb DIR]: compare a re-score tree with the stored check
outputs of each run (<run>/check/, or <run>/check_15H/ when present) or with another tree. Files: *_run1_check.json,
*_run1_inloop.json, *_run1_inloop.txt, *_run1_inloop_stackbag.json, *_run1_inloop_stackbag.txt (and *_check.txt when both
exist). The out-dir path in the scorer's "wrote" line and the cache path are normalised. For a differing _check.json the
check verdicts and info keys that differ are listed; for a differing _inloop*.json the geometry keys and whether every score()
key (everything but "geometry") is identical. Runs present in only one tree are listed as MISSING (a partial tree is normal).
Generalised from stage6/prescore_16A/compare_16A.py (16A)."""
import argparse, glob, json, os, re
SUFF = ["_run1_check.json", "_run1_inloop.json", "_run1_inloop.txt", "_run1_inloop_stackbag.json", "_run1_inloop_stackbag.txt", "_check.txt"]


def norm(txt, d):
    txt = txt.replace(d.rstrip("/") + "/", "<OUT>/")
    return re.sub(r"<OUT>/\.inloop_cache/[^\s\"']+", "<OUT>/.inloop_cache/<key>", txt)


def files(d, label):
    out = {}
    for s in SUFF:
        p = os.path.join(d, label + s)
        if os.path.exists(p):
            out[s] = norm(open(p, errors="replace").read(), d)
    return out


def score_keys(j):
    return {k: v for k, v in j.items() if k != "geometry"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_root"); ap.add_argument("a"); ap.add_argument("b"); ap.add_argument("batches"); ap.add_argument("--usb", default="/mnt/usb/hil_runs")
    x = ap.parse_args()
    batches = [l.split()[1] for l in open(x.batches) if l.strip() and not l.startswith("#")]
    tot = same = 0; diffs = []
    for b in batches:
        bn = os.path.basename(b)
        for rd in sorted(glob.glob(os.path.join(x.usb, b, "0[0-9][0-9]_*"))):
            if not os.path.isdir(os.path.join(rd, "approach_bag")):
                continue
            r = os.path.basename(rd); label = "level2_" + r[4:]
            da = os.path.join(x.out_root, x.a, bn, r)
            db = (rd + "/check_15H" if os.path.isdir(rd + "/check_15H") else rd + "/check") if x.b == "stored" else os.path.join(x.out_root, x.b, bn, r)
            fa, fb = files(da, label), files(db, label)
            if not fa or not fb:
                diffs.append((bn, r, "MISSING", f"a {sorted(fa)} b {sorted(fb)}")); continue
            tot += 1
            d = [s for s in SUFF if (s in fa or s in fb) and fa.get(s) != fb.get(s) and not (s == "_check.txt" and (s not in fa or s not in fb))]
            if not d:
                same += 1; continue
            detail = []
            if "_run1_check.json" in d and fa.get("_run1_check.json") and fb.get("_run1_check.json"):
                ja, jb = json.loads(fa["_run1_check.json"]), json.loads(fb["_run1_check.json"])
                cv = {k: (jb["checks"].get(k), ja["checks"].get(k)) for k in set(ja["checks"]) | set(jb["checks"]) if ja["checks"].get(k) != jb["checks"].get(k)}
                ik = sorted(k for k in set(ja["info"]) | set(jb["info"]) if ja["info"].get(k) != jb["info"].get(k))
                detail.append(f"checks changed (b->a): {cv}; info keys differing: {ik}; pass {jb.get('pass')}->{ja.get('pass')}")
            for s in ("_run1_inloop.json", "_run1_inloop_stackbag.json"):
                if s in d:
                    if fa.get(s) and fb.get(s):
                        ja, jb = json.loads(fa[s]), json.loads(fb[s])
                        gd = {k: (jb["geometry"].get(k), ja["geometry"].get(k)) for k in sorted(set(ja["geometry"]) | set(jb["geometry"])) if ja["geometry"].get(k) != jb["geometry"].get(k)}
                        eq = score_keys(ja) == score_keys(jb)
                        detail.append(f"{s}: geometry keys differing {gd}; score() keys identical: {eq}"
                                      + ("" if eq else f" (differing: {sorted(k for k in set(score_keys(ja)) | set(score_keys(jb)) if score_keys(ja).get(k) != score_keys(jb).get(k))})"))
                    else:
                        detail.append(f"{s}: present in a {bool(fa.get(s))}, in b {bool(fb.get(s))}")
            diffs.append((bn, r, ",".join(d), " | ".join(detail)))
    print(f"compared {tot} runs: identical in every file {same}, differing {tot - same}, missing {sum(1 for x in diffs if x[2] == 'MISSING')}")
    for x_ in diffs:
        if x_[2] != "MISSING":
            print(f"- {x_[0]} {x_[1]}: files {x_[2]}\n    {x_[3]}")
    miss = [f"{x_[0]} {x_[1]}" for x_ in diffs if x_[2] == "MISSING"]
    if miss:
        print(f"missing in one tree ({len(miss)}): " + ", ".join(miss[:10]) + (" ..." if len(miss) > 10 else ""))


if __name__ == "__main__":
    main()
