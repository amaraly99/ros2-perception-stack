"""held_members.py (19A, DEVIATIONS 17.84): the accepted held-render set of a start profile, in one place.
A profile's held-reference json names one primary render (render_sha1) and may name alternates
(accepted_renders: a list of {render_sha1, fixture_png, evidence}). Absent or empty accepted_renders means
the primary alone, the pre-17.84 behaviour, which is every profile except the 6 m one. The gate passes a
final held render if and only if its sha1 is a member (the render logger's sha1 is over the raw bgr8 bytes,
so membership is byte identity). Callers: hold_gate.sh and hold_once.sh (accept), run_one.sh (ref-json: a
per-run copy of the profile whose render_sha1 is the MATCHED member, handed to check_run.py through
HIL_HELD_REF so check_run.py stays byte-unchanged and its existing held_reference_sha1 info key records the
matched member as the run's covariate).
  held_members.py accept <held_reference.json> <sha1>               exit 0 accepted, 1 not
  held_members.py ref-json <held_reference.json> <sha1> <out.json>  exit 0 written, 1 not accepted (or out exists)
"""
import json
import sys


def members(ref):
    return [ref["render_sha1"]] + [m["render_sha1"] for m in ref.get("accepted_renders", [])]


def main():
    cmd, path, sha = sys.argv[1], sys.argv[2], sys.argv[3]
    ref = json.load(open(path))
    ms = members(ref)
    ok = sha in ms
    if cmd == "accept":
        print(f"held_members: {'accepted' if ok else 'NOT ACCEPTED'}: {sha[:8]} against {len(ms)} member(s) "
              f"of {path} (primary {ms[0][:8]})")
        sys.exit(0 if ok else 1)
    if cmd == "ref-json":
        out = sys.argv[4]
        if not ok:
            print(f"held_members: ref-json REFUSED: {sha[:8]} is not an accepted member of {path}")
            sys.exit(1)
        rec = dict(ref)
        rec["render_sha1"] = sha
        rec["matched_from"] = {"profile": path, "primary_render_sha1": ms[0], "matched_member": sha,
                               "doc": "per-run matched reference (held_members.py ref-json, DEVIATIONS 17.84); "
                                      "check_run.py reads render_sha1 only"}
        try:
            with open(out, "x") as f:
                json.dump(rec, f, indent=1)
        except OSError as e:
            print(f"held_members: ref-json REFUSED: {e}")
            sys.exit(1)
        print(f"held_members: ref-json wrote {out} with render_sha1 {sha[:8]} (primary {ms[0][:8]})")
        sys.exit(0)
    sys.exit(f"held_members: unknown command {cmd}")


if __name__ == "__main__":
    main()
