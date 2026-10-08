#!/usr/bin/env python3
"""make_plans.py [--seed N] [--sessions K] [--out DIR]: the fx 554 consolidation plans (5R), one tsv per session, never overwritten.

Rows: 10 detectors (YOLO x 9, CenterNet) x {npu, cpu} x 6 repeats = 120 runs. SSD (sees no sign from 20 m, 5M): 2 runs per
backend in total, role expect_stop. Each backend's runs are shuffled with the seed and dealt round robin over the sessions,
so every session has the same NPU/CPU mix; each session is then shuffled again (NPU and CPU mixed) and starts with an
uncounted YOLO26n NPU sentinel. The seeds are written in each file.
NPU opens per session (5R): 2 (reference check, start and end) + 2 per NPU run (own-config gate + stack); the session gate
has no NPU configuration. Budget: config/protocol_fx554.json npu_open_budget_per_boot (40)."""
import argparse, json, os, random

DET = ["yolo26n", "yolo26s", "yolo26m", "yolov8n", "yolov8s", "yolov8m", "yolov11n", "yolov11s", "yolov11m",
       "centernet_resnet_v1_18"]
SSD = "ssd_mobilenet_v2"
H = "# key\tmodel\tbackend\trole   (batch.sh plan; npu: placement A / ours_fx554_A_ibvs, cpu: B / ours_fx554_B_ibvs; window from config/protocol_fx554.json)\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=20261005)
    ap.add_argument("--sessions", type=int, default=4)
    ap.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    budget = json.load(open(os.path.join(here, "..", "config", "protocol_fx554.json")))["between_runs"]["npu_open_budget_per_boot"]
    K = a.sessions
    sess = [[] for _ in range(K)]
    for i, b in enumerate(("npu", "cpu")):
        runs = [(f"c{r}", m, b, "row") for r in range(1, 7) for m in DET] + [(f"x{k}", SSD, b, "expect_stop") for k in (1, 2)]
        random.Random(a.seed * 10 + i).shuffle(runs)
        for j, r in enumerate(runs):
            sess[j % K].append(r)
    for s, runs in enumerate(sess, 1):
        random.Random(a.seed * 100 + s).shuffle(runs)
        npu = sum(1 for r in runs if r[2] == "npu") + 1
        opens = 2 + 2 * npu
        assert opens <= budget, (s, opens, budget)
        with open(os.path.join(a.out, f"consolidation_fx554_s{s}.tsv"), "x") as f:
            f.write(f"# Consolidation pass fx 554, session {s} of {K}: {len(runs)} runs dealt from 120 row runs + 4 SSD (expect_stop).\n"
                    f"# make_plans.py --seed {a.seed} --sessions {K}; deal seeds {a.seed * 10} (npu) {a.seed * 10 + 1} (cpu), "
                    f"session shuffle seed {a.seed * 100 + s}. Sentinel first (not counted).\n"
                    f"# {len(runs) + 1} runs; NPU opens this session: 2 + 2 x {npu} = {opens} <= {budget}. Power cycle before the session.\n" + H)
            f.write(f"sen{s}\tyolo26n\tnpu\tsentinel\n")
            for r in runs:
                f.write("\t".join(r) + "\n")
        print(f"session {s}: {len(runs) + 1} runs, NPU runs {npu}, opens {opens}")


if __name__ == "__main__":
    main()
