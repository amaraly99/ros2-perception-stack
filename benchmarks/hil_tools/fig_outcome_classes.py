#!/usr/bin/env python3
"""fig_outcome_classes.py (16D figure c; 16F classes): outcome classes per configuration at W = 0.4 s (the pooled rows) and
W = 2.5 s (the ablation rows), from outcome_classes.py's classes_by_config.tsv. Two panels of horizontal stacked bars (one bar
per configuration, segments = runs per class, a 2 px surface gap between segments, counts inside segments that fit). Each
class keeps one fixed palette slot across versions (the six 16D classes slots 1 to 6; 16F ground contact, DEVIATIONS 17.80,
slot 7); the segments are stacked in the order of CLASSES and the legend lists the classes present in the plotted rows.
Sentinel rows and the 1.0 s arm are left out of the figure (they are in the table).
Usage: python3 fig_outcome_classes.py <classes_by_config.tsv> <out stem>"""
import csv, sys
import fig_style as S
import matplotlib.pyplot as plt

CLASSES = ["reached cleanly", "reached after an overshoot", "overshot and lost the sign", "ground contact", "arrived in the window without REACHED", "never locked on", "other"]
LABELS = {"reached cleanly": "reached cleanly", "reached after an overshoot": "reached after an overshoot", "overshot and lost the sign": "overshot, lost the sign",
          "ground contact": "ground contact (vertical descent)", "arrived in the window without REACHED": "arrived in the window, no REACHED", "never locked on": "never locked on", "other": "other"}
SLOT = {"reached cleanly": 0, "reached after an overshoot": 1, "overshot and lost the sign": 2, "arrived in the window without REACHED": 3, "never locked on": 4, "other": 5, "ground contact": 6}
WHITE_TEXT = {0, 5, 6, 7}   # palette slots dark enough for white counts
ORDER = ["yolo26n", "yolo26s", "yolo26m", "yolov8n", "yolov8s", "yolov8m", "yolov11n", "yolov11s", "yolov11m", "centernet_resnet_v1_18"]
DISPLAY = {"yolo26n": "YOLO26n", "yolo26s": "YOLO26s", "yolo26m": "YOLO26m", "yolov8n": "YOLOv8n", "yolov8s": "YOLOv8s", "yolov8m": "YOLOv8m",
           "yolov11n": "YOLOv11n", "yolov11s": "YOLOv11s", "yolov11m": "YOLOv11m", "centernet_resnet_v1_18": "CenterNet"}


def panel(ax, rows, title):
    rows = sorted(rows, key=lambda r: (r["backend"] != "npu", ORDER.index(r["model"]) if r["model"] in ORDER else 99))
    names = [f"{DISPLAY.get(r['model'], r['model'])} {r['backend'].upper()}" for r in rows]
    ys = list(range(len(rows)))[::-1]
    for y, r in zip(ys, rows):
        left = 0.0
        for c in CLASSES:
            n = int(r.get(c, 0) or 0)
            if not n:
                continue
            i = SLOT[c]
            ax.barh(y, n, left=left, height=0.62, color=S.SERIES[i], edgecolor=S.SURFACE, linewidth=1.2, zorder=3)
            ax.text(left + n / 2, y, str(n), ha="center", va="center", fontsize=6, color="white" if i in WHITE_TEXT else S.INK, zorder=4)
            left += n
    ax.set_yticks(ys); ax.set_yticklabels(names); ax.set_xlabel("runs"); ax.set_title(title, loc="left", color=S.INK)
    ax.grid(False); ax.set_axisbelow(True); ax.xaxis.grid(True, color=S.GRID, linewidth=0.5)
    S.despine(ax); nmax = max(int(r["n"]) for r in rows); ax.set_xlim(0, nmax + 0.3); ax.set_xticks(range(0, nmax + 1))


def main():
    src, stem = sys.argv[1], sys.argv[2]
    rows = [r for r in csv.DictReader((l for l in open(src) if not l.startswith("#")), delimiter="\t") if r["role"] != "sentinel"]
    pooled = [r for r in rows if r["set"] == "pooled" and r["W_s"] == "0.4"]
    abl = [r for r in rows if r["set"] == "5a" and r["W_s"] == "2.5"]
    S.setup()
    fig, axes = plt.subplots(1, 2, figsize=(S.HALF_W * 2, 3.4), gridspec_kw={"width_ratios": [1, 1]})
    panel(axes[0], pooled, "W = 0.4 s (pinned rule; 6 runs per row)")
    panel(axes[1], abl, "W = 2.5 s (ablation; 3 runs per row, controls 2)")
    present = [c for c in CLASSES if any(int(r.get(c, 0) or 0) for r in pooled + abl)]
    hs = [plt.Rectangle((0, 0), 1, 1, color=S.SERIES[SLOT[c]]) for c in present]
    fig.legend(hs, [LABELS[c] for c in present], loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.06), handlelength=1.2, columnspacing=1.2)
    fig.text(0.0, -0.13, "DRAFT (16F). Classes fixed before counting (1.0 m line); ground contact = z reached 0 after resume (DEVIATIONS 17.80), precedence over every other class;\n"
             "the W = 1.0 s arm (5 valid runs) and run 032 (rig fault) are in the table only.", fontsize=6, color=S.MUTED, va="top")
    fig.tight_layout(w_pad=1.5)
    S.save(fig, stem)


if __name__ == "__main__":
    main()
