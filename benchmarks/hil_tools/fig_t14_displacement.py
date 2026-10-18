#!/usr/bin/env python3
"""fig_t14_displacement.py (16D figure a): Table 14 centroid displacement by range bin, NPU rows against CPU rows, from
pool_tables.py's DRAFT_table14_bins.tsv. Two panels (at capture, at delivery), x = the four fx 554 bins from far to near,
y = the pooled median displacement in px; one thin line per configuration, NPU rows in slot 1 (blue), CPU rows in slot 2
(orange); the backend legend plus direct labels on the CPU lines and on the one NPU line that separates (CenterNet).
Bins below the reporting floor are not drawn (the table prints them as text). Usage:
  python3 fig_t14_displacement.py <DRAFT_table14_bins.tsv> <out stem> [--final]
17D: --final drops the DRAFT footer line (nothing else changes); in both modes the script prints "plotted_sha256 <hex>", the
sha256 of the arrays it draws (per configuration and panel), so a final figure can be checked against its draft run."""
import csv, sys
import fig_style as S
import matplotlib.pyplot as plt

DISPLAY = {"yolo26n": "YOLO26n", "yolo26s": "YOLO26s", "yolo26m": "YOLO26m", "yolov8n": "YOLOv8n", "yolov8s": "YOLOv8s", "yolov8m": "YOLOv8m",
           "yolov11n": "YOLOv11n", "yolov11s": "YOLOv11s", "yolov11m": "YOLOv11m", "centernet_resnet_v1_18": "CenterNet"}


def main():
    final = "--final" in sys.argv                                      # 17D: no DRAFT footer
    args = [a for a in sys.argv[1:] if a != "--final"]
    src, stem = args[0], args[1]
    plotted = []                                                       # 17D: every series drawn, for the printed hash
    rows = [r for r in csv.DictReader((l for l in open(src) if not l.startswith("#")), delimiter="\t") if r["status"] == "ok" and "(" not in r["model"]]
    bins = []
    for r in rows:
        if r["bin_m"] not in bins:
            bins.append(r["bin_m"])
    bins.sort(key=lambda b: -float(b.split("-")[0]))   # far to near
    cfgs = sorted({(r["model"], r["backend"]) for r in rows}, key=lambda c: (c[1] != "npu", c[0]))
    S.setup()
    fig, axes = plt.subplots(1, 2, figsize=(S.HALF_W * 2, 2.5), sharey=True)
    for ax, col, title in zip(axes, ("cap_med_px", "del_med_px"), ("at capture", "at delivery")):
        labels = []
        for model, bk in cfgs:
            pts = [(bins.index(r["bin_m"]), float(r[col])) for r in rows if (r["model"], r["backend"]) == (model, bk) and r[col]]
            if not pts:
                continue
            pts.sort(); color = S.SERIES[0] if bk == "npu" else S.SERIES[1]
            plotted.append([col, model, bk, [[bins[p[0]], p[1]] for p in pts]])
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, alpha=0.85 if bk == "cpu" else 0.55, marker="o", markersize=3.2,
                    markeredgecolor=S.SURFACE, markeredgewidth=0.6)
            if bk == "cpu" or model.startswith("centernet"):
                labels.append([pts[-1][0], pts[-1][1], f"{DISPLAY.get(model, model)} {bk.upper()}"])
        # direct labels at the right end, pushed apart vertically when they would collide (0.3 px of the axis), leader lines
        labels.sort(key=lambda l: l[1]); ys = [l[1] for l in labels]
        for i in range(1, len(ys)):
            if ys[i] - ys[i - 1] < 0.32:
                ys[i] = ys[i - 1] + 0.32
        for (x, y, txt), yt in zip(labels, ys):
            ax.annotate(txt, (x, y), xytext=(x + 0.12, yt), textcoords="data", fontsize=6, color=S.INK2, va="center",
                        arrowprops={"arrowstyle": "-", "color": S.AXIS, "lw": 0.5, "shrinkA": 0, "shrinkB": 1.5})
        ax.set_xticks(range(len(bins))); ax.set_xticklabels([b.replace("m", " m") for b in bins]); ax.set_xlim(-0.3, len(bins) - 1 + 1.6)
        ax.set_title(f"Centroid displacement {title}", loc="left", color=S.INK); S.despine(ax)
        ax.set_xlabel("range bin at capture (far to near)")
    axes[0].set_ylabel("median displacement (px)")
    h = [plt.Line2D([], [], color=S.SERIES[0]), plt.Line2D([], [], color=S.SERIES[1])]
    axes[0].legend(h, ["NPU rows (10 configurations)", "CPU rows with approach frames (3 n-models)"], loc="upper left", handlelength=1.6)
    if not final:
        fig.text(0.0, -0.10, "DRAFT (16D). Pooled medians over the approach phase, 6 runs per row; CPU s-, m-model and CenterNet rows have no approach frames at W = 0.4 s.",
                 fontsize=6, color=S.MUTED)
    import hashlib, json
    print("plotted_sha256", hashlib.sha256(json.dumps(plotted, sort_keys=True).encode()).hexdigest(), "(final)" if final else "(draft)")
    S.save(fig, stem)


if __name__ == "__main__":
    main()
