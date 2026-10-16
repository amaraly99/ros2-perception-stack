#!/usr/bin/env python3
"""fig_closest_vs_staleness.py (16D figure b): closest approach against v x (latency + mean box gap) for every approaching run,
from staleness_fit.py's staleness_points.tsv. Marker shape = configuration family (n, s, m, CenterNet), color = loss window
(0.4, 1.0, 2.5 s; palette slots 1 to 3, which validate all pairs), the identity model closest = S - x as a thin line, and the
1.0 m outcome line. One run far off the plot (lost mid-approach) is annotated at the top edge. Usage:
  python3 fig_closest_vs_staleness.py <staleness_points.tsv> <out stem>"""
import csv, sys
import fig_style as S
import matplotlib.pyplot as plt

SHAPES = {"n": ("o", "n-models"), "s": ("s", "s-models"), "m": ("^", "m-models"), "centernet": ("D", "CenterNet")}
WCOL = {"0.4": S.SERIES[0], "1.0": S.SERIES[2], "2.5": S.SERIES[1]}


def main():
    src, stem = sys.argv[1], sys.argv[2]
    head = open(src).readline()
    rows = [r for r in csv.DictReader((l for l in open(src) if not l.startswith("#")), delimiter="\t")]
    Sm = float(head.split("S = ")[1].split(" m")[0])
    S.setup()
    fig, ax = plt.subplots(figsize=(S.HALF_W, 2.7))
    ymax = 3.3
    xs = [float(r["x_m"]) for r in rows]; xmax = max(xs) * 1.05
    ax.plot([0, Sm], [Sm, 0], color=S.AXIS, linewidth=0.8, zorder=1)
    ax.text(Sm * 0.55, Sm * 0.55, "closest = S - x", fontsize=6, color=S.MUTED, rotation=-38, ha="center", va="bottom")
    ax.axhline(1.0, color=S.MUTED, linewidth=0.6, zorder=1); ax.text(xmax, 1.03, "1.0 m line", fontsize=6, color=S.MUTED, ha="right", va="bottom")
    ax.axhline(Sm, color=S.GRID, linewidth=0.6, zorder=1); ax.text(2.5, Sm + 0.03, f"S = {Sm:.2f} m", fontsize=6, color=S.MUTED, ha="left", va="bottom")
    for r in rows:
        x, y = float(r["x_m"]), float(r["closest_m"]); mk = SHAPES[r["family"]][0]; c = WCOL[r["W_s"]]
        if y > ymax:
            ax.scatter([x], [ymax - 0.08], marker=mk, s=18, facecolors="none", edgecolors=c, linewidths=0.9, zorder=3)
            ax.annotate(f"{y:.1f} m (lost mid-approach)", (x, ymax - 0.08), xytext=(6, -3), textcoords="offset points", fontsize=5.5, color=S.INK2, ha="left")
            continue
        ax.scatter([x], [y], marker=mk, s=18, color=c, edgecolors=S.SURFACE, linewidths=0.6, zorder=3, alpha=0.9)
    ax.set_xlim(-0.15, xmax); ax.set_ylim(-0.1, ymax)
    ax.set_xlabel("v x (latency + mean target-box gap)  (m)"); ax.set_ylabel("closest approach to the target (m)")
    S.despine(ax)
    hs = [plt.Line2D([], [], marker=SHAPES[k][0], color=S.INK2, linestyle="none", markersize=4, markerfacecolor="none") for k in SHAPES]
    hw = [plt.Line2D([], [], marker="o", color=WCOL[w], linestyle="none", markersize=4) for w in ("0.4", "1.0", "2.5")]
    l1 = ax.legend(hs, [SHAPES[k][1] for k in SHAPES], loc="upper left", bbox_to_anchor=(0.60, 0.99), title="family", title_fontsize=6.5, handletextpad=0.4)
    ax.add_artist(l1)
    ax.legend(hw, ["0.4 s (pooled)", "1.0 s", "2.5 s"], loc="upper left", bbox_to_anchor=(0.29, 0.99), title="loss window W", title_fontsize=6.5, handletextpad=0.4)
    fig.text(0.0, -0.03, f"DRAFT (16D), exploratory. {len(rows)} approaching runs; v at the last box before the closest point; S = vx-zero standoff at the drone origin.",
             fontsize=5.5, color=S.MUTED)
    S.save(fig, stem)


if __name__ == "__main__":
    main()
