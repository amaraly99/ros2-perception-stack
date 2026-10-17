#!/usr/bin/env python3
"""fig_closest_vs_staleness.py (16D figure b, 16F version): closest approach against x = v x (latency + mean box gap) for every
approaching run, from staleness_fit.py's staleness_points.tsv. Marker shape = configuration family (n, s, m, CenterNet), color
= loss window (0.4, 1.0, 2.5 s; palette slots 1 to 3, which validate all pairs). The only model line is the identity law
closest = S - x with S = pred_identity_m + x_m of the points file (the vx-zero standoff at the drone origin, 2.118 m): no
fitted parameter anywhere (16F; the 16D version drew the same line and never the fitted line, which lives in fit.txt only).
Runs whose outcome class is "ground contact" (16F, DEVIATIONS 17.80: the m-models' vertical descent to z = 0) are drawn as
hollow downward triangles with their own legend entry, as a failure mode apart from the horizontal stale-box overshoot, and
are annotated. One run far off the plot (lost mid-approach) is annotated at the top edge. The 1.0 m outcome line is drawn over
the overshoot region (x up to 2.5 m) so that it does not run behind the legends. The residuals closest - (S - x) by
configuration family, backend and loss window are written to <out stem>_caption.txt with a draft caption. Usage: python3 fig_closest_vs_staleness.py <staleness_points.tsv> <out stem>"""
import csv, os, statistics, sys
import fig_style as S
import matplotlib.pyplot as plt

SHAPES = {"n": ("o", "n-models"), "s": ("s", "s-models"), "m": ("^", "m-models"), "centernet": ("D", "CenterNet")}
WCOL = {"0.4": S.SERIES[0], "1.0": S.SERIES[2], "2.5": S.SERIES[1]}
GC = "ground contact"


def main():
    src, stem = sys.argv[1], sys.argv[2]
    cap_path = stem + "_caption.txt"
    if os.path.exists(cap_path):
        sys.exit(f"REFUSE: {cap_path} exists (no overwrite)")
    rows = [r for r in csv.DictReader((l for l in open(src) if not l.startswith("#")), delimiter="\t")]
    Ss = sorted({round(float(r["pred_identity_m"]) + float(r["x_m"]), 3) for r in rows})
    if len(Ss) != 1:
        sys.exit(f"REFUSE: S differs between rows: {Ss}")
    Sm = Ss[0]
    S.setup()
    fig, ax = plt.subplots(figsize=(S.HALF_W, 2.7))
    ymax = 3.3
    xs = [float(r["x_m"]) for r in rows]; xmax = max(xs) * 1.05
    ax.plot([0, Sm], [Sm, 0], color=S.AXIS, linewidth=0.8, zorder=1)
    ax.text(Sm * 0.55, Sm * 0.55, "closest = S - v (L + G)", fontsize=6, color=S.MUTED, rotation=-38, ha="center", va="bottom")
    ax.plot([-0.15, 2.5], [1.0, 1.0], color=S.MUTED, linewidth=0.6, zorder=1); ax.text(0.12, 1.03, "1.0 m line", fontsize=6, color=S.MUTED, ha="left", va="bottom")   # drawn over the overshoot region only, clear of the legends
    ax.axhline(Sm, color=S.GRID, linewidth=0.6, zorder=1); ax.text(0.5, Sm + 0.03, f"S = {Sm:.2f} m", fontsize=6, color=S.MUTED, ha="left", va="bottom")
    gc_pts = []
    for r in rows:
        x, y = float(r["x_m"]), float(r["closest_m"]); mk = SHAPES[r["family"]][0]; c = WCOL[r["W_s"]]
        if r["outcome_class"] == GC:
            ax.scatter([x], [y], marker="v", s=22, facecolors=S.SURFACE, edgecolors=c, linewidths=1.0, zorder=3); gc_pts.append((x, y)); continue
        if y > ymax:
            ax.scatter([x], [ymax - 0.08], marker=mk, s=18, facecolors="none", edgecolors=c, linewidths=0.9, zorder=3)
            ax.annotate(f"{y:.1f} m (lost mid-approach)", (x, ymax - 0.08), xytext=(6, -3), textcoords="offset points", fontsize=5.5, color=S.INK2, ha="left")
            continue
        ax.scatter([x], [y], marker=mk, s=18, color=c, edgecolors=S.SURFACE, linewidths=0.6, zorder=3, alpha=0.9)
    if gc_pts:
        gx = min(p[0] for p in gc_pts); gy = statistics.fmean(p[1] for p in gc_pts)
        ax.annotate("ground contact\n(vertical descent):\nthe law does not apply", (gx, gy), xytext=(-9, 0), textcoords="offset points", fontsize=5.5, color=S.INK2, ha="right", va="center")
    ax.set_xlim(-0.15, xmax); ax.set_ylim(-0.1, ymax)
    ax.set_xlabel("v x (latency + mean target-box gap)  (m)"); ax.set_ylabel("closest approach to the target (m)")
    S.despine(ax)
    hs = [plt.Line2D([], [], marker=SHAPES[k][0], color=S.INK2, linestyle="none", markersize=4, markerfacecolor="none") for k in SHAPES]
    hs.append(plt.Line2D([], [], marker="v", color=S.INK2, linestyle="none", markersize=4.5, markerfacecolor=S.SURFACE))
    hw = [plt.Line2D([], [], marker="o", color=WCOL[w], linestyle="none", markersize=4) for w in ("0.4", "1.0", "2.5")]
    l1 = ax.legend(hs, [SHAPES[k][1] for k in SHAPES] + ["ground contact"], loc="upper left", bbox_to_anchor=(0.71, 0.57), title="family", title_fontsize=6.5, handletextpad=0.4)
    ax.add_artist(l1)
    ax.legend(hw, ["0.4 s (pooled)", "1.0 s", "2.5 s"], loc="upper left", bbox_to_anchor=(0.37, 0.57), title="loss window W", title_fontsize=6.5, handletextpad=0.4)
    fig.text(0.0, -0.03, f"DRAFT (16F), exploratory. {len(rows)} approaching runs; v = horizontal ground speed at the last box before the closest point; the line is the identity law, no fitted parameter.",
             fontsize=5.5, color=S.MUTED)
    S.save(fig, stem)
    # residuals by family, backend and window into the caption file
    groups = {}
    for r in rows:
        groups.setdefault((r["family"], r["backend"], r["W_s"], r["outcome_class"] == GC), []).append(r)
    lines = [f"DRAFT caption (16F, figure b). Closest approach to the target against x = v (L + G) for the {len(rows)} approaching runs (locked on, closest at least 1 m inside the start;",
             f"run 032 set aside): v the horizontal ground speed at the capture of the last target box before the closest point, L the mean capture-to-commit latency, G the mean gap",
             f"between target-box captures. The line is the identity law closest = S - x with S = {Sm:.3f} m, the vx-zero standoff at the drone origin; it has no fitted parameter.",
             "Marker shape = configuration family, colour = loss window. Hollow downward triangles are the ground-contact runs (the m-models at W = 2.5 s): they descended to z = 0 on",
             "stale vertical commands and lost the sign above the frame, a vertical failure to which the horizontal law does not apply (DEVIATIONS 17.80).",
             (f"{sum(1 for r in rows if r['family'] == 'm' and r['backend'] == 'cpu' and r['outcome_class'] != GC)} m-model CPU run(s) drawn as filled triangles descended without reaching z <= 0.01 m and keep the 16D class (see the 16F altitude audit)."),
             "", "Residuals closest - (S - x), in m, by family, backend and window (n; mean; min to max):"]
    def fam_name(k):
        return {"n": "n-models", "s": "s-models", "m": "m-models", "centernet": "CenterNet"}[k]
    for g in sorted(groups, key=lambda g: ("nsmc".index(g[0][0]), g[1], float(g[2]), g[3])):
        rs = groups[g]; res = [float(r["res_identity_m"]) for r in rs]
        tag = " (ground contact; listed, not a test of the law)" if g[3] else ""
        lines.append(f"  {fam_name(g[0])} {g[1].upper()} W {g[2]} s{tag}: n {len(rs)}; mean {statistics.fmean(res):+.2f}; {min(res):+.2f} to {max(res):+.2f}"
                     + (f" (x {min(float(r['x_m']) for r in rs):.2f} to {max(float(r['x_m']) for r in rs):.2f} m, closest {min(float(r['closest_m']) for r in rs):.2f} to {max(float(r['closest_m']) for r in rs):.2f} m)"))
    with open(cap_path, "w") as o:
        o.write("\n".join(lines) + "\n")
    print("wrote", cap_path); print("\n".join(lines[5:]))


if __name__ == "__main__":
    main()
