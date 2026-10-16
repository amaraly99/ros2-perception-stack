"""fig_style.py (16D): the shared plain style of the draft figures (half page width, system python3 with matplotlib).
Colors are the validated default categorical order of the data-viz reference palette (blue, orange, aqua, yellow, magenta,
green, violet, red), used in that fixed order; ink and grid tokens from the same reference. Static PNG and PDF."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#ffffff"
HALF_W = 3.5   # inches, one column of a two-column page


def setup():
    plt.rcParams.update({"font.family": "sans-serif", "font.size": 7.5, "axes.labelsize": 7.5, "axes.titlesize": 8, "legend.fontsize": 6.5,
                         "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.edgecolor": AXIS, "axes.linewidth": 0.6, "xtick.color": MUTED, "ytick.color": MUTED,
                         "axes.labelcolor": INK2, "text.color": INK, "grid.color": GRID, "grid.linewidth": 0.5, "grid.linestyle": "-", "axes.grid": True,
                         "axes.grid.axis": "y", "axes.axisbelow": True, "legend.frameon": False, "savefig.dpi": 300, "figure.facecolor": SURFACE,
                         "axes.facecolor": SURFACE, "pdf.fonttype": 42, "lines.linewidth": 1.4, "lines.solid_joinstyle": "round", "lines.solid_capstyle": "round"})


def despine(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(length=2.5, width=0.5)


def save(fig, stem):
    import os, sys
    for ext in (".png", ".pdf"):
        if os.path.exists(stem + ext):
            sys.exit(f"REFUSE: {stem + ext} exists (no overwrite; use a new stem or folder)")
    fig.savefig(stem + ".png", bbox_inches="tight", pad_inches=0.03)
    fig.savefig(stem + ".pdf", bbox_inches="tight", pad_inches=0.03)
    print("wrote", stem + ".png", stem + ".pdf")
