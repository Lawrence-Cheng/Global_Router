"""Draw the figures in the README from the files in inputs/ and outputs/.

    python3 tools/plot_results.py

Figures are written to figures/:
    routing_case3.png      every routed net, colored by layer
    congestion_case6.png   demand over capacity of every GCell, per layer
    convergence.png        total overflow after each rip-up-and-reroute iteration
"""
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(ROOT, "figures")

INK, MUTED, GRID, AXIS = "#191919", "#666666", "#e7e7e7", "#c2c2c2"
LAYER0, LAYER1, PIN = "#257de0", "#f47230", "#191919"       # horizontal, vertical, pins
SERIES = ["#257de0", "#f47230", "#00b894"]
BLUES = LinearSegmentedColormap.from_list("blues", ["#eef5fe", "#84b9f3", "#257de0", "#0e3d7e"])
OVERFLOW = "#dc3c3d"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10, "text.color": INK,
    "axes.edgecolor": AXIS, "axes.labelcolor": MUTED, "axes.titlesize": 11, "axes.titleweight": "bold",
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
})


def read_cap(case):
    """Return (xSize, ySize, capacity[layer, row, col], direction per layer)."""
    with open(os.path.join(ROOT, "inputs", f"{case}.cap")) as f:
        lines = [ln.split() for ln in f if ln.strip()]
    _, xs, ys = map(int, lines[0])
    cap, dirs, i = [], [], 4
    for _ in range(2):
        dirs.append(lines[i][1]); i += 1
        cap.append([list(map(int, lines[i + r])) for r in range(ys)]); i += ys
    return xs, ys, np.array(cap), dirs


def read_route(case):
    """Return one list of (layer, col, row) points per net."""
    nets, cur = [], None
    with open(os.path.join(ROOT, "outputs", f"{case}.route")) as f:
        for ln in f:
            ln = ln.strip()
            if ln == "(":
                cur = []
            elif ln == ")":
                nets.append(cur); cur = None
            elif cur is not None and ln:
                a = list(map(int, ln.split()))
                if not cur:
                    cur.append(tuple(a[:3]))
                cur.append(tuple(a[3:]))
    return nets


def plot_routing(case):
    xs, ys, _, dirs = read_cap(case)
    nets = read_route(case)
    seg = {0: [], 1: []}
    pins = []
    for path in nets:
        pins += [path[0][1:], path[-1][1:]]
        for (l0, c0, r0), (l1, c1, r1) in zip(path, path[1:]):
            if l0 == l1:
                seg[l0].append([(c0, r0), (c1, r1)])
    fig, ax = plt.subplots(figsize=(7.2, 7.6))
    ax.add_collection(LineCollection(seg[0], colors=LAYER0, linewidths=0.9, alpha=0.85))
    ax.add_collection(LineCollection(seg[1], colors=LAYER1, linewidths=0.9, alpha=0.85))
    px, py = zip(*pins)
    ax.scatter(px, py, s=3, color=PIN, zorder=3, linewidths=0)
    ax.set_xlim(-1, xs); ax.set_ylim(ys, -1); ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(True)
    ax.set_title(f"{case}: {len(nets):,} nets on a {xs}x{ys} grid", loc="left")
    handles = [plt.Line2D([], [], color=LAYER0, lw=2, label=f"Layer 0 ({'horizontal' if dirs[0] == 'H' else 'vertical'})"),
               plt.Line2D([], [], color=LAYER1, lw=2, label=f"Layer 1 ({'horizontal' if dirs[1] == 'H' else 'vertical'})"),
               plt.Line2D([], [], color=PIN, marker="s", ms=4, lw=0, label="Pin")]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=3, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f"routing_{case}.png"), dpi=170)
    plt.close(fig)


def plot_congestion(case):
    xs, ys, cap, dirs = read_cap(case)
    demand = np.zeros_like(cap)
    for path in read_route(case):
        for l, c, r in set(path):            # a net counts once per GCell
            demand[l, r, c] += 1
    over = np.maximum(0, demand - cap)
    util = np.where(cap > 0, demand / np.maximum(cap, 1), np.where(demand > 0, 1.0, 0.0))
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.6), constrained_layout=True)
    for l, ax in enumerate(axes):
        im = ax.imshow(np.clip(util[l], 0, 1), cmap=BLUES, vmin=0, vmax=1, interpolation="nearest")
        rr, cc = np.nonzero(over[l])
        ax.scatter(cc, rr, s=26, facecolors="none", edgecolors=OVERFLOW, linewidths=1.4)
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(f"Layer {l} ({'horizontal' if dirs[l] == 'H' else 'vertical'}): "
                     f"{int(over[l].sum())} overflow in {len(rr)} GCells", loc="left")
    cb = fig.colorbar(im, ax=axes, shrink=0.8, pad=0.02)
    cb.set_label("Demand / capacity")
    cb.outline.set_visible(False)
    axes[1].scatter([], [], s=26, facecolors="none", edgecolors=OVERFLOW, linewidths=1.4, label="GCell with overflow")
    axes[1].legend(loc="upper right", bbox_to_anchor=(1.0, -0.02), frameon=False)
    fig.suptitle(f"{case}: congestion after routing ({xs}x{ys} grid, total overflow {int(over.sum())})\n",
                 x=0.01, ha="left", fontweight="bold", fontsize=12)
    fig.savefig(os.path.join(FIG, f"congestion_{case}.png"), dpi=150)
    plt.close(fig)
    return int(over.sum())


def plot_convergence(cases):
    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    offsets = {0: (8, 10), 1: (6, 26), 2: (10, -22)}       # keep the three labels apart
    for k, (color, case) in enumerate(zip(SERIES, cases)):
        with open(os.path.join(ROOT, "outputs", "logs", f"{case}.log")) as f:
            ov = [int(m.group(1)) for m in re.finditer(r"-> Overflow: (\d+)", f.read())]
        it = np.arange(1, len(ov) + 1)
        ax.plot(it, ov, color=color, lw=2)
        best = int(np.argmin(ov))
        ax.scatter([it[best]], [ov[best]], s=40, color=color, zorder=3, edgecolors="white", linewidths=1.5)
        ax.annotate(f"{case}: best {ov[best]} at iteration {it[best]}", (it[best], ov[best]),
                    xytext=offsets[k], textcoords="offset points", color=INK, fontsize=9)
        ax.plot([], [], color=color, lw=2, label=f"{case} (starts at {ov[0]:,})")
    ax.set_yscale("symlog", linthresh=10)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("Rip-up-and-reroute iteration"); ax.set_ylabel("Total overflow")
    ax.grid(axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
    ax.legend(frameon=False, loc="upper right")
    ax.set_title("Total overflow after each iteration", loc="left")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "convergence.png"), dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(FIG, exist_ok=True)
    plot_routing("case3")
    print("case6 total overflow:", plot_congestion("case6"))
    plot_convergence(["case4", "case5", "case6"])
    print("figures written to", FIG)
