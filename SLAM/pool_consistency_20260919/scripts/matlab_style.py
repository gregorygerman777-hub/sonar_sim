"""A matplotlib rcParams profile that mimics default MATLAB R2019b+ figure styling:
boxed axes (all four spines), inward ticks on all sides, the MATLAB categorical
color order, Helvetica-class sans-serif type, and light gridlines."""
import matplotlib.pyplot as plt

MATLAB_COLORS = [
    "#0072BD", "#D95319", "#EDB120", "#7E2F8E",
    "#77AC30", "#4DBEEE", "#A2142F",
]


def apply():
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
        "font.size": 11,
        "axes.prop_cycle": plt.cycler(color=MATLAB_COLORS),
        "axes.edgecolor": "black",
        "axes.linewidth": 1.0,
        "axes.grid": True,
        "grid.color": "#B0B0B0",
        "grid.linewidth": 0.5,
        "grid.alpha": 0.5,
        "axes.spines.top": True,
        "axes.spines.right": True,
        "xtick.direction": "in",
        "ytick.direction": "in",
        "xtick.top": True,
        "ytick.right": True,
        "xtick.major.size": 5,
        "ytick.major.size": 5,
        "legend.frameon": True,
        "legend.edgecolor": "black",
        "legend.fancybox": False,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "savefig.dpi": 160,
    })
