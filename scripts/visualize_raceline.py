#!/usr/bin/env python3
"""Interactive visualization utility for the CoVAPSy track map and optimal raceline.

Usage:
    python3 scripts/visualize_raceline.py
    python3 scripts/visualize_raceline.py --save maps/track_visualization.png
"""

import argparse
import csv
from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "common"))
from slam import OccupancyGridMap  # noqa: E402


def plot_track(map_path: Path, raceline_path: Path, ref_path: Path, save_path: Path = None):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Error: matplotlib is required. Install via: pip install matplotlib")
        return

    # Load 2D Occupancy Grid Map
    gmap = OccupancyGridMap.load(map_path)
    wall_x, wall_y = [], []
    for y_idx in range(gmap.height):
        for x_idx in range(gmap.width):
            idx = y_idx * gmap.width + x_idx
            if gmap.log_odds[idx] > 0.7:
                wx, wy = gmap.grid_to_world(x_idx, y_idx)
                wall_x.append(wx)
                wall_y.append(wy)

    # Load Reference Path
    ref_x, ref_y, ref_spd = [], [], []
    if ref_path.is_file():
        with open(ref_path) as f:
            for r in csv.DictReader(f):
                ref_x.append(float(r["x"]))
                ref_y.append(float(r["y"]))
                ref_spd.append(float(r.get("speed_mps", 1.0)))

    # Load Optimized Raceline
    race_x, race_y, race_spd, race_k = [], [], [], []
    if raceline_path.is_file():
        with open(raceline_path) as f:
            for r in csv.DictReader(f):
                race_x.append(float(r["x_m"]))
                race_y.append(float(r["y_m"]))
                race_spd.append(float(r["speed_mps"]))
                race_k.append(float(r["curv_radpm"]))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), facecolor="#0f172a")
    for ax in (ax1, ax2):
        ax.set_facecolor("#1e293b")
        ax.scatter(wall_x, wall_y, s=1.8, c="#64748b", alpha=0.7, label=f"LiDAR Walls ({len(wall_x)} cells)")
        ax.set_aspect("equal")
        ax.grid(True, color="#334155", linestyle="--", alpha=0.6)
        ax.set_xlabel("X (meters)", color="#cbd5e1", fontsize=11)
        ax.set_ylabel("Y (meters)", color="#cbd5e1", fontsize=11)
        ax.tick_params(colors="#94a3b8")
        ax.set_xlim(-6.0, 5.5)
        ax.set_ylim(-4.5, 6.0)

    # Left: Comparison
    if ref_x:
        ax1.plot(ref_x, ref_y, color="#f87171", linestyle="--", linewidth=1.2, alpha=0.85, label=f"Raw FTG Path ({len(ref_x)} pts)")
        rev_x = [ref_x[i] for i in range(len(ref_spd)) if ref_spd[i] < 0]
        rev_y = [ref_y[i] for i in range(len(ref_spd)) if ref_spd[i] < 0]
        if rev_x:
            ax1.scatter(rev_x, rev_y, color="#fbbf24", s=30, zorder=5, label=f"FTG Reverse Hooks ({len(rev_x)} pts pruned)")

    if race_x:
        ax1.plot(race_x, race_y, color="#34d399", linewidth=2.4, label=f"Optimized Raceline ({len(race_x)} pts)")

    ax1.set_title("Track Map & Path Comparison", color="#f8fafc", fontsize=13, fontweight="bold", pad=12)
    ax1.legend(loc="upper right", facecolor="#0f172a", edgecolor="#475569", labelcolor="#f1f5f9", fontsize=9)

    # Right: Speed Heatmap
    if race_x:
        sc = ax2.scatter(race_x, race_y, c=race_spd, cmap="turbo", s=20, zorder=4)
        cbar = plt.colorbar(sc, ax=ax2, pad=0.02)
        cbar.set_label("Target Speed (m/s)", color="#cbd5e1", fontsize=11)
        cbar.ax.yaxis.set_tick_params(color="#94a3b8")
        plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="#94a3b8")
        ax2.set_title(f"Optimized Speed Profile (max {max(race_spd):.2f} m/s)", color="#f8fafc", fontsize=13, fontweight="bold", pad=12)

    plt.tight_layout()

    if save_path:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=160, facecolor=fig.get_facecolor(), edgecolor="none")
        print(f"Saved visualization plot to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize CoVAPSy track map and raceline")
    parser.add_argument("--map", type=Path, default=_REPO_ROOT / "common" / "maps" / "track_map.bin")
    parser.add_argument("--raceline", type=Path, default=_REPO_ROOT / "common" / "maps" / "raceline.csv")
    parser.add_argument("--ref", type=Path, default=_REPO_ROOT / "common" / "maps" / "reference_path.csv")
    parser.add_argument("--save", type=Path, default=None, help="Save to image file instead of displaying")
    args = parser.parse_args()

    plot_track(args.map, args.raceline, args.ref, args.save)


if __name__ == "__main__":
    main()

