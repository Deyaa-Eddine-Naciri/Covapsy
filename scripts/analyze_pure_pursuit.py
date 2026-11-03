#!/usr/bin/env python3
"""Telemetry analysis and debugging script for Pure Pursuit path tracking.

Reads /tmp/covapsy_pure_pursuit_telemetry.csv (logged automatically by covapsy_pure_pursuit)
and computes tracking accuracy, cross-track error (CTE), and cornering behavior.

Usage:
    python3 scripts/analyze_pure_pursuit.py
    python3 scripts/analyze_pure_pursuit.py --save maps/tracking_debug.png
"""

import argparse
import csv
from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "common"))


def analyze_telemetry(csv_path: Path, raceline_path: Path, save_path: Path = None):
    if not csv_path.is_file():
        print(f"Error: Telemetry file {csv_path} not found.")
        print("Run a simulation lap in Webots first to generate telemetry.")
        return

    with open(csv_path, "r") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        print("Telemetry file is empty.")
        return

    t = [float(r["time_s"]) for r in rows]
    cx = [float(r["car_x"]) for r in rows]
    cy = [float(r["car_y"]) for r in rows]
    spd = [float(r["measured_spd"]) for r in rows]
    tgt_spd = [float(r["target_spd"]) for r in rows]
    steer = [float(r["steer_cmd_deg"]) for r in rows]
    cte = [float(r["cte_m"]) * 100.0 for r in rows]  # to cm
    wp_idx = [int(r["wp_idx"]) for r in rows]

    mean_cte = sum(cte) / len(cte)
    max_cte = max(cte)
    max_cte_idx = cte.index(max_cte)
    avg_spd = sum(spd) / len(spd)
    max_spd = max(spd)

    print("=" * 60)
    print("  PURE PURSUIT TRACKING & TELEMETRY REPORT")
    print("=" * 60)
    print(f"Total samples recorded: {len(rows)}")
    print(f"Time span:             {t[-1] - t[0]:.2f} s ({t[0]:.2f}s to {t[-1]:.2f}s)")
    print(f"Mean Cross-Track Error:{mean_cte:.1f} cm")
    print(f"Max Cross-Track Error: {max_cte:.1f} cm (at t={t[max_cte_idx]:.2f}s, wp={wp_idx[max_cte_idx]}, pos=({cx[max_cte_idx]:.2f}, {cy[max_cte_idx]:.2f}))")
    print(f"Speed Profile:         Avg = {avg_spd:.2f} m/s ({avg_spd*3.6:.1f} km/h), Peak = {max_spd:.2f} m/s ({max_spd*3.6:.1f} km/h)")
    print("=" * 60)

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot generation.")
        return

    # Load planned raceline
    rx, ry = [], []
    if raceline_path.is_file():
        with open(raceline_path) as f:
            for r in csv.DictReader(f):
                rx.append(float(r["x_m"]))
                ry.append(float(r["y_m"]))

    fig = plt.figure(figsize=(15, 9), facecolor="#0f172a")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1.0])

    # 1. 2D Trajectory Tracking (Left)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_map.set_facecolor("#1e293b")
    if rx:
        ax_map.plot(rx, ry, color="#64748b", linestyle="--", linewidth=1.5, label="Planned Raceline")
    sc = ax_map.scatter(cx, cy, c=cte, cmap="plasma", s=14, zorder=4)
    cbar = plt.colorbar(sc, ax=ax_map, pad=0.02)
    cbar.set_label("Cross-Track Error (cm)", color="#cbd5e1", fontsize=10)
    cbar.ax.yaxis.set_tick_params(color="#94a3b8")
    plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="#94a3b8")

    # Mark start/finish
    ax_map.plot([0, 0], [-1.5, -3.5], color="#38bdf8", linewidth=2.5, label="Start/Finish Line")
    ax_map.set_aspect("equal")
    ax_map.grid(True, color="#334155", linestyle="--", alpha=0.6)
    ax_map.set_xlabel("X (meters)", color="#cbd5e1")
    ax_map.set_ylabel("Y (meters)", color="#cbd5e1")
    ax_map.tick_params(colors="#94a3b8")
    ax_map.set_title("Driven Path vs Raceline (colored by CTE)", color="#f8fafc", fontweight="bold")
    ax_map.legend(loc="upper right", facecolor="#0f172a", edgecolor="#475569", labelcolor="#f1f5f9", fontsize=9)

    # 2. CTE vs Time (Top Right)
    ax_cte = fig.add_subplot(gs[0, 1])
    ax_cte.set_facecolor("#1e293b")
    ax_cte.plot(t, cte, color="#f43f5e", linewidth=1.8, label="Cross-Track Error (cm)")
    ax_cte.axhline(mean_cte, color="#fbbf24", linestyle="--", label=f"Mean CTE ({mean_cte:.1f} cm)")
    ax_cte.grid(True, color="#334155", linestyle="--", alpha=0.6)
    ax_cte.set_ylabel("CTE (cm)", color="#cbd5e1")
    ax_cte.tick_params(colors="#94a3b8")
    ax_cte.set_title("Tracking Accuracy Over Time", color="#f8fafc", fontweight="bold")
    ax_cte.legend(loc="upper right", facecolor="#0f172a", edgecolor="#475569", labelcolor="#f1f5f9", fontsize=9)

    # 3. Speed & Steering vs Time (Bottom Right)
    ax_spd = fig.add_subplot(gs[1, 1])
    ax_spd.set_facecolor("#1e293b")
    ax_spd.plot(t, spd, color="#38bdf8", linewidth=1.8, label="Actual Speed (m/s)")
    ax_spd.plot(t, tgt_spd, color="#34d399", linestyle="--", linewidth=1.5, label="Target Speed (m/s)")
    ax_spd.grid(True, color="#334155", linestyle="--", alpha=0.6)
    ax_spd.set_xlabel("Time (seconds)", color="#cbd5e1")
    ax_spd.set_ylabel("Speed (m/s)", color="#cbd5e1")
    ax_spd.tick_params(colors="#94a3b8")
    ax_spd.legend(loc="upper right", facecolor="#0f172a", edgecolor="#475569", labelcolor="#f1f5f9", fontsize=9)

    plt.tight_layout()
    if save_path:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=160, facecolor=fig.get_facecolor(), edgecolor="none")
        print(f"Saved diagnostic telemetry plot to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analyze Pure Pursuit path tracking telemetry")
    parser.add_argument("--telem", type=Path, default=_REPO_ROOT / "common" / "maps" / "telemetry_pure_pursuit.csv")
    parser.add_argument("--raceline", type=Path, default=_REPO_ROOT / "common" / "maps" / "raceline.csv")
    parser.add_argument("--save", type=Path, default=None)
    args = parser.parse_args()

    analyze_telemetry(args.telem, args.raceline, args.save)


if __name__ == "__main__":
    main()
