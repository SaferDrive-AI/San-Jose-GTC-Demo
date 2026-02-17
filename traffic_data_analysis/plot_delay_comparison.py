#!/usr/bin/env python3
"""
Plot delay comparison for 5-case SUMO simulation results.

Usage:
    python plot_delay_comparison.py                      # defaults to SB_thr
    python plot_delay_comparison.py --direction EB_thr
    python plot_delay_comparison.py --direction WB_thr
"""

import json
import os
import argparse
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# Resolve paths relative to the project root (parent of traffic_data_analysis/)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    parser = argparse.ArgumentParser(description="Plot 5-case delay comparison")
    parser.add_argument("--direction", default="WB_thr",
                        help="Obstacle direction tag, e.g. EB_thr, WB_thr, SB_thr, NB_thr")
    parser.add_argument("--output-dir", default=None,
                        help="Directory containing the delay JSON files")
    args = parser.parse_args()

    d = args.direction
    out_dir = args.output_dir or os.path.join(PROJECT_ROOT, "traffic_data_analysis", "delay_result")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # --- Load data (5 cases) ---
    cases = {
        "Default Plan\n No Stalled Veh":          f"{out_dir}/delay_benchmark.json",
        "Default Plan\n Stalled Veh":             f"{out_dir}/delay_bench_{d}_obstacle.json",
        "Overall-opt Plan\n No Stalled Veh":              f"{out_dir}/delay_opt_{d}_no_obstacle.json",
        "Overall-opt Plan\n Stalled Veh":                 f"{out_dir}/delay_opt_{d}_obstacle.json",
        "lane specific Plan\n Stalled Veh":        f"{out_dir}/delay_dynamic_{d}_obstacle.json",
    }

    labels = list(cases.keys())
    delays = []
    arrived = []
    for name, path in cases.items():
        with open(path) as f:
            r = json.load(f)["results"]
            delays.append(r["average_delay"])
            arrived.append(r["total_arrived"])
            print(f"  {name.replace(chr(10), ' ')}: delay={r['average_delay']:.2f}s, arrived={r['total_arrived']}")

    # --- Plot ---
    fig, ax1 = plt.subplots(figsize=(14, 6))
    # Convert direction tag to readable name (e.g. "SB_thr" -> "SB Thr")
    dir_name = d.replace("_", " ").title()
    fig.suptitle(f"{dir_name} Stalled Vehicle: Mobility Evaluation", fontsize=18, fontweight="bold")

    x = np.arange(len(labels))
    bar_width = 0.30

    # Left y-axis: Average Delay
    bars1 = ax1.bar(x - bar_width / 2, delays, bar_width,
                    color="#5B9BD5", edgecolor="white", label="Avg Delay (s)")
    ax1.set_ylabel("Avg Delay (seconds)", fontsize=13, color="#5B9BD5")
    ax1.tick_params(axis="y", labelcolor="#5B9BD5")
    ax1.set_ylim(0, max(delays) * 1.25)

    for bar, val in zip(bars1, delays):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                 f"{val:.1f}s", ha="center", va="bottom",
                 fontsize=11, fontweight="bold", color="#5B9BD5")

    # Right y-axis: Vehicles Arrived
    ax2 = ax1.twinx()
    bars2 = ax2.bar(x + bar_width / 2, arrived, bar_width,
                    color="#70AD47", edgecolor="white", label="Vehicles Arrived")
    ax2.set_ylabel("Vehicles Arrived", fontsize=13, color="#70AD47")
    ax2.tick_params(axis="y", labelcolor="#70AD47")
    ax2.set_ylim(0, max(arrived) * 1.25)

    for bar, val in zip(bars2, arrived):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 8,
                 f"{val}", ha="center", va="bottom",
                 fontsize=11, fontweight="bold", color="#70AD47")

    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, fontsize=10)
    ax1.spines["top"].set_visible(False)
    ax2.spines["top"].set_visible(False)

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=11)

    plt.tight_layout()
    outpath = f"{out_dir}/delay_comparison_{d}_{timestamp}.png"
    plt.savefig(outpath, dpi=150, bbox_inches="tight")
    print(f"\nSaved: {outpath}")


if __name__ == "__main__":
    main()
