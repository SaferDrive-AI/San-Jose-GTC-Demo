#!/usr/bin/env python3
"""
Plot delay comparison for 4-plan SUMO simulation results (WB stalled vehicle).

Compares plan_1 through plan_4(optimal), each with progressively more
EW green time and less NS green time.

Also includes the benchmark (org plan + stalled veh) as a dashed reference line.

Usage:
    python plot_plan_comparison.py
"""

import json
import os
import argparse
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    parser = argparse.ArgumentParser(description="Plot 4-plan delay comparison")
    parser.add_argument("--output-dir", default=None,
                        help="Directory containing the delay JSON files")
    args = parser.parse_args()

    out_dir = args.output_dir or os.path.join(PROJECT_ROOT, "traffic_data_analysis", "delay_result")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # --- Load data (4 plan cases) ---
    cases = {
        "Plan 1":              f"{out_dir}/delay_plan_1_WB_thr.json",
        "Plan 2":              f"{out_dir}/delay_plan_2_WB_thr.json",
        "Plan 3":              f"{out_dir}/delay_plan_3_WB_thr.json",
        "Plan 4\n(Optimal)":   f"{out_dir}/delay_plan_4_WB_thr.json",
    }

    # Load benchmark for reference line
    bench_path = f"{out_dir}/delay_bench_WB_thr_obstacle.json"
    bench_delay = None
    if os.path.exists(bench_path):
        with open(bench_path) as f:
            bench_delay = json.load(f)["results"]["average_delay"]
            print(f"  Benchmark (org + stalled veh): delay={bench_delay:.2f}s")

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
    fig.suptitle("WB Thr Stalled Vehicle: Signal Plan Comparison", fontsize=18, fontweight="bold")

    x = np.arange(len(labels))
    bar_width = 0.30

    # Left y-axis: Average Delay
    bars1 = ax1.bar(x - bar_width / 2, delays, bar_width,
                    color="#5B9BD5", edgecolor="white", label="Avg Delay (s)")
    ax1.set_ylabel("Avg Delay (seconds)", fontsize=13, color="#5B9BD5")
    ax1.tick_params(axis="y", labelcolor="#5B9BD5")
    y_max_delay = max(delays) * 1.25
    if bench_delay is not None:
        y_max_delay = max(y_max_delay, bench_delay * 1.15)
    ax1.set_ylim(0, y_max_delay)

    for bar, val in zip(bars1, delays):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                 f"{val:.1f}s", ha="center", va="bottom",
                 fontsize=11, fontweight="bold", color="#5B9BD5")

    # Benchmark reference line
    if bench_delay is not None:
        ax1.axhline(y=bench_delay, color="#C00000", linestyle="--", linewidth=1.5, alpha=0.8)
        ax1.text(len(labels) - 0.5, bench_delay + y_max_delay * 0.02,
                 f"Default Plan + Stalled Veh: {bench_delay:.1f}s",
                 ha="right", va="bottom", fontsize=10, color="#C00000", fontweight="bold")

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

    # EW green time annotations below bars
    ew_greens = ["EW: 26.5s", "EW: 27.3s", "EW: 28.2s", "EW: 29.0s"]
    for i, txt in enumerate(ew_greens):
        ax1.text(x[i], -y_max_delay * 0.07, txt, ha="center", va="top",
                 fontsize=9, color="gray", style="italic")

    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, fontsize=10)
    ax1.spines["top"].set_visible(False)
    ax2.spines["top"].set_visible(False)

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left", fontsize=11)

    plt.tight_layout()
    outpath = f"{out_dir}/plan_comparison_WB_thr_{timestamp}.png"
    plt.savefig(outpath, dpi=150, bbox_inches="tight")
    print(f"\nSaved: {outpath}")


if __name__ == "__main__":
    main()
