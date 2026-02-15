#!/bin/bash
# Run 4 SUMO simulations (SB_thr obstacle):
#   Case 1: Benchmark — no obstacle, bench mode (original TLS "org")
#   Case 2: Benchmark + SB_thr obstacle (bench mode - original TLS)
#   Case 3: Optimized (Static) + SB_thr obstacle (opt mode)
#   Case 4: Optimized (Dynamic) + SB_thr obstacle (dynamic mode)
#
# Usage: bash run_five_cases.sh

set -e

PYTHON="${PYTHON:-/home/yilinwang/miniconda3/envs/terasimYW/bin/python}"
export PATH="/home/yilinwang/miniconda3/envs/terasimYW/bin:$PATH"
SCRIPT="main.py"
OUTPUT_DIR="traffic_data_analysis/delay_result"

mkdir -p "$OUTPUT_DIR"

# Obstacle GPS coordinates
SB_THR="37.335594,-121.892218"

TOTAL=4
RUN=0

run_case() {
    local name="$1"
    local obstacles="$2"
    local mode="$3"
    local tag="$4"

    RUN=$((RUN + 1))
    echo ""
    echo "============================================================"
    echo "  Case $RUN/$TOTAL: $name"
    echo "============================================================"

    $PYTHON "$SCRIPT" \
        --obstacles "$obstacles" \
        --mode "$mode" \
        --output "$OUTPUT_DIR/delay_${tag}.json" \
        --tripinfo-output "$OUTPUT_DIR/tripinfo_${tag}.xml" \
        --statistic-output "$OUTPUT_DIR/statistic_${tag}.xml" \
        --no-gui

    echo "  [DONE] $name"
    echo "         delay:     $OUTPUT_DIR/delay_${tag}.json"
    echo "         tripinfo:  $OUTPUT_DIR/tripinfo_${tag}.xml"
    echo "         statistic: $OUTPUT_DIR/statistic_${tag}.xml"
}

# Case 1: Benchmark (no obstacle, bench mode — original TLS)
run_case "Benchmark (no obstacle, original TLS)" "" "bench" "benchmark"

# Case 2: Benchmark + SB_thr obstacle (bench mode - original TLS)
run_case "SB_thr obstacle, original TLS" "$SB_THR" "bench" "bench_SB_thr_obstacle"

# Case 3: Optimized (Static) + SB_thr obstacle (opt mode)
run_case "SB_thr obstacle, optimized TLS" "$SB_THR" "opt" "opt_SB_thr_obstacle"

# Case 4: Optimized (Dynamic) + SB_thr obstacle (dynamic mode)
run_case "SB_thr obstacle, dynamic TLS" "$SB_THR" "dynamic" "dynamic_SB_thr_obstacle"

echo ""
echo "============================================================"
echo "  All $TOTAL cases completed"
echo "============================================================"
