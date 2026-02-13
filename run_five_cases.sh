#!/bin/bash
# Run 4 SUMO simulations (EB_thr obstacle only):
#   Case 1: Benchmark — no obstacle, bench mode (original TLS "org")
#   Case 2: Optimized — no obstacle, opt mode (optimized TLS "opt")
#   Case 3: Optimized + obstacle — EB_thr obstacle, opt mode
#   Case 4: Dynamic + obstacle — EB_thr obstacle, dynamic mode
#
# Usage: bash run_five_cases.sh

set -e

PYTHON="${PYTHON:-/home/yilinwang/miniconda3/envs/terasimYW/bin/python}"
export PATH="/home/yilinwang/miniconda3/envs/terasimYW/bin:$PATH"
SCRIPT="main.py"
OUTPUT_DIR="traffic_data_analysis/delay_result"

mkdir -p "$OUTPUT_DIR"

# Obstacle GPS coordinates
EB_THR="37.335358,-121.892226"

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
        --gui

    echo "  [DONE] $name"
    echo "         delay:     $OUTPUT_DIR/delay_${tag}.json"
    echo "         tripinfo:  $OUTPUT_DIR/tripinfo_${tag}.xml"
    echo "         statistic: $OUTPUT_DIR/statistic_${tag}.xml"
}

# Case 1: Benchmark (no obstacle, bench mode — original TLS)
run_case "Benchmark (no obstacle, original TLS)" "" "bench" "benchmark"

# Case 2: Benchmark + EB_thr obstacle (bench mode - original TLS)
run_case "EB_thr obstacle, optimized TLS" "$EB_THR" "bench" "bench_EB_thr_obstacle"

# Case 3: Optimized + EB_thr obstacle (opt mode — optimized TLS)
run_case "Optimized (no obstacle, optimized TLS)" "$EB_THR" "opt" "opt_EB_thr_obstacle"

## Case 3: Optimized + EB_thr obstacle (opt mode)
#run_case "EB_thr obstacle, optimized TLS" "$EB_THR" "opt" "opt_EB_thr"

# Case 4: Dynamic + EB_thr obstacle (dynamic mode - dynamic TLS)
run_case "EB_thr obstacle, dynamic TLS" "$EB_THR" "dynamic" "dynamic_EB_thr_obstacle"

echo ""
echo "============================================================"
echo "  All $TOTAL cases completed"
echo "============================================================"
