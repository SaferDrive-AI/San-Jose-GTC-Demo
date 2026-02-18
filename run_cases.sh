#!/bin/bash
# Run 4 SUMO simulations (obstacle):
#   Case 1: Benchmark — no obstacle, bench mode (original TLS "org")
#   Case 2: Benchmark + obstacle (bench mode - original TLS)
#   Case 3: Optimized (Static) + obstacle (opt mode)
#   Case 4: Optimized (Dynamic) + obstacle (dynamic mode)
#
# Usage: bash run_cases.sh

set -e

PYTHON="${PYTHON:-/home/yilinwang/miniconda3/envs/terasimYW/bin/python}"
export PATH="/home/yilinwang/miniconda3/envs/terasimYW/bin:$PATH"
SCRIPT="main.py"
OUTPUT_DIR="traffic_data_analysis/delay_result"
mkdir -p "$OUTPUT_DIR"

# Obstacle GPS coordinates
EB_LEFT="37.335379, -121.892249"
EB_THR="37.335358, -121.892248"
EB_RIGHT="37.335338, -121.892208"
WB_LEFT="37.335558, -121.891889"
WB_THR="37.335577, -121.891913"
WB_RIGHT="37.335601, -121.891930"
NB_LEFT="37.335328, -121.891956"
NB_THR="37.335340, -121.891927"
NB_RIGHT="37.335356, -121.891898"
SB_LEFT="37.335605, -121.892187"
SB_THR="37.335353, -121.892234"
SB_RIGHT="37.335578, -121.892244"

# Select the obstacle position
OBSTACLE_POS=${WB_THR}
PREFIX="WB_thr"

TOTAL=5
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

    "$PYTHON" "$SCRIPT" \
        --obstacles "$obstacles" \
        --mode "$mode" \
        --output "$OUTPUT_DIR/delay_${tag}.json" \
        --no-gui
        # --tripinfo-output "$OUTPUT_DIR/tripinfo_${tag}.xml" \
        # --statistic-output "$OUTPUT_DIR/statistic_${tag}.xml" \
        # --no-gui

    echo "  [DONE] $name"
    echo "         delay:     $OUTPUT_DIR/delay_${tag}.json"
    echo "         tripinfo:  $OUTPUT_DIR/tripinfo_${tag}.xml"
    echo "         statistic: $OUTPUT_DIR/statistic_${tag}.xml"
}

# Case 1: Benchmark (no obstacle, bench mode — original TLS)
run_case "Benchmark (no obstacle, original TLS)" "" "bench" "benchmark"

# Case 2: Benchmark + obstacle (bench mode - original TLS)
run_case "${PREFIX} obstacle, original TLS" "$OBSTACLE_POS" "bench" "bench_${PREFIX}_obstacle"

# Case 3: Optimized (Static) + no obstacle  (opt mode)
run_case "Optimized (Static) + no obstacle" "" "opt" "opt_${PREFIX}_no_obstacle"

# Case 4: Optimized (Static) + obstacle (opt mode)
run_case "${PREFIX} obstacle, optimized TLS" "$OBSTACLE_POS" "opt" "opt_${PREFIX}_obstacle"

# Case 5: Optimized (Dynamic) + obstacle (dynamic mode)
run_case "${PREFIX} obstacle, dynamic TLS" "$OBSTACLE_POS" "dynamic" "dynamic_${PREFIX}_obstacle"

echo ""
echo "============================================================"
echo "  All $TOTAL cases completed"
echo "============================================================"

"$PYTHON" traffic_data_analysis/plot_delay_comparison.py --direction "${PREFIX}"