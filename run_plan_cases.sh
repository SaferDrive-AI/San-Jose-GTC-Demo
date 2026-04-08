#!/bin/bash
# Run 4 SUMO simulations with different signal plans + WB stalled vehicle:
#   Case 1: plan_1 — EW green 26.5s, NS green 19.0s (closest to org)
#   Case 2: plan_2 — EW green 27.3s, NS green 18.2s
#   Case 3: plan_3 — EW green 28.2s, NS green 17.3s
#   Case 4: plan_4 — optimal (1418903639#0_2), EW green 29.0s, NS green 16.5s
#
# Usage: bash run_plan_cases.sh

set -e

PYTHON="${PYTHON:-/home/yilinwang/miniconda3/envs/terasimYW/bin/python}"
export PATH="/home/yilinwang/miniconda3/envs/terasimYW/bin:$PATH"
SCRIPT="main.py"
OUTPUT_DIR="traffic_data_analysis/delay_result"
mkdir -p "$OUTPUT_DIR"

# Obstacle GPS coordinates — WB through
WB_THR="37.335566, -121.891927"
OBSTACLE_POS=${WB_THR}

TOTAL=5
RUN=0

run_case() {
    local name="$1"
    local obstacles="$2"
    local program_id="$3"
    local tag="$4"

    RUN=$((RUN + 1))
    echo ""
    echo "============================================================"
    echo "  Case $RUN/$TOTAL: $name"
    echo "  Program ID: $program_id"
    echo "============================================================"

    "$PYTHON" "$SCRIPT" \
        --obstacles "$obstacles" \
        --mode "dynamic" \
        --program-id "$program_id" \
        --output "$OUTPUT_DIR/delay_${tag}.json" \
        --gui

    echo "  [DONE] $name"
    echo "         delay: $OUTPUT_DIR/delay_${tag}.json"
}

## Case 1: plan_1 — EW green 26.5s, NS main 19.0s (closest to org)
#run_case "Plan 1: EW=26.5s, NS=19.0s (closest to default)" \
#         "$OBSTACLE_POS" "wb_plan_1" "plan_1_WB_thr"

# Case 2: plan_2 — EW green 27.3s, NS main 18.2s
run_case "Plan 2: EW=27.3s, NS=18.2s" \
         "$OBSTACLE_POS" "wb_plan_2" "plan_2_WB_thr"

# Case 3: plan_3 — EW green 28.2s, NS main 17.3s
run_case "Plan 3: EW=28.2s, NS=17.3s" \
         "$OBSTACLE_POS" "wb_plan_3" "plan_3_WB_thr"

# Case 4: plan_4 (optimal) — EW green 29.0s, NS main 16.5s
run_case "Plan 4 (Optimal): EW=29.0s, NS=16.5s" \
         "$OBSTACLE_POS" "1418903639#0_2" "plan_4_WB_thr"

# Case 5: org — original TLS plan with stalled vehicle
run_case "Org: original TLS plan with stalled vehicle" \
         "$OBSTACLE_POS" "org" "org_WB_thr"

echo ""
echo "============================================================"
echo "  All $TOTAL plan cases completed"
echo "============================================================"

# Plot comparison
"$PYTHON" traffic_data_analysis/plot_plan_comparison.py
