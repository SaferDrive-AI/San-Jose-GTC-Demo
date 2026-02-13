#!/bin/bash
# Run 25 SUMO simulations:
#   Run 1:     Benchmark (no obstacles, static mode)
#   Runs 2-13: 12 obstacle scenarios, static mode
#   Runs 14-25: 12 obstacle scenarios, dynamic mode
# Usage: bash run_simulations.sh

PYTHON="/home/yilinwang/miniconda3/envs/terasimYW/bin/python"
SCRIPT="main.py"
NET_FILE="san_jose_downtown_gtc/osm.net.xml"
ROUTE_FILE="san_jose_downtown_gtc/directional_traffic.rou.xml"
SIM_TIME=1800
OUTPUT_DIR="traffic_data_analysis/delay_result"

mkdir -p "$OUTPUT_DIR"

# --- Obstacle GPS coordinates (lat,lon) ---
EB_LEFT="37.335379, -121.892249"
EB_THR="37.335358, -121.892226"
EB_RIGHT="37.335338, -121.892208"
WB_LEFT="37.335558, -121.891889"
WB_THR="37.335577, -121.891913"
WB_RIGHT="37.335601, -121.891930"
NB_LEFT="37.335328, -121.891956"
NB_THR="37.335340, -121.891927"
NB_RIGHT="37.335356, -121.891898"
SB_LEFT="37.335605, -121.892187"
SB_THR="37.335594, -121.892218"
SB_RIGHT="37.335578, -121.892244"

CASES=("EB_left" "EB_thr" "EB_right" \
       "WB_left" "WB_thr" "WB_right" \
       "NB_left" "NB_thr" "NB_right" \
       "SB_left" "SB_thr" "SB_right")

COORDS=("$EB_LEFT" "$EB_THR" "$EB_RIGHT" \
        "$WB_LEFT" "$WB_THR" "$WB_RIGHT" \
        "$NB_LEFT" "$NB_THR" "$NB_RIGHT" \
        "$SB_LEFT" "$SB_THR" "$SB_RIGHT")

TOTAL=25

# ============================================================
# Run 1: Benchmark (no obstacles, static mode)
# ============================================================
echo "=========================================="
echo "[1/$TOTAL] Running benchmark (no obstacles, static)..."
echo "=========================================="
$PYTHON $SCRIPT \
    --net-file "$NET_FILE" \
    --route-file "$ROUTE_FILE" \
    --obstacles "" \
    --sim-time $SIM_TIME \
    --mode "static" \
    --output "$OUTPUT_DIR/delay_benchmark.json"

# ============================================================
# Runs 2-13: Obstacle scenarios, static mode
# ============================================================
for i in "${!CASES[@]}"; do
    TAG="${CASES[$i]}"
    OBS="${COORDS[$i]}"
    RUN_NUM=$((i + 2))

    echo ""
    echo "=========================================="
    echo "[$RUN_NUM/$TOTAL] Running ${TAG} (static)..."
    echo "  Obstacle: $OBS"
    echo "=========================================="

    $PYTHON $SCRIPT \
        --net-file "$NET_FILE" \
        --route-file "$ROUTE_FILE" \
        --obstacles "$OBS" \
        --sim-time $SIM_TIME \
        --mode "static" \
        --output "$OUTPUT_DIR/delay_static_${TAG}.json"
done

# ============================================================
# Runs 14-25: Obstacle scenarios, dynamic mode
# ============================================================
for i in "${!CASES[@]}"; do
    TAG="${CASES[$i]}"
    OBS="${COORDS[$i]}"
    RUN_NUM=$((i + 14))

    echo ""
    echo "=========================================="
    echo "[$RUN_NUM/$TOTAL] Running ${TAG} (dynamic)..."
    echo "  Obstacle: $OBS"
    echo "=========================================="

    $PYTHON $SCRIPT \
        --net-file "$NET_FILE" \
        --route-file "$ROUTE_FILE" \
        --obstacles "$OBS" \
        --sim-time $SIM_TIME \
        --mode "dynamic" \
        --output "$OUTPUT_DIR/delay_dynamic_${TAG}.json"
done

echo ""
echo "=========================================="
echo "All $TOTAL simulations completed."
echo "Results saved in: $OUTPUT_DIR/"
echo "=========================================="
