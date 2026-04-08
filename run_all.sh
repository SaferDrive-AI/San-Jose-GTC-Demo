#!/bin/bash
# =============================================================================
# Run both demos sequentially:
#   1. main_demo.py   — zoom-in + rotation video
#   2. main_overview.py — fixed camera overview video
#
# Usage:
#   bash run_all.sh
#   bash run_all.sh --no-gui
# =============================================================================

set -e

PYTHON="${PYTHON:-/home/yilinwang/miniconda3/envs/terasimYW/bin/python}"
export PATH="/home/yilinwang/miniconda3/envs/terasimYW/bin:$PATH"
OUTPUT_DIR="traffic_data_analysis/delay_result"
mkdir -p "$OUTPUT_DIR"

export PROJ_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJ_DIR"
NEW_DIR="${PROJ_DIR}/san_jose_full_new"

NET_FILE="${NEW_DIR}/osm_modified.net.xml"
FLOWS_ROU="${NEW_DIR}/intersection_flows.rou.xml"
SIM_CFG="${NEW_DIR}/simulation.sumocfg"
OBSTACLE_POS="37.335570, -121.891916"

GUI_FLAG="--gui"
MODE="bench"

for arg in "$@"; do
    case "$arg" in
        --no-gui) GUI_FLAG="--no-gui" ;;
    esac
done

# ============================================================
#  Task 1: main_demo.py (zoom-in + rotation)
# ============================================================
echo ""
echo "============================================================"
echo "  Task 1/2: main_demo.py — zoom-in + rotation"
echo "  Config:   $SIM_CFG"
echo "  Network:  $NET_FILE"
echo "  Obstacle: $OBSTACLE_POS"
echo "============================================================"

"$PYTHON" main_demo.py \
    --config "$SIM_CFG" \
    --net-file "$NET_FILE" \
    --route-file "$FLOWS_ROU" \
    --obstacles "$OBSTACLE_POS" \
    --mode "$MODE" \
    --output "$OUTPUT_DIR/delay_full_new_WB_thr.json" \
    --sim-time 40 \
    --demo-config "demo_config.json" \
    --bg-vehicle-count 3500 \
    "$GUI_FLAG"

echo ""
echo "============================================================"
echo "  Task 1/2 completed: main_demo.py"
echo "============================================================"

# ============================================================
#  Task 2: main_overview.py (fixed camera overview)
# ============================================================
echo ""
echo "============================================================"
echo "  Task 2/2: main_overview.py — fixed camera overview"
echo "  Config:   $SIM_CFG"
echo "  Network:  $NET_FILE"
echo "  Obstacle: $OBSTACLE_POS"
echo "============================================================"

"$PYTHON" main_overview.py \
    --config "$SIM_CFG" \
    --net-file "$NET_FILE" \
    --route-file "$FLOWS_ROU" \
    --obstacles "$OBSTACLE_POS" \
    --mode "$MODE" \
    --output "$OUTPUT_DIR/delay_overview.json" \
    --sim-time 30 \
    --demo-config "demo_config.json" \
    --bg-vehicle-count 3500 \
    --warmup 10 \
    "$GUI_FLAG"

echo ""
echo "============================================================"
echo "  Task 2/2 completed: main_overview.py"
echo "============================================================"
echo ""
echo "  All demos finished!"
