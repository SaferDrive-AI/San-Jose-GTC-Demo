#!/bin/bash
# =============================================================================
# Run overview demo — fixed camera, no stalled vehicle, no zoom/rotation
#
# Usage:
#   bash run_overview.sh
#   bash run_overview.sh --no-gui
# =============================================================================

set -e

PYTHON="${PYTHON:-/home/yilinwang/miniconda3/envs/terasimYW/bin/python}"
export PATH="/home/yilinwang/miniconda3/envs/terasimYW/bin:$PATH"
SCRIPT="main_overview.py"
OUTPUT_DIR="traffic_data_analysis/delay_result"
mkdir -p "$OUTPUT_DIR"

export PROJ_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJ_DIR"
NEW_DIR="${PROJ_DIR}/san_jose_full_new"

NET_FILE="${NEW_DIR}/osm_modified.net.xml"
FLOWS_ROU="${NEW_DIR}/intersection_flows.rou.xml"
SIM_CFG="${NEW_DIR}/simulation.sumocfg"

SIM_TIME=30
GUI_FLAG="--gui"
MODE="bench"

for arg in "$@"; do
    case "$arg" in
        --no-gui) GUI_FLAG="--no-gui" ;;
    esac
done

echo ""
echo "============================================================"
echo "  Overview Demo — fixed camera, no stalled vehicle"
echo "  Config:   $SIM_CFG"
echo "  Network:  $NET_FILE"
echo "  Routes:   $FLOWS_ROU"
echo "  Sim time: ${SIM_TIME}s"
echo "============================================================"

"$PYTHON" "$SCRIPT" \
    --config "$SIM_CFG" \
    --net-file "$NET_FILE" \
    --route-file "$FLOWS_ROU" \
    --obstacles "37.335570, -121.891916" \
    --mode "$MODE" \
    --output "$OUTPUT_DIR/delay_overview.json" \
    --sim-time "$SIM_TIME" \
    --demo-config "demo_config.json" \
    --bg-vehicle-count 3500 \
    --warmup 10 \
    "$GUI_FLAG"

echo ""
echo "============================================================"
echo "  Overview demo completed"
echo "  Output: $OUTPUT_DIR/delay_overview.json"
echo "============================================================"
