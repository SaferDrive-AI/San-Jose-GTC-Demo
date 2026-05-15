#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash vss_synth/run_vss_gui.sh [options]

Options:
  --input-json PATH    VSS input JSON. Default: vss_synth/sample_vss_data.json
  --output-dir DIR     Output directory. Default: outputs
  --sim-end SECONDS    SUMO end time. Default: 1800
  --no-shapes          Skip vehicle-type injection and use the clean car_normal route file.
  --no-gui             Generate files only; do not launch sumo-gui.
  --start              Start SUMO immediately after opening GUI.
  --dry-run            Print commands without running them.
  -h, --help           Show this help.

Examples:
  bash vss_synth/run_vss_gui.sh
  bash vss_synth/run_vss_gui.sh --input-json vss_synth/sample_vss_data_3_cameras.json
  bash vss_synth/run_vss_gui.sh --no-gui
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

INPUT_JSON="vss_synth/sample_vss_data.json"
OUTPUT_DIR="outputs"
SIM_END="1800"
USE_SHAPES=1
LAUNCH_GUI=1
START_GUI=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input-json)
      INPUT_JSON="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --sim-end)
      SIM_END="$2"
      shift 2
      ;;
    --no-shapes)
      USE_SHAPES=0
      shift
      ;;
    --no-gui)
      LAUNCH_GUI=0
      shift
      ;;
    --start)
      START_GUI=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

BOUNDARY_FILE="$OUTPUT_DIR/vss_gui_boundary.json"
PATTERNS_FILE="$OUTPUT_DIR/vss_gui_patterns.json"
ROUTE_FILE="$PWD/$OUTPUT_DIR/vss_gui_routes.rou.xml"
SHAPED_ROUTE_FILE="$PWD/$OUTPUT_DIR/vss_gui_routes_with_shapes.rou.xml"
GUI_ROUTE_FILE="$ROUTE_FILE"

run_cmd() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  if [[ "$DRY_RUN" == "0" ]]; then
    "$@"
  fi
}

if [[ "$DRY_RUN" == "0" ]]; then
  mkdir -p "$OUTPUT_DIR"
fi

run_cmd python tools_vss_boundary_pipeline.py \
  --input-json "$INPUT_JSON" \
  --net-file san_jose_downtown_gtc/osm.net.xml \
  --boundary-json-out "$BOUNDARY_FILE" \
  --route-out "$ROUTE_FILE" \
  --patterns-out "$PATTERNS_FILE" \
  --sim-end "$SIM_END" \
  --exit-hops 2 \
  --max-corridor-hops 8 \
  --spawn-upstream-hops 2

if [[ "$USE_SHAPES" == "1" ]]; then
  run_cmd python tools_inject_vehicle_types.py \
    --route-file "$ROUTE_FILE" \
    --vehicle-types-file vss_synth/vehicle_types.xml \
    --output-route-file "$SHAPED_ROUTE_FILE" \
    --flow-type realistic_traffic_mix
  GUI_ROUTE_FILE="$SHAPED_ROUTE_FILE"
fi

if [[ "$LAUNCH_GUI" == "1" ]]; then
  GUI_CMD=(
    sumo-gui
    -c san_jose_downtown_gtc/osm.sumocfg
    --route-files "$GUI_ROUTE_FILE"
    --ignore-route-errors true
    --begin 0
    --end "$SIM_END"
  )
  if [[ "$START_GUI" == "1" ]]; then
    GUI_CMD+=(--start)
  fi
  run_cmd "${GUI_CMD[@]}"
else
  echo "Generated route: $ROUTE_FILE"
  if [[ "$USE_SHAPES" == "1" ]]; then
    echo "Generated shaped route: $SHAPED_ROUTE_FILE"
  fi
fi
