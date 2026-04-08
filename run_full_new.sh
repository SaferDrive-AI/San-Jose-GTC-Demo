#!/bin/bash
# =============================================================================
# Run SUMO simulation on the san_jose_full_new network
#
# This script:
#   1. Generates background random trips for the full network
#   2. Merges them with the intersection-specific flows (from san_jose_downtown_gtc)
#   3. Creates a simulation.sumocfg
#   4. Runs main.py with stalled vehicle + initial vehicles + TLS plan
#
# Usage:
#   bash run_full_new.sh              # default: WB stalled vehicle + optimal plan
#   bash run_full_new.sh --no-gui     # headless mode
#   bash run_full_new.sh --skip-setup # skip trip generation (reuse existing files)
# =============================================================================

set -e

PYTHON="${PYTHON:-/home/yilinwang/miniconda3/envs/terasimYW/bin/python}"
export PATH="/home/yilinwang/miniconda3/envs/terasimYW/bin:$PATH"
SCRIPT="main_demo.py"
OUTPUT_DIR="traffic_data_analysis/delay_result"
mkdir -p "$OUTPUT_DIR"

# --- Directories ---
export PROJ_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJ_DIR"
NEW_DIR="${PROJ_DIR}/san_jose_full_new"
OLD_DIR="${PROJ_DIR}/san_jose_downtown_gtc"

# --- Network and config files ---
NET_FILE="${NEW_DIR}/osm_modified.net.xml"
TLS_FILE="${NEW_DIR}/osm.tls.xml"
OLD_ROU="${OLD_DIR}/osm.rou.xml"
BG_TRIPS="${NEW_DIR}/background_trips.trips.xml"
FLOWS_ROU="${NEW_DIR}/intersection_flows.rou.xml"
SIM_CFG="${NEW_DIR}/simulation.sumocfg"

# --- SUMO tools ---
RANDOM_TRIPS="/home/yilinwang/miniconda3/envs/terasimYW/lib/python3.10/site-packages/sumo/tools/randomTrips.py"

# --- Obstacle GPS coordinates (WB through — same as small network) ---
WB_THR="37.335570, -121.891916"
OBSTACLE_POS="${WB_THR}"

# --- Demo recording parameters ---
SIM_TIME=40            # main simulation duration

# --- Parse arguments ---
SKIP_SETUP=true
GUI_FLAG="--gui"
PROGRAM_ID=""
MODE="bench"

for arg in "$@"; do
    case "$arg" in
        --skip-setup)  SKIP_SETUP=true ;;
        --no-gui)      GUI_FLAG="--no-gui" ;;
        --bench)       MODE="bench"; PROGRAM_ID="" ;;
    esac
done

# ============================================================
#  Step 1: Generate background random trips
# ============================================================
if [ "$SKIP_SETUP" = false ]; then
    echo ""
    echo "============================================================"
    echo "  Step 1: Generating background random trips"
    echo "  Network: $NET_FILE"
    echo "  Period: $BG_PERIOD s  |  Time: ${BG_BEGIN}-${BG_END} s"
    echo "============================================================"

    "$PYTHON" "$RANDOM_TRIPS" \
        -n "$NET_FILE" \
        -o "$BG_TRIPS" \
        -b "$BG_BEGIN" \
        -e "$BG_END" \
        -p "$BG_PERIOD" \
        --seed "$BG_SEED" \
        --fringe-factor 10 \
        --allow-fringe \
        --validate \
        --vehicle-class passenger \
        --prefix "bg"

    echo "  [DONE] Background trips: $BG_TRIPS"

    # ============================================================
    #  Step 2: Create intersection_flows.rou.xml (flows start at WARMUP_TIME)
    # ============================================================
    echo ""
    echo "============================================================"
    echo "  Step 2: Creating intersection flows (begin=${WARMUP_TIME}s)"
    echo "============================================================"

    export WARMUP_TIME SIM_TIME
    "$PYTHON" - <<'FLOWS_SCRIPT'
import xml.etree.ElementTree as ET
import os

proj_dir    = os.environ.get("PROJ_DIR", ".")
warmup_time = int(os.environ.get("WARMUP_TIME", "1000"))
sim_time    = int(os.environ.get("SIM_TIME", "1800"))
old_rou     = os.path.join(proj_dir, "san_jose_downtown_gtc", "osm.rou.xml")
output      = os.path.join(proj_dir, "san_jose_full_new", "intersection_flows.rou.xml")

old_tree = ET.parse(old_rou)
old_root = old_tree.getroot()

routes_elem = ET.Element("routes")
routes_elem.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
routes_elem.set("xsi:noNamespaceSchemaLocation",
                "http://sumo.dlr.de/xsd/routes_file.xsd")

flow_begin = str(warmup_time)
flow_end   = str(warmup_time + sim_time)

for child in old_root:
    if child.tag == "flow":
        child.set("begin", flow_begin)
        child.set("end", flow_end)
    routes_elem.append(child)

tree = ET.ElementTree(routes_elem)
ET.indent(tree, space="    ")
tree.write(output, xml_declaration=True, encoding="UTF-8")

n_routes = len(routes_elem.findall("route"))
n_flows  = len(routes_elem.findall("flow"))
n_vtypes = len(routes_elem.findall("vType"))
print(f"  Created: {n_routes} routes, {n_flows} flows, {n_vtypes} vTypes")
print(f"  Flow window: {flow_begin}s – {flow_end}s")
print(f"  Output: {output}")

# Also assign realistic_traffic_mix to background trips
bg_path = os.path.join(proj_dir, "san_jose_full_new", "background_trips.trips.xml")
bg_tree = ET.parse(bg_path)
bg_root = bg_tree.getroot()
# Remove vType elements first (can't modify list while iterating)
for vt in bg_root.findall("vType"):
    bg_root.remove(vt)
# Then set type on all trips
n_typed = 0
for trip in bg_root.findall("trip"):
    trip.set("type", "realistic_traffic_mix")
    n_typed += 1

# Add warmup-only flows through the intersection corridor
warmup_flows = [
    {"id": "warmup_flow_sb", "from": "-416901153#1", "to": "417034134#1",
     "begin": "0", "end": str(warmup_time), "vehsPerHour": "200",
     "type": "realistic_traffic_mix", "departLane": "best", "departSpeed": "max"},
    {"id": "warmup_flow_nb", "from": "-417034134#1", "to": "157781953#2",
     "begin": "0", "end": str(warmup_time), "vehsPerHour": "200",
     "type": "realistic_traffic_mix", "departLane": "best", "departSpeed": "max"},
]
for f in warmup_flows:
    elem = ET.SubElement(bg_root, "flow")
    for k, v in f.items():
        elem.set(k, v)
print(f"  Added {len(warmup_flows)} warmup-only flows (0–{warmup_time}s)")

bg_tree.write(bg_path, xml_declaration=True, encoding="UTF-8")
print(f"  Assigned type='realistic_traffic_mix' to {n_typed} bg trips")
FLOWS_SCRIPT

    echo "  [DONE] Intersection flows + bg trips configured"

    # ============================================================
    #  Step 3: Create simulation.sumocfg
    # ============================================================
    echo ""
    echo "============================================================"
    echo "  Step 3: Creating simulation.sumocfg"
    echo "============================================================"

    cat > "$SIM_CFG" <<'SUMOCFG'
<?xml version="1.0" encoding="UTF-8"?>
<sumoConfiguration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                   xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">

    <input>
        <net-file value="osm.net.xml.gz"/>
        <route-files value="intersection_flows.rou.xml,background_trips.trips.xml"/>
        <additional-files value="osm.tls.xml"/>
    </input>

    <processing>
        <ignore-route-errors value="true"/>
        <tls.actuated.jam-threshold value="30"/>
        <lateral-resolution value="0.2"/>
    </processing>

    <routing>
        <device.rerouting.probability value="1.0"/>
        <device.rerouting.period value="30"/>
        <device.rerouting.pre-period value="0"/>
        <device.rerouting.adaptation-steps value="18"/>
        <device.rerouting.adaptation-interval value="10"/>
        <device.rerouting.with-taz value="false"/>
    </routing>

    <report>
        <verbose value="true"/>
        <duration-log.statistics value="true"/>
        <no-step-log value="true"/>
    </report>

    <time>
        <step-length value="0.1"/>
    </time>

    <gui_only>
        <gui-settings-file value="osm.view.xml"/>
    </gui_only>

</sumoConfiguration>
SUMOCFG

    echo "  [DONE] Config: $SIM_CFG"

    # Ensure TLS file is in place
    if [ ! -f "$TLS_FILE" ]; then
        cp "$OLD_DIR/osm.tls.xml" "$TLS_FILE"
        echo "  [DONE] Copied TLS settings"
    fi
fi

# ============================================================
#  Step 4: Run simulation
# ============================================================
echo ""
echo "============================================================"
echo "  Step 4: Running simulation on san_jose_full_new"
echo "  Config:   $SIM_CFG"
echo "  Network:  $NET_FILE"
echo "  Routes:   $FLOWS_ROU"
echo "  Obstacle: $OBSTACLE_POS"
echo "  Mode:     $MODE"
if [ -n "$PROGRAM_ID" ]; then
    echo "  Program:  $PROGRAM_ID"
fi
echo "============================================================"

CMD=("$PYTHON" "$SCRIPT" \
    --config "$SIM_CFG" \
    --net-file "$NET_FILE" \
    --route-file "$FLOWS_ROU" \
    --obstacles "$OBSTACLE_POS" \
    --mode "$MODE" \
    --output "$OUTPUT_DIR/delay_full_new_WB_thr.json" \
    --sim-time "$SIM_TIME" \
    --demo-config "demo_config.json" \
    --bg-vehicle-count 3500 \
    "$GUI_FLAG")

if [ -n "$PROGRAM_ID" ]; then
    CMD+=(--program-id "$PROGRAM_ID")
fi

"${CMD[@]}"

echo ""
echo "============================================================"
echo "  Simulation completed"
echo "  Output: $OUTPUT_DIR/delay_full_new_WB_thr.json"
echo "============================================================"

# Video composition is now handled automatically by main_demo.py
# Output: screenshots/<timestamp>.mp4
