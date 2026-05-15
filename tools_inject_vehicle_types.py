#!/usr/bin/env python3
"""Inject SUMO vehicle type definitions into a generated route file."""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_VEHICLE_TYPES_FILE = PROJECT_ROOT / "vss_synth" / "vehicle_types.xml"
DEFAULT_FLOW_TYPE = "realistic_traffic_mix"


def inject_vehicle_types(route_file: Path, vehicle_types_file: Path, output_route_file: Path, flow_type: str) -> dict:
    route_tree = ET.parse(route_file)
    route_root = route_tree.getroot()
    template_root = ET.parse(vehicle_types_file).getroot()

    template_children = [
        child
        for child in list(template_root)
        if child.tag in {"vType", "vTypeDistribution"}
    ]
    if not template_children:
        raise ValueError(f"No vType/vTypeDistribution entries found in {vehicle_types_file}")

    available_ids = {
        child.get("id")
        for child in template_children
        if child.get("id")
    }
    if flow_type not in available_ids:
        raise ValueError(f"Flow type '{flow_type}' is not defined in {vehicle_types_file}")

    for child in list(route_root):
        if child.tag in {"vType", "vTypeDistribution"}:
            route_root.remove(child)

    for idx, child in enumerate(template_children):
        route_root.insert(idx, deepcopy(child))

    updated_flow_count = 0
    for flow in route_root.findall("flow"):
        flow.set("type", flow_type)
        updated_flow_count += 1

    output_route_file.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(route_root).write(output_route_file, encoding="utf-8", xml_declaration=True)

    return {
        "route_file": str(route_file),
        "vehicle_types_file": str(vehicle_types_file),
        "output_route_file": str(output_route_file),
        "vehicle_type_count": sum(1 for c in template_children if c.tag == "vType"),
        "vehicle_distribution_count": sum(1 for c in template_children if c.tag == "vTypeDistribution"),
        "flow_type": flow_type,
        "updated_flow_count": updated_flow_count,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route-file", required=True, help="Generated SUMO route file to decorate.")
    parser.add_argument("--output-route-file", required=True, help="Route file to write after vehicle type injection.")
    parser.add_argument("--vehicle-types-file", default=str(DEFAULT_VEHICLE_TYPES_FILE))
    parser.add_argument("--flow-type", default=DEFAULT_FLOW_TYPE)
    args = parser.parse_args()

    result = inject_vehicle_types(
        route_file=Path(args.route_file),
        vehicle_types_file=Path(args.vehicle_types_file),
        output_route_file=Path(args.output_route_file),
        flow_type=args.flow_type,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
