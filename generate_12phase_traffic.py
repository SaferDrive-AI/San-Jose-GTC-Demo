#!/usr/bin/env python3
"""
Generate directional traffic flow based on 12-phase routes
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import random
import argparse


def generate_directional_traffic(
    base_route_file,
    output_file,
    ew_flow=150,  # East-west traffic flow (vehicles/hour)
    ns_flow=30,   # North-south traffic flow (vehicles/hour)
    sim_time=1800
):
    """
    Generate directional traffic flow (using flow elements)

    Args:
        base_route_file: Base route file (containing 12 route definitions)
        output_file: Output file
        ew_flow: East-west traffic flow density (vehicles/hour)
        ns_flow: North-south traffic flow density (vehicles/hour)
        sim_time: Simulation duration (seconds)
    """

    # Route grouping
    ew_routes = {
        'east': {
            'left': 'r_0',      # Eastbound left turn
            'straight': 'r_1',  # Eastbound straight
            'right': 'r_2'      # Eastbound right turn
        },
        'west': {
            'left': 'r_6',      # Westbound left turn
            'straight': 'r_7',  # Westbound straight
            'right': 'r_8'      # Westbound right turn
        }
    }

    ns_routes = {
        'north': {
            'left': 'r_3',      # Northbound left turn
            'straight': 'r_4',  # Northbound straight
            'right': 'r_5'      # Northbound right turn
        },
        'south': {
            'left': 'r_9',      # Southbound left turn
            'straight': 'r_10', # Southbound straight
            'right': 'r_11'     # Southbound right turn
        }
    }

    # Read base routes
    base_tree = ET.parse(base_route_file)
    base_root = base_tree.getroot()

    # Create new routes file
    routes = ET.Element('routes')
    routes.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
    routes.set('xsi:noNamespaceSchemaLocation', 'http://sumo.dlr.de/xsd/routes_file.xsd')

    # Copy route definitions
    for route in base_root.findall('route'):
        routes.append(route)

    # Add vehicle type
    vtype = ET.SubElement(routes, 'vType')
    vtype.set('id', 'passenger')
    vtype.set('vClass', 'passenger')
    vtype.set('speedFactor', '1.0')
    vtype.set('speedDev', '0.1')

    print(f"Generating directional traffic flow:")
    print(f"  East-west direction: {ew_flow} veh/h")
    print(f"  North-south direction: {ns_flow} veh/h")
    print(f"  Simulation duration: {sim_time} seconds")

    # Allocate proportions for different turns (more straight, less turning)
    turn_weights = {
        'straight': 0.6,  # Straight 60%
        'left': 0.2,      # Left turn 20%
        'right': 0.2      # Right turn 20%
    }

    # Calculate traffic flow for each direction (east-west split in half)
    ew_per_direction = ew_flow / 2.0  # East and west each get half
    ns_per_direction = ns_flow / 2.0  # North and south each get half

    ew_count = 0
    ns_count = 0

    # Generate east-west flows
    for direction_name, direction_routes in ew_routes.items():
        for turn_type, route_id in direction_routes.items():
            # Calculate flow rate for this flow
            flow_rate = ew_per_direction * turn_weights[turn_type]

            if flow_rate > 0:
                flow = ET.SubElement(routes, 'flow')
                flow.set('id', f'flow_ew_{direction_name}_{turn_type}')
                flow.set('type', 'passenger')
                flow.set('begin', '0')
                flow.set('end', str(sim_time))
                flow.set('vehsPerHour', f'{flow_rate:.2f}')
                flow.set('route', route_id)
                flow.set('departLane', 'best')
                flow.set('color', '0,0,1')  # Blue for east-west direction

                # Estimate vehicle count
                estimated_vehs = (flow_rate / 3600.0) * sim_time
                ew_count += int(estimated_vehs)

    print(f"  ✓ Generated east-west flows: 6 (estimated ~{ew_count} vehicles)")

    # Generate north-south flows
    for direction_name, direction_routes in ns_routes.items():
        for turn_type, route_id in direction_routes.items():
            # Calculate flow rate for this flow
            flow_rate = ns_per_direction * turn_weights[turn_type]

            if flow_rate > 0:
                flow = ET.SubElement(routes, 'flow')
                flow.set('id', f'flow_ns_{direction_name}_{turn_type}')
                flow.set('type', 'passenger')
                flow.set('begin', '0')
                flow.set('end', str(sim_time))
                flow.set('vehsPerHour', f'{flow_rate:.2f}')
                flow.set('route', route_id)
                flow.set('departLane', 'best')
                flow.set('color', '1,0,0')  # Red for north-south direction

                # Estimate vehicle count
                estimated_vehs = (flow_rate / 3600.0) * sim_time
                ns_count += int(estimated_vehs)

    print(f"  ✓ Generated north-south flows: 6 (estimated ~{ns_count} vehicles)")
    print(f"  Total 12 flows (estimated ~{ew_count + ns_count} vehicles)")

    # Format and save
    xml_str = minidom.parseString(ET.tostring(routes)).toprettyxml(indent="    ")
    with open(output_file, 'w') as f:
        f.write(xml_str)

    print(f"\n✓ Route file saved: {output_file}")
    print(f"\nTraffic characteristics:")
    interval_ew = 3600 / ew_flow if ew_flow > 0 else 0
    interval_ns = 3600 / ns_flow if ns_flow > 0 else 0
    print(f"  - East-west direction: {ew_flow} veh/h (avg interval {interval_ew:.1f}s)")
    print(f"    → High density, prone to congestion with obstacles")
    print(f"  - North-south direction: {ns_flow} veh/h (avg interval {interval_ns:.1f}s)")
    print(f"    → Low density, current green light time too long")
    print(f"\nAdjustment suggestions:")
    print(f"  - Modify total traffic: Edit --ew-flow and --ns-flow parameters")
    print(f"  - Modify turn proportions: Edit vehsPerHour values in generated file")
    print(f"  - Current allocation: Straight 60%, Left turn 20%, Right turn 20%")


def main():
    parser = argparse.ArgumentParser(
        description='Generate directional traffic flow based on 12-phase routes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate scenario with high east-west flow, low north-south flow
  python generate_12phase_traffic.py \\
      --base-routes 12_phase_route.rou.xml \\
      --ew-flow 150 \\
      --ns-flow 30 \\
      --sim-time 1800 \\
      --output directional_traffic.rou.xml

  # Higher east-west flow (prone to congestion)
  python generate_12phase_traffic.py \\
      --base-routes 12_phase_route.rou.xml \\
      --ew-flow 200 \\
      --ns-flow 20 \\
      --output high_ew_traffic.rou.xml
        """
    )

    parser.add_argument('--base-routes', default='12_phase_route.rou.xml',
                       help='Base route file (containing 12 route definitions)')
    parser.add_argument('--ew-flow', type=int, default=150,
                       help='East-west traffic flow (vehicles/hour), default 150')
    parser.add_argument('--ns-flow', type=int, default=30,
                       help='North-south traffic flow (vehicles/hour), default 30')
    parser.add_argument('--sim-time', type=int, default=1800,
                       help='Simulation duration (seconds), default 1800')
    parser.add_argument('--output', default='directional_traffic.rou.xml',
                       help='Output file path')

    args = parser.parse_args()

    generate_directional_traffic(
        base_route_file=args.base_routes,
        output_file=args.output,
        ew_flow=args.ew_flow,
        ns_flow=args.ns_flow,
        sim_time=args.sim_time
    )

    print("\nNext steps:")
    print(f"python sumo_delay_calculator.py \\")
    print(f"    --net-file san_jose_downtown_gtc/osm.net.xml \\")
    print(f"    --route-file {args.output} \\")
    print(f"    --obstacles \"37.335265,-121.892334\" \\")
    print(f"    --tls-program tls_config_example.json \\")
    print(f"    --sim-time {args.sim_time} \\")
    print(f"    --output results.json")


if __name__ == '__main__':
    main()
