#!/usr/bin/env python3
"""
Generate SUMO route files with directional traffic flow
Adjust traffic flow density in different directions based on obstacle direction
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import argparse
import math
import sys
import os


class DirectionalRouteGenerator:
    """Directional route generator"""

    def __init__(self, net_file, junction_id):
        """
        Initialize generator

        Args:
            net_file: Network file path
            junction_id: Target junction ID
        """
        self.net_file = net_file
        self.junction_id = junction_id
        self.load_network()

    def load_network(self):
        """Load network information"""
        tree = ET.parse(self.net_file)
        root = tree.getroot()

        # Find target junction
        junction = root.find(f".//junction[@id='{self.junction_id}']")
        if junction is None:
            raise ValueError(f"Junction not found: {self.junction_id}")

        # Get junction center position
        self.junction_x = float(junction.get('x'))
        self.junction_y = float(junction.get('y'))
        print(f"✓ Junction position: ({self.junction_x:.2f}, {self.junction_y:.2f})")

        # Get incoming edges to junction
        inc_lanes = junction.get('incLanes', '').split()
        self.incoming_edges = set()
        for lane in inc_lanes:
            if '_w' in lane or lane.startswith(':'):
                continue  # Skip sidewalks and internal edges
            edge_id = lane.rsplit('_', 1)[0]
            self.incoming_edges.add(edge_id)

        print(f"✓ Found {len(self.incoming_edges)} incoming edges")

        # Get information for all edges
        self.edges = {}
        for edge in root.findall('.//edge'):
            edge_id = edge.get('id')
            if edge_id.startswith(':'):
                continue  # Skip internal edges

            # Get shape of first lane to determine direction
            lane = edge.find('lane')
            if lane is not None:
                shape_str = lane.get('shape')
                if shape_str:
                    points = [tuple(map(float, p.split(','))) for p in shape_str.split()]
                    if len(points) >= 2:
                        # Calculate edge direction (angle)
                        start = points[0]
                        end = points[-1]
                        angle = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
                        # Normalize to 0-360
                        angle = angle % 360

                        self.edges[edge_id] = {
                            'shape': points,
                            'angle': angle,
                            'start': start,
                            'end': end
                        }

        print(f"✓ Loaded {len(self.edges)} edges")

    def classify_edge_direction(self, edge_id):
        """
        Classify edge direction

        Returns:
            'east-west' or 'north-south'
        """
        if edge_id not in self.edges:
            return None

        angle = self.edges[edge_id]['angle']

        # East-west direction: -45 to 45 degrees, or 135 to 225 degrees
        # North-south direction: 45 to 135 degrees, or 225 to 315 degrees
        if (angle >= 315 or angle < 45) or (135 <= angle < 225):
            return 'east-west'
        else:
            return 'north-south'

    def get_fringe_edges(self, direction=None):
        """
        Get fringe edges (used as origins and destinations)

        Args:
            direction: 'east-west' or 'north-south' or None (all)

        Returns:
            list of edge IDs
        """
        fringe_edges = []

        for edge_id, edge_info in self.edges.items():
            if edge_id.startswith('-'):
                continue  # Skip reverse edges

            # Check if far from junction (fringe edge)
            start = edge_info['start']
            end = edge_info['end']

            # Calculate distance to junction
            dist_start = math.sqrt((start[0] - self.junction_x)**2 +
                                  (start[1] - self.junction_y)**2)
            dist_end = math.sqrt((end[0] - self.junction_x)**2 +
                                (end[1] - self.junction_y)**2)

            # If start point is far from junction (>50m), likely a fringe edge
            if dist_start > 50:
                edge_dir = self.classify_edge_direction(edge_id)
                if direction is None or edge_dir == direction:
                    fringe_edges.append(edge_id)

        return fringe_edges

    def generate_routes(self, output_file, obstacle_direction='east-west',
                       high_flow=100, low_flow=20, sim_time=3600):
        """
        Generate route file

        Args:
            output_file: Output file path
            obstacle_direction: Direction with obstacles ('east-west' or 'north-south')
            high_flow: Traffic flow in direction with obstacles (vehicles/hour)
            low_flow: Traffic flow in direction without obstacles (vehicles/hour)
            sim_time: Simulation duration (seconds)
        """
        print(f"\nGenerating routes:")
        print(f"  Obstacle direction: {obstacle_direction}")
        print(f"  High flow direction: {obstacle_direction} ({high_flow} veh/h)")
        print(f"  Low flow direction: {'north-south' if obstacle_direction == 'east-west' else 'east-west'} ({low_flow} veh/h)")

        # Determine high and low flow directions
        low_direction = 'north-south' if obstacle_direction == 'east-west' else 'east-west'

        # Get fringe edges for each direction
        high_flow_edges = self.get_fringe_edges(obstacle_direction)
        low_flow_edges = self.get_fringe_edges(low_direction)

        print(f"  {obstacle_direction} direction edges: {len(high_flow_edges)}")
        print(f"  {low_direction} direction edges: {len(low_flow_edges)}")

        # Create route XML
        routes = ET.Element('routes')

        # Add vehicle type
        vtype = ET.SubElement(routes, 'vType')
        vtype.set('id', 'passenger')
        vtype.set('vClass', 'passenger')
        vtype.set('speedFactor', '1.0')
        vtype.set('speedDev', '0.1')

        # Generate high flow trips
        veh_id = 0
        interval_high = 3600 / high_flow if high_flow > 0 else 3600  # Vehicle interval (seconds)

        depart_time = 0
        while depart_time < sim_time:
            # Randomly select origin and destination
            import random
            if len(high_flow_edges) >= 2:
                from_edge = random.choice(high_flow_edges)
                to_edge = random.choice([e for e in high_flow_edges if e != from_edge])

                trip = ET.SubElement(routes, 'trip')
                trip.set('id', f'veh_{veh_id}')
                trip.set('type', 'passenger')
                trip.set('depart', f'{depart_time:.2f}')
                trip.set('from', from_edge)
                trip.set('to', to_edge)
                trip.set('departLane', 'best')

                veh_id += 1

            depart_time += interval_high * random.uniform(0.5, 1.5)  # Add randomness

        print(f"  Generated {obstacle_direction} direction vehicles: {veh_id}")

        # Generate low flow trips
        start_veh_id = veh_id
        interval_low = 3600 / low_flow if low_flow > 0 else 3600

        depart_time = 0
        while depart_time < sim_time:
            import random
            if len(low_flow_edges) >= 2:
                from_edge = random.choice(low_flow_edges)
                to_edge = random.choice([e for e in low_flow_edges if e != from_edge])

                trip = ET.SubElement(routes, 'trip')
                trip.set('id', f'veh_{veh_id}')
                trip.set('type', 'passenger')
                trip.set('depart', f'{depart_time:.2f}')
                trip.set('from', from_edge)
                trip.set('to', to_edge)
                trip.set('departLane', 'best')

                veh_id += 1

            depart_time += interval_low * random.uniform(0.5, 1.5)

        print(f"  Generated {low_direction} direction vehicles: {veh_id - start_veh_id}")
        print(f"  Total vehicles: {veh_id}")

        # Format and save
        xml_str = minidom.parseString(ET.tostring(routes)).toprettyxml(indent="    ")
        with open(output_file, 'w') as f:
            f.write(xml_str)

        print(f"\n✓ Route file saved: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate directional traffic flow route file for specific junction',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Obstacle in east-west direction, high east-west traffic, low north-south traffic
  python generate_directional_routes.py \\
      --net-file san_jose_downtown_gtc/osm.net.xml \\
      --junction cluster_1984576776_3478559735_3478559736_3537422682_#1more \\
      --obstacle-direction east-west \\
      --high-flow 120 \\
      --low-flow 30 \\
      --output custom_routes.rou.xml

  # Obstacle in north-south direction, high north-south traffic, low east-west traffic
  python generate_directional_routes.py \\
      --net-file san_jose_downtown_gtc/osm.net.xml \\
      --junction cluster_1984576776_3478559735_3478559736_3537422682_#1more \\
      --obstacle-direction north-south \\
      --high-flow 100 \\
      --low-flow 20 \\
      --output custom_routes.rou.xml
        """
    )

    parser.add_argument('--net-file', required=True,
                       help='SUMO network file')
    parser.add_argument('--junction', required=True,
                       help='Target junction ID')
    parser.add_argument('--obstacle-direction', choices=['east-west', 'north-south'],
                       required=True,
                       help='Direction with obstacles (this direction will have more traffic)')
    parser.add_argument('--high-flow', type=int, default=100,
                       help='Traffic flow in direction with obstacles (vehicles/hour), default 100')
    parser.add_argument('--low-flow', type=int, default=20,
                       help='Traffic flow in direction without obstacles (vehicles/hour), default 20')
    parser.add_argument('--sim-time', type=int, default=1800,
                       help='Simulation duration (seconds), default 1800')
    parser.add_argument('--output', default='custom_routes.rou.xml',
                       help='Output route file path')

    args = parser.parse_args()

    # Check file
    if not os.path.exists(args.net_file):
        print(f"Error: Network file does not exist: {args.net_file}")
        sys.exit(1)

    # Generate routes
    generator = DirectionalRouteGenerator(args.net_file, args.junction)
    generator.generate_routes(
        output_file=args.output,
        obstacle_direction=args.obstacle_direction,
        high_flow=args.high_flow,
        low_flow=args.low_flow,
        sim_time=args.sim_time
    )

    print("\nComplete! Now you can run the simulation:")
    print(f"python sumo_delay_calculator.py \\")
    print(f"    --net-file {args.net_file} \\")
    print(f"    --route-file {args.output} \\")
    print(f"    --obstacles \"37.33251,-121.892360\" \\")
    print(f"    --tls-program tls_config_example.json \\")
    print(f"    --sim-time {args.sim_time} \\")
    print(f"    --output results.json")


if __name__ == '__main__':
    main()
